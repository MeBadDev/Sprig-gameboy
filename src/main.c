// ---------------------------------------------------------------------------
// Peanut-GB settings
// ---------------------------------------------------------------------------
#define ENABLE_LCD    1
#define ENABLE_SOUND  1
#define ENABLE_SDCARD 1
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
#define PEANUT_GB_USE_BIOS 0
#define USE_DMA 0

#define VSYNC_REDUCTION_FACTOR 16u
#define SCREEN_REFRESH_CYCLES_REDUCED (SCREEN_REFRESH_CYCLES / VSYNC_REDUCTION_FACTOR)
#define DMG_CLOCK_FREQ_REDUCED        (DMG_CLOCK_FREQ / VSYNC_REDUCTION_FACTOR)

// ---------------------------------------------------------------------------
// C headers
// ---------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ---------------------------------------------------------------------------
// RP2040 headers
// ---------------------------------------------------------------------------
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <hardware/timer.h>
#include <hardware/vreg.h>
#include <pico/bootrom.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <sys/unistd.h>
#include <hardware/irq.h>

// ---------------------------------------------------------------------------
// Project headers
// ---------------------------------------------------------------------------
#include "hedley.h"
#include "minigb_apu.h"
#include "peanut_gb.h"
#include "ST7735_TFT.h"   // Sprig screen driver
#include "sdcard.h"
#include "i2s.h"
#include "gbcolors.h"

// ---------------------------------------------------------------------------
// Sprig GPIO definitions
// ---------------------------------------------------------------------------

// Buttons
#define GPIO_W       5    // GB UP
#define GPIO_S       7    // GB DOWN
#define GPIO_A_BTN   6    // GB LEFT (renamed to avoid clash with GB struct 'a')
#define GPIO_D       8    // GB RIGHT
#define GPIO_I      12    // GB START
#define GPIO_K      14    // GB B
#define GPIO_J      13    // GB SELECT
#define GPIO_L      15    // GB A

// SD card CS — shares SPI0 bus with LCD (LCD CS = GP20, defined in ST7735_TFT.h)
#define GPIO_SD_CS  21

// LCD dimensions
#define LCD_WIDTH  160
#define LCD_HEIGHT 128
#define GB_LCD_HEIGHT 144
#define LCD_LINE_SKIP 0xFF
#define MENU_ROW_HEIGHT 8
#define ROMS_PER_PAGE (LCD_HEIGHT / MENU_ROW_HEIGHT)
#define MENU_TEXT_X 2
#define MENU_TEXT_Y_OFFSET 1
#define MENU_CHAR_WIDTH 4
#define MENU_MAX_CHARS ((LCD_WIDTH - MENU_TEXT_X) / MENU_CHAR_WIDTH)

// ---------------------------------------------------------------------------
// LCD helpers built on top of ST7735_TFT.h macros/functions
// ---------------------------------------------------------------------------

/**
 * Set a single-row address window on the ST7735 and begin RAMWR.
 * CS is left LOW after this call; caller must push pixels then call
 * tft_cs_high() when done.
 */
static inline void st7735_set_row_window(uint8_t row)
{
	tft_cs_low();

	// Column: 0 .. LCD_WIDTH-1
	spi_command(ST7735_CASET);
	spi_data(0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH - 1));

	// Row: row .. row  (single-line window)
	spi_command(ST7735_RASET);
	spi_data(0x00, row, 0x00, row);

	// Open RAMWR
	spi_command(ST7735_RAMWR);
	tft_dc_high();
	// CS stays low; caller writes pixels then pulls CS high
}

/**
 * Push LCD_WIDTH RGB565 pixels (big-endian) then deassert CS.
 * ST7735 expects MSB first; RP2040 stores uint16_t little-endian,
 * so we swap bytes manually.
 */
static inline void st7735_write_line(const uint16_t *pixels, uint16_t count)
{
	for (uint16_t i = 0; i < count; i++) {
		uint8_t hi = (uint8_t)(pixels[i] >> 8);
		uint8_t lo = (uint8_t)(pixels[i] & 0xFF);
		spi_write_blocking(SPI_TFT_PORT, &hi, 1);
		spi_write_blocking(SPI_TFT_PORT, &lo, 1);
	}
	tft_cs_high();
}

/** Fill the entire LCD with one colour. */
static void sprig_lcd_clear(uint16_t colour)
{
	st7735_fill_start();
	for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++)
		st7735_fill_send(colour);
	st7735_fill_finish();
}

/** Draw a solid colour rectangle (used by the ROM selector menu). */
static void lcd_fill_rect(uint8_t x0, uint8_t y0,
						  uint8_t w,  uint8_t h, uint16_t colour)
{
	tft_cs_low();
	spi_command(ST7735_CASET);
	spi_data(0x00, x0, 0x00, (uint8_t)(x0 + w - 1));
	spi_command(ST7735_RASET);
	spi_data(0x00, y0, 0x00, (uint8_t)(y0 + h - 1));
	spi_command(ST7735_RAMWR);
	tft_dc_high();
	uint8_t hi = (uint8_t)(colour >> 8);
	uint8_t lo = (uint8_t)(colour & 0xFF);
	for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
		spi_write_blocking(SPI_TFT_PORT, &hi, 1);
		spi_write_blocking(SPI_TFT_PORT, &lo, 1);
	}
	tft_cs_high();
}

// ---------------------------------------------------------------------------
// Audio
// ---------------------------------------------------------------------------
#if ENABLE_SOUND
uint16_t *stream;
#endif

// ---------------------------------------------------------------------------
// ROM / RAM
// ---------------------------------------------------------------------------
#define FLASH_TARGET_OFFSET (1024 * 1024)
const uint8_t *rom = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
static unsigned char rom_bank0[65536];
static uint8_t       ram[32768];

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------
static int lcd_line_busy = 0;
static palette_t palette;
static uint8_t   manual_palette_selected = 0;
static uint8_t gb_to_lcd_line[GB_LCD_HEIGHT];
static bool gb_to_lcd_line_ready = false;

static struct {
	unsigned a      : 1;
	unsigned b      : 1;
	unsigned select : 1;
	unsigned start  : 1;
	unsigned right  : 1;
	unsigned left   : 1;
	unsigned up     : 1;
	unsigned down   : 1;
} prev_joypad_bits;

// ---------------------------------------------------------------------------
// Multicore command structure
// ---------------------------------------------------------------------------
union core_cmd {
	struct {
		#define CORE_CMD_NOP      0
		#define CORE_CMD_LCD_LINE 1
		uint8_t cmd;
		uint8_t unused1;
		uint8_t unused2;
		uint8_t data;
	};
	uint32_t full;
};

static uint8_t pixels_buffer[LCD_WIDTH];

static uint16_t menu_glyph_bits(char c)
{
#define GLYPH(r0, r1, r2, r3, r4) \
	((uint16_t)(((r0) << 12) | ((r1) << 9) | ((r2) << 6) | ((r3) << 3) | (r4)))
	const char uc = (char)toupper((unsigned char)c);
	switch (uc) {
		case 'A': return GLYPH(7, 5, 7, 5, 5);
		case 'B': return GLYPH(6, 5, 6, 5, 6);
		case 'C': return GLYPH(7, 4, 4, 4, 7);
		case 'D': return GLYPH(6, 5, 5, 5, 6);
		case 'E': return GLYPH(7, 4, 6, 4, 7);
		case 'F': return GLYPH(7, 4, 6, 4, 4);
		case 'G': return GLYPH(7, 4, 5, 5, 7);
		case 'H': return GLYPH(5, 5, 7, 5, 5);
		case 'I': return GLYPH(7, 2, 2, 2, 7);
		case 'J': return GLYPH(1, 1, 1, 5, 2);
		case 'K': return GLYPH(5, 5, 6, 5, 5);
		case 'L': return GLYPH(4, 4, 4, 4, 7);
		case 'M': return GLYPH(5, 7, 7, 5, 5);
		case 'N': return GLYPH(5, 7, 7, 7, 5);
		case 'O': return GLYPH(7, 5, 5, 5, 7);
		case 'P': return GLYPH(7, 5, 7, 4, 4);
		case 'Q': return GLYPH(7, 5, 5, 7, 1);
		case 'R': return GLYPH(7, 5, 7, 5, 5);
		case 'S': return GLYPH(7, 4, 7, 1, 7);
		case 'T': return GLYPH(7, 2, 2, 2, 2);
		case 'U': return GLYPH(5, 5, 5, 5, 7);
		case 'V': return GLYPH(5, 5, 5, 5, 2);
		case 'W': return GLYPH(5, 5, 7, 7, 5);
		case 'X': return GLYPH(5, 5, 2, 5, 5);
		case 'Y': return GLYPH(5, 5, 2, 2, 2);
		case 'Z': return GLYPH(7, 1, 2, 4, 7);
		case '0': return GLYPH(7, 5, 5, 5, 7);
		case '1': return GLYPH(2, 6, 2, 2, 7);
		case '2': return GLYPH(7, 1, 7, 4, 7);
		case '3': return GLYPH(7, 1, 7, 1, 7);
		case '4': return GLYPH(5, 5, 7, 1, 1);
		case '5': return GLYPH(7, 4, 7, 1, 7);
		case '6': return GLYPH(7, 4, 7, 5, 7);
		case '7': return GLYPH(7, 1, 1, 1, 1);
		case '8': return GLYPH(7, 5, 7, 5, 7);
		case '9': return GLYPH(7, 5, 7, 1, 7);
		case '.': return GLYPH(0, 0, 0, 0, 2);
		case '-': return GLYPH(0, 0, 7, 0, 0);
		case '_': return GLYPH(0, 0, 0, 0, 7);
		case ' ': return GLYPH(0, 0, 0, 0, 0);
		default:  return GLYPH(7, 1, 2, 0, 2);
	}
#undef GLYPH
}

static void lcd_draw_char3x5(uint8_t x, uint8_t y, char c, uint16_t colour)
{
	const uint16_t glyph = menu_glyph_bits(c);
	for (uint8_t row = 0; row < 5; row++) {
		for (uint8_t col = 0; col < 3; col++) {
			const uint8_t bit_index = (uint8_t)(14 - ((row * 3) + col));
			if ((glyph >> bit_index) & 1u)
				lcd_fill_rect((uint8_t)(x + col), (uint8_t)(y + row), 1, 1, colour);
		}
	}
}

static void lcd_draw_text3x5(uint8_t x, uint8_t y, const char *text, uint16_t colour)
{
	uint8_t i = 0;
	while (text[i] && i < MENU_MAX_CHARS) {
		lcd_draw_char3x5((uint8_t)(x + (i * MENU_CHAR_WIDTH)), y, text[i], colour);
		i++;
	}
}

static void draw_selector_row(uint8_t row, const char *name, bool selected)
{
	const uint8_t y = (uint8_t)(row * MENU_ROW_HEIGHT);
	const uint16_t bg = selected ? ST7735_RED : ST7735_WHITE;
	const uint16_t fg = selected ? ST7735_WHITE : ST7735_BLACK;
	lcd_fill_rect(0, y, LCD_WIDTH, MENU_ROW_HEIGHT - 1, bg);
	lcd_draw_text3x5(MENU_TEXT_X, (uint8_t)(y + MENU_TEXT_Y_OFFSET), name, fg);
}

static void init_lcd_scale_lut(void)
{
	if (gb_to_lcd_line_ready)
		return;

	uint8_t prev_line = LCD_LINE_SKIP;
	for (uint8_t gb_line = 0; gb_line < GB_LCD_HEIGHT; gb_line++) {
		const uint8_t lcd_line =
			(uint8_t)(((uint16_t)gb_line * LCD_HEIGHT) / GB_LCD_HEIGHT);
		if (lcd_line == prev_line) {
			gb_to_lcd_line[gb_line] = LCD_LINE_SKIP;
		} else {
			gb_to_lcd_line[gb_line] = lcd_line;
			prev_line = lcd_line;
		}
	}

	gb_to_lcd_line_ready = true;
}

#define putstdio(x) write(1, x, strlen(x))

// ---------------------------------------------------------------------------
// GB callbacks
// ---------------------------------------------------------------------------
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void)gb;
	if (addr < sizeof(rom_bank0))
		return rom_bank0[addr];
	return rom[addr];
}

uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void)gb;
	return ram[addr];
}

void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
					   const uint8_t val)
{
	ram[addr] = val;
}

void gb_error(struct gb_s *gb, const enum gb_error_e gb_err,
			  const uint16_t addr)
{
	const char *gb_err_str[4] = {
		"UNKNOWN", "INVALID OPCODE", "INVALID READ", "INVALID WRITE"
	};
	printf("Error %d: %s at %04X\n", gb_err, gb_err_str[gb_err], addr);
}

// ---------------------------------------------------------------------------
// SD card helpers
// ---------------------------------------------------------------------------
#if ENABLE_SDCARD

void read_cart_ram_file(struct gb_s *gb)
{
	char filename[16];
	uint_fast32_t save_size;
	UINT br;

	gb_get_rom_name(gb, filename);
	save_size = gb_get_save_size(gb);
	if (save_size > 0) {
		sd_card_t *pSD = sd_get_by_num(0);
		FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
		if (FR_OK != fr) {
			printf("E f_mount: %s (%d)\n", FRESULT_str(fr), fr);
			return;
		}
		FIL fil;
		fr = f_open(&fil, filename, FA_READ);
		if (fr == FR_OK)
			f_read(&fil, ram, f_size(&fil), &br);
		else
			printf("E f_open(%s): %s (%d)\n", filename, FRESULT_str(fr), fr);
		f_close(&fil);
		f_unmount(pSD->pcName);
	}
	printf("I read_cart_ram_file(%s) DONE\n", filename);
}

void write_cart_ram_file(struct gb_s *gb)
{
	char filename[16];
	uint_fast32_t save_size;
	UINT bw;

	gb_get_rom_name(gb, filename);
	save_size = gb_get_save_size(gb);
	if (save_size > 0) {
		sd_card_t *pSD = sd_get_by_num(0);
		FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
		if (FR_OK != fr) {
			printf("E f_mount: %s (%d)\n", FRESULT_str(fr), fr);
			return;
		}
		FIL fil;
		fr = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
		if (fr == FR_OK)
			f_write(&fil, ram, save_size, &bw);
		else
			printf("E f_open(%s): %s (%d)\n", filename, FRESULT_str(fr), fr);
		f_close(&fil);
		f_unmount(pSD->pcName);
	}
	printf("I write_cart_ram_file(%s) DONE\n", filename);
}

void load_cart_rom_file(char *filename)
{
	UINT br;
	uint8_t buffer[FLASH_SECTOR_SIZE];
	sd_card_t *pSD = sd_get_by_num(0);
	FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		printf("E f_mount: %s (%d)\n", FRESULT_str(fr), fr);
		return;
	}
	FIL fil;
	fr = f_open(&fil, filename, FA_READ);
	if (fr == FR_OK) {
		uint32_t offset = FLASH_TARGET_OFFSET;
		for (;;) {
			f_read(&fil, buffer, sizeof buffer, &br);
			if (br == 0) break;
			flash_range_erase(offset, FLASH_SECTOR_SIZE);
			flash_range_program(offset, buffer, FLASH_SECTOR_SIZE);
			offset += FLASH_SECTOR_SIZE;
		}
		printf("I load_cart_rom_file(%s) DONE\n", filename);
	} else {
		printf("E f_open(%s): %s (%d)\n", filename, FRESULT_str(fr), fr);
	}
	f_close(&fil);
	f_unmount(pSD->pcName);
}

/**
 * Scan one page of .gb files and draw rows with filenames on the LCD.
 * Returns the number of files found on this page.
 */
uint16_t rom_file_selector_display_page(char filename[ROMS_PER_PAGE][256],
										uint16_t num_page)
{
	sd_card_t *pSD = sd_get_by_num(0);
	DIR dj;
	FILINFO fno;
	FRESULT fr;

	fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		printf("E f_mount: %s (%d)\n", FRESULT_str(fr), fr);
		return 0;
	}

	for (uint8_t i = 0; i < ROMS_PER_PAGE; i++)
		strcpy(filename[i], "");

	uint16_t num_file = 0;
	fr = f_findfirst(&dj, &fno, "", "*.gb");

	if (num_page > 0) {
		while (num_file < num_page * ROMS_PER_PAGE && fr == FR_OK && fno.fname[0]) {
			num_file++;
			fr = f_findnext(&dj, &fno);
		}
	}

	num_file = 0;
	while (num_file < ROMS_PER_PAGE && fr == FR_OK && fno.fname[0]) {
		strcpy(filename[num_file], fno.fname);
		num_file++;
		fr = f_findnext(&dj, &fno);
	}
	f_closedir(&dj);
	f_unmount(pSD->pcName);

	// Draw list rows and game names
	sprig_lcd_clear(ST7735_BLACK);
	for (uint8_t i = 0; i < num_file; i++)
		draw_selector_row(i, filename[i], false);

	return num_file;
}

void rom_file_selector(void)
{
	uint16_t num_page = 0;
	char filename[ROMS_PER_PAGE][256];
	uint16_t num_file;

	num_file = rom_file_selector_display_page(filename, num_page);

	uint8_t selected = 0;
	if (num_file > 0)
		draw_selector_row(0, filename[0], true);

	while (true) {
		bool up     = gpio_get(GPIO_W);
		bool down   = gpio_get(GPIO_S);
		bool left   = gpio_get(GPIO_A_BTN);
		bool right  = gpio_get(GPIO_D);
		bool btn_a  = gpio_get(GPIO_L);
		bool btn_b  = gpio_get(GPIO_K);
		(void)gpio_get(GPIO_J);   // select — unused in selector
		bool btn_start = gpio_get(GPIO_I);

		// I alone: restart last game (ROM already in flash, skip load)
		if (!btn_start)
			break;

		// L or K: load selected ROM from SD and start
		if ((!btn_a || !btn_b) && num_file > 0) {
			load_cart_rom_file(filename[selected]);
			break;
		}

		// S: move selection down
		if (!down && num_file > 0) {
			draw_selector_row(selected, filename[selected], false);
			selected = (selected + 1 >= num_file) ? 0 : selected + 1;
			draw_selector_row(selected, filename[selected], true);
			sleep_ms(150);
		}

		// W: move selection up
		if (!up && num_file > 0) {
			draw_selector_row(selected, filename[selected], false);
			selected = (selected == 0) ? num_file - 1 : selected - 1;
			draw_selector_row(selected, filename[selected], true);
			sleep_ms(150);
		}

		// D: next page
		if (!right) {
			num_page++;
			num_file = rom_file_selector_display_page(filename, num_page);
			if (num_file == 0) {
				num_page--;
				num_file = rom_file_selector_display_page(filename, num_page);
			}
			selected = 0;
			if (num_file > 0)
				draw_selector_row(0, filename[0], true);
			sleep_ms(150);
		}

		// A: previous page
		if (!left && num_page > 0) {
			num_page--;
			num_file = rom_file_selector_display_page(filename, num_page);
			selected = 0;
			if (num_file > 0)
				draw_selector_row(0, filename[0], true);
			sleep_ms(150);
		}

		tight_loop_contents();
	}
}

#endif /* ENABLE_SDCARD */

// ---------------------------------------------------------------------------
// LCD scanline rendering on core 1
// ---------------------------------------------------------------------------
#if ENABLE_LCD

static void core1_lcd_draw_line(const uint_fast8_t line)
{
	static uint16_t fb[LCD_WIDTH];
	if (line >= LCD_HEIGHT) {
		__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
		return;
	}

	for (unsigned x = 0; x < LCD_WIDTH; x++) {
		fb[x] = palette[(pixels_buffer[x] & LCD_PALETTE_ALL) >> 4]
		[pixels_buffer[x] & 3];
	}

	st7735_set_row_window(line);
	st7735_write_line(fb, LCD_WIDTH);

	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
}

_Noreturn
void main_core1(void)
{
	union core_cmd cmd;

	// st7735_init() was already called on core0; just clear screen here.
	sprig_lcd_clear(ST7735_BLACK);

	while (1) {
		cmd.full = multicore_fifo_pop_blocking();
		switch (cmd.cmd) {
			case CORE_CMD_LCD_LINE:
				core1_lcd_draw_line(cmd.data);
				break;
			case CORE_CMD_NOP:
			default:
				break;
		}
	}
	HEDLEY_UNREACHABLE();
}

void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
				   const uint_fast8_t line)
{
	union core_cmd cmd;
	(void)gb;
	init_lcd_scale_lut();

	if (line >= GB_LCD_HEIGHT)
		return;

	const uint8_t lcd_line = gb_to_lcd_line[line];
	if (lcd_line == LCD_LINE_SKIP)
		return;

	while (__atomic_load_n(&lcd_line_busy, __ATOMIC_SEQ_CST))
		tight_loop_contents();

	memcpy(pixels_buffer, pixels, LCD_WIDTH);

	cmd.cmd  = CORE_CMD_LCD_LINE;
	cmd.data = lcd_line;

	__atomic_store_n(&lcd_line_busy, 1, __ATOMIC_SEQ_CST);
	multicore_fifo_push_blocking(cmd.full);
}

#endif /* ENABLE_LCD */

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------
int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;

	// Overclock to 266 MHz
	{
		// target: 286 MHz = 1716 / (6 * 1)
		const unsigned vco  = 1716 * 1000 * 1000;
		const unsigned div1 = 6, div2 = 1;

		// A slight voltage bump is recommended for stability over 250MHz
		vreg_set_voltage(VREG_VOLTAGE_1_20);
		sleep_ms(2);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(2);
	}

	stdio_init_all();
	time_init();
	putstdio("INIT: ");

	// -----------------------------------------------------------------------
	// Button GPIO
	// -----------------------------------------------------------------------
	const uint button_pins[] = {
		GPIO_W, GPIO_S, GPIO_A_BTN, GPIO_D,
		GPIO_I, GPIO_K, GPIO_J,     GPIO_L
	};
	for (int i = 0; i < (int)(sizeof(button_pins)/sizeof(button_pins[0])); i++) {
		gpio_set_function(button_pins[i], GPIO_FUNC_SIO);
		gpio_set_dir(button_pins[i], false);
		gpio_pull_up(button_pins[i]);
	}

	// -----------------------------------------------------------------------
	// SD CS — pull high BEFORE st7735_init so it can't corrupt the SPI bus
	// -----------------------------------------------------------------------
	#if ENABLE_SDCARD
	gpio_init(GPIO_SD_CS);
	gpio_set_dir(GPIO_SD_CS, GPIO_OUT);
	gpio_put(GPIO_SD_CS, 1);
	#endif

	// -----------------------------------------------------------------------
	// LCD init — st7735_init() handles SPI0 init, all LCD GPIO, and BL (GP17)
	// Do NOT call spi_init() again anywhere else.
	// -----------------------------------------------------------------------
	st7735_init();
	putstdio("LCD ");

	// -----------------------------------------------------------------------
	// Audio
	// -----------------------------------------------------------------------
	#if ENABLE_SOUND
	stream = malloc(AUDIO_BUFFER_SIZE_BYTES);
	assert(stream != NULL);
	memset(stream, 0, AUDIO_BUFFER_SIZE_BYTES);

	i2s_config_t i2s_config    = i2s_get_default_config();
	i2s_config.sample_freq      = AUDIO_SAMPLE_RATE;
	i2s_config.dma_trans_count  = AUDIO_SAMPLES;
	i2s_config.data_pin         = 9;
	i2s_config.clock_pin_base   = 10;
	i2s_volume(&i2s_config, 2);
	i2s_init(&i2s_config);
	putstdio("AUDIO ");
	#endif

	// -----------------------------------------------------------------------
	// Main loop — re-enters after J+L resets to ROM selector
	// -----------------------------------------------------------------------
	while (true)
	{
		#if ENABLE_LCD && ENABLE_SDCARD
		rom_file_selector();
		#endif

		memcpy(rom_bank0, rom, sizeof(rom_bank0));

		ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
					  &gb_cart_ram_write, &gb_error, NULL);
		putstdio("GB ");

		if (ret != GB_INIT_NO_ERROR) {
			printf("Error: %d\n", ret);
			goto out;
		}

		{
			char rom_title[16];
			auto_assign_palette(palette, gb_colour_hash(&gb),
								gb_get_rom_name(&gb, rom_title));
		}

		#if ENABLE_LCD
		gb_init_lcd(&gb, &lcd_draw_line);
		#endif

		#if ENABLE_SDCARD
		// Load save RAM before core1 starts so SD and LCD don't race on SPI0.
		read_cart_ram_file(&gb);
		#endif

		// Restore SPI0 to full LCD speed after any SD access.
		spi_set_baudrate(SPI_TFT_PORT, 30000000);

		#if ENABLE_LCD
		putstdio("CORE1 ");
		multicore_launch_core1(main_core1);
		#endif

		#if ENABLE_SOUND
		audio_init();
		#endif

		putstdio("\n> ");
		uint_fast32_t frames     = 0;
		uint64_t      start_time = time_us_64();
		bool          save_on_exit = false;

		while (1)
		{
			gb.gb_frame = 0;
			do {
				__gb_step_cpu(&gb);
				tight_loop_contents();
			} while (HEDLEY_LIKELY(gb.gb_frame == 0));

			frames++;

			#if ENABLE_SOUND
			if (!gb.direct.frame_skip) {
				audio_callback(NULL, stream, AUDIO_BUFFER_SIZE_BYTES);
				i2s_dma_write(&i2s_config, stream);
			}
			#endif

			// Save previous button state
			prev_joypad_bits.up     = gb.direct.joypad_bits.up;
			prev_joypad_bits.down   = gb.direct.joypad_bits.down;
			prev_joypad_bits.left   = gb.direct.joypad_bits.left;
			prev_joypad_bits.right  = gb.direct.joypad_bits.right;
			prev_joypad_bits.a      = gb.direct.joypad_bits.a;
			prev_joypad_bits.b      = gb.direct.joypad_bits.b;
			prev_joypad_bits.select = gb.direct.joypad_bits.select;
			prev_joypad_bits.start  = gb.direct.joypad_bits.start;

			// Read current button state
			// Active-low: gpio_get returns 0 when pressed.
			// GB joypad_bits: 0 = pressed, 1 = released — matches directly.
			gb.direct.joypad_bits.up     = gpio_get(GPIO_W);
			gb.direct.joypad_bits.down   = gpio_get(GPIO_S);
			gb.direct.joypad_bits.left   = gpio_get(GPIO_A_BTN);
			gb.direct.joypad_bits.right  = gpio_get(GPIO_D);
			gb.direct.joypad_bits.a      = gpio_get(GPIO_L);
			gb.direct.joypad_bits.b      = gpio_get(GPIO_K);
			gb.direct.joypad_bits.select = gpio_get(GPIO_J);
			gb.direct.joypad_bits.start  = gpio_get(GPIO_I);

			// Hotkeys: J (SELECT) held + another button
			if (!gb.direct.joypad_bits.select)
			{
				#if ENABLE_SOUND
				// J+W: volume up
				if (!gb.direct.joypad_bits.up && prev_joypad_bits.up)
					i2s_increase_volume(&i2s_config);
				// J+S: volume down
				if (!gb.direct.joypad_bits.down && prev_joypad_bits.down)
					i2s_decrease_volume(&i2s_config);
				#endif
				// J+D: next palette
				if (!gb.direct.joypad_bits.right && prev_joypad_bits.right) {
					if (manual_palette_selected < 12) {
						manual_palette_selected++;
						manual_assign_palette(palette, manual_palette_selected);
					}
				}
				// J+A: previous palette
				if (!gb.direct.joypad_bits.left && prev_joypad_bits.left) {
					if (manual_palette_selected > 0) {
						manual_palette_selected--;
						manual_assign_palette(palette, manual_palette_selected);
					}
				}
				// J+I: save and return to ROM selector
				if (!gb.direct.joypad_bits.start && prev_joypad_bits.start) {
					save_on_exit = true;
					goto out;
				}
				// J+L: toggle fast-forward
				if (!gb.direct.joypad_bits.a && prev_joypad_bits.a) {
					gb.direct.frame_skip = !gb.direct.frame_skip;
					printf("frame_skip=%d\n", gb.direct.frame_skip);
				}
			}

			// USB serial debug
			int input = getchar_timeout_us(0);
			if (input == PICO_ERROR_TIMEOUT)
				continue;

			switch (input) {
				case 'i': gb.direct.interlace  = !gb.direct.interlace;  break;
				case 'f': gb.direct.frame_skip = !gb.direct.frame_skip; break;
				case 'b': {
					uint64_t end  = time_us_64();
					uint32_t diff = (uint32_t)(end - start_time);
					uint32_t fps  = (uint32_t)(((uint64_t)frames * 1000000) / diff);
					printf("Frames:%u Time:%lu us FPS:%lu\n", frames, diff, fps);
					stdio_flush();
					frames = 0;
					start_time = time_us_64();
					break;
				}
				case '\n': case '\r': gb.direct.joypad_bits.start  = 0; break;
				case '\b':            gb.direct.joypad_bits.select = 0; break;
				case '8': gb.direct.joypad_bits.up    = 0; break;
				case '2': gb.direct.joypad_bits.down  = 0; break;
				case '4': gb.direct.joypad_bits.left  = 0; break;
				case '6': gb.direct.joypad_bits.right = 0; break;
				case 'z': case 'w': gb.direct.joypad_bits.a = 0; break;
				case 'x':           gb.direct.joypad_bits.b = 0; break;
				case 'q': goto out;
				default:  break;
			}
		}

		out:
		puts("\nEmulation Ended");
		multicore_reset_core1();
		#if ENABLE_SDCARD
		if (save_on_exit) {
			write_cart_ram_file(&gb);
		}
		#endif

	} /* outer while */
}
