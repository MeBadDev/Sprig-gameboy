# Sprig Gameboy Emulator

Simple Gameboy Emulator built for the [Hack Club Sprig](https://sprig.hackclub.com/) forked from the [Pico-GB](https://github.com/YouMakeTech/Pico-GB) repository and modified to run on the sprig hardware.




> [!WARNING]
> I have overclocked the RP2040 to 286MHz so I have added this here as a warning just in case anyone needs to know.



## Currently Working:

* Screen
* Controls
* SD Card

## In Progress
* Audio

## Setup Instructions
1. Download the latest released UF2 file
2. Hold down `bootsel` on the sprig and plug it in while doing so
3. Drag and drop the UF2 file onto the device
4. Format a SD Card to `FAT32` and put roms you own legally onto it in a .GB format
5. Insert SD Card into the slot on the back of the screen
6. Start the sprig

If done correctly then you should have your games listed!

# Build Instructions
**I DO NOT RECOMMEND BUILDING IF YOU DONT KNOW WHAT YOU ARE DOING!**
```sh
export PICO_SDK_PATH=~/pico-sdk
cd build
rm -rf *
cmake ..
make -j$(nproc)
```
