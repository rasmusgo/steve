# Notes for building and running from Mac
Basic flow is:
1. Install build chain.
2. Create binary files from .hex files.
3. Install binary over serial connection.

## System installs (build chain)
brew tap osx-cross/avr
brew install avr-gcc

## Build libraries
(cd src/Dynamixel && make)
(cd src/Serial && make)
(cd src/ZigBee && make)

## Build examples
(cd example/Dynamixel_Read_Write && make)
(cd example/Dynamixel_Sync_Write && make)

## Build servo controller program
(cd src/Steve && make)

## Convert from hex to bin
avr-objcopy --input-target=ihex --output-target=binary example/Dynamixel_Read_Write/Read_Write.hex Read_Write.bin
avr-objcopy --input-target=ihex --output-target=binary example/Dynamixel_Sync_Write/Sync_Write.hex Sync_Write.bin
avr-objcopy --input-target=ihex --output-target=binary src/Steve/Steve.hex Steve.bin

## Install binary
Two terminal are used in this example. The first is used to establish the serial connection and reading data. The second is for sending data.

### Open serial connection and read data
stty -f /dev/cu.usbserial-AL02L3L9 57600 | cat /dev/cu.usbserial-AL02L3L9

### Enter bootloader (second terminal)
1. Enter bootloader by sending '#' while restarting the controller:
    yes '#' > /dev/cu.usbserial-AL02L3L9
2. Restart controller by pressing the button labelled "mode".
3. Stop spamming by pressing ctrl+c to terminate "yes".

### Send binary (second terminal)
echo LD > /dev/cu.usbserial-AL02L3L9
cat Steve.bin > /dev/cu.usbserial-AL02L3L9
or
avr-objcopy --input-target=ihex --output-target=binary src/Steve/Steve.hex /dev/cu.usbserial-AL02L3L9

### Start program (second terminal)
Reboot controller by pressing 'mode' or
echo GO > /dev/cu.usbserial-AL02L3L9

## Other useful commands
echo help > /dev/cu.usbserial-AL02L3L9
echo VER > /dev/cu.usbserial-AL02L3L9

### Transfer file from controller to laptop
stty -f /dev/cu.usbserial-AL02L3L9 57600 | cat /dev/cu.usbserial-AL02L3L9 | tee flash.bin

## Open interactive serial connection with screen
screen /dev/cu.usbserial-AL02L3L9 57600

### Useful commands
Be sure to type out the colons.
- Hold # while restarting the controller py pressing the button labelled "mode".
- Ctrl+a : quit
- Ctrl+a : readreg p Read_Write.bin
- Ctrl+a : paste p
