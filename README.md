# MORSEDOT - Flip Dot Clock

>[!TIP]
> If you have issues powering the clock, see the list of [compatible power adapters](https://github.com/marcinsaj/morsedot-clock/blob/main/datasheet/verified-compatible-power-adapters.md).

## Specification
- the clock consists of two modules: controller module & flip-dot display module
- two buttons for setting the time
- one button for turning on the clock
- accurate real-time clock (RTC) on board RX8025T
- the RTC clock memory is backed up by a supercapacitor, so the clock does not require an additional battery in the event of a power failure or turn off
- clock dimensions (W × H × D): 100 × 104 × 56 mm (~3.9" × 4.1" × 2.2")
- power supply from USB-C PD 12V
- ISP programming connector - only for intermediate users

## 3D Printed Enclosure
You can print your own enclosure or modify the design:
- [Download STL files on Printables](https://www.printables.com/model/1650064-modular-flip-disc-clock-enclosure)
- [Download the Fusion 360 source file](https://github.com/marcinsaj/Flipo-Modular-Clock-4x3x3-Flip-Disc-Display/raw/main/datasheet/enclosure-modular-flip-disc-clock.f3d)


## Datasheet
  - CLOCK USER MANUAL - todo
  - [FIRMWARE](https://github.com/marcinsaj/morsedot-clock/blob/main/firmware/firmware-morsedot-flip-dot-clock.ino) - todo
  - Morsedot - Flip Dot Clock - Display Module Diagram - todo
  - Morsedot - Flip Dot Clock - Controller Module Diagram - todo

## How to Read the Display


## Programming and Firmware
If you want to modify the [firmware](https://github.com/marcinsaj/morsedot-clock/blob/main/firmware/firmware-morsedot-flip-dot-clock.ino), you can program the clock via the ISP connector.

### MiniCore Configuration
Before burning the bootloader, install MiniCore using the following Boards Manager URL:
https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json

### In Arduino IDE → Tools, configure the settings as follows:
- Board: ATmega328P / ATmega328PA
- Clock: External 12 MHz ⚠
- Bootloader: Yes (UART0)
- BOD: 2.7V
- EEPROM: Retained
- Compiler LTO: Enabled
- Baud Rate: Default
- Variant: 328P / 328PA

⚠ Make sure the clock is set to External 12 MHz. ⚠

## Programming Procedure:
 1. Select the correct Programmer: AVR ISP MKII (recommended).
 2. Click Burn Bootloader to set the fuse bits and configure the microcontroller.
 3. After completion, select Upload Using Programmer to flash the firmware.


![Morsedot - Flip Dot Clock Animation](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-animation-1.webp)
![Morsedot - Morse Code](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-morse-code.png)
![Morsedot - Morse Code](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-binary-code.png)
![Morsedot - Morse Code](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-stibitz-code.png)
![Morsedot - Morse Code](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-fibonacci-code.png)
![Morsedot - Morse Code](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-postnet-code.png)
