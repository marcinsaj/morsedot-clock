# MORSEDOT - Flip Dot Clock

> [!IMPORTANT]
> [Check out the project on Kickstarter](https://www.kickstarter.com/projects/marcinsaj/morsedot-clock)

## Specification
- the clock consists of two modules: controller module & flip-dot display module
- two control buttons for setting the time and effects
- one button for turning on/off the clock
- accurate real-time clock (RTC) on board RX8025T
- the RTC clock memory is backed up by a supercapacitor, so the clock does not require an additional battery in the event of a power failure or turn off
- clock dimensions (W × H × D): 100 × 104 × 56 mm (~3.9" × 4.1" × 2.2")
- power supply from USB-C PD 12V
- ISP programming connector - only for intermediate users

![Morsedot Clock](https://github.com/marcinsaj/morsedot-clock/blob/main/extras/morsedot-clock-cover-github.png)

## 3D Printed Enclosure
You can print your own enclosure:
- [Download STL files on Printables](https://www.printables.com/model/1742806-morsedot-flip-dot-clock)

>[!TIP]
> If you have issues powering the clock, see the list of [compatible power adapters](https://github.com/marcinsaj/morsedot-clock/blob/main/datasheet/verified-compatible-power-adapters.md).

## Datasheet
  - CLOCK USER MANUAL - todo
  - [FIRMWARE](https://github.com/marcinsaj/morsedot-clock/blob/main/firmware/firmware-morsedot-flip-dot-clock.ino) - todo
  - Morsedot - Flip Dot Clock - Display Module Diagram - todo
  - Morsedot - Flip Dot Clock - Controller Module Diagram - todo

## How to Read the Time
- [Morse Code](https://github.com/marcinsaj/morsedot-clock/raw/main/datasheet/morse-code-flip-dot-clock-morsedot.pdf)
- [Binary Code](https://github.com/marcinsaj/morsedot-clock/raw/main/datasheet/binary-code-flip-dot-clock-morsedot.pdf)
- [Stibitz Code](https://github.com/marcinsaj/morsedot-clock/raw/main/datasheet/stibitz-code-flip-dot-clock-morsedot.pdf)
- [Fibonacci Code](https://github.com/marcinsaj/morsedot-clock/raw/main/datasheet/fibonacci-code-flip-dot-clock-morsedot.pdf)
- [POSTNET Code](https://github.com/marcinsaj/morsedot-clock/raw/main/datasheet/postnet-code-flip-dot-clock-morsedot.pdf)

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
