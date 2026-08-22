/* =============================================================================
 * MORSEDOT Flip-Disc Clock - 5 display modes - controller board MCC REV6
 * =============================================================================
 *
 * HARDWARE
 * --------
 * MCU:       ATmega328P @ 8 MHz external crystal (MiniCore), VCC 3.3 V
 * Display:   20 flip-discs, 4 rows x 5 columns, driven by 3-byte SPI packets
 * RTC:       RX8025T (I2C) - per-minute interrupts, supercap backed
 * Power:     USB-C PD. CH224K asks the charger for 12 V; a 24k configuration
 *            resistor on CF1 selects that gear, PD7 can override it.
 *            TPS22810 (EN-A) feeds VOUT, TPS259631 eFuse (EN-B) feeds the display.
 * Buttons:   BT1 (PD4) - time settings (long), parameter settings (double click)
 *            BT2 (PC3) - value change inside the menus
 *            BT3 (PC2) - clock ON/OFF (long), hard refresh (double click, STANDBY)
 * LED:       one status LED on PD5 - breathing / solid / fast blink
 *
 * BUILD
 * -----
 * MiniCore, ATmega328P, External 8 MHz (must match the crystal), BOD 2.7 V,
 * EEPROM retained, LTO enabled.
 *
 * Upload with "Upload Using Programmer" - the sketch goes straight over ISP.
 *
 * Upload only with the clock switched off (STANDBY). During an ISP session RESET is held,
 * EN-A/EN-B and SPI go high impedance and a disc coil can be left energized, which trips
 * the eFuse.
 *
 * DISPLAY LAYOUT
 * --------------
 * Disc number = row * 5 + col. Disc 0 = bottom right, disc 19 = top left.
 *
 *   Row 3 (top):    19  18  17  16  15   <- tens of hours
 *   Row 2:          14  13  12  11  10   <- units of hours
 *   Row 1:           9   8   7   6   5   <- tens of minutes
 *   Row 0 (bottom):  4   3   2   1   0   <- units of minutes
 *
 * STATE MACHINE
 * -------------
 *   BOOT ---> NEGOTIATE ---> STANDBY <---> RUN
 *                ^    |         |          |
 *                |    v         v          v
 *                +-- FAULT <----+----------+
 *
 * The MCU is powered straight from USB-C, so it boots as soon as the plug goes in and
 * it cannot cut its own supply - switching the clock off only drops the display rails.
 * NEGOTIATE confirms 12 V (see the note above PdRequest12V()); on success the clock
 * parks in STANDBY, or auto-starts after T_AUTOSTART_DELAY if the EEPROM says it was
 * running.
 *
 * FAULT IS NOT A DEAD END, and the way out depends on what failed:
 *
 *   FAULT_NO_12V, FAULT_OVERVOLT   supply faults. Return to NEGOTIATE on their own,
 *                                  T_FAULT_RETRY after the last attempt ENDED - which
 *                                  is why the observed period is ~11.5 s, not 5 s.
 *                                  A BT3 long press retries at once.
 *   FAULT_VOUT, FAULT_EFUSE        rail faults. 12 V itself is fine, so there is no
 *                                  automatic retry - pushing current into a short every
 *                                  few seconds would be worse than waiting for a human.
 *                                  A BT3 long press goes straight back to StartClock().
 *
 * STANDBY and RUN also fall back to NEGOTIATE whenever MonitorVbusLost() fires; from RUN
 * that goes through EmergencyShutdown(), which drops the rails WITHOUT writing the EEPROM
 * so the clock restarts by itself once the supply returns. Bringing the rails up can fail
 * in either direction: from STANDBY through the hard refresh, from STANDBY to RUN through
 * StartClock() - both land in FAULT.
 *
 * OPERATING MODES
 * ---------------
 * Normal:   RTC interrupt every minute -> DisplayTime(). Whether the update uses a
 *           random effect is decided by effect_interval.
 *
 * Time settings (long BT1):
 *           4 digits set one by one, top row down. Inactive rows show all discs ON
 *           except the middle one. Always Morse, whatever display_mode says.
 *
 * Parameter settings (double click BT1):
 *           3 screens, value shown in Morse on row 0: time format, display mode,
 *           effect interval.
 *
 * Hard refresh (double click BT3, STANDBY only):
 *           Rails up, all discs OFF then ON with extended coil pulses to free sticky
 *           discs, rails down. Leaves the panel all ON, which is what STANDBY expects.
 * ============================================================================= */

#include <RTC_RX8025T.h>  // https://github.com/marcinsaj/RTC_RX8025T
#include <TimeLib.h>      // https://github.com/PaulStoffregen/Time
#include <Wire.h>         // https://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <OneButton.h>    // https://github.com/mathertel/OneButton
#include <jled.h>         // https://github.com/jandelgado/jled
#include <EEPROM.h>       // (included with Arduino IDE)
#include <SPI.h>          // (included with Arduino IDE)
#include <avr/pgmspace.h> // (included with Arduino IDE)
#include <avr/wdt.h>      // (included with Arduino IDE)


/* ---------------------------------------------------------------------------------
 * DEBUG - UART trace of the power path. Set the 0 below to 1 and rebuild; a command line
 * build can pass -DDEBUG=1 instead, which is what the #ifndef is there for.
 *
 * Output only: PD1 (TX) on connector PH2, 9600 8N1. The firmware never reads the
 * port, so one wire plus ground is enough and no level shifting is needed.
 *
 * What it traces is the part that cannot be seen on the discs - supply negotiation,
 * rail faults and the start decision. Every line is printed once, where it happens:
 *
 *   MCC REV6 boot        first line of setup(). Seeing it twice = the MCU reset.
 *   PD 12V ok            12 V confirmed (5 consecutive ADC samples in window)
 *   PD no 12V, ADC=nnn   a negotiation pass timed out; nnn is the raw reading,
 *                        ADC ~= V * 28.18, so 143 is 5 V and 343 is 12 V
 *   PD re-request        entering the second and last negotiation pass
 *   auto-start pending   EEPROM says RUN; the T_AUTOSTART_DELAY buffer started
 *   auto-start now       that buffer expired and the clock is starting itself
 *   VOUT fault           EN-A on, but VOUT never reached its window
 *   eFuse fault          TPS259631 pulled INT-FLT low
 *   VBUS lost            N_BAD_SAMPLES consecutive readings outside the 12 V window
 *
 * Silence is the normal state of a running clock: nothing is printed per minute, so
 * anything appearing during a long run is a real event. What it does NOT cover:
 * buttons, menus and disc traffic are deliberately not traced - they are visible on
 * the panel, and printing them would drown the events above.
 *
 * Reading it costs 4 ms per line at 9600 baud, so timestamps taken by the PC are
 * only trustworthy down to tens of milliseconds.
 *
 * Cost: +808 B flash, +4 B RAM. Serial's 128 B buffers are already in the image
 * either way, because the USART_RX vector references them.
 * --------------------------------------------------------------------------------- */
#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#define DBG_BEGIN()    Serial.begin(9600)
#define DBG_PRINT(x)   Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#else
#define DBG_BEGIN()
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
#endif

// =============================================================================
// SPI ADDRESS TABLES
// =============================================================================
//
// Each flip-disc is controlled by a unique 3-byte SPI pattern that energizes
// the correct coil driver output.  Two tables exist:
//   setDisc[n]   - 3 bytes that flip disc n to the ON (yellow) side
//   resetDisc[n] - 3 bytes that flip disc n to the OFF (black) side
//
// The 3 bytes are shifted out MSB-first via SPI and latched to the outputs
// by toggling the EN (SS) pin.  After each flip, ClearOutputs() resets the
// shift register to prevent crosstalk on the next operation.
//
// Disc numbering 0-19 matches the physical layout verified by disc-test.
// Stored in PROGMEM to save RAM (60 + 60 = 120 bytes of flash).

static const uint8_t setDisc[20][3] PROGMEM =
{
  { 0b00010000, 0b00000000, 0b00100000 },  // disc 0  (row 0, col 0, rightmost bottom)
  { 0b00010000, 0b00000000, 0b00001000 },  // disc 1  (row 0, col 1)
  { 0b00010000, 0b00001000, 0b00000000 },  // disc 2  (row 0, col 2, middle)
  { 0b00010000, 0b00000010, 0b00000000 },  // disc 3  (row 0, col 3)
  { 0b10010000, 0b00000000, 0b00000000 },  // disc 4  (row 0, col 4, leftmost)

  { 0b00000010, 0b00000000, 0b00100000 },  // disc 5  (row 1, col 0)
  { 0b00000010, 0b00000000, 0b00001000 },  // disc 6  (row 1, col 1)
  { 0b00000010, 0b00001000, 0b00000000 },  // disc 7  (row 1, col 2, middle)
  { 0b00000010, 0b00000010, 0b00000000 },  // disc 8  (row 1, col 3)
  { 0b10000010, 0b00000000, 0b00000000 },  // disc 9  (row 1, col 4)

  { 0b00000000, 0b00000000, 0b00100010 },  // disc 10 (row 2, col 0)
  { 0b00000000, 0b00000000, 0b00001010 },  // disc 11 (row 2, col 1)
  { 0b00000000, 0b00001000, 0b00000010 },  // disc 12 (row 2, col 2, middle)
  { 0b00000000, 0b00000010, 0b00000010 },  // disc 13 (row 2, col 3)
  { 0b10000000, 0b00000000, 0b00000010 },  // disc 14 (row 2, col 4)

  { 0b00000000, 0b00010000, 0b00100000 },  // disc 15 (row 3, col 0)
  { 0b00000000, 0b00010000, 0b00001000 },  // disc 16 (row 3, col 1)
  { 0b00000000, 0b00011000, 0b00000000 },  // disc 17 (row 3, col 2, middle)
  { 0b00000000, 0b00010010, 0b00000000 },  // disc 18 (row 3, col 3)
  { 0b10000000, 0b00010000, 0b00000000 }   // disc 19 (row 3, col 4, leftmost top)
};

static const uint8_t resetDisc[20][3] PROGMEM =
{
  { 0b00001000, 0b00000000, 0b00000001 },  // disc 0
  { 0b00001000, 0b00000001, 0b00000000 },  // disc 1
  { 0b00001001, 0b00000000, 0b00000000 },  // disc 2
  { 0b00101000, 0b00000000, 0b00000000 },  // disc 3
  { 0b01001000, 0b00000000, 0b00000000 },  // disc 4

  { 0b00000100, 0b00000000, 0b00000001 },  // disc 5
  { 0b00000100, 0b00000001, 0b00000000 },  // disc 6
  { 0b00000101, 0b00000000, 0b00000000 },  // disc 7
  { 0b00100100, 0b00000000, 0b00000000 },  // disc 8
  { 0b01000100, 0b00000000, 0b00000000 },  // disc 9

  { 0b00000000, 0b00000000, 0b01000001 },  // disc 10
  { 0b00000000, 0b00000001, 0b01000000 },  // disc 11
  { 0b00000001, 0b00000000, 0b01000000 },  // disc 12
  { 0b00100000, 0b00000000, 0b01000000 },  // disc 13
  { 0b01000000, 0b00000000, 0b01000000 },  // disc 14

  { 0b00000000, 0b00000100, 0b00000001 },  // disc 15
  { 0b00000000, 0b00000101, 0b00000000 },  // disc 16
  { 0b00000001, 0b00000100, 0b00000000 },  // disc 17
  { 0b00100000, 0b00000100, 0b00000000 },  // disc 18
  { 0b01000000, 0b00000100, 0b00000000 }   // disc 19
};

// =============================================================================
// DIGIT PATTERN TABLES
// =============================================================================
//
// Five encoding tables map digit values 0-9 to 5-disc patterns.
//
// VISUAL ORDER: array index 0 = leftmost disc on display, index 4 = rightmost.
// Reading the array left-to-right matches reading the display left-to-right.
//
// PHYSICAL MAPPING: the display is wired right-to-left (col 0 = rightmost),
// so index 0 in the table corresponds to physical col 4 (leftmost disc).
// GetDigitPattern() compensates by indexing with (4 - col).
//
// 1 = ON (yellow), 0 = OFF (black).

// Morse code: dot = 1 (ON), dash = 0 (OFF).
// Example: digit 1 = ".----" -> {1, 0, 0, 0, 0}
static const uint8_t morseDigit[10][5] PROGMEM =
{
  {0, 0, 0, 0, 0},  // 0: -----
  {1, 0, 0, 0, 0},  // 1: .----
  {1, 1, 0, 0, 0},  // 2: ..---
  {1, 1, 1, 0, 0},  // 3: ...--
  {1, 1, 1, 1, 0},  // 4: ....-
  {1, 1, 1, 1, 1},  // 5: .....
  {0, 1, 1, 1, 1},  // 6: -....
  {0, 0, 1, 1, 1},  // 7: --...
  {0, 0, 0, 1, 1},  // 8: ---..
  {0, 0, 0, 0, 1},  // 9: ----.
};

// Binary (BCD): MSB at index 0 (leftmost on display), LSB at index 4 (rightmost).
// Example: digit 5 = 00101 -> {0, 0, 1, 0, 1}
static const uint8_t binaryDigit[10][5] PROGMEM =
{
  {0, 0, 0, 0, 0},  // 0: 00000
  {0, 0, 0, 0, 1},  // 1: 00001
  {0, 0, 0, 1, 0},  // 2: 00010
  {0, 0, 0, 1, 1},  // 3: 00011
  {0, 0, 1, 0, 0},  // 4: 00100
  {0, 0, 1, 0, 1},  // 5: 00101
  {0, 0, 1, 1, 0},  // 6: 00110
  {0, 0, 1, 1, 1},  // 7: 00111
  {0, 1, 0, 0, 0},  // 8: 01000
  {0, 1, 0, 0, 1},  // 9: 01001
};

// Excess-3 (Stibitz code): each digit encoded as (digit + 3) in binary.
// MSB at index 0 (leftmost on display), LSB at index 4 (rightmost).
// Example: digit 0 -> 0+3 = 3 = 00011 -> {0, 0, 0, 1, 1}
static const uint8_t stibitzDigit[10][5] PROGMEM =
{
  {0, 0, 0, 1, 1},  // 0: 00011 (=3)
  {0, 0, 1, 0, 0},  // 1: 00100 (=4)
  {0, 0, 1, 0, 1},  // 2: 00101 (=5)
  {0, 0, 1, 1, 0},  // 3: 00110 (=6)
  {0, 0, 1, 1, 1},  // 4: 00111 (=7)
  {0, 1, 0, 0, 0},  // 5: 01000 (=8)
  {0, 1, 0, 0, 1},  // 6: 01001 (=9)
  {0, 1, 0, 1, 0},  // 7: 01010 (=10)
  {0, 1, 0, 1, 1},  // 8: 01011 (=11)
  {0, 1, 1, 0, 0},  // 9: 01100 (=12)
};

// Fibonacci sequence: weights 8-5-3-2-1 from left to right on the display.
// Each digit is represented as a sum of Fibonacci numbers.
// Example: digit 4 = 3+1 = 00101 -> {0, 0, 1, 0, 1}
static const uint8_t fibonacciDigit[10][5] PROGMEM =
{
  {0, 0, 0, 0, 0},  // 0: 00000
  {0, 0, 0, 0, 1},  // 1: 00001 (1)
  {0, 0, 0, 1, 0},  // 2: 00010 (2)
  {0, 0, 1, 0, 0},  // 3: 00100 (3)
  {0, 0, 1, 0, 1},  // 4: 00101 (3+1)
  {0, 1, 0, 0, 0},  // 5: 01000 (5)
  {0, 1, 0, 0, 1},  // 6: 01001 (5+1)
  {0, 1, 0, 1, 0},  // 7: 01010 (5+2)
  {1, 0, 0, 0, 0},  // 8: 10000 (8)
  {1, 0, 0, 0, 1},  // 9: 10001 (8+1)
};

// POSTNET: weights 7-4-2-1-0 from left to right on the display.
// Each digit encoded with exactly 2 tall bars (1s). Digit 0 = 7+4 = 11 (special).
// Example: digit 5 = 4+1 = 01010 -> {0, 1, 0, 1, 0}
static const uint8_t postnetDigit[10][5] PROGMEM =
{
  {1, 1, 0, 0, 0},  // 0: 11000 (7+4)
  {0, 0, 0, 1, 1},  // 1: 00011 (1+0)
  {0, 0, 1, 0, 1},  // 2: 00101 (2+0)
  {0, 0, 1, 1, 0},  // 3: 00110 (2+1)
  {0, 1, 0, 0, 1},  // 4: 01001 (4+0)
  {0, 1, 0, 1, 0},  // 5: 01010 (4+1)
  {0, 1, 1, 0, 0},  // 6: 01100 (4+2)
  {1, 0, 0, 0, 1},  // 7: 10001 (7+0)
  {1, 0, 0, 1, 0},  // 8: 10010 (7+1)
  {1, 0, 1, 0, 0},  // 9: 10100 (7+2)
};

// Indicator patterns for the 3 parameter settings screens (rows 2-3).
// Each screen has 2 rows of 5 discs. Index order: [screen][row_offset][col].
// row_offset 0 = row 2, row_offset 1 = row 3.
static const uint8_t paramScreenPattern[3][2][5] PROGMEM =
{
  { {0,1,1,1,0}, {0,1,1,1,0} },  // Screen 0: time format
  { {1,0,1,0,1}, {1,0,1,0,1} },  // Screen 1: display mode
  { {0,1,0,1,0}, {1,0,1,0,1} },  // Screen 2: effect interval
};

// =============================================================================
// DISPLAY CONSTANTS
// =============================================================================

static const uint8_t NUM_DISCS = 20;       // Total number of flip-discs
static const uint8_t DISCS_PER_ROW = 5;    // Discs per row
static const uint8_t NUM_ROWS = 4;         // Number of rows

// Delay (ms) between consecutive disc flips during normal display.
// Range 0-50 gives the best visual effect; higher values slow down updates.
static const uint8_t flip_disc_delay = 50;

// Special codes passed to FlipRow() instead of a digit 0-9:
static const uint8_t CAD = 10;  // Clear All Discs - turn all 5 discs in a row OFF
static const uint8_t SAD = 11;  // Set All Discs   - turn all 5 discs in a row ON

// Time format options (stored in EEPROM)
static const uint8_t HR12 = 12;  // 12-hour display (1:00-12:59)
static const uint8_t HR24 = 24;  // 24-hour display (0:00-23:59)

// Effect interval options (stored in EEPROM).
// The numeric values 1/2/3 double as the Morse digit shown during settings.
static const uint8_t E01M = 1;  // Random visual effect every minute
static const uint8_t E60M = 2;  // Random visual effect only on the full hour
static const uint8_t E00M = 3;  // No visual effect - direct digit update

// Parameter settings screen indices (match paramScreenPattern table rows)
static const uint8_t SCREEN_TIME_FORMAT = 0;
static const uint8_t SCREEN_DISPLAY_MODE = 1;
static const uint8_t SCREEN_EFFECT_INTERVAL = 2;

// Number of middle ON->OFF flicker cycles in RandomTimeEffect()
static const uint8_t EFFECT_MINUTE = 1;
static const uint8_t EFFECT_HOUR = 2;

// Display mode options (stored in EEPROM)
static const uint8_t MODE_MORSE = 0;      // Morse code (dot/dash)
static const uint8_t MODE_BINARY = 1;     // Binary (BCD)
static const uint8_t MODE_STIBITZ = 2;    // Excess-3 (Stibitz)
static const uint8_t MODE_FIBONACCI = 3;  // Fibonacci sequence (8-5-3-2-1)
static const uint8_t MODE_POSTNET = 4;    // POSTNET (7-4-2-1-0)

// =============================================================================
// TIMING CONSTANTS
// =============================================================================

static const uint16_t T_BOOT_SETTLE     = 500;   // ms, minimum settle before touching the CC config

/* Timing for the negotiation. There is at most ONE re-request - see the note above
 * PdRequest12V() for why the count is neither zero nor a retry loop.
 * This is simply how long the rail is watched before the state machine gives up and
 * parks in FAULT, from where it keeps re-checking. The configuration pin is never
 * toggled during that time. */
static const uint16_t T_PD_WAIT_12V     = 3000;  // ms, how long to watch for 12 V to appear
static const uint16_t T_PD_RECOVER      = 1500;  // ms, max wait for VBUS to fall back to 5 V
static const uint16_t T_PD_BACKOFF      = 500;   // ms, quiet time after recovery, before re-request
static const uint16_t T_FAULT_RETRY     = 5000;  // ms, automatic retry period while in FAULT

/* Grace period between confirming 12 V and an AUTOMATIC restart. Manual starts (BT3) are
 * not delayed - those are deliberate. This only covers the auto-start case, where the user
 * has just pushed the plug in and may still be holding the clock: without the pause the
 * discs would begin flipping under their fingers, which both surprises them and risks
 * jamming a disc mid-flip. The LED breathes throughout, so the clock visibly waits. */
static const uint16_t T_AUTOSTART_DELAY = 2000;  // ms, pause before an unattended restart
static const uint16_t T_BT3_LONG        = 1500;  // ms, long press threshold for ON/OFF
static const uint16_t T_BT3_CLICK       = 500;   // ms, double click window for BT3
static const uint16_t T_BT1_LONG        = 800;   // ms, long press threshold for the menus
static const uint16_t T_BT1_CLICK       = 500;   // ms, double click window for BT1
static const uint16_t T_ENA_SETTLE      = 250;   // ms, after EN-A = HIGH
static const uint16_t T_ENB_SETTLE      = 150;   // ms, after EN-B = HIGH
static const uint16_t T_VOUT_TIMEOUT    = 200;   // ms, max wait for VOUT to come up
static const uint16_t T_RAIL_OFF_STEP   = 100;   // ms, between switching the rails off
static const uint16_t T_DISCHARGE       = 1200;  // ms, DIS pulse (1.76 mF through 200 R)
static const uint16_t T_VBUS_POLL       = 1000;  // ms, background supply monitoring period
static const uint8_t  N_BAD_SAMPLES     = 3;     // bad readings before declaring supply lost
static const uint8_t  SAMPLE_DELAY_MS   = 10;    // ms, spacing between ADC samples
static const uint8_t  REQUIRED_OK       = 5;     // consecutive good samples = confirmation

// =============================================================================
// ADC THRESHOLDS
// =============================================================================

/* Both dividers are 100k/10k = 1/11. Reference is AVCC = 3.3 V.
 * AREF (pin 20) is tied to AVCC (pin 18) on the board, so analogReference(DEFAULT)
 * is mandatory - INTERNAL would short the internal 1.1 V reference to 3.3 V.
 *
 *   ADC = V * 10/110 / 3.3 * 1023 = V * 28.18
 *
 * Confirmed on hardware: 5.07 V read as 143, 12.14 V read as 342. */
static const uint16_t VBUS_5V_MIN  = 115;  //  4.08 V - source sitting at its default 5 V
static const uint16_t VBUS_5V_MAX  = 175;  //  6.21 V
static const uint16_t VBUS_12V_MIN = 300;  // 10.65 V
static const uint16_t VBUS_12V_MAX = 380;  // 13.49 V
static const uint16_t VBUS_OVER    = 381;  // 13.52 V - CH224K VBUS pin absolute max
static const uint16_t VOUT_OK_MIN  = 280;  //  9.94 V - allows for droop under load
static const uint16_t VOUT_OK_MAX  = 380;  // 13.49 V

// =============================================================================
// EEPROM MAP
// =============================================================================

static const uint16_t ee_time_format_address     = 0;  // Time format 12/24 hour
static const uint16_t ee_effect_interval_address = 2;  // Random effect interval
static const uint16_t ee_display_mode_address    = 4;  // Digit encoding
static const uint16_t ee_power_state_address     = 6;  // Clock ON/OFF across power cycles

/* Magic value rather than 0/1 on purpose: a blank EEPROM reads 0xFF, which therefore
 * means STANDBY. A factory-fresh board will never start the clock on its own. */
static const uint8_t PWR_STATE_RUN = 0xA5;
static const uint8_t PWR_STATE_OFF = 0x00;

// =============================================================================
// PIN DECLARATIONS
// =============================================================================

// Analog inputs for voltage monitoring
const int ADC_VBUS_PIN = A7;   // ADC7, pin 22 - charger voltage
const int ADC_VOUT_PIN = A6;   // ADC6, pin 19 - voltage behind power switch A

// SPI bus - directly drives the shift registers that control disc coils
const int DIN_PIN = PIN_PB3;   // SPI MOSI - serial data input to shift register
const int EN_PIN  = PIN_PB2;   // SPI SS   - latch enable
const int CLK_PIN = PIN_PB5;   // SPI SCK  - shift register clock
const int CLR_PIN = PIN_PB1;   // Directly connected to shift register /CLR

// User buttons - active low, external 100k pull-up plus 10k/100nF RC filter
const int BT1_PIN = PIN_PD4;   // Right button - time set (long), params (dbl click)
const int BT2_PIN = PIN_PC3;   // Left button  - value change
const int BT3_PIN = PIN_PC2;   // ON/OFF (long), hard refresh (dbl click)

// External interrupt inputs
const int INT_RTC = PIN_PD2;   // RTC per-minute interrupt (INT0, falling edge)
const int INT_FLT = PIN_PD3;   // TPS259631 fault output (INT1, active LOW)

/* Status LED on PD5. Three behaviours, nothing else:
 *   breathing   = 12 V negotiated, clock in standby
 *   solid       = clock running
 *   fast blink  = 12 V not (yet) confirmed, or a hardware fault
 *
 * PD5 is OC0B, a Timer0 PWM output, which is what Breathe() and the per-mode ceilings
 * below need. Series resistor 1k from 3.3 V, so full duty is about 1.5 mA. */
const int LED_PIN = PIN_PD5;

/* Per-mode PWM ceiling, applied by MaxBrightness() in SetLedMode(). The 3.3 V rail
 * comes from a linear regulator fed off 12 V, so every mA on it costs 12 mW at the
 * plug. Perceived brightness follows roughly the cube root of duty, so a quarter of
 * the duty still reads as about two thirds of the brightness. RUN is capped hardest
 * because it is permanent; the fault blink is left brightest because it is an alarm.
 * Peak current through the die is unchanged - this is duty, not current.
 * Tune by editing these three numbers, nothing else depends on them. */
static const uint8_t LED_BRIGHT_RUN     =  64;  // 25 %   - solid
static const uint8_t LED_BRIGHT_STANDBY =  96;  // 37.5 % - breath
static const uint8_t LED_BRIGHT_FAULT   = 160;  // 63 %   - alarm

// USB PD negotiation - drives CF1 of the CH224K through R18 1k
const int PD_PIN = PIN_PD7;

/* ---------------------------------------------------------------------------------
 * CFG1 - the board fits a 24k CONFIGURATION RESISTOR from CF1 to GND, and R18 1k
 * between CF1 and PD7. Nothing else is connected there.
 *
 * CFG1 is an ANALOG input, not a logic one: a resistor to GND picks a voltage gear
 * directly, bypassing CFG2/CFG3.
 *
 *       6.8k -> 9 V     24k -> 12 V     56k -> 15 V     open -> 20 V
 *
 * So 12 V is requested from the moment the CH224K has supply, with no firmware
 * involvement at all. R18 1k is small against 24k, so PD7 still has the last word:
 *
 *       pin Hi-Z -> 24k to GND            -> resistor mode -> 12 V
 *       pin LOW  -> ~1k to GND            -> level 0       -> 12 V (CFG2=0, CFG3=1)
 *       pin HIGH -> 3.3 * 24/25 = 3.17 V  -> level 1       ->  5 V
 *
 * Idle and driven-low both give 12 V, so moving between them cannot glitch the rail.
 * CFG1 tolerates 8 V and the datasheet sanctions driving it push-pull from a 3.3 V
 * GPIO, so HIGH is safe.
 * --------------------------------------------------------------------------------- */

// Power switches
const int ENA_PIN = PIN_PC0;   // TPS22810  - main load switch
const int ENB_PIN = PIN_PB0;   // TPS259631 - display eFuse
const int DIS_PIN = PIN_PC1;   // Discharge of the 1.76 mF bulk capacitor bank

// =============================================================================
// STATE MACHINE TYPES
// =============================================================================

enum ClockState : uint8_t
{
  STATE_NEGOTIATE,  // asking the charger for 12 V
  STATE_STANDBY,    // 12 V present, clock apparently off
  STATE_RUN,        // clock running
  STATE_FAULT       // something is wrong, see faultCode
};

enum FaultCode : uint8_t
{
  FAULT_NONE,
  FAULT_NO_12V,     // both negotiation passes failed to bring up 12 V
  FAULT_OVERVOLT,   // VBUS above VBUS_OVER
  FAULT_VOUT,       // power switch A did not deliver
  FAULT_EFUSE       // TPS259631 reported a fault
};

enum LedMode : uint8_t
{
  LED_MODE_NO_12V,   // fast blink - 12 V missing, negotiation running, or a fault
  LED_MODE_STANDBY,  // breathing - 12 V OK, clock switched off
  LED_MODE_RUN       // solid     - clock running
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Time structure used by the RTC library
tmElements_t tm;

// Current time split into 4 individual digits, indexed by display row.
//   digit[0] = units of minutes -> row 0 (bottom, discs 0-4)
//   digit[1] = tens of minutes  -> row 1 (discs 5-9)
//   digit[2] = units of hours   -> row 2 (discs 10-14)
//   digit[3] = tens of hours    -> row 3 (top, discs 15-19)
uint8_t digit[4] = {0, 0, 0, 0};

uint16_t hour_time = 0;    // Current hour (0-23 or 1-12 depending on format)
uint16_t minute_time = 0;  // Current minute (0-59)

// OneButton instances - (pin, activeLow, pullupActive)
OneButton button1(BT1_PIN, true, true);
OneButton button2(BT2_PIN, true, true);
OneButton button3(BT3_PIN, true, true);

// JLed instance. Never driven with digitalWrite() anywhere else in this file -
// SetLedMode() is the single point of control.
JLed ledStatus = JLed(LED_PIN);

// Button event flags - set by callbacks, consumed by the state handlers or the
// settings loops. Using flags keeps the callbacks short and defers action to the
// main context where it is safe to call FlipDisc() etc.
bool shortPressButton2 = false;
bool longPressButton1 = false;
bool doubleClickButton1 = false;

// BT3 derived flags
bool powerToggleStatus = false;     // BT3 long press - turn the clock on or off
bool serviceRequestStatus = false;  // BT3 double click - hard refresh, STANDBY only
bool shutdownRequestStatus = false; // set inside the menus so they can bail out

// State machine flags - only one settings mode can be active at a time
bool timeSettingsActive = false;
bool paramSettingsActive = false;

// Persistent settings loaded from EEPROM at startup and saved when changed
uint8_t time_format = HR12;          // 12h or 24h display
uint8_t effect_interval = E01M;      // Random effect frequency
uint8_t display_mode = MODE_MORSE;   // Digit encoding

// Power state machine
uint8_t clockState = STATE_NEGOTIATE;
uint8_t faultCode = FAULT_NONE;
uint8_t negotiationFault = FAULT_NO_12V;

unsigned long faultRetryMillis = 0;
unsigned long lastVbusPollMillis = 0;
uint8_t badVbusSamples = 0;

// Deferred auto-start: set when the EEPROM says the clock was running, consumed by
// RunStandbyState() once T_AUTOSTART_DELAY has elapsed.
bool autoStartPending = false;
unsigned long autoStartMillis = 0;

// Interrupt flags - set in ISRs, polled and cleared in the main loop
volatile bool rtcInterrupt = false;  // Set every minute by RTC
volatile bool fltInterrupt = false;  // Set by the eFuse fault output

// Disc state snapshots for the visual transition effect.
// current_state[0..19] = which discs are ON for the OLD time (before update)
// target_state[0..19]  = which discs should be ON for the NEW time (after update)
bool current_state[NUM_DISCS];
bool target_state[NUM_DISCS];

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

void SetLedMode(uint8_t mode);
void ServiceLeds(void);
void TickButtons(void);
void WaitMs(uint16_t duration);
void WatchButtons(void);
void ClearButtonFlags(void);
void DiscardMenuButtonFlags(void);

uint16_t ReadAdc(uint8_t pin);
bool ConfirmVoltage(uint8_t pin, uint16_t lo, uint16_t hi, uint8_t required, uint16_t timeout_ms);
bool MonitorVbusLost(void);

void PdRequest12V(void);
void PdRequest5V(void);
bool NegotiatePd12V(void);

uint8_t PowerUpRails(void);
void PowerDownRails(void);
void DisplayBusBegin(void);
void DisplayBusEnd(void);

void StartClock(void);
void StopClock(void);
void EmergencyShutdown(void);
void EnterFault(uint8_t code);
void RunServiceMode(void);

void RunNegotiateState(void);
void RunStandbyState(void);
void RunRunState(void);
void RunFaultState(void);

void LoadSettingsFromEeprom(void);
void GetRtcTime(void);
uint8_t GetDigitPattern(uint8_t digit_value, uint8_t col);
void FlipRow(uint8_t row, uint8_t digit_or_cmd);
void FlipDisc(uint8_t disc_number, bool disc_status);
void ClearOutputs(void);
void ComputeDisplayState(bool* state);
void RandomFlipDisc(bool disc_status);
void RandomFlipOffCurrent(void);
void RandomFlipOnTarget(void);
void RandomTimeEffect(uint8_t middle_cycles);
void DisplayRawTime(void);
void DisplayTime(void);
void ShowParamScreen(uint8_t screen);
void ParamSettings(void);
void TimeSettings(void);
void HardFlipDiscCleaning(void);

// =============================================================================
// INTERRUPT SERVICE ROUTINES
// =============================================================================
//
// ISRs are kept minimal - they only set a flag.  All real work happens
// in the main loop to avoid issues with SPI, delay(), and other
// non-reentrant functions.

void rtcISR(void) { rtcInterrupt = true; }
void fltISR(void) { fltInterrupt = true; }

// =============================================================================
// BUTTON CALLBACKS
// =============================================================================

void onShortPressButton2(void)  { shortPressButton2 = true; }
void onLongPressButton1(void)   { longPressButton1 = true; }
void onDoubleClickButton1(void) { doubleClickButton1 = true; }

/*
 * onLongPressButton3() - fires after T_BT3_LONG of holding BT3. ON/OFF toggle.
 */
void onLongPressButton3(void) { powerToggleStatus = true; }

/*
 * onDoubleClickButton3() - hard refresh request. Only STANDBY acts on it; every other
 * state discards it, so the flag can never fire late after a state change.
 */
void onDoubleClickButton3(void) { serviceRequestStatus = true; }

// ClearButtonFlags - reset all button event flags.
void ClearButtonFlags(void)
{
  shortPressButton2 = false;
  longPressButton1 = false;
  doubleClickButton1 = false;
  powerToggleStatus = false;
  serviceRequestStatus = false;
  shutdownRequestStatus = false;
}

/*
 * DiscardMenuButtonFlags() - drop BT1/BT2 events without touching the BT3 ones.
 * Used in STANDBY and FAULT, where the menus must not be reachable. Both BT3
 * events survive on purpose: the long press toggles power and the double click
 * asks for a hard refresh, and STANDBY handles both.
 */
void DiscardMenuButtonFlags(void)
{
  shortPressButton2 = false;
  longPressButton1 = false;
  doubleClickButton1 = false;
}

// =============================================================================
// LED CONTROL
// =============================================================================

/*
 * SetLedMode() - the only place that assigns LED effects.
 */
/*
 * MaxBrightness() rescales the effect output from [0..255] to [0..level] on every
 * update (JLed lerp8by8), so it works uniformly on all three effects: it lowers the
 * ON phase of Blink, the peak of Breathe and the level of On() alike, and never
 * touches the timing. See the LED CURRENT LIMITING note above for the numbers.
 */
void SetLedMode(uint8_t mode)
{
  switch(mode)
  {
    case LED_MODE_NO_12V:
      ledStatus = JLed(LED_PIN).Blink(100, 100).MaxBrightness(LED_BRIGHT_FAULT).Forever();
      break;

    case LED_MODE_STANDBY:
      ledStatus = JLed(LED_PIN).Breathe(3000).MaxBrightness(LED_BRIGHT_STANDBY).Forever();
      break;

    case LED_MODE_RUN:
      ledStatus = JLed(LED_PIN).On().MaxBrightness(LED_BRIGHT_RUN);
      break;

    default:            // unreachable by design - the LED always shows one of the three
      ledStatus = JLed(LED_PIN).Off();
      break;
  }

  ledStatus.Update();
}

/*
 * ServiceLeds() - must be called from every loop, including the waiting loops
 * and the fault state, otherwise the effect freezes.
 */
void ServiceLeds(void)
{
  ledStatus.Update();
}

// =============================================================================
// TIMING AND BUTTON SERVICE HELPERS
// =============================================================================

void TickButtons(void)
{
  button1.tick();
  button2.tick();
  button3.tick();
}

/*
 * WaitMs() - blocking wait that keeps the watchdog fed, the LED effect running
 * and the buttons responsive. Replaces every delay() longer than ~100 ms.
 */
void WaitMs(uint16_t duration)
{
  unsigned long startMillis = millis();

  while(millis() - startMillis < duration)
  {
    wdt_reset();
    ServiceLeds();
    TickButtons();
  }
}

/*
 * WatchButtons() - the service call used from inside the blocking settings menus.
 *
 * Keeps the watchdog fed and translates a BT3 long press into shutdownRequestStatus,
 * which the menu loops check so BT3 works everywhere.
 *
 * Note that it does NOT dispatch into the settings modes - that decision lives in
 * RunRunState() only, so a menu can never re-enter itself.
 */
void WatchButtons(void)
{
  wdt_reset();
  ServiceLeds();
  TickButtons();

  if(powerToggleStatus) shutdownRequestStatus = true;
}

// =============================================================================
// ADC HELPERS
// =============================================================================

/*
 * ReadAdc() - one discarded conversion for multiplexer settling, then the real one.
 */
uint16_t ReadAdc(uint8_t pin)
{
  analogRead(pin);
  return analogRead(pin);
}

/*
 * ConfirmVoltage() - returns true as soon as 'required' consecutive samples fall
 * inside [lo, hi]. Returns false if that does not happen within timeout_ms.
 *
 * The early exit is what makes the negotiation fast: a charger that switches to
 * 12 V in 60 ms is confirmed in about 110 ms instead of waiting out a fixed delay.
 */
bool ConfirmVoltage(uint8_t pin, uint16_t lo, uint16_t hi, uint8_t required, uint16_t timeout_ms)
{
  unsigned long startMillis = millis();
  unsigned long lastSampleMillis = millis() - SAMPLE_DELAY_MS;
  uint8_t okCount = 0;

  while(millis() - startMillis < timeout_ms)
  {
    wdt_reset();
    ServiceLeds();
    TickButtons();

    if(millis() - lastSampleMillis >= SAMPLE_DELAY_MS)
    {
      lastSampleMillis = millis();

      uint16_t value = ReadAdc(pin);

      if(value >= lo && value <= hi)
      {
        okCount++;
        if(okCount >= required) return true;
      }
      else
      {
        okCount = 0;
      }
    }
  }

  return false;
}

// =============================================================================
// USB PD NEGOTIATION
// =============================================================================

/* ---------------------------------------------------------------------------------
 * THE PD LINE - WHEN the request is made matters as much as what it asks for.
 *
 * A sink can reach 12 V two ways. As an INITIAL CONTRACT, where CF1 already asks for
 * 12 V when the source sends its first Source_Capabilities, or as a RENEGOTIATION
 * inside an existing 5 V contract. Both are legal, but renegotiation is far more
 * demanding on the source: a strict one answers with a Hard Reset, which drives VBUS
 * to vSafe0V for tSrcRecover (660-1000 ms). That browns out this board, it reboots
 * and asks again - a reset loop with VBUS collapsing about once a second.
 *
 * Hence three rules:
 *
 *   1. Assert the 12 V request as the FIRST executable instruction in setup(), so it
 *      is standing before the CH224K can answer the source.
 *   2. Move CF1 at most ONCE afterwards, and only after VBUS has been MEASURED back
 *      at a stable 5 V. The count cannot be zero either: the CH224K only emits a
 *      Request in answer to a Source_Capabilities, and a source stops advertising
 *      once a contract exists - so a CF1 merely held low generates no further traffic
 *      and the board would sit at 5 V forever. One re-request is the recovery path.
 *   3. The only other reason to move CF1 is overvoltage, where dropping to 5 V is
 *      protection rather than a retry.
 * --------------------------------------------------------------------------------- */

/* Releasing the pin IS the 12 V request, so this is a no-op on a freshly reset MCU -
 * the 24k resistor has been asking for 12 V since the CH224K powered up. Driving LOW
 * first is harmless: ~1k to GND reads as level 0, which also means 12 V, so the rail
 * never moves. */
void PdRequest12V(void)
{
  digitalWrite(PD_PIN, LOW);  // level 0 -> 12 V, same rail as the resistor selects
  pinMode(PD_PIN, INPUT);     // Hi-Z -> resistor mode. NEVER INPUT_PULLUP
}

void PdRequest5V(void)
{
  pinMode(PD_PIN, OUTPUT);
  digitalWrite(PD_PIN, HIGH); // 3.17 V at CFG1 through R18 1k -> level 1 -> 5 V
}

/*
 * WatchFor12V() - poll the rail and report whether 12 V showed up.
 * Sets negotiationFault on overvoltage. Never touches CF1.
 */
static bool WatchFor12V(void)
{
  if(ConfirmVoltage(ADC_VBUS_PIN, VBUS_12V_MIN, VBUS_12V_MAX, REQUIRED_OK, T_PD_WAIT_12V))
  {
    DBG_PRINTLN(F("PD 12V ok"));
    return true;
  }

  uint16_t reading = ReadAdc(ADC_VBUS_PIN);

  DBG_PRINT(F("PD no 12V, ADC="));
  DBG_PRINTLN(reading);

  if(reading >= VBUS_OVER) negotiationFault = FAULT_OVERVOLT;

  return false;
}

/*
 * NegotiatePd12V() - two passes at most, then give up and let FAULT retry.
 *
 * Pass 1 costs nothing on this board: the request is already standing, so this only
 * watches the rail. It also covers the "12 V is already there" case - after a watchdog
 * reset that never disturbed the charger, ConfirmVoltage() returns almost immediately.
 *
 * Pass 2 is the single deliberate re-request. Overvoltage skips it: releasing to 5 V
 * there is protection, not a retry.
 *
 * Sets negotiationFault to the reason on failure.
 */
bool NegotiatePd12V(void)
{
  SetLedMode(LED_MODE_NO_12V);
  negotiationFault = FAULT_NO_12V;

  // Pass 1 - the resistor is already asking for 12 V, so this only watches the rail.
  PdRequest12V();

  if(WatchFor12V()) return true;

  if(negotiationFault == FAULT_OVERVOLT)
  {
    PdRequest5V();   // protective fallback, the one non-negotiation reason to move CF1
    return false;
  }

  /* Pass 2 - one deliberate re-request, the only recovery path there is: a source stops
   * advertising once a contract exists, so a CF1 left alone produces no further PD
   * traffic and the board would sit at 5 V forever. Gated on a MEASURED return to 5 V,
   * because re-requesting mid-transition is what tips a strict source into a Hard
   * Reset. */
  DBG_PRINTLN(F("PD re-request"));

  PdRequest5V();
  ConfirmVoltage(ADC_VBUS_PIN, VBUS_5V_MIN, VBUS_5V_MAX, REQUIRED_OK, T_PD_RECOVER);
  WaitMs(T_PD_BACKOFF);

  PdRequest12V();

  if(WatchFor12V()) return true;

  if(negotiationFault == FAULT_OVERVOLT) PdRequest5V();

  /* Give up for now. CF1 is left asking for 12 V, so a capable source plugged in later
   * is answered by the CH224K without any firmware involvement. */
  return false;
}

// =============================================================================
// POWER RAIL CONTROL
// =============================================================================

/*
 * PowerUpRails() - brings up VOUT then VFD, verifying each step.
 * Returns FAULT_NONE on success, otherwise the fault code, with the rails left off.
 */
uint8_t PowerUpRails(void)
{
  digitalWrite(DIS_PIN, LOW);

  digitalWrite(ENA_PIN, HIGH);
  WaitMs(T_ENA_SETTLE);

  if(!ConfirmVoltage(ADC_VOUT_PIN, VOUT_OK_MIN, VOUT_OK_MAX, REQUIRED_OK, T_VOUT_TIMEOUT))
  {
    digitalWrite(ENA_PIN, LOW);
    DBG_PRINTLN(F("VOUT fault"));
    return FAULT_VOUT;
  }

  digitalWrite(ENB_PIN, HIGH);
  WaitMs(T_ENB_SETTLE);

  fltInterrupt = false;

  if(digitalRead(INT_FLT) == LOW)
  {
    digitalWrite(ENB_PIN, LOW);
    WaitMs(T_RAIL_OFF_STEP);
    digitalWrite(ENA_PIN, LOW);
    DBG_PRINTLN(F("eFuse fault"));
    return FAULT_EFUSE;
  }

  return FAULT_NONE;
}

/*
 * PowerDownRails() - orderly shutdown of both rails plus a discharge pulse.
 *
 * The bank is 8 x 220 uF = 1.76 mF and R26 is 200 R, so tau is about 350 ms.
 * T_DISCHARGE covers three time constants.
 */
void PowerDownRails(void)
{
  digitalWrite(ENB_PIN, LOW);
  WaitMs(T_RAIL_OFF_STEP);

  digitalWrite(ENA_PIN, LOW);
  WaitMs(T_RAIL_OFF_STEP);

  digitalWrite(DIS_PIN, HIGH);
  WaitMs(T_DISCHARGE);
  digitalWrite(DIS_PIN, LOW);
}

void DisplayBusBegin(void)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  WaitMs(100);
  ClearOutputs();
}

/*
 * DisplayBusEnd() - releases SPI and parks every display line LOW.
 *
 * Driving the lines LOW while the display board is unpowered is the safe choice:
 * a HIGH level would forward-bias the ESD diodes of the unpowered shift registers.
 */
void DisplayBusEnd(void)
{
  SPI.endTransaction();
  SPI.end();

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIN_PIN, LOW);
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(CLR_PIN, LOW);
}

// =============================================================================
// CLOCK START / STOP
// =============================================================================

/*
 * StartClock() - STANDBY (or auto-start after a power cut) -> RUN.
 */
void StartClock(void)
{
  /* Switch the LED first, before anything slow happens. The long press fires at
   * T_BT3_LONG and the operator must see the acknowledgement right then - the rails
   * and the startup animation take another few seconds after this point. */
  SetLedMode(LED_MODE_RUN);

  autoStartPending = false;   // consumed, whether this start was automatic or manual
  ClearButtonFlags();

  uint8_t fault = PowerUpRails();

  if(fault != FAULT_NONE)
  {
    EnterFault(fault);
    return;
  }

  /* Only now is it known that the hardware can actually run. Writing PWR_STATE_RUN
   * any earlier would latch "should be running" even for a permanently faulty board,
   * so every following power-up would repeat the same failure. */
  EEPROM.update(ee_power_state_address, PWR_STATE_RUN);

  DisplayBusBegin();

  attachInterrupt(digitalPinToInterrupt(INT_RTC), rtcISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_FLT), fltISR, FALLING);

  /* Seed the shuffle from the RTC. Without this every power-up would replay the exact
   * same "random" disc order, because the AVR PRNG starts from a fixed state. */
  RTC_RX8025T.read(tm);
  randomSeed(((uint32_t)tm.Second << 16) ^ micros());

  // Startup animation. It begins from the all-discs-ON panel left behind by
  // StopClock(), so the first pass reads as a deliberate full wipe.
  RandomFlipDisc(0);  // All OFF in random order
  RandomFlipDisc(1);  // All ON in random order (visual "hello")

  // Display without effect so digit[] matches the physical panel.
  GetRtcTime();
  DisplayRawTime();

  rtcInterrupt = false;
  fltInterrupt = false;
  timeSettingsActive = false;
  paramSettingsActive = false;
  badVbusSamples = 0;
  lastVbusPollMillis = millis();

  ClearButtonFlags();
  clockState = STATE_RUN;
}

/*
 * StopClock() - RUN -> STANDBY, triggered by a BT3 long press.
 *
 * Apparent shutdown only: the MCU keeps running and 12 V stays negotiated.
 * All 20 discs are turned ON first - a full yellow panel is the agreed
 * "switched off" indication, and the discs are bistable so it survives
 * the loss of the display rails.
 */
void StopClock(void)
{
  // Immediate acknowledgement of the long press - see the note in StartClock().
  SetLedMode(LED_MODE_STANDBY);

  autoStartPending = false;
  ClearButtonFlags();

  EEPROM.update(ee_power_state_address, PWR_STATE_OFF);

  RandomFlipDisc(1);  // All discs ON in random order = "clock is off"

  detachInterrupt(digitalPinToInterrupt(INT_RTC));
  detachInterrupt(digitalPinToInterrupt(INT_FLT));

  DisplayBusEnd();
  PowerDownRails();

  rtcInterrupt = false;
  fltInterrupt = false;
  timeSettingsActive = false;
  paramSettingsActive = false;
  badVbusSamples = 0;
  lastVbusPollMillis = millis();

  ClearButtonFlags();
  clockState = STATE_STANDBY;
}

/*
 * EmergencyShutdown() - rails down without touching the display or the EEPROM.
 *
 * Leaving ee_power_state_address at PWR_STATE_RUN is exactly what makes the clock
 * come back on its own once the supply returns. Do not add an EEPROM write here.
 */
void EmergencyShutdown(void)
{
  detachInterrupt(digitalPinToInterrupt(INT_RTC));
  detachInterrupt(digitalPinToInterrupt(INT_FLT));

  digitalWrite(ENB_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);

  DisplayBusEnd();

  digitalWrite(DIS_PIN, HIGH);
  WaitMs(T_DISCHARGE);
  digitalWrite(DIS_PIN, LOW);

  rtcInterrupt = false;
  fltInterrupt = false;
  timeSettingsActive = false;
  paramSettingsActive = false;
  badVbusSamples = 0;
}

// =============================================================================
// FAULT HANDLING
// =============================================================================

void EnterFault(uint8_t code)
{
  faultCode = code;

  if(code == FAULT_OVERVOLT) PdRequest5V();

  SetLedMode(LED_MODE_NO_12V);

  faultRetryMillis = millis();
  ClearButtonFlags();
  clockState = STATE_FAULT;
}

// =============================================================================
// BACKGROUND SUPPLY MONITORING
// =============================================================================

/*
 * MonitorVbusLost() - non-blocking check every T_VBUS_POLL.
 *
 * A single bad reading is ignored on purpose: flipping a disc draws a current pulse
 * that can momentarily sag the rail. Only N_BAD_SAMPLES in a row count as a loss.
 *
 * Called from the state handlers in loop(), never from inside FlipDisc(), so a
 * measurement can never coincide with a disc pulse.
 */
bool MonitorVbusLost(void)
{
  if(millis() - lastVbusPollMillis < T_VBUS_POLL) return false;

  lastVbusPollMillis = millis();

  uint16_t value = ReadAdc(ADC_VBUS_PIN);

  if(value < VBUS_12V_MIN || value > VBUS_12V_MAX) badVbusSamples++;
  else                                             badVbusSamples = 0;

  if(badVbusSamples >= N_BAD_SAMPLES)
  {
    badVbusSamples = 0;
    DBG_PRINTLN(F("VBUS lost"));
    return true;
  }

  return false;
}

// =============================================================================
// SERVICE MODE
// =============================================================================

/*
 * RunServiceMode() - hard refresh from STANDBY (BT3 double click).
 *
 * STANDBY-only by design. In RUN the discs are showing the time, so the gesture is
 * discarded there rather than acted on - see the note at the top of RunRunState().
 *
 * HardFlipDiscCleaning() ends with every disc ON, which happens to be exactly the
 * panel state STANDBY expects, so nothing has to be repainted afterwards.
 */
void RunServiceMode(void)
{
  SetLedMode(LED_MODE_RUN);

  /* Servicing is a deliberate interruption, so a pending auto-start is dropped rather than
   * firing in the middle of it. The EEPROM still says RUN, so the next power-up restarts
   * the clock as usual. */
  autoStartPending = false;

  uint8_t fault = PowerUpRails();

  if(fault != FAULT_NONE)
  {
    EnterFault(fault);
    return;
  }

  DisplayBusBegin();
  HardFlipDiscCleaning();
  DisplayBusEnd();
  PowerDownRails();

  // ee_power_state_address is deliberately untouched - servicing is not "clock on".
  badVbusSamples = 0;
  lastVbusPollMillis = millis();

  SetLedMode(LED_MODE_STANDBY);
  ClearButtonFlags();
  clockState = STATE_STANDBY;
}

// =============================================================================
// SETTINGS PERSISTENCE
// =============================================================================

/*
 * LoadSettingsFromEeprom() - read the three user settings and fall back to defaults
 * on garbage (first boot or corruption).
 *
 * Also used by the menu abort paths: without reloading, a half-entered value would
 * stay active in RAM while the EEPROM still held the old one.
 */
void LoadSettingsFromEeprom(void)
{
  time_format     = EEPROM.read(ee_time_format_address);
  effect_interval = EEPROM.read(ee_effect_interval_address);
  display_mode    = EEPROM.read(ee_display_mode_address);

  if(time_format != HR12 && time_format != HR24) time_format = HR12;
  if(effect_interval != E01M && effect_interval != E60M && effect_interval != E00M) effect_interval = E01M;
  if(display_mode > MODE_POSTNET) display_mode = MODE_MORSE;
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
  /* Belt and braces. The 24k resistor already asks for 12 V and a reset leaves PD7 an
   * input with no pull-up, so this changes nothing on a fresh start - it just makes the
   * released state explicit whatever the pin was doing before. */
  PdRequest12V();

  wdt_disable();  // Disable watchdog in case of WDT reset

  // Power control outputs - everything off before anything else happens
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(DIS_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);
  digitalWrite(DIS_PIN, LOW);

  // Display bus parked low - the display board has no supply yet
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIN_PIN, LOW);
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(CLR_PIN, LOW);

  pinMode(INT_RTC, INPUT_PULLUP);  // RTC interrupt (active low)
  pinMode(INT_FLT, INPUT_PULLUP);  // Fault input (active low)

  /* AREF is tied to AVCC on the board, so DEFAULT is the only legal reference.
   * INTERNAL would short the internal 1.1 V band gap to the 3.3 V rail. */
  analogReference(DEFAULT);
  analogRead(ADC_VBUS_PIN);
  analogRead(ADC_VBUS_PIN);

  DBG_BEGIN();
  DBG_PRINTLN(F("MCC REV6 boot"));

  // --- Button setup ---
  // BT1: long press (menus / confirm), double click (parameter settings)
  button1.attachLongPressStart(onLongPressButton1);
  button1.attachDoubleClick(onDoubleClickButton1);
  button1.setDebounceMs(50);
  button1.setClickMs(T_BT1_CLICK);
  button1.setPressMs(T_BT1_LONG);

  // BT2: short press only - value change inside the menus. BT3 owns ON/OFF.
  button2.attachClick(onShortPressButton2);
  button2.setDebounceMs(50);

  // BT3: long press (ON/OFF), double click (hard refresh - acted on in STANDBY only)
  button3.attachLongPressStart(onLongPressButton3);
  button3.attachDoubleClick(onDoubleClickButton3);
  button3.setDebounceMs(50);
  button3.setClickMs(T_BT3_CLICK);
  button3.setPressMs(T_BT3_LONG);

  /* Blink from the very first moment, not after the settle delay: 12 V is not
   * confirmed yet, and it makes a reset loop immediately visible. */
  SetLedMode(LED_MODE_NO_12V);

  LoadSettingsFromEeprom();

  // Settle time before touching the CH224K: lets the 3.3 V rail and the CH224K VDD
  // stabilise and lets the charger finish its initial 5 V contract.
  WaitMs(T_BOOT_SETTLE);

  /* The RTC sits on the always-on 3.3 V rail, independent of EN-A and EN-B, so it is
   * initialised once here and never re-initialised when the clock is switched on.
   * RX8025T::init() only writes the CONTROL / EXT / STATUS registers and resets the
   * module solely when the VLF/VDET flags are set - it never touches the time. */
  RTC_RX8025T.init();
  RTC_RX8025T.initTUI(INT_MINUTE);  // Configure for per-minute interrupts
  RTC_RX8025T.statusTUI(INT_ON);    // Enable interrupt output

  timeSettingsActive = false;
  paramSettingsActive = false;

  // 8-second watchdog. wdt_reset() is called in WaitMs(), WatchButtons() and FlipDisc().
  wdt_enable(WDTO_8S);

  ClearButtonFlags();
  clockState = STATE_NEGOTIATE;
}

// =============================================================================
// MAIN LOOP - STATE MACHINE DISPATCH
// =============================================================================

void loop()
{
  wdt_reset();
  ServiceLeds();
  TickButtons();

  switch(clockState)
  {
    case STATE_NEGOTIATE: RunNegotiateState(); break;
    case STATE_STANDBY:   RunStandbyState();   break;
    case STATE_RUN:       RunRunState();       break;
    case STATE_FAULT:     RunFaultState();     break;
    default:              clockState = STATE_NEGOTIATE; break;
  }
}

// =============================================================================
// STATE HANDLERS
// =============================================================================

void RunNegotiateState(void)
{
  if(NegotiatePd12V())
  {
    badVbusSamples = 0;
    lastVbusPollMillis = millis();

    /* The clock remembers whether it was running, so an uncontrolled power cut is followed
     * by an automatic restart once 12 V is back - but NOT immediately. The clock parks in
     * STANDBY with the LED breathing and only starts after T_AUTOSTART_DELAY, giving whoever
     * just plugged the cable in time to put the clock down before the discs start moving. */
    if(EEPROM.read(ee_power_state_address) == PWR_STATE_RUN)
    {
      DBG_PRINTLN(F("auto-start pending"));
      autoStartPending = true;
      autoStartMillis = millis();
    }
    else
    {
      autoStartPending = false;
    }

    SetLedMode(LED_MODE_STANDBY);
    ClearButtonFlags();
    clockState = STATE_STANDBY;
  }
  else
  {
    EnterFault(negotiationFault);
  }
}

void RunStandbyState(void)
{
  if(serviceRequestStatus)
  {
    serviceRequestStatus = false;
    RunServiceMode();
    return;
  }

  if(powerToggleStatus)
  {
    powerToggleStatus = false;
    StartClock();   // manual start - no grace period, the press is deliberate
    return;
  }

  // Deferred auto-start after a power cut. Checked after the buttons so the user can
  // always start it early with BT3, or divert into service mode.
  if(autoStartPending && (millis() - autoStartMillis >= T_AUTOSTART_DELAY))
  {
    DBG_PRINTLN(F("auto-start now"));
    StartClock();
    return;
  }

  if(MonitorVbusLost())
  {
    clockState = STATE_NEGOTIATE;
    return;
  }

  // The menus and the effect need display power, so BT1/BT2 do nothing here.
  DiscardMenuButtonFlags();
}

void RunRunState(void)
{
  // BT3 long press - apparent shutdown. Checked first so it works from anywhere.
  if(powerToggleStatus)
  {
    powerToggleStatus = false;
    StopClock();
    return;
  }

  /* The hard refresh is STANDBY-only, so the flag is dropped rather than left pending:
   * a double click landing here must not fire a 4 s cleaning cycle later, once the clock
   * has been switched off and STANDBY starts looking at the flag again. */
  serviceRequestStatus = false;

  if(fltInterrupt)
  {
    fltInterrupt = false;
    EmergencyShutdown();
    EnterFault(FAULT_EFUSE);
    return;
  }

  if(MonitorVbusLost())
  {
    EmergencyShutdown();   // no EEPROM write - that is what enables the auto-restart
    clockState = STATE_NEGOTIATE;
    return;
  }

  // Menu entry decisions live here only, so the menus cannot re-enter themselves.
  if(longPressButton1 && !paramSettingsActive)
  {
    longPressButton1 = false;
    timeSettingsActive = true;
  }

  if(doubleClickButton1 && !timeSettingsActive)
  {
    doubleClickButton1 = false;
    paramSettingsActive = true;
  }

  if(rtcInterrupt)         DisplayTime();      // Per-minute time update
  if(timeSettingsActive)   TimeSettings();     // Time digit setting UI
  if(paramSettingsActive)  ParamSettings();    // Parameter settings UI
}

void RunFaultState(void)
{
  /* A BT3 long press retries immediately instead of waiting for the timer. The double
   * click does NOT - it is a STANDBY-only gesture, so it is discarded here like anywhere
   * else outside STANDBY. Nothing is lost: the long press already covers the retry. */
  serviceRequestStatus = false;

  if(powerToggleStatus)
  {
    powerToggleStatus = false;

    if(faultCode == FAULT_VOUT || faultCode == FAULT_EFUSE)
    {
      // 12 V itself is fine here, so retry powering the display directly.
      StartClock();
    }
    else
    {
      clockState = STATE_NEGOTIATE;
    }
    return;
  }

  /* Supply faults keep retrying on their own. Pulling the USB-C plug resets the MCU
   * anyway, so this only helps the case of a charger that needed a moment. */
  if(faultCode == FAULT_NO_12V || faultCode == FAULT_OVERVOLT)
  {
    if(millis() - faultRetryMillis >= T_FAULT_RETRY)
    {
      faultRetryMillis = millis();
      clockState = STATE_NEGOTIATE;
    }
  }

  DiscardMenuButtonFlags();
}

// =============================================================================
// LOW-LEVEL DISC CONTROL
// =============================================================================
//
// Protocol for flipping one disc:
//   1. Pull EN (latch) LOW  - shift register accepts data
//   2. SPI.transfer() 3 bytes - the pattern for the target disc
//   3. Pull EN HIGH - data is latched to the coil driver outputs
//   4. Wait ~1.2 ms - coil energizes and the disc flips
//   5. ClearOutputs() - de-energize all coils to prevent overheating
//   6. Wait flip_disc_delay ms - visual pacing between consecutive flips

// ClearOutputs - de-energize all coil drivers and flush the shift register.
void ClearOutputs(void)
{
  // Clear the shift register via hardware /CLR
  digitalWrite(CLR_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(CLR_PIN, HIGH);

  // Latch the cleared register to outputs
  digitalWrite(EN_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(EN_PIN, HIGH);

  // Shift three zero bytes and latch - ensures outputs are fully de-energized
  digitalWrite(EN_PIN, LOW);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(EN_PIN, HIGH);
}

/*
 * FlipDisc - flip a single disc to the desired state.
 *
 *   disc_number: 0-19 (physical disc index; >19 is silently ignored)
 *   disc_status: true = ON (yellow side), false = OFF (black side)
 *
 * The pacing waits use WaitMs(), so the watchdog stays fed, the LED effect keeps
 * running and BT3 stays responsive even during a long animation.
 */
void FlipDisc(uint8_t disc_number, bool disc_status)
{
  if(disc_number > 19) return;  // Out of range - ignore

  wdt_reset();

  // Shift out the 3-byte address pattern while EN is LOW
  digitalWrite(EN_PIN, LOW);

  for(uint8_t i = 0; i < 3; i++)
  {
    if(disc_status) SPI.transfer(pgm_read_byte(&setDisc[disc_number][i]));
    else            SPI.transfer(pgm_read_byte(&resetDisc[disc_number][i]));
  }

  // Latch the pattern to outputs - coil driver activates
  digitalWrite(EN_PIN, HIGH);

  // Hold coil energized for 1.2 ms - enough to flip the disc
  delayMicroseconds(1200);

  // De-energize all coils immediately
  ClearOutputs();

  // Visual pacing: 20 ms settling + configurable delay between flips
  WaitMs(20);
  WaitMs(flip_disc_delay);
}

// =============================================================================
// DIGIT PATTERN HELPER
// =============================================================================

/*
 * GetDigitPattern - returns the ON/OFF state for one disc position of a digit.
 *
 *   digit_value: digit value 0-9
 *   col: physical column 0-4 (0 = rightmost disc, 4 = leftmost disc)
 *
 * The pattern tables are stored in VISUAL order (index 0 = leftmost on display).
 * Physical column 0 = rightmost disc = table index 4, so we index with (4 - col).
 */
uint8_t GetDigitPattern(uint8_t digit_value, uint8_t col)
{
  uint8_t index = 4 - col;  // Convert physical column to visual table index

  if(display_mode == MODE_BINARY)
    return pgm_read_byte(&binaryDigit[digit_value][index]);
  else if(display_mode == MODE_STIBITZ)
    return pgm_read_byte(&stibitzDigit[digit_value][index]);
  else if(display_mode == MODE_FIBONACCI)
    return pgm_read_byte(&fibonacciDigit[digit_value][index]);
  else if(display_mode == MODE_POSTNET)
    return pgm_read_byte(&postnetDigit[digit_value][index]);
  else
    return pgm_read_byte(&morseDigit[digit_value][index]);
}

// =============================================================================
// ROW-LEVEL DISPLAY
// =============================================================================

/*
 * FlipRow - display a digit (or special code) on one row of 5 discs.
 *
 *   row: 0-3 (display row; 0 = bottom, 3 = top)
 *   digit_or_cmd: 0-9 (digit in the current display_mode encoding),
 *                 CAD (10) all 5 discs OFF, SAD (11) all ON.
 */
void FlipRow(uint8_t row, uint8_t digit_or_cmd)
{
  if(row > 3) return;            // Invalid row
  if(digit_or_cmd > 11) return;  // Invalid data code

  for(uint8_t col = 0; col < DISCS_PER_ROW; col++)
  {
    uint8_t disc_number = row * DISCS_PER_ROW + col;
    bool disc_state;

    if(digit_or_cmd == CAD)       disc_state = 0;
    else if(digit_or_cmd == SAD)  disc_state = 1;
    else                          disc_state = GetDigitPattern(digit_or_cmd, col);

    FlipDisc(disc_number, disc_state);
  }
}

// =============================================================================
// COMPUTE DISPLAY STATE
// =============================================================================

/*
 * ComputeDisplayState - compute the ON/OFF state of all 20 discs for the current
 * digit[] values and store them in the provided bool[20] array.
 *
 * Called TWICE in DisplayTime():
 *   1. BEFORE GetRtcTime() - fills current_state[] with the OLD time pattern
 *   2. AFTER GetRtcTime()  - fills target_state[] with the NEW time pattern
 */
void ComputeDisplayState(bool* state)
{
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    for(uint8_t col = 0; col < DISCS_PER_ROW; col++)
    {
      state[row * DISCS_PER_ROW + col] = GetDigitPattern(digit[row], col);
    }
  }
}

// =============================================================================
// RANDOM EFFECT FUNCTIONS
// =============================================================================
//
// These create the visual transition effect when the time changes. Instead of
// updating discs in order (which looks mechanical), Fisher-Yates shuffles flip
// discs in a random sequence (looks organic).

/*
 * RandomFlipDisc - flip ALL 20 discs to the given state in random order.
 */
void RandomFlipDisc(bool disc_status)
{
  uint8_t order[NUM_DISCS];
  for(uint8_t i = 0; i < NUM_DISCS; i++) order[i] = i;

  // Fisher-Yates shuffle
  for(uint8_t i = NUM_DISCS - 1; i > 0; i--)
  {
    uint8_t j = random(0, i + 1);
    uint8_t tmp = order[i];
    order[i] = order[j];
    order[j] = tmp;
  }

  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    FlipDisc(order[i], disc_status);
  }
}

/*
 * RandomFlipOffCurrent - randomly turn OFF only the discs that are currently ON.
 * First step of the transition: the old time "disappears" from the display.
 */
void RandomFlipOffCurrent(void)
{
  uint8_t on_discs[NUM_DISCS];
  uint8_t on_count = 0;

  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    if(current_state[i])
    {
      on_discs[on_count] = i;
      on_count++;
    }
  }

  if(on_count > 1)
  {
    for(uint8_t i = on_count - 1; i > 0; i--)
    {
      uint8_t j = random(0, i + 1);
      uint8_t tmp = on_discs[i];
      on_discs[i] = on_discs[j];
      on_discs[j] = tmp;
    }
  }

  for(uint8_t i = 0; i < on_count; i++)
  {
    FlipDisc(on_discs[i], 0);
  }
}

/*
 * RandomFlipOnTarget - randomly turn ON only the discs needed for the new time.
 * Last step of the transition: the new time "appears" on the display.
 */
void RandomFlipOnTarget(void)
{
  uint8_t on_discs[NUM_DISCS];
  uint8_t on_count = 0;

  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    if(target_state[i])
    {
      on_discs[on_count] = i;
      on_count++;
    }
  }

  if(on_count > 1)
  {
    for(uint8_t i = on_count - 1; i > 0; i--)
    {
      uint8_t j = random(0, i + 1);
      uint8_t tmp = on_discs[i];
      on_discs[i] = on_discs[j];
      on_discs[j] = tmp;
    }
  }

  for(uint8_t i = 0; i < on_count; i++)
  {
    FlipDisc(on_discs[i], 1);
  }
}

/*
 * RandomTimeEffect - full visual transition from old time to new time.
 *
 *   middle_cycles: number of full-display ON->OFF flicker cycles.
 *     EFFECT_MINUTE (1) for normal minute changes - subtle.
 *     EFFECT_HOUR (2) for full hour changes - more dramatic.
 *
 * Precondition: current_state[] and target_state[] must be computed BEFORE calling.
 */
void RandomTimeEffect(uint8_t middle_cycles)
{
  // Step 1: Old time disappears - only currently-ON discs flip off randomly
  RandomFlipOffCurrent();

  // Step 2: Dramatic flicker - all 20 discs ON then OFF, repeated N times
  for(uint8_t cycle = 0; cycle < middle_cycles; cycle++)
  {
    RandomFlipDisc(1);
    RandomFlipDisc(0);
  }

  // Step 3: New time appears - only target-ON discs flip on randomly
  RandomFlipOnTarget();
}

// =============================================================================
// TIME READING
// =============================================================================

/*
 * GetRtcTime - read the current time from the RTC and split it into 4 digits.
 * Updates: hour_time, minute_time, digit[0..3].
 */
void GetRtcTime(void)
{
  RTC_RX8025T.read(tm);

  hour_time = tm.Hour;      // 0-23
  minute_time = tm.Minute;  // 0-59

  // Convert to 12-hour format if selected
  if(time_format == HR12)
  {
    if(hour_time > 12) hour_time -= 12;  // 13-23 -> 1-11
    if(hour_time == 0) hour_time = 12;   // midnight -> 12
  }

  // Split into individual digits, one per display row
  digit[0] = minute_time % 10;  // Units of minutes -> row 0 (bottom)
  digit[1] = minute_time / 10;  // Tens of minutes  -> row 1
  digit[2] = hour_time % 10;    // Units of hours   -> row 2
  digit[3] = hour_time / 10;    // Tens of hours    -> row 3 (top)
}

// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================

/*
 * DisplayRawTime - immediately display the current digit[] values on all rows.
 * No visual effect - each row is updated sequentially using FlipRow().
 */
void DisplayRawTime(void)
{
  rtcInterrupt = false;

  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    FlipRow(row, digit[row]);
  }
}

/*
 * DisplayTime - handle a per-minute RTC interrupt: read new time and display it.
 *
 *   1. Compute current_state[] from the OLD digit[] values (still on display)
 *   2. GetRtcTime() reads the NEW time into digit[]
 *   3. Compute target_state[] from the NEW digit[] values
 *   4. Choose the display method based on effect_interval
 */
void DisplayTime(void)
{
  rtcInterrupt = false;

  // Snapshot the display state BEFORE reading new time
  ComputeDisplayState(current_state);

  // Read new time from RTC - updates digit[]
  GetRtcTime();

  // Snapshot the target display state for the NEW time
  ComputeDisplayState(target_state);

  if(effect_interval == E00M)
  {
    DisplayRawTime();
  }
  else if(effect_interval == E01M)
  {
    // Effect every minute. Full hour gets a more dramatic effect (2 cycles).
    RandomTimeEffect(minute_time == 0 ? EFFECT_HOUR : EFFECT_MINUTE);
  }
  else if(effect_interval == E60M)
  {
    if(minute_time == 0) RandomTimeEffect(EFFECT_HOUR);
    else DisplayRawTime();
  }
}

// =============================================================================
// PARAMETER SETTINGS (double click BT1)
// =============================================================================
//
// Presents 3 sequential configuration screens.  On each screen:
//   - Rows 2-3 show a unique indicator pattern (identifies which setting)
//   - Row 1 is always OFF (visual separator)
//   - Row 0 shows the current value encoded in Morse code
//   - BT2 short press cycles through available values
//   - BT1 long press confirms and moves to the next screen
//   - BT3 long press aborts everything and switches the clock off
//
// Screen 1 - Time format:   1 = 12h, 2 = 24h
// Screen 2 - Display mode:  1 = Morse, 2 = Binary, 3 = Stibitz, 4 = Fibonacci, 5 = POSTNET
// Screen 3 - Effect:        1 = every minute, 2 = every hour, 3 = off

/*
 * ShowParamScreen - clear the display, then draw the indicator pattern on rows 2-3.
 */
void ShowParamScreen(uint8_t screen)
{
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    FlipRow(row, CAD);
  }

  for(uint8_t row_offset = 0; row_offset < 2; row_offset++)
  {
    for(uint8_t col = 0; col < DISCS_PER_ROW; col++)
    {
      FlipDisc((2 + row_offset) * DISCS_PER_ROW + col,
               pgm_read_byte(&paramScreenPattern[screen][row_offset][col]));
    }
  }
}

/*
 * ShowMorseValueRow0 - draw a value 0-9 on row 0 in Morse, regardless of display_mode.
 *
 * The parameter screens always use Morse so the numbers stay readable while the user
 * is in the middle of choosing a different encoding.
 */
static void ShowMorseValueRow0(uint8_t value)
{
  for(uint8_t col = 0; col < DISCS_PER_ROW; col++)
  {
    FlipDisc(col, pgm_read_byte(&morseDigit[value][4 - col]));
  }
}

void ParamSettings(void)
{
  ClearButtonFlags();

  // ---- SCREEN 1: Time format (12/24h) ----
  ShowParamScreen(SCREEN_TIME_FORMAT);

  uint8_t format_value = (time_format == HR12) ? 1 : 2;
  ShowMorseValueRow0(format_value);

  do
  {
    WatchButtons();

    /* BT3 long press aborts. Nothing is written to the EEPROM and the working copies
     * are reloaded from it - otherwise a half-entered value would stay active in RAM
     * while the EEPROM still held the old one. */
    if(shutdownRequestStatus)
    {
      LoadSettingsFromEeprom();
      paramSettingsActive = false;
      return;
    }

    if(shortPressButton2)
    {
      shortPressButton2 = false;
      time_format = (time_format == HR12) ? HR24 : HR12;
      format_value = (time_format == HR12) ? 1 : 2;
      ShowMorseValueRow0(format_value);
    }

    if(longPressButton1)
    {
      longPressButton1 = false;
      break;  // Confirmed - move to next screen
    }
  } while(true);

  ClearButtonFlags();

  // ---- SCREEN 2: Display mode ----
  ShowParamScreen(SCREEN_DISPLAY_MODE);

  // display_mode is 0-based internally, so add 1 for the Morse digit (1-5).
  ShowMorseValueRow0(display_mode + 1);

  do
  {
    WatchButtons();

    if(shutdownRequestStatus)
    {
      LoadSettingsFromEeprom();
      paramSettingsActive = false;
      return;
    }

    if(shortPressButton2)
    {
      shortPressButton2 = false;
      display_mode++;
      if(display_mode > MODE_POSTNET) display_mode = MODE_MORSE;  // Wrap around
      ShowMorseValueRow0(display_mode + 1);
    }

    if(longPressButton1)
    {
      longPressButton1 = false;
      break;
    }
  } while(true);

  ClearButtonFlags();

  // ---- SCREEN 3: Effect interval ----
  ShowParamScreen(SCREEN_EFFECT_INTERVAL);

  // effect_interval values (E01M=1, E60M=2, E00M=3) are directly valid Morse digits.
  ShowMorseValueRow0(effect_interval);

  do
  {
    WatchButtons();

    if(shutdownRequestStatus)
    {
      LoadSettingsFromEeprom();
      paramSettingsActive = false;
      return;
    }

    if(shortPressButton2)
    {
      shortPressButton2 = false;
      effect_interval++;
      if(effect_interval > E00M) effect_interval = E01M;  // Wrap: 3 -> 1
      ShowMorseValueRow0(effect_interval);
    }

    if(longPressButton1)
    {
      longPressButton1 = false;
      break;
    }
  } while(true);

  // Persist all three settings to EEPROM (only writes if the value changed)
  EEPROM.update(ee_time_format_address, time_format);
  EEPROM.update(ee_display_mode_address, display_mode);
  EEPROM.update(ee_effect_interval_address, effect_interval);

  ClearButtonFlags();
  paramSettingsActive = false;

  // Transition: blank display, then show current time
  RandomFlipDisc(0);

  GetRtcTime();
  DisplayRawTime();
  rtcInterrupt = false;
}

// =============================================================================
// TIME SETTINGS (long press BT1)
// =============================================================================
//
// The user sets 4 digits one by one, from the top row (tens of hours) down.
// During time settings the display is forced to Morse for clarity; the original
// display_mode is saved and restored afterwards - including on the abort path.
//
// Controls:
//   - BT2 short press: increment the digit (wraps, respects limits)
//   - BT1 long press: confirm digit, move to the next row below
//   - BT3 long press: abort without writing the RTC, then switch the clock off

void TimeSettings(void)
{
  ClearButtonFlags();

  // Force Morse code display during settings for consistent readability
  uint8_t saved_display_mode = display_mode;
  display_mode = MODE_MORSE;

  // Read current time from RTC to use as initial values
  RTC_RX8025T.read(tm);
  hour_time = tm.Hour;
  minute_time = tm.Minute;

  if(time_format == HR12)
  {
    if(hour_time > 12) hour_time -= 12;
    if(hour_time == 0) hour_time = 12;
  }

  digit[0] = minute_time % 10;
  digit[1] = minute_time / 10;
  digit[2] = hour_time % 10;
  digit[3] = hour_time / 10;

  // Entry animation: blank -> flash -> blank
  RandomFlipDisc(0);
  RandomFlipDisc(1);
  RandomFlipDisc(0);

  // settingsLevel 0->3 maps to digitIndex 3->0 (top row first, bottom last)
  uint8_t settingsLevel = 0;
  uint8_t digitIndex = 3 - settingsLevel;
  uint8_t prevDigitIndex = 255;              // Sentinel: no previous active row yet
  uint8_t currentValue = digit[digitIndex];
  bool needsRedraw = true;

  do
  {
    WatchButtons();

    /* BT3 long press aborts. The RTC is NOT written, so a half-entered time can never
     * reach the clock, and display_mode is restored - forgetting that would leave the
     * clock stuck in Morse. digit[] needs no cleanup: the shutdown path and the next
     * StartClock() both rebuild it via GetRtcTime(). */
    if(shutdownRequestStatus)
    {
      display_mode = saved_display_mode;
      LoadSettingsFromEeprom();
      timeSettingsActive = false;
      return;
    }

    // ---- BT2 short press: increment digit value ----
    if(shortPressButton2)
    {
      shortPressButton2 = false;
      currentValue++;
      if(currentValue > 9) currentValue = 0;  // Wrap 9 -> 0

      // Enforce digit limits based on time format and already-set digits
      if(settingsLevel == 0)  // Tens of hours
      {
        if(time_format == HR12 && currentValue > 1) currentValue = 0;
        if(time_format == HR24 && currentValue > 2) currentValue = 0;
      }

      if(settingsLevel == 1)  // Units of hours
      {
        if(time_format == HR12 && digit[3] == 0 && currentValue == 0) currentValue = 1;
        if(time_format == HR12 && digit[3] == 1 && currentValue > 2) currentValue = 0;
        if(time_format == HR24 && digit[3] == 2 && currentValue > 3) currentValue = 0;
      }

      if(settingsLevel == 2 && currentValue > 5) currentValue = 0;  // Tens of minutes

      FlipRow(digitIndex, currentValue);
    }

    // ---- BT1 long press: confirm digit and advance ----
    if(longPressButton1)
    {
      longPressButton1 = false;

      digit[digitIndex] = currentValue;

      settingsLevel++;
      digitIndex = 3 - settingsLevel;

      if(settingsLevel <= 3) currentValue = digit[digitIndex];
      else currentValue = 0;

      // 12h mode does not allow hour 00 - minimum is 01
      if(settingsLevel == 1 && time_format == HR12 && digit[3] == 0 && currentValue == 0)
        currentValue = 1;

      needsRedraw = true;
    }

    // ---- Redraw display when the active row changes ----
    if(needsRedraw)
    {
      needsRedraw = false;

      if(prevDigitIndex == 255)
      {
        // First draw: set up all 4 rows. Inactive rows show all discs ON except
        // the middle one (col 2), which distinguishes them from the active row.
        for(uint8_t row = 0; row < NUM_ROWS; row++)
        {
          if(row != digitIndex)
          {
            FlipRow(row, CAD);
            FlipDisc(row * DISCS_PER_ROW + 0, 1);
            FlipDisc(row * DISCS_PER_ROW + 1, 1);
            // col 2 (middle) stays OFF
            FlipDisc(row * DISCS_PER_ROW + 3, 1);
            FlipDisc(row * DISCS_PER_ROW + 4, 1);
          }
        }
      }
      else
      {
        // Subsequent draws: only the row that just became inactive needs the pattern
        FlipRow(prevDigitIndex, CAD);
        FlipDisc(prevDigitIndex * DISCS_PER_ROW + 0, 1);
        FlipDisc(prevDigitIndex * DISCS_PER_ROW + 1, 1);
        FlipDisc(prevDigitIndex * DISCS_PER_ROW + 3, 1);
        FlipDisc(prevDigitIndex * DISCS_PER_ROW + 4, 1);
      }

      // Flash the active row: all ON briefly, then show the digit value
      if(settingsLevel <= 3)
      {
        FlipRow(digitIndex, SAD);
        FlipRow(digitIndex, currentValue);
      }

      prevDigitIndex = digitIndex;
    }
  } while(settingsLevel <= 3);

  // Reconstruct the full time from the 4 confirmed digits
  hour_time = digit[3] * 10 + digit[2];
  minute_time = digit[1] * 10 + digit[0];

  // Write the new time to the RTC (seconds reset to 0, date is irrelevant)
  setTime(hour_time, minute_time, 0, 1, 1, 1);
  RTC_RX8025T.set(now());

  // Restore the original display mode (was forced to Morse during settings)
  display_mode = saved_display_mode;

  timeSettingsActive = false;
  ClearButtonFlags();

  // Exit animation: flash all ON then OFF
  RandomFlipDisc(1);
  RandomFlipDisc(0);

  GetRtcTime();
  DisplayRawTime();
  rtcInterrupt = false;
}

// =============================================================================
// MAINTENANCE
// =============================================================================

/*
 * HardFlipDiscCleaning - exercise all discs with extended coil pulses.
 *
 * Flips all 20 discs OFF then ON with longer coil energize times (1.4-1.5 ms vs
 * the normal 1.2 ms) and 100 ms settling delays, to free sticky discs.
 *
 * Ends with every disc ON, which is exactly the panel state STANDBY expects.
 */
void HardFlipDiscCleaning(void)
{
  // Phase 1: Reset all discs (OFF) with extended pulse
  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    digitalWrite(EN_PIN, LOW);
    SPI.transfer(pgm_read_byte(&resetDisc[i][0]));
    SPI.transfer(pgm_read_byte(&resetDisc[i][1]));
    SPI.transfer(pgm_read_byte(&resetDisc[i][2]));
    digitalWrite(EN_PIN, HIGH);

    delayMicroseconds(1400);
    ClearOutputs();
    WaitMs(100);
  }

  // Phase 2: Set all discs (ON) with extended pulse
  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    digitalWrite(EN_PIN, LOW);
    SPI.transfer(pgm_read_byte(&setDisc[i][0]));
    SPI.transfer(pgm_read_byte(&setDisc[i][1]));
    SPI.transfer(pgm_read_byte(&setDisc[i][2]));
    digitalWrite(EN_PIN, HIGH);

    delayMicroseconds(1500);
    ClearOutputs();
    WaitMs(100);
  }
}
