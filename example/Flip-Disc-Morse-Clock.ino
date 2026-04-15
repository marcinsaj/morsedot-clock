/* Flip-Disc Morse Clock – Programming and Firmware

MiniCore Configuration
Before burning the bootloader, install MiniCore using the following Boards Manager URL:
https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json

In Arduino IDE → Tools, configure the settings as follows:
Board: ATmega328P / ATmega328PA
Clock: External 12 MHz
Bootloader: Yes (UART0)
BOD: 2.7V
EEPROM: Retained
Compiler LTO: Enabled
Baud Rate: Default
Variant: 328P / 328PA

⚠ Make sure the clock is set to External 12 MHz. ⚠

Programming Procedure:
1. Select the correct Programmer: AVR ISP MKII (recommended).
2. Click Burn Bootloader to set the fuse bits and configure the microcontroller.
3. After completion, select Upload Using Programmer to flash the firmware. */

#include <RTC_RX8025T.h>  // https://github.com/marcinsaj/RTC_RX8025T
#include <TimeLib.h>      // https://github.com/PaulStoffregen/Time
#include <Wire.h>         // https://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <OneButton.h>    // https://github.com/mathertel/OneButton
#include <EEPROM.h>       // (included with Arduino IDE)
#include <SPI.h>          // (included with Arduino IDE)
#include <avr/pgmspace.h> // (included with Arduino IDE)
#include <avr/wdt.h>      // (included with Arduino IDE)


#define DEBUG 0

#if DEBUG
#include <SoftwareSerial.h>

const int RX_PIN = PIN_PD0;
const int TX_PIN = PIN_PD1;

SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif

// ============================================================================
// FLIP-DISC DISPLAY DATA (PROGMEM)
// ============================================================================

/* 20-disc Morse display (4 rows x 5 discs).
 * setDisc[n][3] - SPI bytes to turn ON disc n (yellow/dot side).
 * resetDisc[n][3] - SPI bytes to turn OFF disc n (black/dash side).
 *
 * Physical layout (top to bottom):
 * Row 3 (discs 15-19): tens of hours   ← physical top
 * Row 2 (discs 10-14): units of hours
 * Row 1 (discs 5-9):   tens of minutes
 * Row 0 (discs 0-4):   units of minutes ← physical bottom
 *
 * Within each row, disc order is right-to-left:
 * e.g. Row 3: [d19][d18][d17][d16][d15] (d19 = leftmost) */
static const uint8_t setDisc[20][3] PROGMEM =
{
  { 0b00010000, 0b00000000, 0b00100000 },
  { 0b00010000, 0b00000000, 0b00001000 },
  { 0b00010000, 0b00001000, 0b00000000 },
  { 0b00010000, 0b00000010, 0b00000000 },
  { 0b10010000, 0b00000000, 0b00000000 },

  { 0b00000010, 0b00000000, 0b00100000 },
  { 0b00000010, 0b00000000, 0b00001000 },
  { 0b00000010, 0b00001000, 0b00000000 },
  { 0b00000010, 0b00000010, 0b00000000 },
  { 0b10000010, 0b00000000, 0b00000000 },

  { 0b00000000, 0b00000000, 0b00100010 },
  { 0b00000000, 0b00000000, 0b00001010 },
  { 0b00000000, 0b00001000, 0b00000010 },
  { 0b00000000, 0b00000010, 0b00000010 },
  { 0b10000000, 0b00000000, 0b00000010 },

  { 0b00000000, 0b00010000, 0b00100000 },
  { 0b00000000, 0b00010000, 0b00001000 },
  { 0b00000000, 0b00011000, 0b00000000 },
  { 0b00000000, 0b00010010, 0b00000000 },
  { 0b10000000, 0b00010000, 0b00000000 }
};

static const uint8_t resetDisc[20][3] PROGMEM =
{
  { 0b00001000, 0b00000000, 0b00000001 },
  { 0b00001000, 0b00000001, 0b00000000 },
  { 0b00001001, 0b00000000, 0b00000000 },
  { 0b00101000, 0b00000000, 0b00000000 },
  { 0b01001000, 0b00000000, 0b00000000 },

  { 0b00000100, 0b00000000, 0b00000001 },
  { 0b00000100, 0b00000001, 0b00000000 },
  { 0b00000101, 0b00000000, 0b00000000 },
  { 0b00100100, 0b00000000, 0b00000000 },
  { 0b01000100, 0b00000000, 0b00000000 },

  { 0b00000000, 0b00000000, 0b01000001 },
  { 0b00000000, 0b00000001, 0b01000000 },
  { 0b00000001, 0b00000000, 0b01000000 },
  { 0b00100000, 0b00000000, 0b01000000 },
  { 0b01000000, 0b00000000, 0b01000000 },

  { 0b00000000, 0b00000100, 0b00000001 },
  { 0b00000000, 0b00000101, 0b00000000 },
  { 0b00000001, 0b00000100, 0b00000000 },
  { 0b00100000, 0b00000100, 0b00000000 },
  { 0b01000000, 0b00000100, 0b00000000 }
};

/* Standard ITU Morse code for digits 0-9.
 * 1 = dot (disc ON / yellow), 0 = dash (disc OFF / black).
 *
 * Array stored in physical disc order: index [0] = rightmost disc,
 * index [4] = leftmost disc. Morse reads left-to-right: [4][3][2][1][0].
 *
 * | Digit | Morse | Array [0]..[4]   |
 * |-------|-------|------------------|
 * |   0   | ----- | 0, 0, 0, 0, 0   |
 * |   1   | .---- | 0, 0, 0, 0, 1   |
 * |   2   | ..--- | 0, 0, 0, 1, 1   |
 * |   3   | ...-- | 0, 0, 1, 1, 1   |
 * |   4   | ....- | 0, 1, 1, 1, 1   |
 * |   5   | ..... | 1, 1, 1, 1, 1   |
 * |   6   | -.... | 1, 1, 1, 1, 0   |
 * |   7   | --... | 1, 1, 1, 0, 0   |
 * |   8   | ---.. | 1, 1, 0, 0, 0   |
 * |   9   | ----. | 1, 0, 0, 0, 0   | */
static const uint8_t morseDigit[10][5] PROGMEM =
{
  {0, 0, 0, 0, 0},  // 0: -----
  {0, 0, 0, 0, 1},  // 1: .----
  {0, 0, 0, 1, 1},  // 2: ..---
  {0, 0, 1, 1, 1},  // 3: ...--
  {0, 1, 1, 1, 1},  // 4: ....-
  {1, 1, 1, 1, 1},  // 5: .....
  {1, 1, 1, 1, 0},  // 6: -....
  {1, 1, 1, 0, 0},  // 7: --...
  {1, 1, 0, 0, 0},  // 8: ---..
  {1, 0, 0, 0, 0},  // 9: ----.
};

// ============================================================================
// CONSTANTS
// ============================================================================

// Delay between flipping consecutive discs
// Best visual effect in range 0-50ms, maximum value 255
static const uint8_t flip_disc_delay = 50;

// Delay between disc operations in sweep effect (ms)
static const uint16_t sweep_delay = 50;

// Display dimensions
static const uint8_t NUM_DISCS = 20;
static const uint8_t DISCS_PER_ROW = 5;
static const uint8_t NUM_ROWS = 4;

// Row mapping: index 0 = bottom row, index 3 = top row.
// ROW_MAP[0] = row 0 (discs 0-4,   units of minutes, physical bottom)
// ROW_MAP[1] = row 1 (discs 5-9,   tens of minutes)
// ROW_MAP[2] = row 2 (discs 10-14, units of hours)
// ROW_MAP[3] = row 3 (discs 15-19, tens of hours, physical top)
static const uint8_t ROW_MAP[4] = {0, 1, 2, 3};

// Codenames for the flip-disc display
static const uint8_t CAD = 10; // Clear all discs in a row (all OFF)
static const uint8_t SAD = 11; // Set all discs in a row (all ON)

// Voltage monitoring thresholds
const int LOWER_THRESHOLD = 200;    // 180 for 10V lower threshold (in ADC units)
const int UPPER_THRESHOLD = 250;   // upper threshold (in ADC units)
const int REQUIRED_OK_SAMPLE = 5;  // number of consecutive in-range readings required
const int SAMPLE_DELAY_MS = 10;    // delay between measurements

// Error / state codes
const int ERROR_NONE = 0;
const int ERROR_VBUS_LOW = 1;   // Vbus <= LOWER_THRESHOLD
const int ERROR_VBUS_HIGH = 2;  // Vbus >= UPPER_THRESHOLD
const int ERROR_VFD_NO_DISPLAY = 3;
const int ERROR_VFD_OVER_CURRENT = 4;

// Aliases for individual option settings
static const uint8_t HR12 = 12;  // Display time in 12 hour format
static const uint8_t HR24 = 24;  // Display time in 24 hour format
static const uint8_t E01M = 1;   // Sweep Effect every 1 minute
static const uint8_t E60M = 2;   // Sweep Effect every full hour
static const uint8_t E00M = 3;   // Sweep Effect turn off
 
// Eeprom addresses where settings are stored
static const uint16_t ee_time_format_address = 0;       // Time format 12/24 hour
static const uint16_t ee_effect_interval_address = 2;   // Sweep Effect interval

// ============================================================================
// PIN DECLARATIONS
// ============================================================================

// Analog voltage monitor pins
const int ADC_VBUS_PIN = A7;
const int ADC_VOUT_PIN = A6;

// SPI pins
const int DIN_PIN = PIN_PB3;  // SPI - MOSI
const int EN_PIN  = PIN_PB2;  // SPI - SS
const int CLK_PIN = PIN_PB5;  // SPI - SCK
const int CLR_PIN = PIN_PB1;  // SPI - MISO

// Buttons
const int BT1_PIN = PIN_PD4;  // Right button
const int BT2_PIN = PIN_PC3;  // Left button

// RTC
const int INT_RTC = PIN_PD2;  // RTC interrupt input

// Fault input
const int INT_FLT = PIN_PD3;  // Diagnostic input

// LED - Used to indicate potential hardware issues during the boot process
const int LVCC_PIN = PIN_PD6;   // VCC
const int LVFD_PIN = PIN_PD5;   // VFD
const int LDIAG_PIN = PIN_PD7;  // DIAG

// Power switches
const int ENA_PIN = PIN_PC0;  // High side power switch TPS1H100B
const int ENB_PIN = PIN_PB0;  // Last stand power switch

// Discharge
const int DIS_PIN = PIN_PC1;

// Power ON
const int PWR_PIN = PIN_PC2;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Time structure and variables
tmElements_t tm;
uint8_t currentTimeDigit[4] = {0, 0, 0, 0};
uint16_t hour_time = 0;
uint16_t minute_time = 0;

// OneButton instances
OneButton button1(BT1_PIN, true, true);
OneButton button2(BT2_PIN, true, true);

// Button press status flags
bool shortPressButton1Status = false;
bool shortPressButton2Status = false;
bool longPressButton1Status = false;
bool longPressButton2Status = false;
bool doubleClickButton1Status = false;

// Settings and status flags
bool powerClockStatus = false;
bool timeSettingsStatus = false;
bool effectSettingsStatus = false;

// Settings values (stored in eeprom, read during setup)
uint8_t time_format = 12;     // HR12 - Time format
uint8_t effect_interval = 1;  // Sweep Effect every 1 minute

// Interrupt flags
volatile bool interruptFltStatus = false;
volatile bool interruptRtcStatus = false;

// Boolean state arrays for all 20 discs (true = ON, false = OFF).
// current_state[] - current display state (old time, before update)
// target_state[]  - target display state (new time, after update)
// Populated by ComputeDisplayState(), used by the sweep effect.
bool current_state[20];
bool target_state[20];

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

void fltInterruptISR(void)
{
  interruptFltStatus = true;
  // Feature planned for future implementation
}

void rtcInterruptISR(void)
{
  interruptRtcStatus = true;
}

// ============================================================================
// BUTTON CALLBACKS
// ============================================================================

void ShortPressButton1(void)
{
  shortPressButton1Status = true;
}

void ShortPressButton2(void)
{
  shortPressButton2Status = true;
}

void LongPressButton1(void)
{
  longPressButton1Status = true;
}

void LongPressButton2(void)
{
  longPressButton2Status = true;
}

void DoubleClickButton1(void)
{
  doubleClickButton1Status = true;
}

void ClearPressButtonFlags(void)
{
  shortPressButton1Status = false;
  shortPressButton2Status = false;
  longPressButton1Status = false;
  longPressButton2Status = false;
  doubleClickButton1Status = false;
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  wdt_disable();

  #if DEBUG
  mySerial.begin(9600);  // baud rate
  #endif

  // Set pin modes
  pinMode(PWR_PIN, OUTPUT);
  pinMode(DIS_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLR_PIN, OUTPUT);
  pinMode(LVCC_PIN, OUTPUT);
  pinMode(LVFD_PIN, OUTPUT);
  pinMode(LDIAG_PIN, OUTPUT);
  pinMode(ADC_VBUS_PIN, INPUT);
  pinMode(ADC_VOUT_PIN, INPUT);
  pinMode(INT_RTC, INPUT_PULLUP);
  pinMode(INT_FLT, INPUT_PULLUP);

  // Set initial pin states
  digitalWrite(PWR_PIN, LOW);
  digitalWrite(DIS_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIN_PIN, LOW);
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(CLR_PIN, LOW);
  digitalWrite(LVCC_PIN, LOW);
  digitalWrite(LVFD_PIN, LOW);
  digitalWrite(LDIAG_PIN, LOW);

  // Link the button functions
  button1.attachClick(ShortPressButton1);
  button1.attachLongPressStart(LongPressButton1);
  button1.attachDoubleClick(DoubleClickButton1);
  button1.setDebounceMs(50);
  button1.setPressMs(800);

  button2.attachClick(ShortPressButton2);
  button2.attachLongPressStart(LongPressButton2);
  button2.setDebounceMs(50);
  button2.setPressMs(2000);

  // Active 2-second wait — poll buttons to detect long press
  {
    unsigned long startWait = millis();
    while(millis() - startWait < 2000)
    {
      button1.tick();
    }
  }

  powerClockStatus = true;
  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(LVCC_PIN, HIGH);

  delay(100);
  WaitForUsbPowerStable();

  // SPI initialization
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  delay(100);

  ClearOutputs();

  digitalWrite(ENA_PIN, HIGH);
  delay(500);

  digitalWrite(ENB_PIN, HIGH);
  delay(500);

  // If button1 was long-pressed during the 2s wait — enter test mode
  if(longPressButton1Status == true)
  {
    ClearPressButtonFlags();
    HardFlipDiscCleaning();
    PowerOffClock();
  }

  ClearPressButtonFlags();

  attachInterrupt(digitalPinToInterrupt(INT_FLT), fltInterruptISR, FALLING);

  // RTC RX8025T initialization
  RTC_RX8025T.init();

  // Time update interrupt initialization. Interrupt generated by RTC (INT output):
  // "INT_SECOND" - every second,
  // "INT_MINUTE" - every minute.
  RTC_RX8025T.initTUI(INT_MINUTE);

  // "INT_ON" - turn ON interrupt generated by RTC (INT output),
  // "INT_OFF" - turn OFF interrupt.
  RTC_RX8025T.statusTUI(INT_ON);

  // Assign an interrupt handler to the RTC output,
  // an interrupt will be generated every minute to display the time
  attachInterrupt(digitalPinToInterrupt(INT_RTC), rtcInterruptISR, FALLING);

  // Read setting options from eeprom memory
  time_format = EEPROM.read(ee_time_format_address);
  effect_interval = EEPROM.read(ee_effect_interval_address);

  // If the read values are incorrect, set the default values
  if(time_format != HR12 && time_format != HR24) time_format = HR12;
  if(effect_interval != E00M && effect_interval != E01M && effect_interval != E60M) effect_interval = E01M;

  // Startup animation: all OFF then all ON
  ClearPressButtonFlags();
  ResetAll();
  SetAll();

  // After startup animation, display time without effect (DisplayRawTime instead of DisplayTime).
  // This ensures currentTimeDigit[] matches the physical display state.
  // On the next RTC interrupt, DisplayTime() will compute correct current_state.
  GetRtcTime();
  DisplayRawTime();

  timeSettingsStatus = false;
  effectSettingsStatus = false;

  wdt_enable(WDTO_8S);
}

// ============================================================================
// LOOP
// ============================================================================

void loop()
{
  wdt_reset();
  WatchButtons();

  // Handle the per-minute RTC interrupt.
  // Every minute the RTC generates an interrupt that sets the interruptRtcStatus flag.
  //
  // Two visual effect modes:
  // - E01M (every minute): sweep effect on every interrupt
  // - E60M (every hour): sweep effect only on the full hour (minutes == 0),
  //   otherwise DisplayRawTime.
  if(interruptRtcStatus == true) DisplayTime();
  if(timeSettingsStatus == true) TimeSettings();
  if(effectSettingsStatus == true) EffectSettings();
}

// ============================================================================
// GET RTC TIME
// ============================================================================

void GetRtcTime(void)
{
  RTC_RX8025T.read(tm);

  hour_time = tm.Hour;
  minute_time = tm.Minute;

  // 12-Hour conversion
  if(time_format == HR12)
  {
    if(hour_time > 12) hour_time = hour_time - 12;
    if(hour_time == 0) hour_time = 12;
  }

  // Split time into 4 individual digits (indexed bottom-to-top)
  // currentTimeDigit[0] = units of minutes (row 0, bottom) (e.g. 5 from "14:25")
  // currentTimeDigit[1] = tens of minutes  (row 1)         (e.g. 2 from "14:25")
  // currentTimeDigit[2] = units of hours   (row 2)         (e.g. 4 from "14:25")
  // currentTimeDigit[3] = tens of hours    (row 3, top)    (e.g. 1 from "14:25")
  currentTimeDigit[3] = hour_time / 10;
  currentTimeDigit[2] = hour_time % 10;
  currentTimeDigit[1] = minute_time / 10;
  currentTimeDigit[0] = minute_time % 10;
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

/*
 * FlipRow() - Display a Morse digit pattern on one row of 5 discs.
 *
 * row:   0-3 (row on the display)
 * digit: 0-9 (Morse pattern), CAD=10 (all OFF), SAD=11 (all ON)
 */
void FlipRow(uint8_t row, uint8_t digit)
{
  if(row > 3) return;
  if(digit > 11) return;

  // Iterate from highest disc in the row to lowest (physically left-to-right)
  for(uint8_t i = 0; i < DISCS_PER_ROW; i++)
  {
    uint8_t col = DISCS_PER_ROW - 1 - i;
    uint8_t disc_number = row * DISCS_PER_ROW + col;
    bool disc_status;

    if(digit == CAD)       disc_status = 0;
    else if(digit == SAD)  disc_status = 1;
    else                   disc_status = pgm_read_byte(&morseDigit[digit][col]);

    FlipDisc(disc_number, disc_status);
  }
}

// ============================================================================
// DISPLAY RAW TIME
// ============================================================================

void DisplayRawTime(void)
{
  interruptRtcStatus = false;

  // Display bottom-to-top: units min (row 0) → tens hr (row 3)
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    FlipRow(row, currentTimeDigit[row]);
  }
}

// ============================================================================
// DISPLAY TIME
// ============================================================================

void DisplayTime(void)
{
  interruptRtcStatus = false;

  // Compute current display state BEFORE reading new time
  // (currentTimeDigit[] still contains the OLD time)
  ComputeDisplayState(current_state);

  GetRtcTime();

  // Compute target display state for the NEW time
  // (currentTimeDigit[] has been updated by GetRtcTime)
  ComputeDisplayState(target_state);

  if(effect_interval == E00M)
  {
    DisplayRawTime();
  }
  else if(effect_interval == E01M)
  {
    SweepEffect();
  }
  else if(effect_interval == E60M)
  {
    if(minute_time == 0) SweepEffect();
    else DisplayRawTime();
  }
}

// ============================================================================
// SWEEP EFFECT
// ============================================================================

/*
 * ComputeDisplayState() - Computes the state of all 20 discs based on currentTimeDigit[].
 *
 * The state_array parameter is a pointer to a bool[20] array where the result is stored.
 * Called twice in DisplayTime():
 * - BEFORE GetRtcTime() with current_state[] -> OLD time state
 * - AFTER GetRtcTime() with target_state[]    -> NEW time state
 */
void ComputeDisplayState(bool* state_array)
{
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    uint8_t digit = currentTimeDigit[row];

    for(uint8_t disc = 0; disc < DISCS_PER_ROW; disc++)
    {
      state_array[row * DISCS_PER_ROW + disc] = pgm_read_byte(&morseDigit[digit][disc]);
    }
  }
}

/*
 * SweepEffect() - Visual sweep effect for time changes.
 *
 * Phase 1 - Sweep ON:
 *   All 20 discs light up sequentially from left to right, top to bottom.
 *   (disc 0, 1, 2, ... 19 — each turned ON with sweep_delay between)
 *
 * Phase 2 - Sweep OFF:
 *   Discs that should be OFF (dash) for the new time are turned off
 *   sequentially from left to right, top to bottom.
 *   Discs that should be ON (dot) remain lit.
 *
 * target_state[] must be computed BEFORE calling this function (in DisplayTime()).
 */
void SweepEffect(void)
{
  // Phase 1: Sweep ON — all discs light up left-to-right, bottom-to-top
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    for(uint8_t i = 0; i < DISCS_PER_ROW; i++)
    {
      uint8_t disc = row * DISCS_PER_ROW + (DISCS_PER_ROW - 1 - i);
      FlipDisc(disc, 1);
      delay(sweep_delay);
    }
  }

  // Phase 2: Sweep OFF — unnecessary discs turn off left-to-right, bottom-to-top
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    for(uint8_t i = 0; i < DISCS_PER_ROW; i++)
    {
      uint8_t disc = row * DISCS_PER_ROW + (DISCS_PER_ROW - 1 - i);
      if(target_state[disc] == 0)
      {
        FlipDisc(disc, 0);
        delay(sweep_delay);
      }
    }
  }
}

// ============================================================================
// EFFECT SETTINGS
// ============================================================================

void EffectSettings(void)
{
  ClearPressButtonFlags();

  // Effect interval values match display directly:
  //   E01M=1 (every minute), E60M=2 (every hour), E00M=3 (off)

  // Clear entire display starting from bottom row
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    FlipRow(row, CAD);
  }

  // Sweep bottom row: light up, then turn off, then show value
  FlipRow(0, SAD);
  FlipRow(0, CAD);

  // Indicator: discs 10, 12, 16, 18 ON to show "effect" mode
  FlipDisc(11, 1);
  FlipDisc(13, 1);
  FlipDisc(16, 1);
  FlipDisc(18, 1);

  FlipRow(0, effect_interval);

  do
  {
    WatchButtons();

    if(shortPressButton2Status == true)
    {
      shortPressButton2Status = false;
      effect_interval++;
      if(effect_interval > E00M) effect_interval = E01M;

      // Direct update on bottom row
      FlipRow(0, effect_interval);
    }

    if(longPressButton1Status == true)
    {
      longPressButton1Status = false;
      break;
    }
  } while(true);

  EEPROM.update(ee_effect_interval_address, effect_interval);

  ClearPressButtonFlags();
  effectSettingsStatus = false;

  ResetAll();

  // After blanking, display time without effect — sync currentTimeDigit with display
  GetRtcTime();
  DisplayRawTime();
  interruptRtcStatus = false;
}

// ============================================================================
// TIME SETTINGS - set time format, then 4 digits one by one (digit 0 -> 1 -> 2 -> 3)
// ============================================================================

void TimeSettings(void)
{
  ClearPressButtonFlags();

  uint8_t timeSettingsLevel = 0;
  uint8_t currentValue = 0;

  // Read current time from RTC
  RTC_RX8025T.read(tm);
  hour_time = tm.Hour;
  minute_time = tm.Minute;

  // 12-Hour conversion for display
  if(time_format == HR12)
  {
    if(hour_time > 12) hour_time = hour_time - 12;
    if(hour_time == 0) hour_time = 12;
  }

  currentTimeDigit[3] = hour_time / 10;
  currentTimeDigit[2] = hour_time % 10;
  currentTimeDigit[1] = minute_time / 10;
  currentTimeDigit[0] = minute_time % 10;

  ResetAll();

  // Time format selection: sweep bottom rows (row 0 then row 1), then show 12/24
  FlipRow(0, SAD);
  FlipRow(1, SAD);

  // Indicator: discs 11 and 17 ON to show "time format" mode
  FlipDisc(12, 1);
  FlipDisc(17, 1);

  FlipRow(1, time_format / 10);
  FlipRow(0, time_format % 10);

  do
  {
    WatchButtons();

    if(shortPressButton2Status == true)
    {
      shortPressButton2Status = false;
      if(time_format == HR12) time_format = HR24;
      else time_format = HR12;

      // Direct update — no CAD, just overwrite
      FlipRow(1, time_format / 10);
      FlipRow(0, time_format % 10);
    }

    if(longPressButton1Status == true)
    {
      longPressButton1Status = false;
      break;
    }
  } while(true);

  ClearPressButtonFlags();

  // Transition animation between format selection and digit setting
  SetAll();
  ResetAll();

  // Settings order: tens hours (row 3) → units hours (row 2) → tens min (row 1) → units min (row 0)
  // timeSettingsLevel 0→3 maps to digitIndex 3→0 via: digitIndex = 3 - timeSettingsLevel
  uint8_t digitIndex = 3 - timeSettingsLevel;  // = 3 (tens hours)
  currentValue = currentTimeDigit[digitIndex];
  bool updateDisplayStatus = true;

  do
  {
    WatchButtons();

    // Short press button 2: increment displayed value (0-9, wraps around)
    if(shortPressButton2Status == true)
    {
      shortPressButton2Status = false;
      currentValue++;

      if(currentValue > 9) currentValue = 0;

      // Tens of hours limits (digitIndex == 3)
      if(timeSettingsLevel == 0)
      {
        if(time_format == HR12 && currentValue > 1) currentValue = 0;
        if(time_format == HR24 && currentValue > 2) currentValue = 0;
      }

      // Units of hours limits (digitIndex == 2)
      if(timeSettingsLevel == 1)
      {
        if(time_format == HR12 && currentTimeDigit[3] == 0 && currentValue == 0) currentValue = 1;
        if(time_format == HR12 && currentTimeDigit[3] == 1 && currentValue > 2) currentValue = 0;
        if(time_format == HR24 && currentTimeDigit[3] == 2 && currentValue > 3) currentValue = 0;
      }

      // Tens of minutes limit (digitIndex == 1)
      if(timeSettingsLevel == 2 && currentValue > 5) currentValue = 0;

      // Direct update — just overwrite current row
      FlipRow(digitIndex, currentValue);
    }

    // Long press button 1: confirm value, move to next row
    if(longPressButton1Status == true)
    {
      longPressButton1Status = false;
      currentTimeDigit[digitIndex] = currentValue;
      timeSettingsLevel++;
      digitIndex = 3 - timeSettingsLevel;

      if(timeSettingsLevel <= 3) currentValue = currentTimeDigit[digitIndex];
      else currentValue = 0;

      // Hour 00 is not allowed in HR12 mode — minimum value is 01
      if(timeSettingsLevel == 1 && time_format == HR12 && currentTimeDigit[3] == 0 && currentValue == 0) currentValue = 1;

      updateDisplayStatus = true;
    }

    if(updateDisplayStatus == true)
    {
      updateDisplayStatus = false;

      // Clear other rows, sweep (SAD) active row, then show value
      for(uint8_t row = 0; row < NUM_ROWS; row++)
      {
        if(row != digitIndex)
        {
          FlipRow(row, CAD);
        }
      }
      if(timeSettingsLevel <= 3)
      {
        FlipRow(digitIndex, SAD);
        FlipRow(digitIndex, currentValue);
      }
    }
  } while(timeSettingsLevel <= 3);

  hour_time = (currentTimeDigit[3] * 10) + currentTimeDigit[2];
  minute_time = (currentTimeDigit[1] * 10) + currentTimeDigit[0];

  // setTime(hh, mm, ss, day, month, year)
  // The date is skipped and the seconds are set by default to 0
  // We are only interested in hours and minutes
  setTime(hour_time, minute_time, 0, 1, 1, 1);

  // Set the RTC from the system time
  RTC_RX8025T.set(now());

  EEPROM.update(ee_time_format_address, time_format);

  timeSettingsStatus = false;
  ClearPressButtonFlags();

  // Transition animation
  SetAll();
  ResetAll();

  // After animation (ON->OFF), display time without effect — sync currentTimeDigit with display
  GetRtcTime();
  DisplayRawTime();
  interruptRtcStatus = false;
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void WatchButtons(void)
{
  wdt_reset();
  button1.tick();
  button2.tick();

  if(longPressButton1Status == true && effectSettingsStatus == false) timeSettingsStatus = true;
  if(doubleClickButton1Status == true && timeSettingsStatus == false) effectSettingsStatus = true;
  if(longPressButton2Status == true) PowerOffClock();
}

// ============================================================================
// POWER
// ============================================================================

void PowerOffClock(void)
{
  digitalWrite(ENB_PIN, LOW);
  delay(100);
  digitalWrite(ENA_PIN, LOW);
  delay(100);
  digitalWrite(PWR_PIN, LOW);
  delay(2000);
}

// ============================================================================
// LOW-LEVEL DISC CONTROL
// ============================================================================

// Clear shift register and latch cleared data to outputs
void ClearOutputs(void)
{
  // CLR resets shift register contents
  digitalWrite(CLR_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(CLR_PIN, HIGH);

  // EN pulse latches cleared register to outputs
  digitalWrite(EN_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(EN_PIN, HIGH);

  // Send zeros through SPI and latch again
  digitalWrite(EN_PIN, LOW);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(EN_PIN, HIGH);
}

/*
 * FlipDisc() - Control a single disc via SPI.
 *
 * disc_number: 0-19 (0-4 = row 0, 5-9 = row 1, 10-14 = row 2, 15-19 = row 3)
 * disc_status: 1 = ON (yellow/dot), 0 = OFF (black/dash)
 */
void FlipDisc(uint8_t disc_number, bool disc_status)
{
  // Range validation: discs numbered 0-19, out of range -> ignore
  if(disc_number > 19) return;

  wdt_reset();

  // Start of SPI data transfer
  digitalWrite(EN_PIN, LOW);

  for(uint8_t byte_number = 0; byte_number < 3; byte_number++)
  {
    if(disc_status == 1) SPI.transfer(pgm_read_byte(&setDisc[disc_number][byte_number]));
    else                 SPI.transfer(pgm_read_byte(&resetDisc[disc_number][byte_number]));
  }

  // End of SPI data transfer
  digitalWrite(EN_PIN, HIGH);

  delayMicroseconds(1200);
  ClearOutputs();
  delay(20);
  delay(flip_disc_delay);
}

/*
 * SetAll() - Turn ON all 20 discs sequentially (physically left-to-right, bottom-to-top).
 */
void SetAll(void)
{
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    for(uint8_t i = 0; i < DISCS_PER_ROW; i++)
    {
      FlipDisc(row * DISCS_PER_ROW + (DISCS_PER_ROW - 1 - i), 1);
    }
  }
}

/*
 * ResetAll() - Turn OFF all 20 discs sequentially (physically left-to-right, bottom-to-top).
 */
void ResetAll(void)
{
  for(uint8_t row = 0; row < NUM_ROWS; row++)
  {
    for(uint8_t i = 0; i < DISCS_PER_ROW; i++)
    {
      FlipDisc(row * DISCS_PER_ROW + (DISCS_PER_ROW - 1 - i), 0);
    }
  }
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void WaitForUsbPowerStable(void)
{
  unsigned long previousSampleMillis = 0;
  unsigned long previousBlinkMillis  = 0;

  unsigned long blinkInterval = 500;
  unsigned long currentMillis;

  bool ledState = false;
  bool errorActive = false;

  int consecutiveOkCount = 0;   // counts consecutive in-range readings
  int sampleCounter = 0;        // counts samples in current window (max 10)

  while(true)
  {
    wdt_reset();
    currentMillis = millis();

    // =====================================================
    // 1. Measurement every SAMPLE_DELAY_MS
    // =====================================================
    if(currentMillis - previousSampleMillis >= SAMPLE_DELAY_MS)
    {
      previousSampleMillis = currentMillis;

      int measuredVbus = analogRead(ADC_VBUS_PIN);

#if DEBUG
      mySerial.print(F("VBUS: "));
      mySerial.println(measuredVbus);
#endif

      sampleCounter++;

      // Check individual measurement
      if(measuredVbus > LOWER_THRESHOLD &&
         measuredVbus < UPPER_THRESHOLD)
      {
        consecutiveOkCount++;   // another consecutive in-range reading

        // If we reached 5 consecutive in-range readings
        if(consecutiveOkCount >= REQUIRED_OK_SAMPLE)
        {
          digitalWrite(LVFD_PIN, HIGH);
          digitalWrite(LDIAG_PIN, LOW);
          break;   // exit immediately
        }
      }
      else
      {
        consecutiveOkCount = 0;   // streak broken
      }

      // =====================================================
      // 2. If 10 samples taken without 5 consecutive OK
      // =====================================================
      if(sampleCounter >= 10)
      {
        errorActive = true;   // start blinking only now

        // Determine error type from the last measurement
        if(measuredVbus <= LOWER_THRESHOLD)
        {
          blinkInterval = 1000;   // slow blink
        }
        else if(measuredVbus >= UPPER_THRESHOLD)
        {
          blinkInterval = 250;    // fast blink
        }

        // Reset for next 10-sample window
        sampleCounter = 0;
        consecutiveOkCount = 0;
      }
    }

    // =====================================================
    // 3. Non-blocking LED blinking
    // =====================================================
    if(errorActive)
    {
      if(currentMillis - previousBlinkMillis >= blinkInterval)
      {
        previousBlinkMillis = currentMillis;
        ledState = !ledState;

        digitalWrite(LVFD_PIN, ledState);
        digitalWrite(LDIAG_PIN, ledState);
      }
    }

    // =====================================================
    // 4. Button handling runs continuously
    // =====================================================
    WatchButtons();
  }
}

// ============================================================================
// MAINTENANCE
// ============================================================================

void HardFlipDiscCleaning(void)
{
  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    digitalWrite(EN_PIN, LOW);
    SPI.transfer(pgm_read_byte(&resetDisc[i][0]));
    SPI.transfer(pgm_read_byte(&resetDisc[i][1]));
    SPI.transfer(pgm_read_byte(&resetDisc[i][2]));
    digitalWrite(EN_PIN, HIGH);

    delayMicroseconds(1400);
    ClearOutputs();
    delay(100);
  }

  for(uint8_t i = 0; i < NUM_DISCS; i++)
  {
    digitalWrite(EN_PIN, LOW);
    SPI.transfer(pgm_read_byte(&setDisc[i][0]));
    SPI.transfer(pgm_read_byte(&setDisc[i][1]));
    SPI.transfer(pgm_read_byte(&setDisc[i][2]));
    digitalWrite(EN_PIN, HIGH);

    delayMicroseconds(1500);
    ClearOutputs();
    delay(100);
  }
}
