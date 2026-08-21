/*
 * ==========================================================================
 * PIXELSAT DIY - Solar-powered e-paper display
 * ==========================================================================
 * Licence : CC BY-NC-SA 4.0
 * 
 Autonomous low-power display showing time, date, temperature and humidity
 * on a 1.02" e-paper screen. Powered by solar cells + supercapacitor.
 *
 * Hardware:
 *   - MCU      : Microchip ATtiny1616 (also supports ATmega328P for dev)
 *   - Display  : Waveshare 1.02" e-paper (UC8175, 80x128)
 *   - RTC      : NXP PCF8563T
 *   - Sensor   : Sensirion SHT31 (temperature + humidity)
 *   - Solar    : 2x SM111K08L cells in parallel
 *
 * UI:
 *   - Drawn with Lopaka-style syntax (see drawUI() at the bottom of file)
 *   - Customisable via the Lopaka visual editor (https://lopaka.app)
 *
 * Version history: see CHANGELOG.md
 *
 * Creator : Maxime Loiseau Labs
 * ==========================================================================
 */

#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>          // sleep modes + sleep_cpu()
#include <avr/wdt.h>            // watchdog (interrupt mode on UNO, reset-only on tinyAVR)
#include <avr/interrupt.h>      // cli(), sei(), ISR()
#include <string.h>
#include <EEPROM.h>             // for RTC init-flag persistence (build hash)

// ==========================================================================
// USER CONFIGURATION
// ==========================================================================

// Refresh cadence: a full refresh is forced every N partial refreshes.
// Waveshare panels must alternate full/partial to avoid ghosting and
// pixel burn-in (mfg recommendation).
#define FULL_REFRESH_EVERY    10

// PCF8563T I2C address (7-bit).
#define RTC_ADDR              0x51

// Software watchdog timeout. On tinyAVR the WDT is reset-only, so we
// emulate the interrupt mode with RTC.PIT @ 1Hz (1 tick = 1s).
// On ATmega328P the WDT runs in interrupt mode @ 8s (10 ticks = 80s).
// Both give an ~80s timeout, giving 20s of margin over the 60s RTC IRQ.
#if defined(MEGATINYCORE)
    #define WDT_MAX_TICKS    80
#else
    #define WDT_MAX_TICKS    10
#endif

// VCC thresholds (mV) for charge-percentage mapping and SLEEPING entry.
// VCC_PCT_LOW_MV is both the 0% point on the gauge AND V_OFF (sleeping
// trigger): when the user sees 0%, the system is about to shut down.
// Tune these to your supercap + solar conditions after measuring.
#define VCC_PCT_LOW_MV        2300      // 0% / V_OFF
#define VCC_PCT_HIGH_MV       5000      // 100% / supercap full

// Hysteresis on the SLEEPING transition. V_ON = V_OFF + this value.
// Prevents fast oscillation when the solar input is just borderline.
#define VCC_HYSTERESIS_MV     150

// Charge-trend window. Compare instant VCC to the moving average of the
// last N minutes; |delta| > VCC_TREND_HYST_MV decides up/down arrow.
#define VCC_TREND_WINDOW      5
#define VCC_TREND_HYST_MV     30

// Internal bandgap reference (mV). Nominal 1.1V on ATmega328P / 1.024 or
// 1.1V on tinyAVR, but varies ~10% chip-to-chip. Calibrate once by
// comparing reported VCC to a multimeter reading and adjust this value.
#define BANDGAP_MV            1103L

// Secondary timezone (shown in the date box). Offset is in minutes
// relative to the primary time. Default = Delhi (CET → UTC+5:30 = +270 min).
#define TZ_OFFSET_MIN         330
#define TZ_LABEL              "Delhi"

// Automatic European DST. The RTC always stores winter time (CET);
// the display adds +1h when the current date is in summer time.
// Set to 0 for a timezone without DST.
#define ENABLE_DST            1

// Serial debug. Disable for production on ATtiny1616 (saves Flash + RAM).
#define DEBUG_SERIAL          0

// Auto-set the RTC from the firmware's build time (__DATE__/__TIME__).
// A hash of those strings is stored in EEPROM; on boot, if the hash
// differs, the new build time is written to the RTC (compensating for
// the compile+flash delay via BUILD_TIME_OFFSET_S).
#define ENABLE_AUTO_SET_RTC      1
#define BUILD_TIME_OFFSET_S     10
#define EEPROM_BUILD_HASH_ADDR   0     // 4 bytes (uint32_t)

// ==========================================================================
// PIN CONFIGURATION (depends on the compile target)
// ==========================================================================

#if defined(__AVR_ATmega328P__)
    // UNO target (initial dev). Only D2/D3 wake from PWR_DOWN sleep,
    // so RTC INT must be on one of these.
    #define PIN_CS         10
    #define PIN_DC          9
    #define PIN_RST         8
    #define PIN_BUSY        7
    #define PIN_RTC_INT     2     // INT0

#elif defined(MEGATINYCORE)
    // ATtiny1616 target (production). megaTinyCore standard pinout.
    // On tinyAVR any pin can wake from PWR_DOWN via level interrupt.
    #define PIN_BUSY       10     // PC0 (physical pin 12)
    #define PIN_RST        11     // PC1 (physical pin 13)
    #define PIN_DC         12     // PC2 (physical pin 14)
    #define PIN_CS         13     // PC3 (physical pin 15)
    #define PIN_RTC_INT     5     // PB4 (physical pin 7)

#else
    #error "Target MCU not supported. Add pin mapping for your board."
#endif

// ==========================================================================
// SCREEN DIMENSIONS + COLOR CONSTANTS
// ==========================================================================

#define EPD_W    80    // pixel width  (source lines)
#define EPD_H   128    // pixel height (gate lines)
#define EPD_RB   10    // bytes per row (80 / 8)

// Colours, Lopaka/GxEPD2 convention: 0 = black, 1 = white.
// The actual bit polarity sent to the panel depends on full/partial mode
// (handled in setPixel()).
#define COLOR_BLACK  0
#define COLOR_WHITE  1

// Lopaka-generated code uses GxEPD_BLACK / GxEPD_WHITE — keep those as
// aliases so pasted Lopaka snippets compile unchanged.
#define GxEPD_BLACK  COLOR_BLACK
#define GxEPD_WHITE  COLOR_WHITE

// Sentinel for &Org_01 in setFont(&Org_01). The Lopaka shim only needs to
// compare the pointer, not read an actual GFXfont struct — our rendering
// is hard-coded in drawCharOrg01().
static const uint8_t Org_01 = 0;

// ==========================================================================
// E-PAPER WAVEFORM LUTs - taken verbatim from EPD_1in02d.cpp (Waveshare).
// 4 x 42 bytes in PROGMEM, 0 byte SRAM.
//
// Image bit conventions (intentional asymmetry from the Waveshare driver):
//   FULL refresh    : 0=black, 1=white, base 0xFF
//   PARTIAL refresh : 1=black, 0=white, base 0x00
// ==========================================================================

const uint8_t LUT_W1[42] PROGMEM = {
    0x60,0x5A,0x5A,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00
};
const uint8_t LUT_B1[42] PROGMEM = {
    0x90,0x5A,0x5A,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00
};
const uint8_t LUT_W[42] PROGMEM = {
    0x60,0x01,0x01,0x00,0x00,0x01,
    0x80,0x0f,0x00,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00
};
const uint8_t LUT_B[42] PROGMEM = {
    0x90,0x01,0x01,0x00,0x00,0x01,
    0x40,0x0f,0x00,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00
};

// ==========================================================================
// Org_01 FONT (Adafruit GFX format) - embedded to avoid pulling in the
// full Adafruit_GFX library (30+ KB).
//
// Format: 1 bit/pixel packed bitmaps + a glyph table (offset/w/h/advance/
// xOffset/yOffset per character). setCursor(x, y) puts (x, y) at the
// glyph BASELINE; the glyph is drawn at (x + xOffset, y + yOffset).
//
// Org_01 by Orgdot (www.orgdot.com/aliasfonts), public domain.
// ==========================================================================

// Minimal Adafruit-compatible GFXglyph (7 bytes/glyph).
typedef struct {
    uint16_t bitmapOffset;
    uint8_t  width;
    uint8_t  height;
    uint8_t  xAdvance;
    int8_t   xOffset;
    int8_t   yOffset;
} GFXglyph;

const uint8_t Org_01Bitmaps[] PROGMEM = {
    0xE8, 0xA0, 0x57, 0xD5, 0xF5, 0x00, 0xFD, 0x3E, 0x5F, 0x80, 0x88, 0x88,
    0x88, 0x80, 0xF4, 0xBF, 0x2E, 0x80, 0x80, 0x6A, 0x40, 0x95, 0x80, 0xAA,
    0x80, 0x5D, 0x00, 0xC0, 0xF0, 0x80, 0x08, 0x88, 0x88, 0x00, 0xFC, 0x63,
    0x1F, 0x80, 0xF8, 0xF8, 0x7F, 0x0F, 0x80, 0xF8, 0x7E, 0x1F, 0x80, 0x8C,
    0x7E, 0x10, 0x80, 0xFC, 0x3E, 0x1F, 0x80, 0xFC, 0x3F, 0x1F, 0x80, 0xF8,
    0x42, 0x10, 0x80, 0xFC, 0x7F, 0x1F, 0x80, 0xFC, 0x7E, 0x1F, 0x80, 0x90,
    0xB0, 0x2A, 0x22, 0xF0, 0xF0, 0x88, 0xA8, 0xF8, 0x4E, 0x02, 0x00, 0xFD,
    0x6F, 0x0F, 0x80, 0xFC, 0x7F, 0x18, 0x80, 0xF4, 0x7D, 0x1F, 0x00, 0xFC,
    0x21, 0x0F, 0x80, 0xF4, 0x63, 0x1F, 0x00, 0xFC, 0x3F, 0x0F, 0x80, 0xFC,
    0x3F, 0x08, 0x00, 0xFC, 0x2F, 0x1F, 0x80, 0x8C, 0x7F, 0x18, 0x80, 0xF9,
    0x08, 0x4F, 0x80, 0x78, 0x85, 0x2F, 0x80, 0x8D, 0xB1, 0x68, 0x80, 0x84,
    0x21, 0x0F, 0x80, 0xFD, 0x6B, 0x5A, 0x80, 0xFC, 0x63, 0x18, 0x80, 0xFC,
    0x63, 0x1F, 0x80, 0xFC, 0x7F, 0x08, 0x00, 0xFC, 0x63, 0x3F, 0x80, 0xFC,
    0x7F, 0x29, 0x00, 0xFC, 0x3E, 0x1F, 0x80, 0xF9, 0x08, 0x42, 0x00, 0x8C,
    0x63, 0x1F, 0x80, 0x8C, 0x62, 0xA2, 0x00, 0xAD, 0x6B, 0x5F, 0x80, 0x8A,
    0x88, 0xA8, 0x80, 0x8C, 0x54, 0x42, 0x00, 0xF8, 0x7F, 0x0F, 0x80, 0xEA,
    0xC0, 0x82, 0x08, 0x20, 0x80, 0xD5, 0xC0, 0x54, 0xF8, 0x80, 0xF1, 0xFF,
    0x8F, 0x99, 0xF0, 0xF8, 0x8F, 0x1F, 0x99, 0xF0, 0xFF, 0x8F, 0x6B, 0xA4,
    0xF9, 0x9F, 0x10, 0x8F, 0x99, 0x90, 0xF0, 0x55, 0xC0, 0x8A, 0xF9, 0x90,
    0xF8, 0xFD, 0x63, 0x10, 0xF9, 0x99, 0xF9, 0x9F, 0xF9, 0x9F, 0x80, 0xF9,
    0x9F, 0x20, 0xF8, 0x88, 0x47, 0x1F, 0x27, 0xC8, 0x42, 0x00, 0x99, 0x9F,
    0x99, 0x97, 0x8C, 0x6B, 0xF0, 0x96, 0x69, 0x99, 0x9F, 0x10, 0x2E, 0x8F,
    0x2B, 0x22, 0xF8, 0x89, 0xA8, 0x0F, 0xE0
};

// 95 glyphs: ASCII 0x20 (' ') to 0x7E ('~'), 7 bytes per glyph.
const GFXglyph Org_01Glyphs[] PROGMEM = {
    {0, 0, 0, 6, 0, 1},     // 0x20 ' '
    {0, 1, 5, 2, 0, -4},    // 0x21 '!'
    {1, 3, 1, 4, 0, -4},    // 0x22 '"'
    {2, 5, 5, 6, 0, -4},    // 0x23 '#'
    {6, 5, 5, 6, 0, -4},    // 0x24 '$'
    {10, 5, 5, 6, 0, -4},   // 0x25 '%'
    {14, 5, 5, 6, 0, -4},   // 0x26 '&'
    {18, 1, 1, 2, 0, -4},   // 0x27 '''
    {19, 2, 5, 3, 0, -4},   // 0x28 '('
    {21, 2, 5, 3, 0, -4},   // 0x29 ')'
    {23, 3, 3, 4, 0, -3},   // 0x2A '*'
    {25, 3, 3, 4, 0, -3},   // 0x2B '+'
    {27, 1, 2, 2, 0, 0},    // 0x2C ','
    {28, 4, 1, 5, 0, -2},   // 0x2D '-'
    {29, 1, 1, 2, 0, 0},    // 0x2E '.'
    {30, 5, 5, 6, 0, -4},   // 0x2F '/'
    {34, 5, 5, 6, 0, -4},   // 0x30 '0'
    {38, 1, 5, 2, 0, -4},   // 0x31 '1'
    {39, 5, 5, 6, 0, -4},   // 0x32 '2'
    {43, 5, 5, 6, 0, -4},   // 0x33 '3'
    {47, 5, 5, 6, 0, -4},   // 0x34 '4'
    {51, 5, 5, 6, 0, -4},   // 0x35 '5'
    {55, 5, 5, 6, 0, -4},   // 0x36 '6'
    {59, 5, 5, 6, 0, -4},   // 0x37 '7'
    {63, 5, 5, 6, 0, -4},   // 0x38 '8'
    {67, 5, 5, 6, 0, -4},   // 0x39 '9'
    {71, 1, 4, 2, 0, -3},   // 0x3A ':'
    {72, 1, 4, 2, 0, -3},   // 0x3B ';'
    {73, 3, 5, 4, 0, -4},   // 0x3C '<'
    {75, 4, 3, 5, 0, -3},   // 0x3D '='
    {77, 3, 5, 4, 0, -4},   // 0x3E '>'
    {79, 5, 5, 6, 0, -4},   // 0x3F '?'
    {83, 5, 5, 6, 0, -4},   // 0x40 '@'
    {87, 5, 5, 6, 0, -4},   // 0x41 'A'
    {91, 5, 5, 6, 0, -4},   // 0x42 'B'
    {95, 5, 5, 6, 0, -4},   // 0x43 'C'
    {99, 5, 5, 6, 0, -4},   // 0x44 'D'
    {103, 5, 5, 6, 0, -4},  // 0x45 'E'
    {107, 5, 5, 6, 0, -4},  // 0x46 'F'
    {111, 5, 5, 6, 0, -4},  // 0x47 'G'
    {115, 5, 5, 6, 0, -4},  // 0x48 'H'
    {119, 5, 5, 6, 0, -4},  // 0x49 'I'
    {123, 5, 5, 6, 0, -4},  // 0x4A 'J'
    {127, 5, 5, 6, 0, -4},  // 0x4B 'K'
    {131, 5, 5, 6, 0, -4},  // 0x4C 'L'
    {135, 5, 5, 6, 0, -4},  // 0x4D 'M'
    {139, 5, 5, 6, 0, -4},  // 0x4E 'N'
    {143, 5, 5, 6, 0, -4},  // 0x4F 'O'
    {147, 5, 5, 6, 0, -4},  // 0x50 'P'
    {151, 5, 5, 6, 0, -4},  // 0x51 'Q'
    {155, 5, 5, 6, 0, -4},  // 0x52 'R'
    {159, 5, 5, 6, 0, -4},  // 0x53 'S'
    {163, 5, 5, 6, 0, -4},  // 0x54 'T'
    {167, 5, 5, 6, 0, -4},  // 0x55 'U'
    {171, 5, 5, 6, 0, -4},  // 0x56 'V'
    {175, 5, 5, 6, 0, -4},  // 0x57 'W'
    {179, 5, 5, 6, 0, -4},  // 0x58 'X'
    {183, 5, 5, 6, 0, -4},  // 0x59 'Y'
    {187, 5, 5, 6, 0, -4},  // 0x5A 'Z'
    {191, 2, 5, 3, 0, -4},  // 0x5B '['
    {193, 5, 5, 6, 0, -4},  // 0x5C '\'
    {197, 2, 5, 3, 0, -4},  // 0x5D ']'
    {199, 3, 2, 4, 0, -4},  // 0x5E '^'
    {200, 5, 1, 6, 0, 1},   // 0x5F '_'
    {201, 1, 1, 2, 0, -4},  // 0x60 '`'
    {202, 4, 4, 5, 0, -3},  // 0x61 'a'
    {204, 4, 5, 5, 0, -4},  // 0x62 'b'
    {207, 4, 4, 5, 0, -3},  // 0x63 'c'
    {209, 4, 5, 5, 0, -4},  // 0x64 'd'
    {212, 4, 4, 5, 0, -3},  // 0x65 'e'
    {214, 3, 5, 4, 0, -4},  // 0x66 'f'
    {216, 4, 5, 5, 0, -3},  // 0x67 'g'
    {219, 4, 5, 5, 0, -4},  // 0x68 'h'
    {222, 1, 4, 2, 0, -3},  // 0x69 'i'
    {223, 2, 5, 3, 0, -3},  // 0x6A 'j'
    {225, 4, 5, 5, 0, -4},  // 0x6B 'k'
    {228, 1, 5, 2, 0, -4},  // 0x6C 'l'
    {229, 5, 4, 6, 0, -3},  // 0x6D 'm'
    {232, 4, 4, 5, 0, -3},  // 0x6E 'n'
    {234, 4, 4, 5, 0, -3},  // 0x6F 'o'
    {236, 4, 5, 5, 0, -3},  // 0x70 'p'
    {239, 4, 5, 5, 0, -3},  // 0x71 'q'
    {242, 4, 4, 5, 0, -3},  // 0x72 'r'
    {244, 4, 4, 5, 0, -3},  // 0x73 's'
    {246, 5, 5, 6, 0, -4},  // 0x74 't'
    {250, 4, 4, 5, 0, -3},  // 0x75 'u'
    {252, 4, 4, 5, 0, -3},  // 0x76 'v'
    {254, 5, 4, 6, 0, -3},  // 0x77 'w'
    {257, 4, 4, 5, 0, -3},  // 0x78 'x'
    {259, 4, 5, 5, 0, -3},  // 0x79 'y'
    {262, 4, 4, 5, 0, -3},  // 0x7A 'z'
    {264, 3, 5, 4, 0, -4},  // 0x7B '{'
    {266, 1, 5, 2, 0, -4},  // 0x7C '|'
    {267, 3, 5, 4, 0, -4},  // 0x7D '}'
    {269, 5, 3, 6, 0, -3}   // 0x7E '~'
};

#define ORG01_FIRST   0x20    // first ASCII glyph
#define ORG01_LAST    0x7E    // last  ASCII glyph

// ==========================================================================
// MINI 5x7 FONT - only digits 0-9 and '%' (extracted from glcdfont.c).
// Format: 5 column-bytes per glyph, LSB at top. setCursor(x, y) puts the
// TOP-LEFT corner at (x, y). xAdvance = 6 (5px glyph + 1px gap).
// 11 chars × 5 bytes = 55 B Flash (vs 1280 B for the full glcdfont).
// ==========================================================================

#define GFX5x7_W       5
#define GFX5x7_H       8
#define GFX5x7_XADV    6

const uint8_t GFX5x7_DATA[11][5] PROGMEM = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x72, 0x49, 0x49, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x49, 0x4D, 0x33}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x31}, // '6'
    {0x41, 0x21, 0x11, 0x09, 0x07}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x46, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
};

/** Glyph index in GFX5x7_DATA, -1 if unsupported. */
static int8_t gfx5x7Idx(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == '%')             return 10;
    return -1;
}

// ==========================================================================
// BITMAPS - icons (Adafruit drawBitmap format: 1 bpp, MSB top-left,
// row-by-row, byte-aligned padding at end of each row).
// ==========================================================================

/**
 * 3x3 "degree" dot for the temperature symbol.
 *  XXX   0xE0
 *  XXX   0xE0
 *  XXX   0xE0
 */
const uint8_t IMG_POINT_DEGRE[3] PROGMEM = { 0xE0, 0xE0, 0xE0 };

/**
 * 7x9 charge-status arrow (UP = charging, DOWN = discharging).
 * 3px-wide shaft for visibility on the small 80x128 display.
 *
 *  ..X....   0x10   apex
 *  .XXX...   0x38
 *  XXXXX..   0x7C
 *  XXXXXXX   0xFE   full-width head
 *  .XXX...   0x38   3-px shaft
 *  .XXX...   0x38
 *  .XXX...   0x38
 *  .XXX...   0x38
 *  .XXX...   0x38
 *
 * DOWN = same pattern flipped vertically.
 */
const uint8_t IMG_CHARGE_UP[9] PROGMEM = {
    0x10, 0x38, 0x7C, 0xFE, 0x38, 0x38, 0x38, 0x38, 0x38
};
const uint8_t IMG_CHARGE_DOWN[9] PROGMEM = {
    0x38, 0x38, 0x38, 0x38, 0x38, 0xFE, 0x7C, 0x38, 0x10
};

// Layout note: there are no position #defines anymore. All UI coordinates
// live directly in drawUI() at the bottom of this file (Lopaka-style).
// To modify the layout: either edit drawUI() directly or redesign in Lopaka
// (https://lopaka.app) and paste the generated code.

// ==========================================================================
// GLOBAL STATE - data shown on screen + power-saving + ISR flags.
// SRAM = ~30 B total. All initial values are placeholders overwritten on
// the first loop pass.
// ==========================================================================

// Date/time (mirror of the RTC, updated each minute by readRTC()).
static uint8_t  g_hour    = 0;     // 0-23
static uint8_t  g_minute  = 0;
static uint8_t  g_day     = 1;     // 1-31
static uint8_t  g_month   = 1;     // 1-12
static uint16_t g_year    = 2026;
static uint8_t  g_dow     = 0;     // 0=Sun, 1=Mon, ... 6=Sat
static uint8_t  g_hourCET = 0;     // raw CET hour from RTC, before DST offset

// SHT31 sensor readings. int16_t to allow negative temperatures
// (SHT31 spec: -45..+130 °C).
static int16_t g_temp  = 0;
static int16_t g_humid = 0;

// Charge state. g_chargePct mapped to 15 UI bars in prepareUIData().
// g_charging = true if VCC is trending up vs last 5min average.
static uint8_t g_chargePct = 50;
static bool    g_charging  = false;

// Partial-refresh counter (resets on each full refresh).
static uint8_t g_partialCnt = 0;

// Power-saving state machine. uint8_t (not enum) to save 1 byte SRAM.
//   NORMAL   = standard operation, refresh every minute
//   SLEEPING = screen frozen on "SLEEP" frame, MCU still wakes every
//              minute but only measures VCC and waits for V_ON
#define STATE_NORMAL    0
#define STATE_SLEEPING  1
static uint8_t g_state = STATE_NORMAL;

// Time at which the system entered SLEEPING (displayed on the SLEEP frame).
static uint8_t g_sleepHour   = 0;
static uint8_t g_sleepMinute = 0;

// ISR flags (volatile = forces re-read on each access).
//   g_rtcInt   : set TRUE by rtcIntISR(), cleared in loop()
//   g_wdtCount : incremented every WDT/PIT tick; reset by wdtKick() on a
//                valid RTC wake. Reaches WDT_MAX_TICKS → forced HW reset
//                (recovers from stuck I2C / RTC INT line).
static volatile bool    g_rtcInt   = false;
static volatile uint8_t g_wdtCount = 0;

// ==========================================================================
// LABELS - stored in PROGMEM (Flash) to keep SRAM free.
// ==========================================================================

// Month abbreviations, indexed 1..12 (index 0 is unused padding).
const char MON_JAN[] PROGMEM = "JAN";
const char MON_FEB[] PROGMEM = "FEB";
const char MON_MAR[] PROGMEM = "MAR";
const char MON_APR[] PROGMEM = "APR";
const char MON_MAY[] PROGMEM = "MAY";
const char MON_JUN[] PROGMEM = "JUN";
const char MON_JUL[] PROGMEM = "JUL";
const char MON_AUG[] PROGMEM = "AUG";
const char MON_SEP[] PROGMEM = "SEP";
const char MON_OCT[] PROGMEM = "OCT";
const char MON_NOV[] PROGMEM = "NOV";
const char MON_DEC[] PROGMEM = "DEC";
const char* const MONTH_TABLE[] PROGMEM = {
    MON_JAN, MON_JAN, MON_FEB, MON_MAR, MON_APR, MON_MAY, MON_JUN,
    MON_JUL, MON_AUG, MON_SEP, MON_OCT, MON_NOV, MON_DEC
};

// Weekday names (truncated to 8 chars max to fit the date box in Org_01).
const char DOW_SUN[] PROGMEM = "Sunday";
const char DOW_MON[] PROGMEM = "Monday";
const char DOW_TUE[] PROGMEM = "Tuesday";
const char DOW_WED[] PROGMEM = "Wednesda";
const char DOW_THU[] PROGMEM = "Thursday";
const char DOW_FRI[] PROGMEM = "Friday";
const char DOW_SAT[] PROGMEM = "Saturday";
const char* const DOW_TABLE[] PROGMEM = {
    DOW_SUN, DOW_MON, DOW_TUE, DOW_WED, DOW_THU, DOW_FRI, DOW_SAT
};

// Secondary-timezone label (defined via TZ_LABEL macro at top).
const char STR_TZ_LABEL[] PROGMEM = TZ_LABEL;

// ==========================================================================
// BCD <-> DECIMAL helpers (PCF8563T registers are BCD-encoded).
// ==========================================================================
static inline uint8_t bcd2dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }
static inline uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

// ==========================================================================
// AUTO-SET RTC from build time (__DATE__ / __TIME__).
// On boot, we hash those strings and compare with a hash stored in EEPROM.
// If different, this is a fresh firmware → write build time to the RTC.
// Otherwise (same hash) the RTC has already been set by this firmware.
// ==========================================================================

/**
 * Cheap 32-bit hash of __DATE__ + __TIME__ (DJB2-ish). Only needs to be
 * deterministic and collision-free across builds — not crypto.
 */
static uint32_t computeBuildHash() {
    uint32_t h = 5381;
    const char* s = __DATE__;
    while (*s) { h = ((h << 5) + h) ^ (uint8_t)(*s); s++; }
    s = __TIME__;
    while (*s) { h = ((h << 5) + h) ^ (uint8_t)(*s); s++; }
    return h;
}

/**
 * Convert __DATE__'s 3-letter month abbreviation to 1-12.
 * Returns 0 on unknown input.
 */
static uint8_t parseMonthAbbrev(const char* m3) {
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (uint8_t i = 0; i < 12; i++) {
        if (m3[0] == months[i*3] &&
            m3[1] == months[i*3+1] &&
            m3[2] == months[i*3+2]) {
            return i + 1;
        }
    }
    return 0;
}

/**
 * Parse __DATE__ / __TIME__ and write to the PCF8563T.
 * Adds BUILD_TIME_OFFSET_S to compensate for compile+flash delay.
 * On DST builds, subtracts 1h to store winter time (CET reference).
 * Writes registers 0x02..0x08 (seconds → year). Weekday (0x06) is left
 * at 0 — it's recomputed at read time via Sakamoto's algorithm.
 * Returns true on success.
 */
static bool rtcSetFromBuild() {
    // Parse __DATE__ ("MMM DD YYYY", e.g. "Apr 27 2026").
    const char* d = __DATE__;
    uint8_t  month = parseMonthAbbrev(d);
    if (month == 0) return false;

    // Day: chars 4-5. Char 4 is a space for single-digit days.
    uint8_t day = (d[4] == ' ' ? 0 : (d[4] - '0') * 10) + (d[5] - '0');

    // Year: chars 7-10.
    uint16_t year = (d[7] - '0') * 1000 + (d[8] - '0') * 100
                  + (d[9] - '0') * 10  + (d[10] - '0');

    // Parse __TIME__ ("HH:MM:SS").
    const char* t = __TIME__;
    uint8_t hour   = (t[0] - '0') * 10 + (t[1] - '0');
    uint8_t minute = (t[3] - '0') * 10 + (t[4] - '0');
    uint8_t second = (t[6] - '0') * 10 + (t[7] - '0');

    // Add compile+flash offset. We only propagate the carry up to the
    // hour level — crossing day boundaries is rare and will be corrected
    // on the next compile/flash anyway.
    uint16_t totalSec = (uint16_t)second + BUILD_TIME_OFFSET_S;
    second = totalSec % 60;
    uint16_t carryMin = totalSec / 60;
    uint16_t totalMin = (uint16_t)minute + carryMin;
    minute = totalMin % 60;
    uint16_t carryHour = totalMin / 60;
    hour = (hour + carryHour) % 24;

    // Convert from host's local time to CET (winter time) for storage.
    // If the build was made during summer time (CEST = UTC+2), subtract
    // 1h to get CET = UTC+1. We handle the rare midnight wrap by walking
    // the day back, but stop there — month/year wraps would only happen
    // on April 1st at 00:xx and will self-correct on next reflash.
#if ENABLE_DST
    if (isDST_EU(year, month, day, hour)) {
        if (hour > 0) {
            hour--;
        } else {
            hour = 23;
            if (day > 1) {
                day--;
            }
        }
    }
#endif
    
    // Write 7 RTC registers (BCD-encoded). PCF8563 stores year as 2 digits
    // (00-99 = 2000-2099).
    uint8_t yearBCD = dec2bcd((uint8_t)(year - 2000));

    bool ok = true;
    ok &= rtcWriteReg(0x02, dec2bcd(second));    // VL=0, seconds
    ok &= rtcWriteReg(0x03, dec2bcd(minute));
    ok &= rtcWriteReg(0x04, dec2bcd(hour));
    ok &= rtcWriteReg(0x05, dec2bcd(day));
    ok &= rtcWriteReg(0x06, 0);                  // weekday: recomputed on read
    ok &= rtcWriteReg(0x07, dec2bcd(month));
    ok &= rtcWriteReg(0x08, yearBCD);

#if DEBUG_SERIAL
    if (!ok) Serial.println(F("ERR: rtcSetFromBuild I2C failure"));
#endif

    return ok;
}

/**
 * Set the RTC from build time only if the firmware has changed
 * (build-hash mismatch in EEPROM). Call once in setup().
 */
static void checkAndSetRtcFromBuild() {
    uint32_t currentHash = computeBuildHash();
    uint32_t storedHash;

    EEPROM.get(EEPROM_BUILD_HASH_ADDR, storedHash);

    if (storedHash == currentHash) {
        // Same firmware as last boot → RTC already initialised.
        return;
    }

    // New firmware → write build time to RTC and update hash.
    if (rtcSetFromBuild()) {
        // Only update the hash on a successful I²C write; otherwise we
        // retry on next boot (helps recover from I²C glitches).
        EEPROM.put(EEPROM_BUILD_HASH_ADDR, currentHash);
    }
}

// ==========================================================================
// DAY-OF-WEEK - Sakamoto's algorithm.
// Independent of the PCF8563's weekday register (which is not guaranteed
// to be correctly initialised by the chip). Returns 0=Sun..6=Sat.
// ==========================================================================
static uint8_t computeDayOfWeek(uint16_t y, uint8_t m, uint8_t d) {
    static const uint8_t t[] PROGMEM = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y -= 1;
    return (y + y/4 - y/100 + y/400 + pgm_read_byte(&t[m-1]) + d) % 7;
}

// ==========================================================================
// EUROPEAN DST CALCULATION
//
// EU rules:
//   - Summer time starts on the LAST SUNDAY OF MARCH at 02:00 UTC+1
//     (clock jumps from 02:00 to 03:00)
//   - Winter time resumes on the LAST SUNDAY OF OCTOBER at 03:00 UTC+2
//     (clock falls back from 03:00 to 02:00)
//
// We store CET (UTC+1) in the RTC, so the transitions are at 02:00 in
// March and 03:00 in October in the "stored time" reference frame.
// ==========================================================================

/**
 * Last Sunday of a given month. We only call this with month = 3 or 10,
 * both 31-day months, so no need to handle the 28/30 day cases.
 */
static uint8_t lastSundayOfMonth(uint16_t year, uint8_t month) {
    uint8_t lastDay = 31;
    uint8_t dow31   = computeDayOfWeek(year, month, lastDay);
    // dow31 is 0=Sun..6=Sat → walk back `dow31` days to reach Sunday.
    return lastDay - dow31;
}

/**
 * Returns true if the given CET date/time falls in European summer time.
 * Simple cases: Jan/Feb/Nov/Dec = winter, Apr..Sep = summer.
 * Boundary months: March (DST starts at last Sunday 02:00 CET),
 * October (DST ends at last Sunday 03:00 CET).
 */
static bool isDST_EU(uint16_t year, uint8_t month, uint8_t day, uint8_t hour) {
    if (month < 3 || month > 10) return false;   // Jan/Feb/Nov/Dec: winter
    if (month > 3 && month < 10) return true;    // Apr..Sep: summer

    uint8_t lastSun = lastSundayOfMonth(year, month);

    if (month == 3) {
        // March: summer time from last Sunday at 02:00 CET onwards.
        if (day > lastSun) return true;
        if (day < lastSun) return false;
        return (hour >= 2);
    } else {
        // October: winter time from last Sunday at 03:00 CET onwards
        // (= 02:00 CEST after the clock falls back).
        if (day > lastSun) return false;
        if (day < lastSun) return true;
        return (hour < 3);
    }
}

/**
 * Apply DST offset to display variables (g_hour, g_day, g_dow).
 * Call AFTER readRTC() and BEFORE display logic.
 *
 * Does NOT touch the RTC itself — it always holds reference winter time.
 * On the next readRTC() we re-read CET and re-apply the offset, so no drift.
 *
 * Midnight wrap: 23:xx + 1h = 00:xx the next day. We bump g_day and
 * recompute g_dow, but stop there (month/year wrap would only happen on
 * March 31st → April 1st at 00:xx, which never occurs because the DST
 * transition is at 02:00, not 23:00).
 */
static void applyDSTOffset() {
    if (!isDST_EU(g_year, g_month, g_day, g_hour)) return;

    g_hour++;
    if (g_hour >= 24) {
        g_hour = 0;
        g_day++;
        g_dow = computeDayOfWeek(g_year, g_month, g_day);
    }
}

// ==========================================================================
// PCF8563T READ - registers 0x02..0x08 (7 bytes).
// Updates g_hour, g_minute, g_day, g_month, g_year, g_dow.
// Returns false on I2C failure (used by the software watchdog).
// ==========================================================================
static bool readRTC() {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(0x02);                 // point to 'seconds' register
    if (Wire.endTransmission() != 0) return false;

    if (Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7) != 7) return false;

    uint8_t rawSec   = Wire.read();   // bit 7 = VL (low-voltage flag)
    uint8_t rawMin   = Wire.read();
    uint8_t rawHour  = Wire.read();
    uint8_t rawDay   = Wire.read();
    Wire.read();                      // weekday: ignored (recomputed)
    uint8_t rawMonth = Wire.read();   // bit 7 = century (0 = 20xx)
    uint8_t rawYear  = Wire.read();

    (void)rawSec;                     // not displayed but must be read

    g_minute = bcd2dec(rawMin   & 0x7F);
    g_hour   = bcd2dec(rawHour  & 0x3F);
    g_day    = bcd2dec(rawDay   & 0x3F);
    g_month  = bcd2dec(rawMonth & 0x1F);
    g_year   = (uint16_t)bcd2dec(rawYear) + 2000;
    g_dow    = computeDayOfWeek(g_year, g_month, g_day);
    return true;
}

// ==========================================================================
// PCF8563T LOW-LEVEL HELPERS
// Single-register write/read + timer config + clear flag.
// ==========================================================================

/** Write 1 byte to a PCF8563T register. Returns true on I2C success. */
static bool rtcWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

/** Read 1 byte from a PCF8563T register. Returns true on success. */
static bool rtcReadReg(uint8_t reg, uint8_t* out) {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)1) != 1) return false;
    *out = Wire.read();
    return true;
}

/**
 * Configure the PCF8563T timer for 1Hz countdown from 60 → IRQ every 60s
 * on the INT pin. Must be called at every MCU boot (the backup battery
 * preserves time but NOT the control/timer registers).
 * Datasheet ref: NXP PCF8563 §8.6.2 + §8.6.7.
 *
 * Register 0x01 (Control_Status_2) layout:
 *   bit 7-5: reserved
 *   bit 4  : TI/TP   (0 = INT follows TF)
 *   bit 3  : AF      (alarm flag)
 *   bit 2  : TF      (timer flag)
 *   bit 1  : AIE     (alarm IRQ enable)
 *   bit 0  : TIE     (timer IRQ enable)  ← we want this
 * → 0x01 enables timer IRQ only.
 *
 * Returns true if all I2C writes succeeded.
 */
static bool rtcConfigureTimer() {
    bool ok = true;
    ok &= rtcWriteReg(0x0E, 0x00);   // 1) Timer OFF (TE=0)
    ok &= rtcWriteReg(0x01, 0x00);   // 2) Clear CtrlStatus2 (clears TF/AF)
    ok &= rtcWriteReg(0x0F, 60);     // 3) Countdown = 60
    ok &= rtcWriteReg(0x01, 0x01);   // 4) TIE = 1 (enable timer IRQ)
    ok &= rtcWriteReg(0x0E, 0x82);   // 5) TE=1, source=1Hz (TD=10b)
    return ok;
}

/**
 * Clear the TF (timer flag) in Control_Status_2 (register 0x01).
 *
 * MUST be called on every valid RTC wake. Otherwise:
 *   - TF stays 1
 *   - INT (open-drain) stays LOW
 *   - MCU wakes immediately after every sleep → stuck / max current
 *
 * Sequence (read-modify-write):
 *   - read Control_Status_2
 *   - clear TF (bit 2) by writing 0
 *   - preserve AF (bit 3) by writing 1 (PCF8563 quirk: on TF/AF flags,
 *     write 0 = clear, write 1 = unchanged → OR 0x08 = "leave AF alone")
 *   - write the register back (TIE/AIE/TI_TP preserved)
 *
 * Returns true on I2C success.
 */
static bool rtcClearTimerFlag() {
    uint8_t cs2;
    if (!rtcReadReg(0x01, &cs2)) return false;
    cs2 &= ~0x04;        // bit 2 = TF -> 0  (clear)
    cs2 |=  0x08;        // bit 3 = AF -> 1  (unchanged, defensive)
    return rtcWriteReg(0x01, cs2);
}

/**
 * ISR triggered when the PCF8563T's INT line goes LOW.
 *
 * Minimal ISR: just set a flag and detach the interrupt (otherwise the
 * ISR re-fires continuously while INT stays LOW, which is until we clear
 * TF in the main loop).
 *
 * Do NOT use Serial/Wire/SPI here — they're not reentrant and some need
 * interrupts enabled.
 */
static void rtcIntISR() {
    g_rtcInt = true;
    rtcIntDetach();   // helper: detachInterrupt on UNO, PIN4CTRL on tinyAVR
}

// ==========================================================================
// SHT31 MINIMAL DRIVER (temperature + humidity)
// ==========================================================================
//
// No external lib (Adafruit_SHT31 would add ~3 KB Flash). I2C sequence:
//   1. Address 0x44 (default; 0x45 if ADDR pin is tied to VCC)
//   2. Command 0x24 0x00 = single shot, high repeatability, no clock stretch
//   3. Wait ~15 ms (datasheet: 12.5 ms typ for high rep)
//   4. Read 6 bytes: temp_MSB, temp_LSB, CRC, hum_MSB, hum_LSB, CRC
//   5. Integer-only conversion (no float):
//        T_celsius   = -45 + 175 * raw / 65535
//        Humidity_%  = 100 * raw / 65535
//      Implemented as ((175L * raw) >> 16) — faster than division.
//
// Cost: ~150 B Flash, ~25 ms per reading (mostly the 15 ms wait).
// CRCs are ignored to save ~80 B Flash; I2C ACK already catches bus errors.
//
// Returns 1 on success, 0 on I2C failure.
static uint8_t readSHT31(int16_t &temperature, int16_t &humidity) {
    Wire.beginTransmission(0x44);
    Wire.write(0x24);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return 0;

    delay(15);

    Wire.requestFrom((uint8_t)0x44, (uint8_t)6);
    if (Wire.available() != 6) return 0;

    uint16_t t_raw = ((uint16_t)Wire.read() << 8) | Wire.read();
    Wire.read();    // temp CRC, ignored

    uint16_t h_raw = ((uint16_t)Wire.read() << 8) | Wire.read();
    Wire.read();    // hum CRC, ignored

    // Signed int conversion: -45..130 °C, 0..100 %
    temperature = (int16_t)(((175L * t_raw) >> 16) - 45);
    humidity    = (int16_t)((100L * h_raw) >> 16);
    return 1;
}

// ==========================================================================
// VCC MEASUREMENT via internal bandgap reference
// ==========================================================================
//
// Trick: the ADC can sample the internal bandgap (~1.1V) using VCC as the
// reference. If the bandgap reads N counts out of 1024 (or 1023 on ATmega):
//      VCC = 1024 * BANDGAP_MV / N
//
// Tune BANDGAP_MV per chip for ~1% accuracy (default 1103 mV).
// Returns VCC in millivolts. Typical range: 2700..5200 mV.
// ==========================================================================
static uint16_t readVccMv() {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) \
 || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) \
 || defined(__AVR_ATmega1280__)
    // ----- ATmega328P (UNO) -----
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
    uint16_t adc = ADCL;
    adc |= ((uint16_t)ADCH) << 8;
    if (adc == 0) return 0;
    return (uint16_t)((1024L * BANDGAP_MV) / adc);

#elif defined(MEGATINYCORE)
    // ----- ATtiny1616 ADC0 type B -----
    // Same idea: sample the internal 1.1V ref using VDD as reference.
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;

    ADC0.CTRLC = ADC_PRESC_DIV16_gc       // prescaler /16
               | ADC_REFSEL_VDDREF_gc      // ref = VDD (indirect measurement)
               | (1 << ADC_SAMPCAP_bp);    // reduced SAMPCAP (recommended for Vref >= 1V)

    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;    // channel = internal 1.1V bandgap

    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;

    delay(2);                              // reference settling time

    ADC0.COMMAND = ADC_STCONV_bm;          // start conversion
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

    uint16_t adc = ADC0.RES;

    ADC0.CTRLA = 0;                        // disable ADC (saves ~150 µA)

    if (adc == 0) return 0;
    return (uint16_t)((1024L * BANDGAP_MV) / adc);

#else
    return 4000;   // dummy value, MCU not supported
#endif
}

// ==========================================================================
// CIRCULAR BUFFER for VCC smoothing and trend detection
// ==========================================================================

// Last N VCC readings (mV) — uint16_t holds up to 65535 mV, plenty.
static uint16_t g_vccHistory[VCC_TREND_WINDOW] = { 0 };
static uint8_t  g_vccHistoryIdx   = 0;
static uint8_t  g_vccHistoryCount = 0;   // valid samples accumulated so far

/** Convert mV to 0..100% with clamping outside [LOW, HIGH] thresholds. */
static uint8_t vccToPct(uint16_t mv) {
    if (mv <= VCC_PCT_LOW_MV)  return 0;
    if (mv >= VCC_PCT_HIGH_MV) return 100;
    // 32-bit math to avoid overflow: (mv - LOW) * 100 can reach 150000.
    return (uint8_t)(((uint32_t)(mv - VCC_PCT_LOW_MV) * 100UL)
                     / (VCC_PCT_HIGH_MV - VCC_PCT_LOW_MV));
}

/** Average of stored history (returns 0 if fewer than 2 samples). */
static uint16_t vccHistoryAverage() {
    if (g_vccHistoryCount < 2) return 0;
    uint32_t sum = 0;
    uint8_t  n   = (g_vccHistoryCount < VCC_TREND_WINDOW)
                   ? g_vccHistoryCount : VCC_TREND_WINDOW;
    for (uint8_t i = 0; i < n; i++) sum += g_vccHistory[i];
    return (uint16_t)(sum / n);
}

/**
 * Update g_chargePct and g_charging from a new VCC sample. Call once per
 * minute (on RTC wake).
 *
 * Trend with hysteresis:
 *   vcc > avg + HYST  → charging
 *   vcc < avg - HYST  → discharging
 *   |vcc - avg| < HYST → keep previous state (dead zone)
 */
static void updateVccStatus(uint16_t vccMv) {
    g_chargePct = vccToPct(vccMv);

    uint16_t avg = vccHistoryAverage();
    if (avg > 0) {
        if (vccMv > avg + VCC_TREND_HYST_MV)       g_charging = true;
        else if (vccMv + VCC_TREND_HYST_MV < avg)  g_charging = false;
        // else: keep previous g_charging (dead zone)
    }
    // If avg==0 (not enough samples yet), keep init value of g_charging.
    // First few minutes may show wrong arrow, then it self-corrects.

    // Insert new sample into circular buffer.
    g_vccHistory[g_vccHistoryIdx] = vccMv;
    g_vccHistoryIdx = (g_vccHistoryIdx + 1) % VCC_TREND_WINDOW;
    if (g_vccHistoryCount < VCC_TREND_WINDOW) g_vccHistoryCount++;
}

// ==========================================================================
// SOFTWARE WATCHDOG - conditional implementation
//
// On UNO     : ATmega WDT in interrupt mode at 8s.
// On tinyAVR : RTC.PIT at 1 Hz (tinyAVR WDT is reset-only). PIT runs from
//              OSCULP32K (32 kHz, ultra-low power), stays alive in PDOWN,
//              draws < 1 µA.
// ==========================================================================

#if defined(MEGATINYCORE)

/**
 * Configure the PIT (Periodic Interrupt Timer) to fire at 1 Hz.
 * Writes to PIT registers must wait on RTC.STATUS / RTC.PITSTATUS sync
 * bits or they're silently ignored.
 */
static void wdtSetup8s_INT() {
    while (RTC.STATUS > 0);                  // wait sync
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;       // clock = OSCULP32K
    RTC.PITINTCTRL = RTC_PI_bm;              // enable PIT IRQ
    while (RTC.PITSTATUS > 0);               // wait PIT sync
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;  // 1 Hz, ON
}

/**
 * Atomic reset of the software tick counter after a valid RTC wake.
 * cli/sei around it because g_wdtCount is volatile (modified in ISR).
 */
static void wdtKick() {
    cli();
    g_wdtCount = 0;
    sei();
}

/**
 * Force a hardware reset via the RSTCTRL module. Cleaner than re-arming
 * the WDT in short-reset mode: immediate, deterministic.
 */
static void wdtForceReset() {
    cli();
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    while (1) { /* wait for reset */ }
}

/**
 * PIT ISR — fires every 1 s. Must acknowledge PITINTFLAGS or the IRQ
 * re-fires. If g_wdtCount exceeds WDT_MAX_TICKS, force a HW reset.
 */
ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;             // ack flag

    g_wdtCount++;
    if (g_wdtCount >= WDT_MAX_TICKS) {
        // Stuck condition: force immediate HW reset.
        _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
        while (1) {}
    }
}

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) \
   || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) \
   || defined(__AVR_ATmega1280__)

// ----- UNO / ATmega implementation -----

/**
 * Configure the WDT in interrupt-only mode at 8s.
 * The cli/sei + timed-write sequence is mandatory on AVR.
 */
static void wdtSetup8s_INT() {
    cli();
    asm volatile("wdr");
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);                  // timed-write window
    WDTCSR  = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);   // 8s, INT-only
    sei();
}

/** Reset the software watchdog counter. */
static void wdtKick() {
    cli();
    g_wdtCount = 0;
    asm volatile("wdr");
    sei();
}

/** Force HW reset via WDT short-reset (15 ms). */
static void wdtForceReset() {
    cli();
    wdt_enable(WDTO_15MS);
    while (1) { /* wait for reset */ }
}

/**
 * WDT ISR (fires every 8s). The hardware automatically switches back to
 * RESET mode after firing, so we must re-enable WDIE here — otherwise
 * the next 8s timeout would reset us instead of generating an IRQ.
 */
ISR(WDT_vect) {
    g_wdtCount++;
    if (g_wdtCount >= WDT_MAX_TICKS) {
        wdt_enable(WDTO_15MS);
        while (1) {}
    }
    WDTCSR |= (1 << WDIE);   // re-arm INT mode
}

#endif

// ==========================================================================
// RTC EXTERNAL-INTERRUPT HELPERS
// UNO     : attachInterrupt on INT0 (D2), LEVEL_LOW.
// tinyAVR : direct PORTB.PIN4CTRL config (LEVEL_LOW is the only mode
//           that wakes from PWR_DOWN/PDOWN).
// ==========================================================================

/** Enable the RTC INT line wake-up (level LOW). */
static void rtcIntAttach() {
#if defined(MEGATINYCORE)
    // ISC_LEVEL_gc = wakes the CPU on LOW level (works in PDOWN).
    // PULLUPEN_bm = internal pull-up (PCF8563 INT is open-drain).
    PORTB.INTFLAGS = PIN4_bm;
    PORTB.PIN4CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm;
#else
    attachInterrupt(digitalPinToInterrupt(PIN_RTC_INT), rtcIntISR, LOW);
#endif
}

/**
 * Disable the RTC INT wake-up. Called from the ISR to avoid re-firing
 * while INT stays LOW (until we clear TF in the main loop).
 */
static void rtcIntDetach() {
#if defined(MEGATINYCORE)
    // Keep the pull-up active (otherwise the line would float during
    // wake-up processing).
    PORTB.PIN4CTRL = PORT_ISC_INTDISABLE_gc | PORT_PULLUPEN_bm;
#else
    detachInterrupt(digitalPinToInterrupt(PIN_RTC_INT));
#endif
}

#if defined(MEGATINYCORE)
/**
 * PORTB ISR on ATtiny1616. We test PIN4 explicitly because other pins
 * on the same port might use interrupts in the future. The flag MUST be
 * acknowledged (write 1 = clear) BEFORE calling rtcIntISR().
 */
ISR(PORTB_PORT_vect) {
    if (PORTB.INTFLAGS & PIN4_bm) {
        PORTB.INTFLAGS = PIN4_bm;
        rtcIntISR();
    }
}
#endif

// ==========================================================================
// LOW-LEVEL SPI / EPD FUNCTIONS
// Mirror EPD_SendCommand / EPD_SendData / EPD_WaitUntilIdle / EPD_Reset
// from EPD_1in02d.cpp + DEV_Config.cpp (Waveshare reference driver).
// ==========================================================================

static void epdCmd(uint8_t cmd) {
    digitalWrite(PIN_DC, LOW);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(cmd);
    digitalWrite(PIN_CS, HIGH);
}

static void epdDat(uint8_t dat) {
    digitalWrite(PIN_DC, HIGH);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(dat);
    digitalWrite(PIN_CS, HIGH);
}

/**
 * Mirror of EPD_WaitUntilIdle().
 * Quirks: BUSY HIGH = ready, BUSY LOW = busy.
 *   - Command 0x71 must precede every BUSY pin read.
 *   - 200 ms delay after BUSY=HIGH is required for stabilisation.
 *   - On tinyAVR we sleep in IDLE during the wait (saves ~3 mA × 2 s).
 */
// SLEEP_MODE_IDLE on tinyAVR: CPU stops but all peripherals (SPI, TWI,
// timers, millis) keep running. Draws ~200 µA vs ~3 mA active (15x less).
// RTC INT can also wake from IDLE; the g_rtcInt flag will be processed
// on the next loop() iteration after the refresh.
static void epdIdleMs(uint16_t ms) {
    uint32_t deadline = millis() + ms;
    while ((int32_t)(millis() - deadline) < 0) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sleep_cpu();        // wake on any IRQ (e.g. millis timer)
        sleep_disable();
    }
}

static void epdWait() {
    uint32_t t = millis();
    do {
        epdCmd(0x71);
        if (millis() - t > 6000UL) {
#if DEBUG_SERIAL
            Serial.println(F("WARN: epdWait timeout"));
#endif
            break;
        }
        // IDLE sleep instead of busy-wait: CPU sleeps, SPI stays alive.
        epdIdleMs(10);
    } while (digitalRead(PIN_BUSY) == LOW);

    // delay(200) replaced with IDLE sleep.
    epdIdleMs(200);
}

/** Mirror of EPD_Reset(): 20ms LOW pulse on RST. */
static void epdReset() {
    digitalWrite(PIN_RST, LOW);  delay(20);
    digitalWrite(PIN_RST, HIGH); delay(20);
}

// LUT loading via registers 0x23 (white waveform) / 0x24 (black waveform)
static void loadLUTFull() {
    epdCmd(0x23); for (uint8_t i=0;i<42;i++) epdDat(pgm_read_byte(&LUT_W1[i]));
    epdCmd(0x24); for (uint8_t i=0;i<42;i++) epdDat(pgm_read_byte(&LUT_B1[i]));
}
static void loadLUTPart() {
    epdCmd(0x23); for (uint8_t i=0;i<42;i++) epdDat(pgm_read_byte(&LUT_W[i]));
    epdCmd(0x24); for (uint8_t i=0;i<42;i++) epdDat(pgm_read_byte(&LUT_B[i]));
}

/**
 * Panel init — mirror of EPD_Init() / EPD_Part_Init().
 * Common sequence with three full/partial differences:
 *   0x30 (clock)      : 0x13 full (~50Hz) vs 0x17 partial
 *   0x50 (VCOM/CDI)   : 0x97 full         vs 0xb2 partial
 *   0x61 (resolution) : sent only in full
 *
 * Register 0x50 (VCOM AND DATA INTERVAL, UC8175):
 *   bits 7-6 : VBD (Border Voltage Detection)
 *                00 = floating
 *                01 = LUT_VCOM   (neutral, no visible border)
 *                10 = data-dependent
 *                11 = -VS         (solid BLACK border!)
 *   bits 5-4 : DDX (Data polarity)
 *   bits 3-0 : CDI (Data interval in hclk)
 *
 * Gotcha: a previous iteration used 0xf2 (VBD=11) which painted a black
 * border on every partial refresh — the border slowly turned permanently
 * black over many cycles. 0xb2 keeps DDX/CDI but flips VBD to 10 (data
 * dependent), giving a neutral border.
 */
static void epdInit(bool partialMode) {
    epdReset();                          // reset + wake from deep sleep

    epdCmd(0xD2); epdDat(0x3F);

    epdCmd(0x00); epdDat(0x6F);          // Panel Setting

    epdCmd(0x01);                        // Power Setting
    epdDat(0x03); epdDat(0x00);
    epdDat(0x2b); epdDat(0x2b);

    epdCmd(0x06); epdDat(0x3f);

    epdCmd(0x2A); epdDat(0x00); epdDat(0x00);

    epdCmd(0x30); epdDat(partialMode ? 0x17 : 0x13);

    // 0x50: VBD bits set to avoid a permanent black border on partial.
    epdCmd(0x50); epdDat(partialMode ? 0xb2 : 0x97);

    epdCmd(0x60); epdDat(0x22);

    if (!partialMode) {
        epdCmd(0x61);
        epdDat(0x50);  // source = 80px
        epdDat(0x80);  // gate   = 128px
    }

    epdCmd(0x82); epdDat(0x12);
    epdCmd(0xe3); epdDat(0x33);

    if (partialMode) loadLUTPart();
    else             loadLUTFull();

    epdCmd(0x04); epdWait();             // Power On
}

/**
 * Trigger the refresh and power down — mirror of EPD_TurnOnDisplay().
 * The 0x02 (Power Off) at the end is MANDATORY — leaving the panel under
 * high voltage damages it permanently.
 */
static void epdTurnOnDisplay() {
    epdCmd(0x04); epdWait();             // Power On
    epdCmd(0x12); epdWait();             // Display Refresh (~2s full / ~0.3s partial)
    epdCmd(0x02); epdWait();             // Power Off — MANDATORY
}

/**
 * EPD deep sleep (~0.017 mW). Image is retained without power (e-ink
 * persistence). Required between every refresh per Waveshare's manual.
 * Wake-up: epdInit() → epdReset() (RST HIGH).
 *
 * Note: this Waveshare module requires RST kept LOW during deep sleep;
 * leaving RST HIGH causes excess current draw.
 */
static void epdDeepSleep() {
    epdCmd(0x07); epdDat(0xA5);          // Deep Sleep
    digitalWrite(PIN_RST, LOW);          // RST LOW (required on this module)
}

// ==========================================================================
// FULL-BUFFER RENDERING - 1280 bytes image buffer in SRAM
//
//  - `g_imgBuf` holds the complete image (1280 bytes = 80 × 128 / 8 px).
//  - `drawUI()` fills it using Lopaka-style primitives (drawRoundRect,
//    drawLine, drawTextOrg01, etc.) without any row/fullMode params.
//  - `sendFrame()` ships the whole buffer to the panel over SPI in one
//    transaction (1280 consecutive bytes).
//
// Bit convention (kept identical to row-by-row implementations):
//  - g_drawingFullMode true  : base 0xFF, 0 = black (full refresh)
//  - g_drawingFullMode false : base 0x00, 1 = black (partial refresh)
//
// g_drawingFullMode must be set by displayFull()/displayPartial() BEFORE
// calling drawUI(). setPixel() reads it; the higher primitives (drawLine,
// drawRect, etc.) go through setPixel() and don't see the mode directly.
//
// SRAM cost: 1280 bytes. On ATtiny1616 (2 KB) ~600 B remain for stack
// and other globals.
// ==========================================================================

// Current draw mode. Updated by displayFull/displayPartial before each
// call to drawUI() / drawUISleep().
static bool g_drawingFullMode = true;

// Full image buffer: 80 × 128 / 8 = 1280 bytes.
// Pixel (x, y) addressing:
//   byte: g_imgBuf[y * EPD_RB + (x >> 3)]
//   bit : 0x80 >> (x & 7)
static uint8_t g_imgBuf[EPD_W * EPD_H / 8];   // 1280 bytes

/**
 * Set pixel (x, y) to `color` in g_imgBuf, applying the bit convention
 * for the current draw mode (read from g_drawingFullMode).
 *   x, y  : pixel coordinates (x: 0..79, y: 0..127)
 *   color : COLOR_BLACK (0) or COLOR_WHITE (1)
 */
static inline void setPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= EPD_W || y >= EPD_H) return;
    uint16_t idx = (uint16_t)y * EPD_RB + (x >> 3);
    uint8_t mask = 0x80 >> (x & 7);
    bool drawBlack = (color == COLOR_BLACK);
    if (g_drawingFullMode) {
        // Base = 0xFF (all white). 0 = black.
        if (drawBlack) g_imgBuf[idx] &= ~mask;
        else           g_imgBuf[idx] |=  mask;
    } else {
        // Base = 0x00 (all white). 1 = black.
        if (drawBlack) g_imgBuf[idx] |=  mask;
        else           g_imgBuf[idx] &= ~mask;
    }
}

// ----- GEOMETRIC PRIMITIVES (full-buffer, Lopaka-style signatures) -----
//
// All primitives below write directly into g_imgBuf via setPixel(). No
// row/fullMode params — minimal signatures compatible with Lopaka-generated
// code. The bit convention (full vs partial) is read by setPixel() from
// the g_drawingFullMode global, set by displayFull/displayPartial BEFORE
// calling drawUI().

/** Horizontal line from x0 to x1 at row y. */
static void hLine(int8_t x0, int8_t x1, int8_t y, uint8_t color) {
    if (y < 0 || y >= EPD_H) return;
    if (x0 > x1) { int8_t t = x0; x0 = x1; x1 = t; }
    if (x0 < 0) x0 = 0;
    if (x1 >= EPD_W) x1 = EPD_W - 1;
    for (int8_t x = x0; x <= x1; x++) setPixel((uint8_t)x, (uint8_t)y, color);
}

/** Vertical line from y0 to y1 at column x. */
static void vLine(int8_t x, int8_t y0, int8_t y1, uint8_t color) {
    if (x < 0 || x >= EPD_W) return;
    if (y0 > y1) { int8_t t = y0; y0 = y1; y1 = t; }
    if (y0 < 0) y0 = 0;
    if (y1 >= EPD_H) y1 = EPD_H - 1;
    for (int8_t y = y0; y <= y1; y++) setPixel((uint8_t)x, (uint8_t)y, color);
}

/** Arbitrary line from (x0, y0) to (x1, y1), Bresenham. */
static void drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1,
                     uint8_t color) {
    int8_t dx =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int8_t sx = (x0 < x1) ? 1 : -1;
    int8_t dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int8_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)dx + (int16_t)dy;
    while (1) {
        if (x0 >= 0 && x0 < EPD_W && y0 >= 0 && y0 < EPD_H)
            setPixel((uint8_t)x0, (uint8_t)y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = err << 1;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/**
 * [v8] Cadre rectangulaire (sans remplissage).
 * Compatible Lopaka : signature drawRect(x, y, w, h, color).
 */
static void drawRect(int8_t x, int8_t y, int8_t w, int8_t h, uint8_t color) {
    hLine(x, x + w - 1, y, color);              // top
    hLine(x, x + w - 1, y + h - 1, color);      // bottom
    vLine(x, y, y + h - 1, color);              // left
    vLine(x + w - 1, y, y + h - 1, color);      // right
}

/**
 * Rounded rectangle (Adafruit-compatible with r=1: standard frame minus
 * the 4 corners, left white). Only r=1 is supported here — the parameter
 * is accepted for Lopaka syntax compatibility but ignored.
 */
static void drawRoundRect(int8_t x, int8_t y, int8_t w, int8_t h,
                          int8_t r, uint8_t color) {
    (void)r;   // ignored (assumes r=1)
    // Horizontal edges, minus the 2 corners
    hLine(x + 1, x + w - 2, y, color);
    hLine(x + 1, x + w - 2, y + h - 1, color);
    // Vertical edges (full length, including side corners)
    vLine(x, y + 1, y + h - 2, color);
    vLine(x + w - 1, y + 1, y + h - 2, color);
}

/** Filled rectangle. */
static void fillRect(int8_t x, int8_t y, int8_t w, int8_t h, uint8_t color) {
    for (int8_t yy = y; yy < y + h; yy++) {
        hLine(x, x + w - 1, yy, color);
    }
}

/**
 * Bitmap blit at (x, y), Adafruit format: 1 bpp, MSB top-left, row by
 * row, byte-aligned padding at end of each row.
 */
static void drawBitmap(int8_t x, int8_t y,
                       const uint8_t* bmp, uint8_t w, uint8_t h,
                       uint8_t color) {
    uint8_t bytesPerRow = (w + 7) >> 3;
    for (uint8_t yy = 0; yy < h; yy++) {
        for (uint8_t xx = 0; xx < w; xx++) {
            uint8_t b = pgm_read_byte(&bmp[yy * bytesPerRow + (xx >> 3)]);
            if (b & (0x80 >> (xx & 7))) {
                setPixel((uint8_t)(x + xx), (uint8_t)(y + yy), color);
            }
        }
    }
}

// ----- Org_01 CUSTOM FONT RENDERING (Adafruit GFX format) -----

/**
 * Render Org_01 character `c` at (xb, yb) where yb is the baseline.
 * `size` is the scale factor (1, 2, 3...).
 */
static void drawCharOrg01(int16_t xb, int16_t yb, char c, uint8_t size,
                          uint8_t color) {
    if ((uint8_t)c < ORG01_FIRST || (uint8_t)c > ORG01_LAST) return;

    const GFXglyph* g = &Org_01Glyphs[(uint8_t)c - ORG01_FIRST];
    uint16_t bo = pgm_read_word(&g->bitmapOffset);
    uint8_t  w  = pgm_read_byte(&g->width);
    uint8_t  h  = pgm_read_byte(&g->height);
    int8_t   xo = (int8_t)pgm_read_byte(&g->xOffset);
    int8_t   yo = (int8_t)pgm_read_byte(&g->yOffset);

    if (w == 0 || h == 0) return;             // glyphe invisible (espace)

    // Top-left corner of the glyph on screen (after scaling).
    int16_t topX = xb + (int16_t)xo * size;
    int16_t topY = yb + (int16_t)yo * size;

    // Adafruit packs bits row by row with no padding between rows; we
    // walk bits sequentially across all rows.
    uint16_t bitIdx = 0;
    for (uint8_t yy = 0; yy < h; yy++) {
        for (uint8_t xx = 0; xx < w; xx++) {
            uint16_t byteIdx = bitIdx >> 3;
            uint8_t  bitInByte = bitIdx & 7;
            uint8_t  b = pgm_read_byte(&Org_01Bitmaps[bo + byteIdx]);
            if (b & (0x80 >> bitInByte)) {
                // Active pixel: render a size×size block.
                int16_t px = topX + (int16_t)xx * size;
                int16_t py = topY + (int16_t)yy * size;
                for (uint8_t dy = 0; dy < size; dy++) {
                    for (uint8_t dx = 0; dx < size; dx++) {
                        int16_t qx = px + dx;
                        int16_t qy = py + dy;
                        if (qx >= 0 && qx < EPD_W && qy >= 0 && qy < EPD_H)
                            setPixel((uint8_t)qx, (uint8_t)qy, color);
                    }
                }
            }
            bitIdx++;
        }
    }
}

/**
 * Return the xAdvance (cursor horizontal advance) for an Org_01 character,
 * scaled by `size`. Helper used for centering since Org_01 has variable
 * glyph widths (e.g. '1' = 2px, other digits = 6px at size 1).
 */
static int8_t org01XAdv(char c, uint8_t size) {
    if ((uint8_t)c < ORG01_FIRST || (uint8_t)c > ORG01_LAST) return 0;
    return (int8_t)(pgm_read_byte(&Org_01Glyphs[(uint8_t)c - ORG01_FIRST].xAdvance)
                    * size);
}

// ----- MINI 5x7 FONT RENDERING (digits + '%') -----

/**
 * Render a GFX 5x7 character at (x, y) where (x, y) is the TOP-LEFT
 * (Adafruit convention for the default font).
 */
static void drawCharGFX(int16_t x, int16_t y, char c, uint8_t size,
                        uint8_t color) {
    int8_t idx = gfx5x7Idx(c);
    if (idx < 0) return;

    // Each glyph column is a byte (LSB = top pixel, per glcdfont format).
    for (uint8_t col = 0; col < GFX5x7_W; col++) {
        uint8_t colByte = pgm_read_byte(&GFX5x7_DATA[idx][col]);
        for (uint8_t bmpRow = 0; bmpRow < GFX5x7_H; bmpRow++) {
            if (colByte & (1 << bmpRow)) {
                // Active pixel: render a size×size block.
                int16_t px = x + (int16_t)col * size;
                int16_t py = y + (int16_t)bmpRow * size;
                for (uint8_t dy = 0; dy < size; dy++) {
                    for (uint8_t dx = 0; dx < size; dx++) {
                        int16_t qx = px + dx;
                        int16_t qy = py + dy;
                        if (qx >= 0 && qx < EPD_W && qy >= 0 && qy < EPD_H)
                            setPixel((uint8_t)qx, (uint8_t)qy, color);
                    }
                }
            }
        }
    }
}

// Note: there are no drawTextOrg01 / drawTextGFX helpers. String rendering
// happens directly in print() (in the Lopaka shim below), which reads the
// current font from g_currentFont and loops char by char.

// ==========================================================================
// UI COMPOSITION - Lopaka-style syntax, full buffer
//
// Two-stage pipeline:
//
//   1. prepareUIData() — format every dynamic string into a set of text
//      globals (Hours_text, Minutes_text, Date_text, etc.) and compute
//      the bar count + charge direction. Call ONCE before drawUI().
//
//   2. drawUI() — draw ALL graphical elements into g_imgBuf using pure
//      Lopaka syntax (display.drawRoundRect, display.drawLine, etc.),
//      without worrying about the full/partial bit convention. One pass
//      per refresh (vs 128 row calls in row-by-row implementations).
//
// This separation keeps drawUI() readable as-is: it mirrors Lopaka-generated
// code, so the layout is visible without data-formatting clutter.
// ==========================================================================

// ==========================================================================
// LOPAKA TEXT VARIABLES (char[] arrays sized tight to save SRAM)
// Populated by prepareUIData(), consumed by drawUI() via display.print().
// ==========================================================================

static char Hours_text[3];          // "HH\0"
static char Minutes_text[3];        // "MM\0"
static char Date_text[3];           // "DD\0" (numero du jour)
static char Day_text[9];            // "Wednesda\0" max 8 chars
static char Month_text[4];          // "JAN\0"
static char Year_text[5];           // "2026\0"
static char Clock_2_text[12];       // "Delhi-HH:MM\0"
static char CHARGE___text[5];       // "100%\0" max
static char Num_temp_text[5];       // "-45" to "130" + null
static char Num_humid_text[3];      // "99" + null
static char Refresh_count_text[2];  // "9" single digit + null
static char SleepTime_text[6];      // "HH:MM" + null (SLEEP screen)

// Helpers used by drawUI() for dynamic content that Lopaka can't express
// directly (conditionals, arrow direction).
//   barsToShow              : 0..15 charge bars to draw
//   image_STATUT_CHARGE_bits: points to IMG_CHARGE_UP or IMG_CHARGE_DOWN
//                             depending on g_charging (set in prepareUIData)
//   image_POINT_DEGR__bits  : alias for IMG_POINT_DEGRE (Lopaka naming).
static uint8_t barsToShow = 0;
static const uint8_t* image_STATUT_CHARGE_bits = IMG_CHARGE_DOWN;
#define image_POINT_DEGR__bits IMG_POINT_DEGRE

// ==========================================================================
// "LOPAKA ENGINE" INTERNAL STATE
//
// setTextSize / setFont / setTextColor / setCursor mutate these globals.
// print() reads them to render text with the right attributes.
//
// Behaviour mirrors Adafruit_GFX / GxEPD2:
//   - setFont(nullptr) or setFont() : use the default 5x7 font
//   - setFont(&Org_01)              : use the custom Org_01 font
//   - print() advances g_cursorX by the rendered glyph's xAdvance
// ==========================================================================

static const void* g_currentFont  = nullptr;        // nullptr = 5x7 font
static uint8_t     g_currentSize  = 1;
static uint8_t     g_currentColor = COLOR_BLACK;
static int16_t     g_cursorX      = 0;
static int16_t     g_cursorY      = 0;

// ==========================================================================
// LOPAKA / GxEPD2-STYLE SETTERS
// ==========================================================================

static void setTextSize(uint8_t size) {
    g_currentSize = (size == 0) ? 1 : size;
}

/**
 * setFont(&Org_01) selects Org_01.
 * setFont(nullptr) or setFont() falls back to the default 5x7 font.
 */
static void setFont(const void* font = nullptr) {
    g_currentFont = font;
}

static void setTextColor(uint16_t color) {
    // GxEPD_BLACK / GxEPD_WHITE are #defined to COLOR_BLACK / COLOR_WHITE.
    g_currentColor = (uint8_t)color;
}

/** Stub kept for Lopaka syntax compatibility — no wrapping needed here. */
static void setTextWrap(bool wrap) { (void)wrap; }

static void setCursor(int16_t x, int16_t y) {
    g_cursorX = x;
    g_cursorY = y;
}

/**
 * Render a RAM string at the cursor with the current font/size/color.
 * Advances g_cursorX by xAdvance per glyph (matches Adafruit_GFX behaviour).
 */
static void print(const char* s) {
    if (s == nullptr) return;

    if (g_currentFont == &Org_01) {
        // Org_01: g_cursorY is the baseline.
        while (*s) {
            char c = *s++;
            if ((uint8_t)c >= ORG01_FIRST && (uint8_t)c <= ORG01_LAST) {
                const GFXglyph* g = &Org_01Glyphs[(uint8_t)c - ORG01_FIRST];
                drawCharOrg01(g_cursorX, g_cursorY, c, g_currentSize, g_currentColor);
                g_cursorX += (int16_t)pgm_read_byte(&g->xAdvance) * g_currentSize;
            }
        }
    } else {
        // Default 5x7: g_cursorY is the top-left.
        while (*s) {
            drawCharGFX(g_cursorX, g_cursorY, *s, g_currentSize, g_currentColor);
            g_cursorX += (int16_t)GFX5x7_XADV * g_currentSize;
            s++;
        }
    }
}

/** Overload for integers (kept for Lopaka compatibility — rarely used here). */
static void print(int n) {
    char buf[8];
    itoa(n, buf, 10);
    print((const char*)buf);
}

// ==========================================================================
// "display" SHIM - lets us paste Lopaka code verbatim
//
// Lopaka emits "display.drawRect(...)", "display.print(...)" etc. We keep
// that syntax by exposing a struct with inline methods that forward to
// the globals above. inline → zero runtime cost, compiler removes it.
//
// SRAM cost  : 0 (empty struct)
// Flash cost : 0 (calls are inlined away)
// ==========================================================================

struct DisplayShim {
    // ----- Geometric primitives -----
    inline void drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1, uint8_t color) {
        ::drawLine(x0, y0, x1, y1, color);
    }
    inline void drawRect(int8_t x, int8_t y, int8_t w, int8_t h, uint8_t color) {
        ::drawRect(x, y, w, h, color);
    }
    inline void drawRoundRect(int8_t x, int8_t y, int8_t w, int8_t h,
                              int8_t r, uint8_t color) {
        ::drawRoundRect(x, y, w, h, r, color);
    }
    inline void fillRect(int8_t x, int8_t y, int8_t w, int8_t h, uint8_t color) {
        ::fillRect(x, y, w, h, color);
    }
    inline void drawBitmap(int8_t x, int8_t y, const uint8_t* bmp,
                           uint8_t w, uint8_t h, uint8_t color) {
        ::drawBitmap(x, y, bmp, w, h, color);
    }

    // ----- Text (persistent state) -----
    inline void setTextSize(uint8_t size)             { ::setTextSize(size); }
    inline void setFont(const void* font = nullptr)   { ::setFont(font); }
    inline void setTextColor(uint16_t color)          { ::setTextColor(color); }
    inline void setTextWrap(bool wrap)                { ::setTextWrap(wrap); }
    inline void setCursor(int16_t x, int16_t y)       { ::setCursor(x, y); }
    inline void print(const char* s)                  { ::print(s); }
    inline void print(int n)                          { ::print(n); }

    // ----- Final refresh (no-op here) -----
    // Lopaka code ends with display.display(). We push to the panel from
    // displayFull() / displayPartial() instead, after drawUI() has run.
    inline void display() {}
};

static DisplayShim display;

// ==========================================================================
// PREPARE UI DATA
//
// Format every displayed string (time, date, sensor values, etc.) into
// the global text variables. Also compute the bar count and the charge
// arrow direction.
//
// Call ONCE before drawUI() (handled inside displayFull / displayPartial).
//
// This isolates all the data formatting, leaving drawUI() focused purely
// on layout — i.e. matching what Lopaka emits.
// ==========================================================================

static void prepareUIData() {
    // Hours / minutes split into HH and MM strings.
    Hours_text[0]   = '0' + (g_hour / 10);
    Hours_text[1]   = '0' + (g_hour % 10);
    Hours_text[2]   = '\0';
    Minutes_text[0] = '0' + (g_minute / 10);
    Minutes_text[1] = '0' + (g_minute % 10);
    Minutes_text[2] = '\0';

    // Day number (DD)
    Date_text[0] = '0' + (g_day / 10);
    Date_text[1] = '0' + (g_day % 10);
    Date_text[2] = '\0';

    // Day-of-week name (copied from PROGMEM)
    {
        const char* dowP = (const char*)pgm_read_word(&DOW_TABLE[g_dow]);
        uint8_t i = 0;
        char c;
        while ((c = pgm_read_byte(dowP + i)) != 0 && i < sizeof(Day_text) - 1) {
            Day_text[i++] = c;
        }
        Day_text[i] = '\0';
    }

    // 3-letter month (copied from PROGMEM)
    {
        const char* monP = (const char*)pgm_read_word(&MONTH_TABLE[g_month]);
        Month_text[0] = pgm_read_byte(monP);
        Month_text[1] = pgm_read_byte(monP + 1);
        Month_text[2] = pgm_read_byte(monP + 2);
        Month_text[3] = '\0';
    }

    // 4-digit year
    Year_text[0] = '0' + (g_year / 1000);
    Year_text[1] = '0' + ((g_year / 100) % 10);
    Year_text[2] = '0' + ((g_year / 10) % 10);
    Year_text[3] = '0' + (g_year % 10);
    Year_text[4] = '\0';

    // Secondary timezone (Delhi-HH:MM). Uses g_hourCET (raw winter
    // reference) so the offset stays correct year-round, regardless of DST.
    {
        int16_t mins = (int16_t)g_hourCET * 60 + (int16_t)g_minute + TZ_OFFSET_MIN;
        mins = mins % 1440;
        if (mins < 0) mins += 1440;
        uint8_t h2 = (uint8_t)(mins / 60);
        uint8_t m2 = (uint8_t)(mins % 60);

        // Copy PROGMEM label ("Delhi" by default).
        uint8_t i = 0;
        char c;
        while ((c = pgm_read_byte(&STR_TZ_LABEL[i])) != 0) Clock_2_text[i++] = c;
        Clock_2_text[i++] = '-';
        Clock_2_text[i++] = '0' + (h2 / 10);
        Clock_2_text[i++] = '0' + (h2 % 10);
        Clock_2_text[i++] = ':';
        Clock_2_text[i++] = '0' + (m2 / 10);
        Clock_2_text[i++] = '0' + (m2 % 10);
        Clock_2_text[i]   = '\0';
    }

    // Charge percentage ("8", "78", "100")
    {
        uint8_t pct = g_chargePct;
        uint8_t i = 0;
        if (pct >= 100) { CHARGE___text[i++] = '1';
                          CHARGE___text[i++] = '0';
                          CHARGE___text[i++] = '0'; }
        else if (pct >= 10) { CHARGE___text[i++] = '0' + (pct / 10);
                              CHARGE___text[i++] = '0' + (pct % 10); }
        else { CHARGE___text[i++] = '0' + pct; }
        CHARGE___text[i++] = '%';
        CHARGE___text[i]   = '\0';
    }

    // Temperature (1-3 digits + optional minus sign)
    {
        int16_t t = g_temp;
        char* p = Num_temp_text;
        if (t < 0) { *p++ = '-'; t = -t; }
        if (t >= 100) *p++ = '0' + (t / 100);
        if (t >= 10)  *p++ = '0' + ((t / 10) % 10);
        *p++ = '0' + (t % 10);
        *p   = '\0';
    }

    // Humidity (clamped 0..99)
    {
        int16_t h = g_humid;
        if (h > 99) h = 99;
        if (h < 0)  h = 0;
        Num_humid_text[0] = '0' + (h / 10);
        Num_humid_text[1] = '0' + (h % 10);
        Num_humid_text[2] = '\0';
    }

    // Partial-refresh counter (1 digit)
    Refresh_count_text[0] = '0' + (g_partialCnt % 10);
    Refresh_count_text[1] = '\0';

    // Sleep-time stamp (shown on the SLEEP screen)
    SleepTime_text[0] = '0' + (g_sleepHour / 10);
    SleepTime_text[1] = '0' + (g_sleepHour % 10);
    SleepTime_text[2] = ':';
    SleepTime_text[3] = '0' + (g_sleepMinute / 10);
    SleepTime_text[4] = '0' + (g_sleepMinute % 10);
    SleepTime_text[5] = '\0';

    // ----- Dynamic helpers not handled by Lopaka directly -----
    // Bar count: rounded mapping of chargePct → 0..15 bars.
    {
        uint16_t v = (uint16_t)g_chargePct * 15 + 50;
        barsToShow = (uint8_t)(v / 100);
        if (barsToShow > 15) barsToShow = 15;
    }

    // Charge-arrow bitmap: UP if charging, DOWN if discharging.
    image_STATUT_CHARGE_bits = g_charging ? IMG_CHARGE_UP : IMG_CHARGE_DOWN;
}

// ==========================================================================
// drawUI() — PURE LOPAKA SYNTAX
//
// Adapted from a Lopaka sketch of the mini sat UI. Minor modifications vs
// raw Lopaka output:
//   - 15 charge bars rendered conditionally on barsToShow
//   - image_STATUT_CHARGE_bits picks UP or DOWN bitmap (from g_charging,
//     in prepareUIData()) so Lopaka's static drawBitmap call works.
//   - Main clock is centered on the ':' (computed before the Lopaka block
//     to feed setCursor a dynamic X). This replaces Lopaka's fixed
//     setCursor(3, 35) for hours / setCursor(45, 35) for minutes.
//
// The background memset is done first (base 0xFF in full, 0x00 in partial).
// ==========================================================================

static void drawUI() {
    // Clear to white (base value depends on the current bit convention).
    memset(g_imgBuf, g_drawingFullMode ? 0xFF : 0x00, sizeof(g_imgBuf));

    // Dynamic centering of the main clock (HH:MM):
    // Compute the starting X of "HH" so the ':' falls at x=39 (center).
    // Needed because Org_01 has variable glyph widths ('1' = 2 px,
    // others = 6 px at size 1) — fixed setCursor would mis-center.
    int8_t hourPrefixW = org01XAdv(Hours_text[0], 3) + org01XAdv(Hours_text[1], 3);
    int8_t hourX = (int8_t)(EPD_W / 2) - 1 - hourPrefixW;

    // ----- [BEGIN lopaka generated] -----
    // Humidity box
    display.drawRoundRect(0, 90, 38, 38, 1, GxEPD_BLACK);
    // Temperature box
    display.drawRoundRect(42, 90, 38, 38, 1, GxEPD_BLACK);
    // Date box
    display.drawRoundRect(0, 55, 80, 32, 1, GxEPD_BLACK);
    // Hours (dynamic centering: setCursor(hourX, 35) instead of (3, 35))
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(3);
    display.setTextWrap(false);
    display.setFont(&Org_01);
    display.setCursor(hourX, 35);
    display.print(Hours_text);
    // Temp label
    display.setTextSize(1);
    display.setCursor(50, 99);
    display.print("Temp");
    // Temp value
    display.setTextSize(2);
    display.setFont();
    display.setCursor(50, 106);
    display.print(Num_temp_text);
    // Humidity value
    display.setCursor(7, 106);
    display.print(Num_humid_text);
    // Humid label
    display.setTextSize(1);
    display.setFont(&Org_01);
    display.setCursor(7, 99);
    display.print("Humid");
    // Degree dot
    display.drawBitmap(74, 105, image_POINT_DEGR__bits, 3, 3, GxEPD_BLACK);
    // Clock 2 (secondary timezone)
    display.setCursor(17, 49);
    display.print(Clock_2_text);
    // % sign
    display.setFont();
    display.setCursor(31, 113);
    display.print("%");
    // Month
    display.setTextSize(2);
    display.setFont(&Org_01);
    display.setCursor(36, 66);
    display.print(Month_text);
    // Day of the week
    display.setTextSize(1);
    display.setCursor(36, 75);
    display.print(Day_text);
    // Year
    display.setCursor(36, 83);
    display.print(Year_text);
    // Filled black rect behind the day number
    display.fillRect(3, 58, 29, 26, GxEPD_BLACK);
    // Day number in white on black
    display.setTextColor(GxEPD_WHITE);
    display.setTextSize(2);
    display.setFont();
    display.setCursor(7, 64);
    display.print(Date_text);
    // Decorative lines around the refresh counter
    display.drawLine(62, 86, 68, 80, GxEPD_BLACK);
    display.drawLine(69, 79, 79, 79, GxEPD_BLACK);
    display.drawLine(79, 86, 63, 86, GxEPD_WHITE);
    display.drawLine(79, 85, 79, 79, GxEPD_WHITE);
    // Charge bar enclosure (top strip)
    display.drawRoundRect(0, 0, 80, 16, 1, GxEPD_BLACK);
    // Charge percentage text
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);
    display.setCursor(3, 5);
    display.print(CHARGE___text);
    // Decorative lines around the percentage text
    display.drawLine(22, 3, 20, 1, GxEPD_BLACK);
    display.drawLine(23, 3, 78, 3, GxEPD_BLACK);
    display.drawLine(20, 0, 79, 0, GxEPD_WHITE);
    display.drawLine(79, 0, 79, 3, GxEPD_WHITE);
    // BAR1..BAR15 (conditional on barsToShow)
    if (barsToShow >= 1)  display.drawRect(24, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 2)  display.drawRect(27, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 3)  display.drawRect(30, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 4)  display.drawRect(33, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 5)  display.drawRect(36, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 6)  display.drawRect(39, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 7)  display.drawRect(42, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 8)  display.drawRect(45, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 9)  display.drawRect(48, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 10) display.drawRect(51, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 11) display.drawRect(54, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 12) display.drawRect(57, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 13) display.drawRect(60, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 14) display.drawRect(63, 5, 2, 9, GxEPD_BLACK);
    if (barsToShow >= 15) display.drawRect(66, 5, 2, 9, GxEPD_BLACK);
    // Central ':' of the clock
    display.setTextSize(3);
    display.setFont(&Org_01);
    display.setCursor(39, 35);
    display.print(":");
    // Charge-status arrow (UP or DOWN depending on g_charging)
    display.drawBitmap(70, 5, image_STATUT_CHARGE_bits, 7, 9, GxEPD_BLACK);
    // Partial-refresh counter (0..9)
    display.setTextSize(1);
    display.setCursor(71, 87);
    display.print(Refresh_count_text);
    // Minutes (positioned to the right of the central ':')
    display.setTextSize(3);
    display.setCursor(45, 35);
    display.print(Minutes_text);
    // [END lopaka generated]
}


// ==========================================================================
// SLEEP SCREEN (full buffer + Lopaka syntax)
//
// Composes a simple full-refresh frame:
//   - "SLEEP" Org_01 size 3, horizontally centered
//   - Timestamp Org_01 size 2, centered below
//
// Full refresh because the image must stay sharp for hours without any
// further refresh (the e-paper holds the image without power).
// ==========================================================================

/**
 * [v8] Calcule la largeur totale d'une chaine en Org_01 (en pixels).
 * Necessite de sommer les xAdvance reels de chaque glyphe (largeurs
 * variables : '1' = 2px, autres chiffres = 6px en size 1).
 * Helper utilise pour le centrage de drawUISleep().
 */
static int16_t org01StringWidth(const char* s, uint8_t size) {
    int16_t w = 0;
    while (*s) {
        w += (int16_t)org01XAdv(*s, size);
        s++;
    }
    return w;
}

/**
 * [v8] Dessine l'ecran SLEEP dans g_imgBuf, syntaxe Lopaka.
 * Equivalent buffer plein de v7 buildRowSleep(), en une seule passe.
 *
 * Note : on calcule le centrage dynamique a la volee (largeurs variables
 * Org_01), puis on utilise la syntaxe Lopaka classique avec setCursor.
 */
static void drawUISleep() {
    // ----- Centrage dynamique de heure (size 2) -----
    int16_t timeW    = org01StringWidth(SleepTime_text, 2);
    int16_t timeX    = (EPD_W - timeW) / 2;
    
    // [BEGIN lopaka generated]
    
    // black background
    display.fillRect(0, 0, 80, 128, GxEPD_BLACK);
    
    // sleep
    display.setTextColor(GxEPD_WHITE);
    display.setTextSize(2);
    display.setTextWrap(false);
    display.setFont(&Org_01);
    display.setCursor(11, 40);
    display.print("SLEEP");
    
    // text2
    display.setTextSize(1);
    display.setCursor(19, 70);
    display.print("two ray ");
    
    // text3
    display.setCursor(21, 79);
    display.print("of light ...");
    
    // text1
    display.setCursor(19, 61);
    display.print("Between ");
    
    // clock
    display.setTextSize(2);
    display.setCursor(timeX, 110);
    display.print(SleepTime_text);
    // [END lopaka generated]
}

// ==========================================================================
// SPI FRAME SEND (full 1280-byte buffer)
//
// CS is held LOW for the whole frame (single CS toggle, more efficient).
//
//   cmd   : 0x10 (DTM1) or 0x13 (DTM2)
//   allFF : if true, send 0xFF for every byte (the neutral state UC8175
//           expects before the new image in full refresh).
//           If false, send the contents of g_imgBuf.
//
// Note: drawUI() must have run BEFORE sendFrame() to populate the buffer,
// except when allFF=true (in which case g_imgBuf is ignored).
// ==========================================================================
static void sendFrame(uint8_t cmd, bool allFF) {
    epdCmd(cmd);
    digitalWrite(PIN_DC, HIGH);
    digitalWrite(PIN_CS, LOW);
    if (allFF) {
        for (uint16_t i = 0; i < sizeof(g_imgBuf); i++) {
            SPI.transfer(0xFF);
        }
    } else {
        for (uint16_t i = 0; i < sizeof(g_imgBuf); i++) {
            SPI.transfer(g_imgBuf[i]);
        }
    }
    digitalWrite(PIN_CS, HIGH);
}


/**
 * Render the SLEEP screen in full refresh.
 * Same sequence as displayFull() but calls drawUISleep() instead of
 * drawUI() to populate DTM2.
 */
static void displaySleepScreen() {
    epdInit(false);                       // full refresh mode

    prepareUIData();                      // for SleepTime_text
    g_drawingFullMode = true;
    drawUISleep();                        // fill g_imgBuf in full convention

    sendFrame(0x10, true);                // DTM1: 0xFF (neutral)
    sendFrame(0x13, false);               // DTM2: g_imgBuf

    epdTurnOnDisplay();
    epdDeepSleep();
}

// ==========================================================================
// [v8] AFFICHAGE HAUT NIVEAU - full refresh + partial refresh
// ==========================================================================

/**
 * Full refresh (~2s, scintillement normal).
 * Obligatoire au demarrage et toutes les FULL_REFRESH_EVERY iterations.
 *   DTM1 (0x10) = 0xFF partout (etat neutre)
 *   DTM2 (0x13) = image en convention full (0=noir, base 0xFF)
 *
 * [v8] drawUI() appelee UNE SEULE FOIS (vs 128 appels buildRow() en v7).
 */
static void displayFull() {
    epdInit(false);
    prepareUIData();                  // format dynamic strings
    g_drawingFullMode = true;
    drawUI();                          // fill g_imgBuf in full convention
    sendFrame(0x10, true);             // DTM1: 0xFF
    sendFrame(0x13, false);            // DTM2: g_imgBuf
    epdTurnOnDisplay();
    epdDeepSleep();
}

/**
 * Partial refresh (~0.3s, no global flashing).
 *
 * The UC8175 only updates pixels that DIFFER between DTM1 and DTM2.
 * To make partial refresh work we therefore need to feed it two different
 * frames. We call drawUI() TWICE:
 *   - first  in partial convention (g_drawingFullMode=false) → DTM1
 *   - second in full convention   (g_drawingFullMode=true)   → DTM2
 * Both encode the same visual image but with OPPOSITE bit polarity, which
 * creates the "diff" the controller expects.
 */
static void displayPartial() {
    epdInit(true);

    // Set up the partial window to cover the whole screen.
    epdCmd(0x91);              // PTIN
    epdCmd(0x90); epdDat(0); epdDat(EPD_W - 1); epdDat(0); epdDat(EPD_H - 1); epdDat(0x00);

    prepareUIData();           // format strings once, used by both passes

    // DTM1: image in partial convention (1=black, base 0x00)
    g_drawingFullMode = false;
    drawUI();
    sendFrame(0x10, false);

    // DTM2: same image in full convention (0=black, base 0xFF)
    g_drawingFullMode = true;
    drawUI();
    sendFrame(0x13, false);

    epdTurnOnDisplay();
    epdDeepSleep();
}

// ==========================================================================
// POWER-SAVING STATE TRANSITIONS
//
// transitionToSleeping() : NORMAL -> SLEEPING
//   1. Save g_hour/g_minute for display under "SLEEP"
//   2. Render the SLEEP frame (full refresh + epdDeepSleep)
//   3. Set g_state = STATE_SLEEPING
//
// transitionToNormal() : SLEEPING -> NORMAL
//   1. Set g_state = STATE_NORMAL
//   2. Force a full refresh on next cycle (g_partialCnt = max), so we
//      start from a clean slate after the frozen SLEEP image.
// ==========================================================================

static void transitionToSleeping() {
    // Capture the time the system entered sleep, for display.
    g_sleepHour   = g_hour;
    g_sleepMinute = g_minute;

#if DEBUG_SERIAL
    Serial.print(F("-> SLEEPING at "));
    Serial.print(g_sleepHour);
    Serial.print(':');
    if (g_sleepMinute < 10) Serial.print('0');
    Serial.println(g_sleepMinute);
#endif

    displaySleepScreen();   // show "SLEEP + time" then put EPD in deep sleep
    g_state = STATE_SLEEPING;
}

static void transitionToNormal() {
#if DEBUG_SERIAL
    Serial.println(F("-> NORMAL (VCC restored)"));
#endif

    g_state = STATE_NORMAL;
    // Force a full refresh — a partial on the frozen "SLEEP" frame would
    // leave artefacts. Full starts from a clean white slate.
    g_partialCnt = FULL_REFRESH_EVERY;
}

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
#if DEBUG_SERIAL
    Serial.begin(115200);
    Serial.println(F("=== Mini Sat - boot ==="));
#endif

    // ----- GPIO -----
    pinMode(PIN_CS,   OUTPUT); digitalWrite(PIN_CS,   HIGH);
    pinMode(PIN_DC,   OUTPUT); digitalWrite(PIN_DC,   HIGH);
    pinMode(PIN_RST,  OUTPUT); digitalWrite(PIN_RST,  HIGH);
    pinMode(PIN_BUSY, INPUT);

    // PCF8563T INT line is open-drain active-LOW; we use the MCU's
    // internal pull-up. The external interrupt is attached AFTER the
    // RTC timer config to avoid a stale TF=1 causing an immediate wake.
    pinMode(PIN_RTC_INT, INPUT_PULLUP);

    // ----- I2C (PCF8563T) -----
    // 100kHz default is plenty; transactions are <10 bytes.
    Wire.begin();

    // ----- SPI MODE3 (CPOL=1, CPHA=1), 4 MHz - Waveshare default
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE3));

    delay(100);

    // Initial RTC read. On failure, keep init values (useful for visual
    // debugging — the user can tell the RTC isn't responding).
    bool initialRtcOk = readRTC();
#if DEBUG_SERIAL
    if (!initialRtcOk) Serial.println(F("RTC KO at boot"));
#endif
    (void)initialRtcOk;

    // Apply DST on the first RTC read so the very first frame shows the
    // correct local time.
#if ENABLE_DST
    g_hourCET = g_hour;
    applyDSTOffset();
#endif

    // Auto-set RTC from build time. Only writes if the EEPROM hash
    // differs from the current build hash (i.e. fresh firmware).
    // Must run BEFORE rtcConfigureTimer() which assumes a valid time.
#if ENABLE_AUTO_SET_RTC
    checkAndSetRtcFromBuild();
#endif

    // Configure the PCF8563T timer for periodic 60s wake-ups via INT.
    // On failure we keep going — the software watchdog will reset us
    // after ~80s.
    if (!rtcConfigureTimer()) {
#if DEBUG_SERIAL
        Serial.println(F("WARN: rtcConfigureTimer failed"));
#endif
    }

    // First VCC reading to decide NORMAL vs SLEEPING boot.
    // If VCC < V_ON, the supercap isn't full enough to safely do a full
    // refresh, so we go straight to the SLEEP screen.
    {
        uint16_t vccMv = readVccMv();
        if (vccMv > 0) updateVccStatus(vccMv);

        // Require V_ON (not V_OFF) at boot — same threshold as returning
        // from SLEEPING during operation, gives a symmetric robust behaviour.
        if (vccMv > 0 && vccMv < (VCC_PCT_LOW_MV + VCC_HYSTERESIS_MV)) {
#if DEBUG_SERIAL
            Serial.print(F("Boot SLEEPING (VCC="));
            Serial.print(vccMv); Serial.println(F(" mV)"));
#endif
            transitionToSleeping();
            // No displayFull(): the SLEEP frame is already shown.
        } else {
            displayFull();
            g_partialCnt = 0;
        }
    }

    // Software watchdog: initialised LAST so the first WDT tick can't
    // fire during the ~1-2s of init work above.
    wdtSetup8s_INT();

    // Attach RTC INT (level-LOW, the only level that wakes from deep
    // sleep on both ATmega328P PWR_DOWN and tinyAVR PDOWN). Done AFTER
    // the first refresh so a stale INT doesn't fire too early.
    rtcIntAttach();

#if DEBUG_SERIAL
    Serial.flush();   // flush before first sleep
#endif
}

// ==========================================================================
// MAIN LOOP
//
// Event-driven sleep architecture:
//
//   - MCU sleeps in PWR_DOWN/PDOWN mode (~µA range).
//   - Two wake sources:
//        * PIN_RTC_INT pulled LOW by the PCF8563T → minute elapsed,
//          process normally.
//        * WDT (every 8s on UNO, 1s PIT on tinyAVR) → short wake; if no
//          RTC INT, increment g_wdtCount and go back to sleep.
//
//   - If I2C locks up (TF can't be cleared, INT stuck LOW, etc.),
//     wdtKick() is never called → g_wdtCount grows → hard reset at ~80s.
// ==========================================================================

// Peripheral management around sleep.
// SPI and Wire are stopped before each sleep and restarted on each RTC
// wake (saves ~50-150 µA, cost is ~50 µs of re-init).

static void peripheralsStop() {
    // Wire: make sure the bus is idle before turning off TWI.
    Wire.end();

    // SPI: release MOSI/SCK/MISO. After SPI.end() the pins revert to GPIO.
    SPI.endTransaction();
    SPI.end();

    // MISO (PA2) needs INPUT_DISABLE reasserted — SPI.end() may have
    // switched it to a floating INPUT.
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

static void peripheralsStart() {
    // Force CS HIGH before any SPI re-init (SPI.begin() can glitch pins).
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_CS, OUTPUT);

    Wire.begin();
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE3));
}

/**
 * Put the MCU in deep sleep (PWR_DOWN on ATmega328P, PDOWN on tinyAVR).
 * Serial.flush() is mandatory — otherwise UART bytes get truncated when
 * the CPU clock stops.
 */
static void enterSleep() {
#if DEBUG_SERIAL
    Serial.flush();
#endif

#if defined(MEGATINYCORE)
    // ----- ATtiny1616: SLPCTRL native -----
    // Configure power-down mode and arm the sleep enable bit.
    SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm;
    cli();
    sei();
    sleep_cpu();                              // SLEEP instruction
    SLPCTRL.CTRLA &= ~SLPCTRL_SEN_bm;        // disable sleep after wake

#else
    // ----- ATmega328P (UNO): classic AVR idiom -----
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    #if defined(BODS) && defined(BODSE)
    sleep_bod_disable();   // saves ~17 µA when supported
    #endif
    sei();
    sleep_cpu();           // <-- sleeps HERE. RTC INT or WDT/PIT wakes.
    sleep_disable();
#endif
}

void loop() {
    // -------- 1) On RTC wake: process one minute --------------------------
    if (g_rtcInt) {
        g_rtcInt = false;

        // SPI and Wire are restarted only on a real RTC wake, not on every
        // PIT tick (1 Hz). Avoids unnecessary current spikes and bus
        // re-init every second.
        peripheralsStart();

        // Read date/time from the RTC.
        bool rtcOk = readRTC();

        // Apply DST if needed. The RTC stores CET (winter reference);
        // when the current date falls in summer time we bump g_hour
        // (and g_day if midnight wraps) for display only — the RTC
        // itself is never touched, so no drift between cycles.
#if ENABLE_DST
        if (rtcOk) {
            // Save the raw CET hour so the secondary timezone (Delhi)
            // is computed from a stable reference, season-independent.
            g_hourCET = g_hour;
            applyDSTOffset();
        }
#endif

        // Read SHT31 on every RTC wake (every 60s). Keep previous values
        // on I2C failure.
        int16_t newTemp, newHum;
        if (readSHT31(newTemp, newHum)) {
            g_temp  = newTemp;
            g_humid = newHum;
        }

        // Read VCC: drives both the charge gauge and the NORMAL <-> SLEEPING
        // state transitions.
        uint16_t vccMv = readVccMv();
        if (vccMv > 0) {
            updateVccStatus(vccMv);
        }

        // Clear the TF flag (CRITICAL) — otherwise INT stays LOW.
        bool clearOk = rtcOk ? rtcClearTimerFlag() : false;

        // Explicitly reload the countdown to 60 (defensive).
        if (clearOk) {
            rtcWriteReg(0x0F, 60);
        }
        
        if (rtcOk && clearOk) {
            // Valid wake → reset the software watchdog.
            wdtKick();

            // Power-saving state machine:
            //   NORMAL   : screen refresh (partial, full every N).
            //              Drops to SLEEPING when VCC < V_OFF.
            //   SLEEPING : no screen refresh (e-paper persistence keeps
            //              the SLEEP frame visible). Only checks VCC.
            //              Goes back to NORMAL when VCC >= V_ON.
            //
            // vccMv == 0 (ADC failure) → no state change, to avoid
            // flapping on bad readings.

            if (g_state == STATE_NORMAL) {
                if (vccMv > 0 && vccMv < VCC_PCT_LOW_MV) {
                    // VCC below V_OFF → switch to SLEEPING.
                    transitionToSleeping();
                } else {
                    // NORMAL: refresh as usual.
                    g_partialCnt++;
                    if (g_partialCnt >= FULL_REFRESH_EVERY) {
                        g_partialCnt = 0;
                        displayFull();
                    } else {
                        displayPartial();
                    }
                }
            } else {
                // STATE_SLEEPING: only VCC check.
                if (vccMv >= (VCC_PCT_LOW_MV + VCC_HYSTERESIS_MV)) {
                    // VCC back above V_ON → return to NORMAL.
                    transitionToNormal();
                    displayFull();        // immediate full to leave SLEEP screen
                    g_partialCnt = 0;
                }
                // Otherwise: stay in SLEEPING, screen untouched.
            }
        } else {
#if DEBUG_SERIAL
            Serial.println(F("WARN: RTC wake but I2C failed - WDT will reset"));
#endif
            // wdtKick() NOT called → g_wdtCount grows → HW reset at ~80s.
        }

        // Stop SPI/Wire BEFORE re-attaching the RTC INT, so the next PIT
        // tick won't trigger a peripheral re-init.
        peripheralsStop();

        // Re-attach EXT interrupt after TF was cleared (INT line back HIGH).
        // Doing it earlier would re-fire the ISR if INT was still LOW.
        rtcIntAttach();
    }

    // -------- 2) Back to deep sleep -------------------------------------
    // Reached after RTC processing above, OR on every PIT tick.
    // On a PIT tick: g_rtcInt=false, we do nothing and sleep again.
    // peripheralsStop/Start are NOT called here (handled in the RTC
    // block above) to avoid SPI glitches on every PIT tick.
    enterSleep();
}

/*
 * ==========================================================================
 * ROADMAP
 * ==========================================================================
 *
 * Pending items:
 *   - Live RTC set via Serial: Wake-on-RX (USART start bit) to enter a
 *     config mode. Listen for "SET YYYY M D h m s", write to RTC, sleep.
 *   - NFC sync via ST25DV04K: phone-side companion app (web NFC) to write
 *     time/config into the tag, MCU reads it on next wake.
 *   - Compensation temperature for e-paper waveforms (TSE register 0x41 +
 *     SHT31 reading) — useful for wide temperature deployments.
 *
 * Target final memory budget:
 *   Flash  : ~13 KB / 16 KB
 *   SRAM   : depends on full vs row buffer (v8 full buffer ~1430 B)
 *   EEPROM : 4 B used (build hash), 252 B free
 * ==========================================================================
 */
