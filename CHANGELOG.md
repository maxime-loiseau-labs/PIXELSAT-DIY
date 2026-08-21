# Changelog

All notable changes to the Mini Sat project are documented here.
Format inspired by [Keep a Changelog](https://keepachangelog.com/).

## [1.0.0] - v8 — Full buffer + Lopaka syntax

Refactor of the rendering pipeline to make the UI editable through
[Lopaka](https://lopaka.app) (visual e-paper UI editor).

### Changed
- Image buffer: `g_rowBuf[10]` (row-by-row) → `g_imgBuf[1280]` (full frame).
- UI code (`buildRow` → `drawUI`) called once per refresh, not 128 times.
- UI written in pure Lopaka syntax with the `display.` prefix:
  - `display.drawRoundRect(x, y, w, h, r, color)`
  - `display.setFont(&Org_01)` / `display.setCursor(x, y)` / `display.print(...)`
  - Compatibility aliases: `GxEPD_BLACK`, `GxEPD_WHITE`.
- Data formatting isolated in `prepareUIData()`, called before `drawUI()`.
- `displayPartial()` now calls `drawUI()` twice (one per bit convention)
  to produce the differential image the UC8175 controller expects.

### Removed
- Layout `#define`s (`BAR_RECT_X`, `BOX_DATE_Y`, etc.) — coordinates now
  live directly in `drawUI()` as in the Lopaka-generated code.
- Helpers `formatHHMM`, `formatSecondaryTime`, `formatDay2`, `formatYear4`,
  `formatPct`, `computeBarsCount` — inlined into `prepareUIData()`.
- Unused `IMG_VOLTAGE` bitmap.

### Memory impact
- SRAM: ~150 B → ~1430 B (~70% of ATtiny1616's 2 KB).
- Flash: equivalent to v7 (~12-13 KB).

---

## [0.7.0] - v7 — Automatic European DST

### Added
- Automatic European DST handling. RTC always stores winter time (CET).
  Display code adds +1h when current date falls in summer time (last
  Sunday of March 02:00 → last Sunday of October 03:00).
- `applyDSTOffset()` runs after every `readRTC()` in the loop.
- `g_hourCET` keeps CET hour for correct Delhi (UTC+5:30) calculation
  year-round.
- Partial refresh counter (0–9) displayed in the date box.

### Fixed
- `PORTB.INTFLAGS = PIN4_bm` written before each `rtcIntAttach()` to
  prevent double partial refresh on the next wake.

---

## [0.6.0] - v6 — Auto-set RTC from build time

### Added
- Auto-set RTC from compiler's `__DATE__` / `__TIME__` macros. A hash of
  the build strings is stored in EEPROM (4 bytes). On boot the firmware
  compares the stored hash to the current build's hash:
  - Mismatch → new firmware, write build time to RTC, update hash.
  - Match    → RTC already initialised by this firmware, skip.
- `BUILD_TIME_OFFSET_S` (default 10 s) compensates for the compile+flash
  delay between `__TIME__` capture and actual boot.

### Changed
- EPD GPIOs moved to PORTC (PC0=BUSY, PC1=RST, PC2=DC, PC3=CS) on
  ATtiny1616 to make pin assignment cleaner.

---

## [0.5.0] - v5 — Power-saving state machine

### Added
- Two-state machine `STATE_NORMAL` / `STATE_SLEEPING`.
- Hysteresis on VCC: enter SLEEPING when `VCC < V_OFF` (default 2300 mV),
  return to NORMAL when `VCC >= V_OFF + 150 mV`. Prevents fast oscillation
  around the threshold.
- "SLEEP" full-refresh screen with timestamp of when the system went to
  sleep (displayed under the SLEEP text).
- VCC trend detection: charging/discharging arrow based on moving average
  over the last 5 minutes (with 30 mV hysteresis).

### Fixed
- Wake time reduced from 8–10 s to ~3 s (redundant `epdDeepSleep` call
  removed).
- Current spikes 12–15 mA fixed: `peripheralsStart/Stop` moved inside the
  RTC-wake branch so SPI/Wire are no longer reinitialised on every PIT
  tick (1 Hz).

---

## [0.4.0] - v4 — Port to ATtiny1616

### Changed
- Migration from ATmega328P (Arduino UNO dev) to ATtiny1616 (production
  target, SOIC-20, 2 KB SRAM, 16 KB Flash).
- ADC: ATtiny ADC0 type B implementation (`VREF.CTRLA`, `ADC0.CTRLC`,
  `ADC0.MUXPOS = INTREF`, etc.).
- Watchdog: WDT interrupt mode replaced by RTC.PIT at 1 Hz on ATtiny1616
  (the tinyAVR WDT is reset-only).
- Sleep: `SLPCTRL.CTRLA = SMODE_PDOWN | SEN` for tinyAVR.
- External interrupt on PB4 via `PORTB.PIN4CTRL = ISC_LEVEL | PULLUPEN`
  (only level interrupts wake from PDOWN on tinyAVR).
- Bandgap reference calibrated for the specific chip: `BANDGAP_MV = 1103`.

### Added
- Conditional compilation `__AVR_ATmega328P__` vs `MEGATINYCORE` for both
  targets in the same source tree.
- SHT31 temperature/humidity driver (~150 B Flash, single-shot at every
  RTC wake, no external lib).

### Fixed (v4.1)
- EPD wait loops switched from busy-wait `delay()` to `epdIdleMs()` using
  `SLEEP_MODE_IDLE` (CPU sleeps but SPI keeps running). Saves ~3 mA × 2 s
  per refresh.
- SPI and Wire are stopped before each sleep and restarted on each RTC
  wake, freeing the bus pins (saves ~50–150 µA in deep sleep).

---

## [0.3.0] - v3 — Display robustness fixes

### Fixed
- Font upgraded from Font8 (5×8px) to Font16 (11×16px) for the counter
  digits. Font8 was too small for partial refresh to drive the particles
  properly, causing blurry digits.
- Black frame artefact at the panel edges fixed by changing the VBD bits
  of register `0x50` from `0xf2` (VBD=11 → -VS = solid black border) to
  `0x72` (VBD=01 → follows LUT_VCOM = neutral).
- Mandatory `epdDeepSleep(0x07 + 0xA5)` after every refresh cycle
  (Waveshare manual requirement, prevents long-term panel damage).
- Partial refresh: pass-1 inversion ("shakes" the particles for reliable
  small-text rendering).

---

## [0.2.0] - v2 — Event-driven architecture

### Changed
- Replaced `delay(60000)` polling loop with event-driven design.
- MCU stays in `SLEEP_MODE_PWR_DOWN` between refreshes (~µA range).
- Wake-up triggered by PCF8563T `INT` line (open-drain, active-LOW) on
  pin INT0 (D2 on UNO) — only level interrupts wake from PWR_DOWN.
- PCF8563T configured for periodic interrupts: timer in countdown mode,
  1 Hz × 60 (one IRQ per minute).

### Added
- Software watchdog: 8 s WDT-interrupt + software counter on ATmega328P.
  If RTC fails to wake the MCU within ~80 s (≈10 ticks), the WDT forces
  a hardware reset. Recovers from I²C lock-up and stuck `TF` flag.
- Centring of the main clock on the colon character (`:` always at x=40)
  to handle Org_01's variable glyph widths (e.g. `1`=2px, others=6px).
- 3 px wide arrow shafts for the charge status icon (was 1 px, hard to
  see at 80×128).

---

## [0.1.0] - v1 — Initial UI

### Added
- Lopaka-inspired UI for the 1.02" e-paper:
  - Top charge bar with percentage, 15 segments, charging/discharging
    arrow icon.
  - Large HH:MM clock (Org_01 font at size 3).
  - Date box with day number (white-on-black), month abbreviation,
    weekday name, year, and secondary timezone (Delhi by default).
  - Bottom temperature and humidity boxes with degree mark.
- Custom Org_01 font embedded (95 glyphs in PROGMEM).
- Minimal GFX 5×7 font reduced to digits + `%` (55 B Flash vs 1280 B
  for the full glcdfont).
- Row-by-row rendering with a 10 B buffer (zero image buffer in SRAM).
- PCF8563T RTC read every minute via I²C.
- Sakamoto's algorithm for day-of-week (independent of the PCF8563
  weekday register, which is not guaranteed correct).
