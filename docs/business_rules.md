# Confirmed business rules

## Eye shield

- Product identity, version and the eye-shield fuse switch are centralized in
  `App/Inc/product_config.h`. `PRODUCT_EYE_FUSE_ENABLED=1U` keeps the production
  consumed-marker behavior; `0U` neither writes nor enforces that marker.
- With eye-shield fusing enabled, a shield is usable only when TMP112 responds
  and its EEPROM usage marker is `0xFFFF`.
- Preheating does not consume the shield.
- When formal heat or pressure treatment is started, the marker is immediately
  written as consumed and verified by readback.
- The current uninterrupted insertion session remains usable after the marker is
  written. The current treatment may finish, and another treatment may start after
  homing without writing the marker again.
- Once removal is observed, reinserting that shield exposes its consumed marker and
  treatment is rejected. The LCD receives the same eye-shield-offline value for a
  consumed shield as for a physically absent shield. A controller restart also reloads
  the marker, so an already marked shield is rejected after restart even when it was
  not physically removed.
- The service-shield marker `0x0202` remains exempt for production/service work.

## Countdown compatibility

- The screen continues to own the countdown.
- A blank main-board EEPROM is initialized once with Chinese at `0x06`, sound
  enabled at `0xF8`, preset 1/2/3 pressures of 250/350/450 mmHg, a 2-minute
  runtime for all presets, and preset 1 selected. The commit marker at `0xFE`
  is written last.
  Later boots preserve the preset selection and values written by the screen.
- `PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM=1U` fills the complete main-board I2C
  EEPROM with `0xFF` once after each firmware download, then writes new-machine defaults.
  Ordinary power cycles preserve the EEPROM. The eye-shield EEPROM is not affected.
- Runtime from prepare command `0x1041` is stored and displayed but is not used by
  the main board to generate a second countdown.
- A confirmed in-use eye shield has a 1-second I2C reconnect window. During a
  transient loss the treatment outputs are paused, the LCD continues to see the
  eye as online, and the consumed marker does not invalidate the current insertion
  session. Continuous loss beyond the window is reported as eye offline.
- Screen command `0x8900` is the authoritative natural-finish event.
- A following legacy mode-stop command is accepted harmlessly while homing.

## Stop and count matrix

| Cause | Immediate heater/motor safe stop | Home | Count | Final action |
|---|---:|---:|---:|---|
| User stop | yes | yes | no | idle when home succeeds |
| Natural finish (`0x8900`) | yes | yes | yes, once | idle when home succeeds |
| Fault | yes | yes | no | fault remains latched |
| Power-loss signal | yes | yes | no | request hardware power latch off after home |
| Charging inserted | yes | yes | no | remain powered, treatment blocked |
| Critical battery | yes | yes | no | request hardware power latch off after home |
| Eye removed | yes | yes | no | idle, but treatment remains blocked |

## Charging and battery

- Any valid external input reported by BQ25895, including charge-done, blocks treatment.
- Inserting charging power during preparation or treatment aborts and homes the mechanism.
- Low-battery warning is raised when BQ27441 SOC is below 10%.
- Warning is shown by flashing WS2812 LEDs.
- While formal treatment is RUNNING, low-voltage samples do not stop treatment
  or request automatic shutdown; the confirmation count is reset on each poll.
  Low-battery indications remain active.
- Outside RUNNING (including preheat, idle and homing), voltage <= 3150 mV for
  five consecutive power checks requests shutdown while external power is absent.
  Checks are scheduled every 100 ms; device I/O can lengthen the actual interval.
  After treatment stops, confirmation starts afresh. A real power-loss input,
  external-power interlock and fault stops remain active during treatment.
- Critical shutdown first attempts home, then writes BQ25895 register `0x09 = 0x64`.

## Trigger button

- A press must remain stable for at least 30 ms before producing one action.
- Holding the trigger does not repeat. A stable release of at least 30 ms is
  required before another press can act; falling edges during press confirmation
  restart that confirmation to reject bounce between main-loop polls.
- Debouncing is nonblocking and runs in the application task. Interrupts only
  record an edge hint; physical pin levels and elapsed time determine events.

## Safety ownership

- Only the application task may operate TMC5130, ADS1220, TMP112, heater PWM,
  treatment state, and homing state.
- Interrupts only enqueue/capture UART bytes and set button/power-loss flags.
- No application task is dynamically deleted, suspended, or resumed.
- A treatment cannot start unless home and the unloaded ADS1220 zero are valid.
- Overpressure fault 0x0402 uses unfiltered pressure >= 700 mmHg for at least
  900 ms from the first over-limit sample, confirmed on a fresh sample. A valid
  below-limit sample or acquisition failure resets the timer. The application
  backup check and motor recovery pressure check use the same timing rule;
  the motor remains disabled during recovery confirmation. Nonfinite recovery
  pressure remains an immediate rejection. Actual response is at the first
  eligible pressure check after 900 ms, not a fixed number of samples.
- Pressure target commands retain their existing upper bound of 650 mmHg.
- Temperature >= 44 C retains its independent overtemperature protection.
