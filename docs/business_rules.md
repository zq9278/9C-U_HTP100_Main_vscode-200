# Confirmed business rules

## Eye shield

- A shield is usable only when TMP112 responds and its EEPROM usage marker is `0xFFFF`.
- Preheating does not consume the shield.
- When formal heat or pressure treatment is started, the marker is immediately
  written as consumed and verified by readback.
- The current uninterrupted insertion session remains usable after the marker is
  written. The current treatment may finish, and another treatment may start after
  homing without writing the marker again.
- Once removal is observed, reinserting that shield exposes its consumed marker and
  treatment is rejected. A controller restart also reloads the marker, so an already
  marked shield is rejected after restart even when it was not physically removed.
- The service-shield marker `0x0202` remains exempt for production/service work.

## Countdown compatibility

- The screen continues to own the countdown.
- Runtime from prepare command `0x1041` is stored and displayed but is not used by
  the main board to generate a second countdown.
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
- The current low-battery warning threshold is retained: BQ27441 SOC <= 1%.
- Warning is shown by flashing WS2812 LEDs.
- The current shutdown threshold is retained: voltage <= 3150 mV for five consecutive
  one-second checks while external power is absent.
- Critical shutdown first attempts home, then writes BQ25895 register `0x09 = 0x64`.

## Safety ownership

- Only the application task may operate TMC5130, ADS1220, TMP112, heater PWM,
  treatment state, and homing state.
- Interrupts only enqueue/capture UART bytes and set button/power-loss flags.
- No application task is dynamically deleted, suspended, or resumed.
- A treatment cannot start unless home and the unloaded ADS1220 zero are valid.
- Temperature >= 44 C and pressure >= 650 mmHg independently stop treatment and latch a fault.
