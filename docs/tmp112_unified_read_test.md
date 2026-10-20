# TMP112 unified-read bench build

Branch: `test/tmp112-unified-read`; base: `0114475`, before the <=0 C
shutdown change. There is NO nonpositive-temperature protection in this build.
Disconnect heater power physically before reproducing the sensor GND fault.
The existing communication-failure confirmation window and overtemperature
logic remain as in the base version. Zero readings can drive PID output high
and cancel the existing overtemperature timer.

## Unified acquisition

`tmp112_read_register()` is used by control, telemetry and register diagnostics.
All use the existing I2C2 read helper: 50 ms timeout per attempt, up to two
attempts, 2 ms delay before retry, and the same bus-recovery conditions.
Diagnostics no longer use a direct HAL call with different retry behavior.

Every physical TMP112 read attempt prints `[TMP112 READ]` on USART1 at
115200 8N1, including register, attempt (1 or 2), HAL status, error mask and
raw bytes on success. A failed transfer prints `raw=INVALID`; this is neither
zero nor FF. Recovery is reported by existing `[I2C2]` logs.

Register mapping: 00=TEMP, 01=CONFIG, 02=TLOW, 03=THIGH.
HAL status: 0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT. Error 0x04 is ACK failure.

Approximately every second, after the main control tick, `[TMP112 DIAG]`
references the latest temperature attempt from control/telemetry, including
its sample timestamp, age and final result. Failed attempts invalidate the
cache; old successful bytes are never presented as the failed sample.
If no temperature attempt occurred in the last 500 ms, diagnostics acquire a
new temperature using the same helper. Registers 01..03 are then read using
that helper. These register reads are sequential, not simultaneous samples.
The EEPROM flag is the last eye probe/identity result, not a new EEPROM read
at the snapshot timestamp. A diagnostic read is never used to clear a fault
or automatically restart treatment.

## Test

1. Physically disconnect heater power, preserving main-board/logger ground.
2. Flash `build/Debug/9C-HTP100-tmp112-UNIFIED-READ-NO-LOW-TEMP-STOP.hex`.
3. Verify the boot marker `UNIFIED READ BENCH BUILD; no <=0C shutdown protection`.
4. Capture normal operation, sensor GND interruption and recovery in one log.
5. Inspect `reg=0x00` attempts immediately preceding a zero `[Temp]` value.
   Compare first-attempt failures, recovery, second-attempt bytes, and the
   diagnostic cached timestamp. Also compare registers 01..03.

This firmware actively changes bus traffic and timing: per-attempt UART logs
and three additional register reads can delay the main loop, especially on
failures/recovery. It is a bench diagnostic, not production firmware. Reading
sensor registers can acknowledge an alert in interrupt mode. No configuration
or threshold register writes are added.
