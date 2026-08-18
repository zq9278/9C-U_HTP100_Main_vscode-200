# 9C-U HTP100 main-control firmware v2

This is a clean sibling project for the STM32G070CBT6 main-control board. It
reuses the CubeMX peripheral initialization and STM32 HAL, but does not reuse
the old `UserApp`, `Button`, `communication`, task-delete, or shared-global
application architecture.

## Implemented baseline

- one application owner for treatment state and all actuator transitions;
- heat, pressure, and automatic treatment modes;
- non-blocking TMC5130 homing owned by the application task;
- ADS1220 DRDY wait, register readback, and unloaded zero calibration after home;
- TMP112 temperature control with an independent 44 C cutoff;
- pressure control with an independent 650 mmHg cutoff;
- current 5AA5/6AA6 screen framing, CRC, commands, telemetry, presets, and screen-owned countdown;
- new-eye-shield consumption at the first formal treatment start;
- treatment prohibited whenever valid external charging power is present;
- low-battery warning at SOC <= 1%, then shutdown at <= 3150 mV after five confirmations;
- user stop, natural finish, fault, power loss, charging insertion, critical battery,
  and eye removal all pass through safe output stop and motor homing;
- only natural finish (`0x8900`) increments a persistent treatment counter;
- J-Link and ST-Link launch profiles without RTOS awareness.

The exact business decisions are recorded in [docs/business_rules.md](docs/business_rules.md).

## Build

```powershell
cmake --preset Debug
cmake --build build/Debug --parallel
```

Outputs are created in `build/Debug`:

- `9C-HTP100-main-v2.elf`
- `9C-HTP100-main-v2.hex`
- `9C-HTP100-main-v2.bin`

The current Debug build uses about 70 KB flash and 20 KB RAM.

## Hardware validation gate

This source compiles, but it must not be used for treatment before bench testing:

1. verify motor directions and the REFR home switch without a patient/load;
2. verify ADS1220 polarity, pressure coefficient, and all five pressure levels;
3. verify heater polarity and the independent over-temperature cutoff;
4. verify eye EEPROM marking and replacement count across power cycles;
5. verify plug-in charging interrupts a running treatment and blocks restart;
6. verify SOC warning and five-sample 3150 mV shutdown behavior;
7. verify the power-hold circuit supplies enough energy to complete homing after
   `PWR_SENSE`; firmware cannot home after electrical power has already collapsed;
8. run the endurance host and manually exercise every screen command.

The old protocol and storage workbooks are retained under `docs/` as migration
references. The v2 application code is under `App/`; board-specific code is
isolated under `Bsp/`.
