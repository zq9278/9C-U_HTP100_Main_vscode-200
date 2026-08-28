# Business-logic tests

`test_app_controller.c` tests the application state machine without STM32 HAL:

- preheating does not consume a new eye shield;
- formal treatment consumes it exactly once;
- the same uninterrupted insertion session can start another treatment;
- removal followed by reinsertion exposes the consumed marker and blocks treatment;
- only a natural finish increments the treatment counter;
- successful heat sampling exposes a valid measured temperature for telemetry,
  and homing clears that validity;
- user stop, charging, fault, power loss and critical battery all enter homing;
- external power blocks treatment;
- critical battery performs homing before requesting the hardware power latch off.
- a user/natural stop can cooperatively cancel homing and reuse the previous
  valid pressure zero for a quick second treatment;
- fault and power-related stops cannot use that quick-resume path.

The firmware build compiles the same `App/Src/app_controller.c`. A native C compiler
can run this test by compiling the test together with that source and including
`App/Inc`.
