# Business-logic tests

`test_app_controller.c` tests the application state machine without STM32 HAL:

- preheating does not consume a new eye shield;
- formal treatment consumes it exactly once;
- only a natural finish increments the treatment counter;
- user stop, charging, fault, power loss and critical battery all enter homing;
- external power blocks treatment;
- critical battery performs homing before requesting the hardware power latch off.

The firmware build compiles the same `App/Src/app_controller.c`. A native C compiler
can run this test by compiling the test together with that source and including
`App/Inc`.
