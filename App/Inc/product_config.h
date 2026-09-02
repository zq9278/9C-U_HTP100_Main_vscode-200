#ifndef PRODUCT_CONFIG_H
#define PRODUCT_CONFIG_H

/* Product identity. PRODUCT_VERSION_NUMBER is sent to the LCD and tuning host. */
#define PRODUCT_MODEL_NAME          "9C-HTP100"
#define PRODUCT_VERSION_NUMBER      20260828UL

/* Add this value to the heater control setpoint and subtract it from the LCD's
 * measured-temperature display. Safety checks and tuning-host telemetry keep
 * using the uncompensated sensor measurement. */
#define PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C  0.5f

/* Test only: fill the complete main-board EEPROM with 0xFF on every boot,
 * then run the normal new-machine initialization. Set to 0U for production. */
#ifndef PRODUCT_MAIN_EEPROM_FILL_FF_ON_BOOT
#define PRODUCT_MAIN_EEPROM_FILL_FF_ON_BOOT  1U
#endif

#if PRODUCT_MAIN_EEPROM_FILL_FF_ON_BOOT != 0U && \
    PRODUCT_MAIN_EEPROM_FILL_FF_ON_BOOT != 1U
#error "PRODUCT_MAIN_EEPROM_FILL_FF_ON_BOOT must be 0U or 1U"
#endif

/* 1U: consume/fuse an eye shield at the first formal treatment actuation.
 * 0U: do not write or enforce the eye-shield consumed marker. */
#ifndef PRODUCT_EYE_FUSE_ENABLED
#define PRODUCT_EYE_FUSE_ENABLED    0U
#endif

#if PRODUCT_EYE_FUSE_ENABLED != 0U && PRODUCT_EYE_FUSE_ENABLED != 1U
#error "PRODUCT_EYE_FUSE_ENABLED must be 0U or 1U"
#endif

#endif
