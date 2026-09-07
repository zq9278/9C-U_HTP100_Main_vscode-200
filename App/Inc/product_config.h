#ifndef PRODUCT_CONFIG_H
#define PRODUCT_CONFIG_H

/* Product identity. PRODUCT_VERSION_NUMBER is sent to the LCD and tuning host. */
#define PRODUCT_MODEL_NAME          "9C-HTP100"
#define PRODUCT_VERSION_NUMBER      20260904UL

/* Add this value to the heater control setpoint and subtract it from the LCD's
 * measured-temperature display. Safety checks and tuning-host telemetry keep
 * using the uncompensated sensor measurement. */
#define PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C  0.5f

/* Reset the complete main-board I2C EEPROM once after each firmware download.
 * Ordinary power cycles and software resets preserve the EEPROM. */
#ifndef PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM
#define PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM  1U
#endif

#if PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM != 0U && \
    PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM != 1U
#error "PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM must be 0U or 1U"
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
