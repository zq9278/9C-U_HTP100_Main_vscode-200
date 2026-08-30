#ifndef PRODUCT_CONFIG_H
#define PRODUCT_CONFIG_H

/* Product identity. PRODUCT_VERSION_NUMBER is sent to the LCD and tuning host. */
#define PRODUCT_MODEL_NAME          "9C-HTP100"
#define PRODUCT_VERSION_NUMBER      20260828UL

/* Factory branch: inserting any confirmed I2C eye shield starts a heater
 * temperature-rise test. This bypasses the normal treatment state machine. */
#ifndef PRODUCT_I2C_EYE_TEST_ENABLED
#define PRODUCT_I2C_EYE_TEST_ENABLED          1U
#endif
#define PRODUCT_I2C_EYE_TEST_SAMPLE_PERIOD_MS 500U
#define PRODUCT_I2C_EYE_TEST_REQUIRED_RISES   3U
#define PRODUCT_I2C_EYE_TEST_PASS_TEMP_C       30.0f
#define PRODUCT_I2C_EYE_TEST_TIMEOUT_MS        10000U

/* 1U: consume/fuse an eye shield at the first formal treatment actuation.
 * 0U: do not write or enforce the eye-shield consumed marker. */
#ifndef PRODUCT_EYE_FUSE_ENABLED
#define PRODUCT_EYE_FUSE_ENABLED    0U
#endif

#if PRODUCT_EYE_FUSE_ENABLED != 0U && PRODUCT_EYE_FUSE_ENABLED != 1U
#error "PRODUCT_EYE_FUSE_ENABLED must be 0U or 1U"
#endif

#if PRODUCT_I2C_EYE_TEST_ENABLED != 0U && PRODUCT_I2C_EYE_TEST_ENABLED != 1U
#error "PRODUCT_I2C_EYE_TEST_ENABLED must be 0U or 1U"
#endif

#if PRODUCT_I2C_EYE_TEST_ENABLED && PRODUCT_EYE_FUSE_ENABLED
#error "I2C eye test mode must never consume an eye shield"
#endif

#endif
