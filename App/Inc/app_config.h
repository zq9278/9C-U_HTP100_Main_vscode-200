#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_PREHEAT_TARGET_C              40.0f
#define APP_DEFAULT_TREATMENT_TEMP_C      42.5f
#define APP_MAX_DISPLAY_TEMPERATURE_C     42.9f
#define APP_MAX_SAFE_TEMPERATURE_C        44.0f
#define APP_OVER_TEMPERATURE_CONFIRM_MS   5000U
#define APP_MAX_SAFE_PRESSURE_MMHG        650.0f
#define APP_OVER_PRESSURE_CONFIRM_SAMPLES 3U

/* Measured effective compression area. Pressure is derived from load-cell
 * force with 1 mmHg = 133.322 Pa and area expressed in mm^2. */
#define APP_PRESSURE_EFFECTIVE_AREA_MM2    84.8f
#define APP_PASCAL_PER_MMHG                133.322f
#define APP_STANDARD_GRAVITY_M_S2          9.80665f

/* Keep the thresholds used by the current released logic. */
#define APP_LOW_BATTERY_WARNING_SOC       1U
#define APP_LOW_BATTERY_SHUTDOWN_MV       3150U
#define APP_LOW_BATTERY_CONFIRM_SAMPLES   5U

#define APP_HEAT_CONTROL_PERIOD_MS        150U
#define APP_TEMP_COMM_FAULT_CONFIRM_MS    400U
#define APP_PRESSURE_CONTROL_PERIOD_MS    20U
#define APP_PRESSURE_SENSOR_CHECK_MS      1000U
/* ADS1220 BCS diagnostic limits. Validate against the production bridge and
 * input RC network; healthy BCS samples must remain between these limits. */
#define APP_PRESSURE_SHORT_RAW_LIMIT      100000L
#define APP_PRESSURE_OPEN_RAW_LIMIT       8200000L
#define APP_TEMP_DISPLAY_PERIOD_MS        300U
#define APP_TEMP_DISPLAY_FILTER_ALPHA     0.25f
#define APP_PRESSURE_DISPLAY_PERIOD_MS    250U
#define APP_SENSOR_LOG_PERIOD_MS          1000U
#define APP_BATTERY_POLL_PERIOD_MS        100U
#define APP_EYE_POLL_PERIOD_MS            100U
#define APP_EYE_INSERT_CONFIRM_SAMPLES    5U
#define APP_EYE_REMOVE_CONFIRM_SAMPLES    3U
#define APP_EYE_STARTUP_ABSENT_SAMPLES    15U
#define APP_SCREEN_TIMEOUT_MS             2000U
#define APP_HOME_TIMEOUT_MS               8000U

/* Set to 1U to send new-screen fault status frames (0x2F01) to the LCD. */
#define APP_ENABLE_SCREEN_FAULT_REPORT    1U

#endif
