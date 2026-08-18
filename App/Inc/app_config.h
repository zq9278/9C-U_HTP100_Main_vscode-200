#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_PREHEAT_TARGET_C              40.0f
#define APP_DEFAULT_TREATMENT_TEMP_C      42.5f
#define APP_MAX_SAFE_TEMPERATURE_C        44.0f
#define APP_MAX_SAFE_PRESSURE_MMHG        650.0f

/* Keep the thresholds used by the current released logic. */
#define APP_LOW_BATTERY_WARNING_SOC       1U
#define APP_LOW_BATTERY_SHUTDOWN_MV       3150U
#define APP_LOW_BATTERY_CONFIRM_SAMPLES   5U

#define APP_HEAT_CONTROL_PERIOD_MS        150U
#define APP_PRESSURE_CONTROL_PERIOD_MS    20U
#define APP_TELEMETRY_PERIOD_MS           400U
#define APP_BATTERY_POLL_PERIOD_MS        1000U
#define APP_EYE_POLL_PERIOD_MS            100U
#define APP_SCREEN_TIMEOUT_MS             2000U
#define APP_HOME_TIMEOUT_MS               8000U

#define APP_SOFTWARE_VERSION              20260818UL

#endif
