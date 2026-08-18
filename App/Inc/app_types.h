#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_MODE_NONE = 0,
    APP_MODE_HEAT,
    APP_MODE_PRESSURE,
    APP_MODE_AUTO
} AppMode;

typedef enum {
    APP_STATE_BOOT = 0,
    APP_STATE_HOMING,
    APP_STATE_IDLE,
    APP_STATE_PREHEAT,
    APP_STATE_READY,
    APP_STATE_RUNNING,
    APP_STATE_STOPPING,
    APP_STATE_FAULT,
    APP_STATE_SHUTDOWN
} AppState;

typedef enum {
    APP_EYE_ABSENT = 0,
    APP_EYE_NEW,
    APP_EYE_IN_USE,
    APP_EYE_CONSUMED,
    APP_EYE_SERVICE
} AppEyeState;

typedef enum {
    APP_STOP_NONE = 0,
    APP_STOP_USER,
    APP_STOP_NATURAL,
    APP_STOP_FAULT,
    APP_STOP_POWER_LOSS,
    APP_STOP_CHARGING,
    APP_STOP_EYE_REMOVED,
    APP_STOP_LOW_BATTERY
} AppStopReason;

typedef enum {
    APP_FAULT_NONE = 0x0000,
    APP_FAULT_TMP112_COMM = 0x0101,
    APP_FAULT_MOTOR_NOT_FOUND = 0x0201,
    APP_FAULT_MOTOR_COMM = 0x0202,
    APP_FAULT_MOTOR_HOME = 0x0203,
    APP_FAULT_PRESSURE_SENSOR = 0x0301,
    APP_FAULT_PRESSURE_COMM = 0x0302,
    APP_FAULT_PRESSURE_ZERO = 0x0303,
    APP_FAULT_OVER_TEMPERATURE = 0x0401,
    APP_FAULT_OVER_PRESSURE = 0x0402,
    APP_FAULT_SCREEN_TIMEOUT = 0x0501
} AppFault;

typedef struct {
    float temperature_c;
    float pressure_mmhg;
    uint16_t runtime_minutes;
} AppTreatmentSettings;

typedef struct {
    AppState state;
    AppMode mode;
    AppEyeState eye;
    AppStopReason stop_reason;
    AppFault fault;
    AppTreatmentSettings settings;
    bool charging;
    bool low_battery_warning;
    bool home_valid;
    bool pressure_zero_valid;
    uint16_t battery_soc;
    uint16_t battery_mv;
} AppSnapshot;

#endif
