#ifndef APP_PORT_H
#define APP_PORT_H

#include "app_types.h"

typedef enum {
    APP_ASYNC_BUSY = 0,
    APP_ASYNC_OK,
    APP_ASYNC_FAILED
} AppAsyncResult;

typedef enum {
    APP_LED_IDLE = 0,
    APP_LED_WORKING,
    APP_LED_WARNING,
    APP_LED_CHARGING,
    APP_LED_FULL,
    APP_LED_FAULT
} AppLedState;

typedef struct {
    uint32_t (*now_ms)(void);
    void (*safe_outputs_off)(void);
    bool (*heater_control)(float target_c, float *measured_c);
    bool (*pressure_control_start)(float target_mmhg);
    bool (*pressure_control_step)(float target_mmhg, float *measured_mmhg);
    void (*pressure_control_stop)(void);
    bool (*home_begin)(void);
    AppAsyncResult (*home_poll)(void);
    void (*home_cancel)(void);
    bool (*pressure_zero_calibrate)(void);
    AppEyeState (*eye_read_state)(void);
    bool (*eye_mark_consumed)(void);
    bool (*power_read)(bool *charging, bool *full, uint16_t *soc, uint16_t *millivolts);
    void (*power_latch_off)(void);
    void (*led_set)(AppLedState state, bool blink);
    void (*counter_increment)(AppMode mode);
    void (*screen_boot_sync)(void);
    uint16_t (*storage_read_u16)(uint8_t address, uint16_t default_value);
    bool (*storage_write_u16)(uint8_t address, uint16_t value);
    void (*storage_erase_main)(void);
    void (*storage_erase_eye)(void);
    void (*screen_float)(uint16_t command, float value);
    void (*screen_u16)(uint16_t command, uint16_t value);
    void (*screen_u32)(uint16_t command, uint32_t value);
    void (*fault_report)(AppFault fault);
} AppPort;

#endif
