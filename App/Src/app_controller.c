#include "app_controller.h"

#include <stddef.h>
#include <string.h>

#include "app_config.h"

#define SCREEN_TEMP_HEAT             0x2041U
#define SCREEN_TEMP_AUTO             0x2037U
#define SCREEN_PRESSURE              0x2005U
#define SCREEN_PRESSURE_AUTO         0x2047U
#define SCREEN_SOC                   0x2050U
#define SCREEN_WORK_QUIT             0x2051U
#define SCREEN_TIMER_START           0x2052U
#define SCREEN_EYE_STATE             0x2055U
#define SCREEN_TIMER_STOP            0x2056U
#define SCREEN_NEW_EYE               0x2057U
#define SCREEN_FAULT                 0x2070U
#define SCREEN_HOME_COMPLETE         0x20F0U

typedef struct {
    AppSnapshot status;
    const AppPort *port;
    uint32_t last_heat_ms;
    uint32_t last_pressure_ms;
    uint32_t last_telemetry_ms;
    uint8_t low_voltage_samples;
    bool count_recorded;
    bool pending_power_off;
    bool home_started;
} AppContext;

static AppContext g_app;

static uint32_t now_ms(void)
{
    return (g_app.port != NULL && g_app.port->now_ms != NULL) ? g_app.port->now_ms() : 0U;
}

static bool mode_uses_heat(AppMode mode)
{
    return mode == APP_MODE_HEAT || mode == APP_MODE_AUTO;
}

static bool mode_uses_pressure(AppMode mode)
{
    return mode == APP_MODE_PRESSURE || mode == APP_MODE_AUTO;
}

static bool treatment_is_open(void)
{
    return g_app.status.state == APP_STATE_PREHEAT ||
           g_app.status.state == APP_STATE_READY ||
           g_app.status.state == APP_STATE_RUNNING;
}

static void send_float(uint16_t command, float value)
{
    if (g_app.port != NULL && g_app.port->screen_float != NULL) {
        g_app.port->screen_float(command, value);
    }
}

static void set_led(void)
{
    AppLedState led = APP_LED_IDLE;
    bool blink = false;

    if (g_app.port == NULL || g_app.port->led_set == NULL) {
        return;
    }
    if (g_app.status.fault != APP_FAULT_NONE) {
        led = APP_LED_FAULT;
    } else if (g_app.status.charging) {
        led = APP_LED_CHARGING;
    } else if (g_app.status.low_battery_warning) {
        led = APP_LED_WARNING;
        blink = true;
    } else if (treatment_is_open()) {
        led = APP_LED_WORKING;
    }
    g_app.port->led_set(led, blink);
}

static void safe_outputs_off(void)
{
    if (g_app.port == NULL) {
        return;
    }
    if (g_app.port->safe_outputs_off != NULL) {
        g_app.port->safe_outputs_off();
    }
    if (g_app.port->pressure_control_stop != NULL) {
        g_app.port->pressure_control_stop();
    }
}

static void record_natural_finish_once(void)
{
    if (g_app.count_recorded || g_app.status.stop_reason != APP_STOP_NATURAL) {
        return;
    }
    g_app.count_recorded = true;
    if (g_app.port != NULL && g_app.port->counter_increment != NULL) {
        g_app.port->counter_increment(g_app.status.mode);
    }
}

static void begin_home(void)
{
    g_app.status.state = APP_STATE_HOMING;
    g_app.status.home_valid = false;
    g_app.status.pressure_zero_valid = false;
    g_app.home_started = false;

    if (g_app.port != NULL && g_app.port->home_begin != NULL && g_app.port->home_begin()) {
        g_app.home_started = true;
    } else {
        g_app.status.fault = APP_FAULT_MOTOR_COMM;
        g_app.status.state = APP_STATE_FAULT;
        if (g_app.port != NULL && g_app.port->fault_report != NULL) {
            g_app.port->fault_report(APP_FAULT_MOTOR_COMM);
        }
        if (g_app.pending_power_off && g_app.port != NULL && g_app.port->power_latch_off != NULL) {
            g_app.port->power_latch_off();
        }
    }
}

static bool consume_eye_at_first_actuation(void)
{
    if (g_app.status.eye == APP_EYE_SERVICE) {
        return true;
    }
    if (g_app.status.eye != APP_EYE_NEW || g_app.port == NULL ||
        g_app.port->eye_mark_consumed == NULL || !g_app.port->eye_mark_consumed()) {
        AppController_RaiseFault(APP_FAULT_TMP112_COMM);
        return false;
    }
    /* The current treatment may finish, but no second treatment is allowed. */
    g_app.status.eye = APP_EYE_IN_USE;
    if (g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_NEW_EYE, 0.0f);
    }
    return true;
}

void AppController_Init(const AppPort *port)
{
    memset(&g_app, 0, sizeof(g_app));
    g_app.port = port;
    g_app.status.state = APP_STATE_BOOT;
    g_app.status.mode = APP_MODE_NONE;
    g_app.status.eye = APP_EYE_ABSENT;
    g_app.status.settings.temperature_c = APP_DEFAULT_TREATMENT_TEMP_C;
    g_app.status.settings.pressure_mmhg = 350.0f;
    g_app.status.settings.runtime_minutes = 1U;
    safe_outputs_off();
    begin_home();
    set_led();
}

bool AppController_Prepare(AppMode mode, float pressure_mmhg)
{
    if (g_app.status.state != APP_STATE_IDLE || mode == APP_MODE_NONE ||
        g_app.status.charging || g_app.status.fault != APP_FAULT_NONE ||
        !g_app.status.home_valid ||
        (g_app.status.eye != APP_EYE_NEW && g_app.status.eye != APP_EYE_SERVICE)) {
        return false;
    }
    if (mode_uses_pressure(mode) && !g_app.status.pressure_zero_valid) {
        return false;
    }

    g_app.status.mode = mode;
    g_app.status.stop_reason = APP_STOP_NONE;
    g_app.status.fault = APP_FAULT_NONE;
    g_app.count_recorded = false;
    if (pressure_mmhg > 0.0f) {
        g_app.status.settings.pressure_mmhg = pressure_mmhg;
    }

    if (mode_uses_heat(mode)) {
        g_app.status.state = APP_STATE_PREHEAT;
    } else {
        g_app.status.state = APP_STATE_READY;
    }
    set_led();
    return true;
}

bool AppController_Start(void)
{
    if ((g_app.status.state != APP_STATE_PREHEAT && g_app.status.state != APP_STATE_READY) ||
        g_app.status.charging || !g_app.status.home_valid ||
        (mode_uses_pressure(g_app.status.mode) && !g_app.status.pressure_zero_valid)) {
        return false;
    }
    if (mode_uses_pressure(g_app.status.mode) &&
        (g_app.port == NULL || g_app.port->pressure_control_start == NULL ||
         !g_app.port->pressure_control_start(g_app.status.settings.pressure_mmhg))) {
        AppController_RaiseFault(APP_FAULT_MOTOR_COMM);
        return false;
    }
    if (!consume_eye_at_first_actuation()) {
        if (g_app.port != NULL && g_app.port->pressure_control_stop != NULL) {
            g_app.port->pressure_control_stop();
        }
        return false;
    }

    g_app.status.state = APP_STATE_RUNNING;
    g_app.last_heat_ms = now_ms();
    g_app.last_pressure_ms = g_app.last_heat_ms;
    if (g_app.port != NULL && g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_TIMER_START, 0.0f);
    }
    set_led();
    return true;
}

void AppController_Stop(AppStopReason reason)
{
    if (g_app.status.state == APP_STATE_HOMING || g_app.status.state == APP_STATE_SHUTDOWN) {
        return;
    }
    if ((reason == APP_STOP_NATURAL && g_app.status.state != APP_STATE_RUNNING) ||
        (reason == APP_STOP_USER && !treatment_is_open())) {
        return;
    }
    g_app.status.stop_reason = reason;
    if (g_app.status.eye == APP_EYE_IN_USE) {
        g_app.status.eye = APP_EYE_CONSUMED;
    }
    safe_outputs_off();
    record_natural_finish_once();
    if (g_app.port != NULL && g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_TIMER_STOP, 0.0f);
        g_app.port->screen_float(SCREEN_WORK_QUIT, 0.0f);
    }
    if (reason == APP_STOP_POWER_LOSS || reason == APP_STOP_LOW_BATTERY) {
        g_app.pending_power_off = true;
    }
    begin_home();
    set_led();
}

void AppController_RaiseFault(AppFault fault)
{
    if (fault == APP_FAULT_NONE) {
        return;
    }
    g_app.status.fault = fault;
    if (g_app.port != NULL) {
        if (g_app.port->fault_report != NULL) {
            g_app.port->fault_report(fault);
        } else if (g_app.port->screen_u32 != NULL) {
            g_app.port->screen_u32(SCREEN_FAULT, (uint32_t)fault);
        }
    }
    if (g_app.status.state != APP_STATE_HOMING && g_app.status.state != APP_STATE_SHUTDOWN) {
        AppController_Stop(APP_STOP_FAULT);
    }
}

void AppController_SetEyeState(AppEyeState eye)
{
    AppEyeState previous = g_app.status.eye;

    if (previous == APP_EYE_IN_USE && eye != APP_EYE_ABSENT) {
        return;
    }
    g_app.status.eye = eye;
    if (g_app.port != NULL && g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_EYE_STATE, eye != APP_EYE_ABSENT ? 1.0f : 0.0f);
        if (eye == APP_EYE_NEW && previous != APP_EYE_NEW) {
            g_app.port->screen_float(SCREEN_NEW_EYE, 0.0f);
        }
    }
    if (eye == APP_EYE_ABSENT && treatment_is_open()) {
        AppController_Stop(APP_STOP_EYE_REMOVED);
    }
}

void AppController_SetPower(bool charging, uint16_t soc, uint16_t millivolts)
{
    g_app.status.charging = charging;
    g_app.status.battery_soc = soc;
    g_app.status.battery_mv = millivolts;
    g_app.status.low_battery_warning = soc <= APP_LOW_BATTERY_WARNING_SOC;

    if (charging) {
        g_app.low_voltage_samples = 0U;
        if (treatment_is_open()) {
            AppController_Stop(APP_STOP_CHARGING);
        }
    } else if (millivolts != 0U && millivolts <= APP_LOW_BATTERY_SHUTDOWN_MV) {
        if (g_app.low_voltage_samples < APP_LOW_BATTERY_CONFIRM_SAMPLES) {
            g_app.low_voltage_samples++;
        }
        if (g_app.low_voltage_samples >= APP_LOW_BATTERY_CONFIRM_SAMPLES &&
            g_app.status.state != APP_STATE_SHUTDOWN) {
            if (g_app.status.state == APP_STATE_HOMING) {
                g_app.status.stop_reason = APP_STOP_LOW_BATTERY;
                g_app.pending_power_off = true;
            } else {
                AppController_Stop(APP_STOP_LOW_BATTERY);
            }
        }
    } else {
        g_app.low_voltage_samples = 0U;
    }
    send_float(SCREEN_SOC, (float)soc);
    set_led();
}

void AppController_NotifyPowerLoss(void)
{
    if (g_app.status.state == APP_STATE_HOMING) {
        g_app.status.stop_reason = APP_STOP_POWER_LOSS;
        g_app.pending_power_off = true;
    } else if (g_app.status.state != APP_STATE_SHUTDOWN) {
        AppController_Stop(APP_STOP_POWER_LOSS);
    }
}

void AppController_SetTemperature(float temperature_c)
{
    if (temperature_c > 0.0f) {
        g_app.status.settings.temperature_c = temperature_c;
    }
}

void AppController_SetPressure(float pressure_mmhg)
{
    if (pressure_mmhg > 0.0f) {
        g_app.status.settings.pressure_mmhg = pressure_mmhg;
    }
}

void AppController_SetRuntime(uint16_t minutes)
{
    if (minutes > 0U) {
        /* The existing screen remains the countdown owner. */
        g_app.status.settings.runtime_minutes = minutes;
    }
}

void AppController_ScreenBoot(void)
{
    if (g_app.port != NULL && g_app.port->screen_boot_sync != NULL) {
        g_app.port->screen_boot_sync();
    }
}

uint16_t AppController_StorageRead(uint8_t address, uint16_t default_value)
{
    return g_app.port != NULL && g_app.port->storage_read_u16 != NULL ?
           g_app.port->storage_read_u16(address, default_value) : default_value;
}

bool AppController_StorageWrite(uint8_t address, uint16_t value)
{
    return g_app.port != NULL && g_app.port->storage_write_u16 != NULL &&
           g_app.port->storage_write_u16(address, value);
}

void AppController_StorageEraseMain(void)
{
    if (g_app.port != NULL && g_app.port->storage_erase_main != NULL) {
        g_app.port->storage_erase_main();
    }
}

void AppController_StorageEraseEye(void)
{
    if (g_app.port != NULL && g_app.port->storage_erase_eye != NULL) {
        g_app.port->storage_erase_eye();
    }
}

static void tick_homing(void)
{
    AppAsyncResult result;

    if (!g_app.home_started || g_app.port == NULL || g_app.port->home_poll == NULL) {
        return;
    }
    result = g_app.port->home_poll();
    if (result == APP_ASYNC_BUSY) {
        return;
    }
    g_app.home_started = false;
    if (result == APP_ASYNC_FAILED) {
        g_app.status.fault = APP_FAULT_MOTOR_HOME;
        if (g_app.port->fault_report != NULL) {
            g_app.port->fault_report(APP_FAULT_MOTOR_HOME);
        }
        if (g_app.port->screen_float != NULL) {
            g_app.port->screen_float(SCREEN_HOME_COMPLETE, 0.0f);
        }
        if (g_app.pending_power_off && g_app.port->power_latch_off != NULL) {
            g_app.port->power_latch_off();
        }
        g_app.status.state = APP_STATE_FAULT;
        set_led();
        return;
    }

    g_app.status.home_valid = true;
    g_app.status.pressure_zero_valid = g_app.port->pressure_zero_calibrate != NULL &&
                                       g_app.port->pressure_zero_calibrate();
    if (g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_HOME_COMPLETE, 1.0f);
    }
    if (g_app.pending_power_off) {
        g_app.status.state = APP_STATE_SHUTDOWN;
        if (g_app.port->power_latch_off != NULL) {
            g_app.port->power_latch_off();
        }
        return;
    }
    g_app.status.mode = APP_MODE_NONE;
    g_app.status.state = g_app.status.fault == APP_FAULT_NONE ? APP_STATE_IDLE : APP_STATE_FAULT;
    set_led();
}

static void tick_heat(uint32_t now)
{
    float measured = 0.0f;
    float target;

    if (!mode_uses_heat(g_app.status.mode) ||
        (g_app.status.state != APP_STATE_PREHEAT && g_app.status.state != APP_STATE_RUNNING) ||
        now - g_app.last_heat_ms < APP_HEAT_CONTROL_PERIOD_MS) {
        return;
    }
    g_app.last_heat_ms = now;
    target = g_app.status.state == APP_STATE_PREHEAT ? APP_PREHEAT_TARGET_C :
                                                     g_app.status.settings.temperature_c;
    if (g_app.port == NULL || g_app.port->heater_control == NULL ||
        !g_app.port->heater_control(target, &measured)) {
        AppController_RaiseFault(APP_FAULT_TMP112_COMM);
        return;
    }
    if (measured >= APP_MAX_SAFE_TEMPERATURE_C) {
        AppController_RaiseFault(APP_FAULT_OVER_TEMPERATURE);
        return;
    }
    if (now - g_app.last_telemetry_ms >= APP_TELEMETRY_PERIOD_MS) {
        send_float(g_app.status.mode == APP_MODE_AUTO ? SCREEN_TEMP_AUTO : SCREEN_TEMP_HEAT, measured);
    }
}

static void tick_pressure(uint32_t now)
{
    float measured = 0.0f;

    if (!mode_uses_pressure(g_app.status.mode) || g_app.status.state != APP_STATE_RUNNING ||
        now - g_app.last_pressure_ms < APP_PRESSURE_CONTROL_PERIOD_MS) {
        return;
    }
    g_app.last_pressure_ms = now;
    if (g_app.port == NULL || g_app.port->pressure_control_step == NULL ||
        !g_app.port->pressure_control_step(g_app.status.settings.pressure_mmhg, &measured)) {
        AppController_RaiseFault(APP_FAULT_PRESSURE_COMM);
        return;
    }
    if (measured >= APP_MAX_SAFE_PRESSURE_MMHG) {
        AppController_RaiseFault(APP_FAULT_OVER_PRESSURE);
        return;
    }
    if (now - g_app.last_telemetry_ms >= APP_TELEMETRY_PERIOD_MS) {
        send_float(g_app.status.mode == APP_MODE_AUTO ? SCREEN_PRESSURE_AUTO : SCREEN_PRESSURE, measured);
    }
}

void AppController_Tick(void)
{
    uint32_t now = now_ms();

    if (g_app.status.state == APP_STATE_HOMING) {
        tick_homing();
        return;
    }
    tick_heat(now);
    tick_pressure(now);
    if (now - g_app.last_telemetry_ms >= APP_TELEMETRY_PERIOD_MS) {
        g_app.last_telemetry_ms = now;
    }
}

const AppSnapshot *AppController_Status(void)
{
    return &g_app.status;
}
