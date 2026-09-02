#include "app_controller.h"

#include <stddef.h>
#include <string.h>

#include "app_config.h"
#include "app_log.h"
#include "product_config.h"

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
#define SCREEN_HOME_COMPLETE         0x20F0U
#define SCREEN_FAULT_STATUS          0x2F01U

typedef struct {
    AppSnapshot status;
    const AppPort *port;
    uint32_t last_heat_ms;
    uint32_t last_pressure_ms;
    uint32_t last_temp_display_ms;
    uint32_t last_pressure_display_ms;
    uint32_t last_temp_log_ms;
    float display_temperature_c;
    bool display_temperature_valid;
    bool tmp112_recovery_confirmed;
    uint32_t over_temperature_started_ms;
    bool over_temperature_timing;
    uint8_t low_voltage_samples;
    bool count_recorded;
    bool pending_power_off;
    bool home_started;
    bool quick_resume_active;
    bool reusable_pressure_zero;
    bool power_state_logged;
    bool debug_mode;
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

static bool eye_is_screen_online(AppEyeState eye)
{
    return eye == APP_EYE_NEW || eye == APP_EYE_IN_USE ||
           eye == APP_EYE_SERVICE;
}

static bool treatment_is_open(void)
{
    return g_app.status.state == APP_STATE_PREHEAT ||
           g_app.status.state == APP_STATE_READY ||
           g_app.status.state == APP_STATE_RUNNING;
}

static bool stop_allows_quick_resume(AppStopReason reason)
{
    return reason == APP_STOP_USER || reason == APP_STOP_NATURAL;
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
    if (g_app.status.charging && g_app.status.charge_full) {
        led = APP_LED_FULL;
    } else if (g_app.status.charging) {
        led = APP_LED_CHARGING;
    } else if (g_app.status.fault != APP_FAULT_NONE) {
        led = APP_LED_FAULT;
    } else if (g_app.status.low_battery_warning) {
        led = APP_LED_WARNING;
        blink = true;
    }
    g_app.port->led_set(led, blink);
}

static void try_clear_tmp112_fault(void)
{
    if (!g_app.tmp112_recovery_confirmed ||
        g_app.status.fault != APP_FAULT_TMP112_COMM ||
        g_app.status.state != APP_STATE_FAULT || !g_app.status.home_valid) {
        return;
    }
    g_app.tmp112_recovery_confirmed = false;
    g_app.status.fault = APP_FAULT_NONE;
    g_app.status.mode = APP_MODE_NONE;
    g_app.status.state = APP_STATE_IDLE;
    LOGI("[Fault] TMP112 communication recovered; fault cleared eye=%u",
         (unsigned)g_app.status.eye);
    set_led();
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
    LOGI("[Motor] Homing started");
    g_app.reusable_pressure_zero =
        stop_allows_quick_resume(g_app.status.stop_reason) &&
        g_app.status.pressure_zero_valid;
    g_app.quick_resume_active = false;
    g_app.status.state = APP_STATE_HOMING;
    g_app.status.home_valid = false;
    g_app.status.pressure_zero_valid = false;
    g_app.status.temperature_valid = false;
    g_app.home_started = false;

    if (g_app.port != NULL && g_app.port->home_begin != NULL && g_app.port->home_begin()) {
        g_app.home_started = true;
    } else {
        LOGE("[Motor] Homing start failed");
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
    if (g_app.status.eye == APP_EYE_SERVICE || g_app.status.eye == APP_EYE_IN_USE) {
        return true;
    }
    if (g_app.status.eye != APP_EYE_NEW) {
        AppController_RaiseFault(APP_FAULT_TMP112_COMM);
        return false;
    }
#if PRODUCT_EYE_FUSE_ENABLED
    if (g_app.port == NULL || g_app.port->eye_mark_consumed == NULL ||
        !g_app.port->eye_mark_consumed()) {
        AppController_RaiseFault(APP_FAULT_TMP112_COMM);
        return false;
    }
#endif
    /* Keep the current uninterrupted insertion session usable. */
    g_app.status.eye = APP_EYE_IN_USE;
    /* SCREEN_NEW_EYE is an insertion event. It was already sent when the
     * eye shield changed to APP_EYE_NEW, so do not send it again at start. */
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
    LOGI("[App] Controller initialized");
}

bool AppController_Prepare(AppMode mode, float pressure_mmhg)
{
    bool eye_valid = g_app.status.eye == APP_EYE_NEW ||
                     g_app.status.eye == APP_EYE_IN_USE ||
                     g_app.status.eye == APP_EYE_SERVICE;
    bool quick_request = g_app.status.state == APP_STATE_HOMING &&
                         stop_allows_quick_resume(g_app.status.stop_reason);

    if (mode == APP_MODE_NONE ||
        (g_app.status.charging && !g_app.debug_mode) ||
        g_app.status.fault != APP_FAULT_NONE || !eye_valid ||
        (!quick_request &&
         (g_app.status.state != APP_STATE_IDLE || !g_app.status.home_valid))) {
        LOGW("[App] Prepare rejected: state=%u mode=%u eye=%u charging=%u fault=0x%04X home=%u",
             (unsigned)g_app.status.state, (unsigned)mode, (unsigned)g_app.status.eye,
             g_app.status.charging ? 1U : 0U, (unsigned)g_app.status.fault,
             g_app.status.home_valid ? 1U : 0U);
        return false;
    }
    if (quick_request) {
        if (!g_app.home_started || g_app.port == NULL ||
            g_app.port->home_cancel == NULL ||
            (mode_uses_pressure(mode) && !g_app.reusable_pressure_zero)) {
            LOGW("[App] Quick resume rejected: home_started=%u zero_reusable=%u mode=%u",
                 g_app.home_started ? 1U : 0U,
                 g_app.reusable_pressure_zero ? 1U : 0U, (unsigned)mode);
            return false;
        }
        g_app.port->home_cancel();
        g_app.home_started = false;
        g_app.quick_resume_active = true;
        g_app.status.pressure_zero_valid = g_app.reusable_pressure_zero;
        g_app.status.state = APP_STATE_IDLE;
        LOGW("[App] Quick resume accepted before home; previous pressure zero reused");
    } else {
        g_app.quick_resume_active = false;
    }
    if (mode_uses_pressure(mode) && !g_app.status.pressure_zero_valid) {
        LOGW("[App] Prepare rejected: pressure zero invalid");
        return false;
    }

    g_app.status.mode = mode;
    g_app.status.stop_reason = APP_STOP_NONE;
    g_app.status.fault = APP_FAULT_NONE;
    g_app.count_recorded = false;
    g_app.display_temperature_valid = false;
    g_app.status.temperature_valid = false;
    if (pressure_mmhg > 0.0f) {
        g_app.status.settings.pressure_mmhg = pressure_mmhg;
    }

    if (mode_uses_heat(mode)) {
        g_app.status.state = APP_STATE_PREHEAT;
    } else {
        g_app.status.state = APP_STATE_READY;
    }
    set_led();
    LOGI("[App] Prepared: mode=%u pressure=%ld", (unsigned)mode,
         (long)g_app.status.settings.pressure_mmhg);
    return true;
}

bool AppController_Start(void)
{
    if ((g_app.status.state != APP_STATE_PREHEAT && g_app.status.state != APP_STATE_READY) ||
        (g_app.status.charging && !g_app.debug_mode) ||
        (!g_app.status.home_valid && !g_app.quick_resume_active) ||
        (mode_uses_pressure(g_app.status.mode) && !g_app.status.pressure_zero_valid)) {
        LOGW("[App] Start rejected: state=%u charging=%u home=%u zero=%u",
             (unsigned)g_app.status.state, g_app.status.charging ? 1U : 0U,
             g_app.status.home_valid ? 1U : 0U,
             g_app.status.pressure_zero_valid ? 1U : 0U);
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
    LOGI("[App] Treatment started: mode=%u", (unsigned)g_app.status.mode);
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
    g_app.over_temperature_timing = false;
    LOGI("[App] Treatment stop: reason=%u state=%u", (unsigned)reason,
         (unsigned)g_app.status.state);
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
    if (fault == APP_FAULT_TMP112_COMM) {
        g_app.tmp112_recovery_confirmed = false;
    }
    LOGE("[Fault] code=0x%08lX state=%u", (unsigned long)fault,
         (unsigned)g_app.status.state);
    if (g_app.status.state != APP_STATE_HOMING && g_app.status.state != APP_STATE_SHUTDOWN) {
        /* Let the screen close its treatment UI before reporting the fault. */
        AppController_Stop(APP_STOP_FAULT);
    }
    /* If homing itself failed, begin_home() has replaced and reported the
     * more relevant motor fault already. */
    if (g_app.status.fault == fault &&
        g_app.port != NULL && g_app.port->fault_report != NULL) {
        g_app.port->fault_report(fault);
    }
}

void AppController_SetEyeState(AppEyeState eye)
{
    AppEyeState previous = g_app.status.eye;
    bool changed = eye != previous;

    if (previous == APP_EYE_IN_USE && eye != APP_EYE_ABSENT) {
        if (g_app.port != NULL && g_app.port->screen_float != NULL) {
            g_app.port->screen_float(SCREEN_EYE_STATE, 1.0f);
        }
        return;
    }
    g_app.status.eye = eye;
    if (changed) {
        LOGI("[Eye] state=%u -> %u", (unsigned)previous, (unsigned)eye);
    }
    if (previous == APP_EYE_ABSENT && eye != APP_EYE_ABSENT) {
        g_app.tmp112_recovery_confirmed = true;
    }
    /* Keep refreshing the LCD eye state on every eye poll. The LCD protocol
     * has no acknowledgement, so a one-shot state-change frame is not enough
     * to guarantee that an unplug event is displayed. */
    if (g_app.port != NULL && g_app.port->screen_float != NULL) {
        g_app.port->screen_float(SCREEN_EYE_STATE,
                                eye_is_screen_online(eye) ? 1.0f : 0.0f);
        if (eye == APP_EYE_NEW && changed) {
            g_app.port->screen_float(SCREEN_NEW_EYE, 0.0f);
        }
    }
    if (eye == APP_EYE_ABSENT && treatment_is_open()) {
        /* Eye removal is a recoverable treatment interruption, not a latched
         * fault. Reuse the board's three-flash warning pattern before the
         * normal stop path returns the LEDs to idle white. */
        if (!g_app.status.charging &&
            g_app.port != NULL && g_app.port->led_set != NULL) {
            g_app.port->led_set(APP_LED_FAULT, false);
        }
        LOGW("[Eye] Removed during treatment; warning flash requested");
        AppController_Stop(APP_STOP_EYE_REMOVED);
    }
}

void AppController_SetPower(bool charging, bool full, uint16_t soc, uint16_t millivolts)
{
    bool changed = !g_app.power_state_logged ||
                   g_app.status.charging != charging ||
                   g_app.status.charge_full != (charging && full);

    g_app.status.charging = charging;
    g_app.status.charge_full = charging && full;
    g_app.status.battery_soc = soc;
    g_app.status.battery_mv = millivolts;
    g_app.status.low_battery_warning = soc <= APP_LOW_BATTERY_WARNING_SOC;
    if (changed) {
        LOGI("[Power] external=%u full=%u soc=%u voltage=%umV",
             charging ? 1U : 0U, g_app.status.charge_full ? 1U : 0U,
             (unsigned)soc, (unsigned)millivolts);
        g_app.power_state_logged = true;
    }

    if (charging) {
        g_app.low_voltage_samples = 0U;
        if (g_app.pending_power_off &&
            g_app.status.stop_reason == APP_STOP_POWER_LOSS) {
            /* External power has priority.  A PWR_SENSE edge during charger
             * insertion/termination must not disconnect the battery FET. */
            g_app.pending_power_off = false;
            LOGW("[Power] Cancelled pending shutdown: external power present");
        }
        if (treatment_is_open() && !g_app.debug_mode) {
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
    if (g_app.port != NULL && g_app.port->screen_u32 != NULL) {
        /* New screens treat this as a renewable fault status. Old screens do
         * not know 0x2F01 and safely ignore it. A zero value clears the popup. */
        g_app.port->screen_u32(SCREEN_FAULT_STATUS, (uint32_t)g_app.status.fault);
    }
    set_led();
}

void AppController_NotifyPowerLoss(void)
{
    if (g_app.status.charging) {
        LOGW("[Power] Ignored PWR_SENSE edge while external power is present");
        return;
    }
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

void AppController_SetDebugMode(bool enabled)
{
    if (g_app.debug_mode == enabled) {
        return;
    }
    g_app.debug_mode = enabled;
    LOGW("[Debug] Test mode %s; charging treatment interlock %s",
         enabled ? "enabled" : "disabled",
         enabled ? "bypassed" : "active");
    if (!enabled && treatment_is_open()) {
        AppController_Stop(g_app.status.charging ? APP_STOP_CHARGING : APP_STOP_USER);
    }
}

bool AppController_DebugMode(void)
{
    return g_app.debug_mode;
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
        LOGE("[Motor] Homing failed");
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
    LOGI("[Motor] Homing complete, pressure_zero=%u",
         g_app.status.pressure_zero_valid ? 1U : 0U);
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
    if (g_app.status.fault == APP_FAULT_OVER_PRESSURE &&
        g_app.status.pressure_zero_valid) {
        LOGW("[Fault] Overpressure latch cleared after successful home and zero calibration");
        g_app.status.fault = APP_FAULT_NONE;
    } else if (g_app.status.fault == APP_FAULT_OVER_TEMPERATURE) {
        LOGW("[Fault] Overtemperature latch cleared after successful home");
        g_app.status.fault = APP_FAULT_NONE;
    }
    g_app.status.mode = APP_MODE_NONE;
    g_app.status.state = g_app.status.fault == APP_FAULT_NONE ? APP_STATE_IDLE : APP_STATE_FAULT;
    set_led();
}

static void tick_heat(uint32_t now)
{
    float measured = 0.0f;
    float target;
    float control_target;
    float screen_temperature;

    if (!mode_uses_heat(g_app.status.mode) ||
        (g_app.status.state != APP_STATE_PREHEAT && g_app.status.state != APP_STATE_RUNNING) ||
        now - g_app.last_heat_ms < APP_HEAT_CONTROL_PERIOD_MS) {
        return;
    }
    g_app.last_heat_ms = now;
    target = g_app.status.state == APP_STATE_PREHEAT ? APP_PREHEAT_TARGET_C :
                                                     g_app.status.settings.temperature_c;
    control_target = target + PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C;
    if (g_app.port == NULL || g_app.port->heater_control == NULL ||
        !g_app.port->heater_control(control_target, &measured)) {
        g_app.status.temperature_valid = false;
        AppController_RaiseFault(APP_FAULT_TMP112_COMM);
        return;
    }
    g_app.status.measured_temperature_c = measured;
    g_app.status.temperature_valid = true;
    if (measured >= APP_MAX_SAFE_TEMPERATURE_C) {
        if (!g_app.over_temperature_timing) {
            g_app.over_temperature_timing = true;
            g_app.over_temperature_started_ms = now;
            LOGW("[Temp] Overtemperature timing started measured_x10=%ld threshold_x10=%ld",
                 (long)(measured * 10.0f),
                 (long)(APP_MAX_SAFE_TEMPERATURE_C * 10.0f));
        } else if (now - g_app.over_temperature_started_ms >=
                   APP_OVER_TEMPERATURE_CONFIRM_MS) {
            LOGE("[Temp] Overtemperature confirmed for %lu ms measured_x10=%ld",
                 (unsigned long)(now - g_app.over_temperature_started_ms),
                 (long)(measured * 10.0f));
            g_app.over_temperature_timing = false;
            AppController_RaiseFault(APP_FAULT_OVER_TEMPERATURE);
            return;
        }
    } else if (g_app.over_temperature_timing) {
        LOGI("[Temp] Overtemperature timing cancelled measured_x10=%ld",
             (long)(measured * 10.0f));
        g_app.over_temperature_timing = false;
    }
    /* Smooth only the value shown to the user. Safety and heater control keep
     * using the unfiltered sensor value above. */
    if (!g_app.display_temperature_valid) {
        g_app.display_temperature_c = measured;
        g_app.display_temperature_valid = true;
    } else {
        g_app.display_temperature_c += APP_TEMP_DISPLAY_FILTER_ALPHA *
                                       (measured - g_app.display_temperature_c);
    }
    if (now - g_app.last_temp_display_ms >= APP_TEMP_DISPLAY_PERIOD_MS) {
        screen_temperature = g_app.display_temperature_c -
                             PRODUCT_TEMPERATURE_CONTROL_COMPENSATION_C;
        if (screen_temperature > APP_MAX_DISPLAY_TEMPERATURE_C) {
            screen_temperature = APP_MAX_DISPLAY_TEMPERATURE_C;
        }
        send_float(g_app.status.mode == APP_MODE_AUTO ? SCREEN_TEMP_AUTO : SCREEN_TEMP_HEAT,
                   screen_temperature);
        g_app.last_temp_display_ms = now;
    }
    if (now - g_app.last_temp_log_ms >= APP_SENSOR_LOG_PERIOD_MS) {
        int32_t measured_x10 = (int32_t)(measured * 10.0f);
        int32_t target_x10 = (int32_t)(target * 10.0f);
        int32_t control_target_x10 = (int32_t)(control_target * 10.0f);
        LOGI("[Temp] measured=%ld.%01ldC target=%ld.%01ldC control_target=%ld.%01ldC",
             (long)(measured_x10 / 10), (long)(measured_x10 % 10),
             (long)(target_x10 / 10), (long)(target_x10 % 10),
             (long)(control_target_x10 / 10),
             (long)(control_target_x10 % 10));
        g_app.last_temp_log_ms = now;
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
        /* Stop the motor before formatting or transmitting fault diagnostics. */
        if (g_app.port != NULL && g_app.port->pressure_control_stop != NULL) {
            g_app.port->pressure_control_stop();
        }
        AppController_RaiseFault(APP_FAULT_OVER_PRESSURE);
        return;
    }
    if (now - g_app.last_pressure_display_ms >= APP_PRESSURE_DISPLAY_PERIOD_MS) {
        send_float(g_app.status.mode == APP_MODE_AUTO ? SCREEN_PRESSURE_AUTO : SCREEN_PRESSURE, measured);
        g_app.last_pressure_display_ms = now;
    }
}

void AppController_Tick(void)
{
    uint32_t now = now_ms();

    try_clear_tmp112_fault();
    if (g_app.status.state == APP_STATE_HOMING) {
        tick_homing();
        return;
    }
    tick_heat(now);
    tick_pressure(now);
}

const AppSnapshot *AppController_Status(void)
{
    return &g_app.status;
}
