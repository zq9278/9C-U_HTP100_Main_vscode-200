#include "treatment_hw.h"

#include <math.h>
#include <stdint.h>

#include "ads1220_driver.h"
#include "app_config.h"
#include "app_log.h"
#include "eye_driver.h"
#include "pressure_tuning.h"
#include "pressure_limit.h"
#include "tim.h"
#include "tmc5130_driver.h"

typedef enum {
    PRESS_STAGE_FAST = 0,
    PRESS_STAGE_SLOW,
    PRESS_STAGE_HOLD,
    PRESS_STAGE_RETRACT
} PressureStage;

#define PRESSURE_FILTER_ALPHA          0.20f
#define PRESSURE_CONTROL_DEADBAND      3.0f
#define PRESSURE_CONTROL_MAX_SPEED     8000.0f
#define PRESSURE_CONTROL_MAX_I_OUTPUT  2000.0f
#define PRESSURE_CONTROL_SLEW_PER_S    100000.0f
#define PRESSURE_CONTROL_MIN_SPEED     300.0f
#define PRESSURE_HOLD_TIME_MS          1000U
#define PRESSURE_APPROACH_TIMEOUT_MS   5000U
#define PRESSURE_STEP_DEFAULT_DT_S     0.02f
#define PRESSURE_STEP_MIN_DT_S         0.005f
#define PRESSURE_STEP_MAX_DT_S         0.10f
#define TMC_RUNTIME_PROBE_PERIOD_MS     100U
#define TMC_HOME_COMM_FAILURE_LIMIT     3U
#define TMC_HOME_POLL_MS                10U
#define TMC_HOME_CONFIRM_MS             50U

#define HEAT_PID_DEFAULT_KP 30.0f
#define HEAT_PID_DEFAULT_KI 5.0f
#define HEAT_PID_DEFAULT_KD 5.0f
#define HEAT_PID_MAX_KP     100.0f
#define HEAT_PID_MAX_KI     50.0f
#define HEAT_PID_MAX_KD     20.0f
#define HEAT_PID_DT_S       0.15f
#define HEAT_PID_MAX_I      20.0f
#define HEAT_PID_MIN_I      (-20.0f)
#define HEAT_PID_MAX_I_OUT  40.0f
#define HEAT_PID_MIN_I_OUT  (-40.0f)

static bool g_home_active;
static uint32_t g_home_started_ms;
static uint32_t g_home_last_poll_ms;
static uint32_t g_home_last_log_ms;
static uint32_t g_home_spi_errors;
static bool g_home_stop_requested;
static bool g_home_confirming;
static uint32_t g_home_confirm_started_ms;
static bool g_home_low_confirming;
static uint32_t g_home_low_started_ms;
static uint32_t g_home_low_sample_ms;
static float g_home_pressure_target = 350.0f;
static bool g_home_recovering;
static uint8_t g_home_recovery_attempts;
static uint32_t g_home_recovery_next_ms;

static uint32_t g_ads_last_log_ms;
static int32_t g_ads_last_raw;
static int32_t g_ads_last_pressure_x10;
static int32_t g_ads_last_unfiltered_pressure_x10;
static bool g_ads_sample_valid;
static bool g_ads_overpressure_pending;
static PressureLimit g_overpressure;
static PressureStage g_pressure_stage;
static uint32_t g_pressure_stage_ms;
static uint32_t g_pressure_approach_started_ms;
static uint32_t g_pressure_last_step_ms;
static uint32_t g_pressure_hold_started_ms;
static uint32_t g_pressure_last_tmc_probe_ms;
static uint32_t g_pressure_last_sensor_check_ms;
static float g_pressure_integral;
static float g_pressure_command;
static float g_pressure_error;
static float g_pressure_filter_samples[3];
static float g_pressure_filtered;
static uint8_t g_pressure_filter_count;
static uint8_t g_pressure_filter_index;
static bool g_pressure_filter_valid;
static bool g_pressure_hold_timer_started;
static bool g_pressure_active;
static bool g_pressure_recovering;
static uint8_t g_pressure_recovery_attempts;
static uint32_t g_pressure_recovery_next_ms;
static uint8_t g_pressure_fault_attempts;
static uint32_t g_pressure_fault_next_ms;

static float g_heat_integral;
static float g_heat_previous_error;
static float g_heat_kp;
static float g_heat_ki;
static float g_heat_kd;
static float g_heat_power_percent;
static bool g_heat_previous_valid;
static bool g_heat_pwm_started;

static AppFault pressure_begin_recovery(const char *reason);
static AppFault pressure_recovery_poll(float target_mmhg,
                                       float *measured_mmhg);

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float median3(float a, float b, float c)
{
    if (a > b) {
        float temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c) {
        float temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b) b = a;
    return b;
}

static void pressure_filter_reset(void)
{
    g_pressure_filter_count = 0U;
    g_pressure_filter_index = 0U;
    g_pressure_filtered = 0.0f;
    g_pressure_filter_valid = false;
    PressureLimit_Reset(&g_overpressure);
}

static float pressure_filter_update(float sample)
{
    float selected;

    g_pressure_filter_samples[g_pressure_filter_index] = sample;
    g_pressure_filter_index = (uint8_t)((g_pressure_filter_index + 1U) % 3U);
    if (g_pressure_filter_count < 3U) g_pressure_filter_count++;

    if (g_pressure_filter_count == 1U) {
        selected = sample;
    } else if (g_pressure_filter_count == 2U) {
        selected = (g_pressure_filter_samples[0] +
                    g_pressure_filter_samples[1]) * 0.5f;
    } else {
        selected = median3(g_pressure_filter_samples[0],
                           g_pressure_filter_samples[1],
                           g_pressure_filter_samples[2]);
    }

    if (!g_pressure_filter_valid) {
        g_pressure_filtered = selected;
        g_pressure_filter_valid = true;
    } else {
        g_pressure_filtered += PRESSURE_FILTER_ALPHA *
                               (selected - g_pressure_filtered);
    }
    return g_pressure_filtered;
}

static void pressure_control_reset(void)
{
    g_pressure_integral = 0.0f;
    g_pressure_error = 0.0f;
    g_pressure_hold_started_ms = 0U;
    g_pressure_hold_timer_started = false;
}

static bool pressure_set_speed(float command)
{
    int32_t speed = (int32_t)command;

    if (!Tmc5130Driver_SetSpeed(speed)) return false;
    g_pressure_command = (float)speed;
    return true;
}

static float pressure_control_command(const PressureTuningProfile *profile,
                                      float error, float dt_s,
                                      bool use_deadband)
{
    (void)dt_s;
    g_pressure_integral = 0.0f;
    if (use_deadband && fabsf(error) <= PRESSURE_CONTROL_DEADBAND)
        return 0.0f;
    /* Pure P velocity control; allow both compression and unloading. */
    const float limit = 3000.0f; /* Internal P-control output limit. */
    return clamp_float(profile->kp * error, -limit, limit);
}

void TreatmentHw_Init(void)
{
    g_heat_kp = HEAT_PID_DEFAULT_KP;
    g_heat_ki = HEAT_PID_DEFAULT_KI;
    g_heat_kd = HEAT_PID_DEFAULT_KD;
    Tmc5130Driver_Init();
    TreatmentHw_SafeOutputsOff();
}

void TreatmentHw_SafeOutputsOff(void)
{
    PressureLimit_Reset(&g_overpressure);
    g_pressure_fault_attempts = 0U;
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    (void)HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
    g_heat_integral = 0.0f;
    g_heat_previous_error = 0.0f;
    g_heat_power_percent = 0.0f;
    g_heat_previous_valid = false;
    g_heat_pwm_started = false;
    g_pressure_active = false;
    g_pressure_recovering = false;
    g_pressure_command = 0.0f;
    pressure_control_reset();
    pressure_filter_reset();
    Tmc5130Driver_Stop();
}

bool TreatmentHw_HeaterControl(float target_c, float *measured_c)
{
    float error;
    float derivative;
    float next_integral;
    float integral_output;
    float pwm;

    if (!EyeDriver_ReadTemperature(measured_c)) {
        /* Keep the previous PWM and controller state during the application's
         * bounded temperature-communication confirmation window. */
        return false;
    }
    error = target_c - *measured_c;
    /* Temporarily disabled for PID tuning: reaching the target must not
     * discard the holding-power integral and force PWM to zero.
     *
     * if (error <= 0.0f) {
     *     g_heat_integral = 0.0f;
     *     g_heat_previous_valid = false;
     *     pwm = 0.0f;
     * } else
     */
    derivative = g_heat_previous_valid ?
                 (error - g_heat_previous_error) / HEAT_PID_DT_S : 0.0f;
    next_integral = g_heat_integral + error * HEAT_PID_DT_S;
    if (next_integral > HEAT_PID_MAX_I) next_integral = HEAT_PID_MAX_I;
    if (next_integral < HEAT_PID_MIN_I) next_integral = HEAT_PID_MIN_I;
    integral_output = g_heat_ki * next_integral;
    if (integral_output > HEAT_PID_MAX_I_OUT) {
        integral_output = HEAT_PID_MAX_I_OUT;
        next_integral = integral_output / g_heat_ki;
    }
    if (integral_output < HEAT_PID_MIN_I_OUT) {
        integral_output = HEAT_PID_MIN_I_OUT;
        next_integral = integral_output / g_heat_ki;
    }
    pwm = g_heat_kp * error + integral_output + g_heat_kd * derivative;
    /* Integrate only when it moves a saturated output back toward range. */
    if ((pwm > 0.0f && pwm < 254.0f) ||
        (pwm <= 0.0f && error > 0.0f) ||
        (pwm >= 254.0f && error < 0.0f)) {
        g_heat_integral = next_integral;
    }
    g_heat_previous_valid = true;
    g_heat_previous_error = error;
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > 254.0f) pwm = 254.0f;
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, (uint32_t)pwm);
    if (!g_heat_pwm_started) {
        g_heat_pwm_started = HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) == HAL_OK;
    }
    g_heat_power_percent = g_heat_pwm_started ? pwm * (100.0f / 254.0f) : 0.0f;
    return g_heat_pwm_started;
}

bool TreatmentHw_SetHeatPid(float kp, float ki, float kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp < 0.0f || kp > HEAT_PID_MAX_KP ||
        ki < 0.0f || ki > HEAT_PID_MAX_KI ||
        kd < 0.0f || kd > HEAT_PID_MAX_KD) {
        return false;
    }
    g_heat_kp = kp;
    g_heat_ki = ki;
    g_heat_kd = kd;
    g_heat_integral = 0.0f;
    g_heat_previous_error = 0.0f;
    g_heat_previous_valid = false;
    return true;
}

void TreatmentHw_GetHeatPid(float *kp, float *ki, float *kd)
{
    if (kp != NULL) *kp = g_heat_kp;
    if (ki != NULL) *ki = g_heat_ki;
    if (kd != NULL) *kd = g_heat_kd;
}

void TreatmentHw_HeatTelemetry(float *power_percent, float *integral_output)
{
    if (power_percent != NULL) *power_percent = g_heat_power_percent;
    if (integral_output != NULL) *integral_output = g_heat_ki * g_heat_integral;
}

static bool home_configure_driver(bool verify)
{
    uint32_t previous_status;

    Tmc5130Driver_Enable(false);
    return Tmc5130Driver_Configure() &&
           (!verify || Tmc5130Driver_Probe()) &&
           /* Drain read-to-clear events left by an earlier homing attempt. */
           Tmc5130Driver_Read(0x35U, &previous_status) == HAL_OK &&
           /* REFR high active, positive-direction hard stop, edge latch. */
           Tmc5130Driver_Write(0x34U, 0x82U) == HAL_OK &&
           Tmc5130Driver_Write(0x27U, 0x00010000U) == HAL_OK &&
           Tmc5130Driver_Write(0x20U, 1U) == HAL_OK;
}

static AppAsyncResult home_begin_recovery(uint32_t now, const char *reason)
{
    Tmc5130Driver_Enable(false);
    g_home_recovering = true;
    g_home_recovery_next_ms = now + APP_TMC_RECOVERY_RETRY_MS;
    g_home_spi_errors = 0U;
    g_home_confirming = false;
    g_home_stop_requested = false;
    LOGW("[Motor] Home recovery scheduled: %s attempt=%u/%u elapsed=%lums",
         reason, (unsigned)g_home_recovery_attempts,
         (unsigned)APP_TMC_HOME_RECOVERY_TRIES,
         (unsigned long)(now - g_home_started_ms));
    return APP_ASYNC_BUSY;
}

static AppAsyncResult home_recovery_poll(uint32_t now)
{
    if ((int32_t)(now - g_home_recovery_next_ms) < 0) {
        return APP_ASYNC_BUSY;
    }
    if (g_home_recovery_attempts >= APP_TMC_HOME_RECOVERY_TRIES) {
        g_home_active = false;
        g_home_recovering = false;
        LOGE("[Motor] Home recovery exhausted attempts=%u elapsed=%lums",
             (unsigned)g_home_recovery_attempts,
             (unsigned long)(now - g_home_started_ms));
        return APP_ASYNC_COMM_FAILED;
    }
    g_home_recovery_attempts++;
    if (!home_configure_driver(true)) {
        LOGW("[Motor] Home recovery configure/verify failed attempt=%u/%u",
             (unsigned)g_home_recovery_attempts,
             (unsigned)APP_TMC_HOME_RECOVERY_TRIES);
        g_home_recovery_next_ms = HAL_GetTick() + APP_TMC_RECOVERY_RETRY_MS;
        return APP_ASYNC_BUSY;
    }
    g_home_recovering = false;
    g_home_last_poll_ms = HAL_GetTick();
    Tmc5130Driver_Enable(true);
    LOGI("[Motor] Home recovery restarted attempt=%u elapsed=%lums",
         (unsigned)g_home_recovery_attempts,
         (unsigned long)(HAL_GetTick() - g_home_started_ms));
    return APP_ASYNC_BUSY;
}

bool TreatmentHw_HomeBegin(void)
{
    g_home_started_ms = HAL_GetTick();
    g_home_last_poll_ms = 0U;
    g_home_last_log_ms = g_home_started_ms;
    g_home_spi_errors = 0U;
    g_home_stop_requested = false;
    g_home_confirming = false;
    g_home_low_confirming = false;
    g_home_low_sample_ms = 0U;
    g_home_recovering = false;
    g_home_recovery_attempts = 0U;
    g_home_active = true;
    if (!home_configure_driver(false)) {
        LOGW("[Motor] Home initial configuration failed; scheduling recovery");
        (void)home_begin_recovery(g_home_started_ms,
                                  "initial configuration failed");
        return true;
    }
    Tmc5130Driver_Enable(true);
    LOGI("[Motor] Home command accepted");
    return true;
}

AppAsyncResult TreatmentHw_HomePoll(void)
{
    uint32_t ramp_status;
    uint32_t now = HAL_GetTick();

    if (!g_home_active) {
        return APP_ASYNC_FAILED;
    }
    if (now - g_home_started_ms >= APP_HOME_TIMEOUT_MS) {
        Tmc5130Driver_Stop();
        g_home_active = false;
        LOGE("[Motor] Home timeout elapsed=%lu ms spi_errors=%lu",
             (unsigned long)(now - g_home_started_ms),
             (unsigned long)g_home_spi_errors);
        return APP_ASYNC_FAILED;
    }
    if (g_home_recovering) {
        return home_recovery_poll(now);
    }
    if (now - g_home_last_poll_ms < TMC_HOME_POLL_MS) {
        return APP_ASYNC_BUSY;
    }
    g_home_last_poll_ms = now;
    if (Tmc5130Driver_Read(0x35U, &ramp_status) != HAL_OK) {
        g_home_confirming = false;
        g_home_spi_errors++;
        if (now - g_home_last_log_ms >= 500U) {
            LOGW("[Motor] Home poll SPI failure elapsed=%lu ms count=%lu",
                 (unsigned long)(now - g_home_started_ms),
                 (unsigned long)g_home_spi_errors);
            g_home_last_log_ms = now;
        }
        if (g_home_spi_errors >= TMC_HOME_COMM_FAILURE_LIMIT) {
            return home_begin_recovery(now, "RAMP_STAT read failed");
        }
        return APP_ASYNC_BUSY;
    }
    if (!Tmc5130Driver_Probe()) {
        g_home_confirming = false;
        g_home_spi_errors++;
        if (g_home_spi_errors >= TMC_HOME_COMM_FAILURE_LIMIT) {
            LOGE("[Motor] TMC5130 disconnected during home");
            return home_begin_recovery(now, "identity probe failed");
        }
        return APP_ASYNC_BUSY;
    }
    g_home_spi_errors = 0U;
    /* Sample fresh data only near the fallback deadline. A missed/invalid
     * sample or a sampling gap breaks the continuous-low interval. */
#if APP_HOME_LOW_PRESSURE_SUCCESS
    if (now - g_home_started_ms >=
        APP_HOME_LOW_PRESSURE_AFTER_MS - APP_HOME_LOW_PRESSURE_CONFIRM_MS) {
        int32_t raw;
        float pressure = NAN;
        uint32_t sampled;
        if (Ads1220Driver_Ready() && Ads1220Driver_ZeroValid() &&
            Ads1220Driver_ReadRaw(&raw)) {
            pressure = Ads1220Driver_PressureFromRaw(raw, g_home_pressure_target);
        }
        sampled = HAL_GetTick();
        if (!isfinite(pressure) || pressure >= APP_HOME_LOW_PRESSURE_MMHG) {
            g_home_low_confirming = false;
        } else {
            if (!g_home_low_confirming || sampled - g_home_low_sample_ms > 100U) {
                g_home_low_started_ms = sampled;
                g_home_low_confirming = true;
            }
            if (sampled - g_home_started_ms >= APP_HOME_LOW_PRESSURE_AFTER_MS &&
                sampled - g_home_low_started_ms >= APP_HOME_LOW_PRESSURE_CONFIRM_MS) {
                uint32_t actual_position;
                uint32_t target_position;
                uint32_t velocity = 0U;
                bool velocity_valid = Tmc5130Driver_Read(0x22U, &velocity) == HAL_OK;
                Tmc5130Driver_Stop();
                if (Tmc5130Driver_Write(0x21U, 0U) != HAL_OK ||
                    Tmc5130Driver_Write(0x2DU, 0U) != HAL_OK ||
                    Tmc5130Driver_Read(0x21U, &actual_position) != HAL_OK ||
                    Tmc5130Driver_Read(0x2DU, &target_position) != HAL_OK ||
                    actual_position != 0U || target_position != 0U) {
                    LOGE("[Motor] Low-pressure home zero write/readback failed");
                    return home_begin_recovery(sampled,
                                               "low-pressure zero readback failed");
                }
                g_home_active = false;
                Ads1220Result sensor = Ads1220Driver_CheckSensor();
                LOGI("[Motor] Home accepted by low pressure: elapsed=%lums low=%lums pressure_x10=%ld raw=%ld RAMP_STAT=0x%08lX VACTUAL=0x%08lX velocity_valid=%u trigger_seen=%u sensor=%u",
                     (unsigned long)(sampled - g_home_started_ms),
                     (unsigned long)(sampled - g_home_low_started_ms),
                     (long)(pressure * 10.0f), (long)raw,
                     (unsigned long)ramp_status, (unsigned long)velocity,
                     velocity_valid ? 1U : 0U, g_home_stop_requested ? 1U : 0U,
                     (unsigned)sensor);
                return APP_ASYNC_OK;
            }
        }
        g_home_low_sample_ms = sampled;
    }
#endif
    /* Capture both the current switch and latched transient events. Revoke
     * motion immediately so switch bounce cannot restart the hardware ramp. */
    if (!g_home_stop_requested && (ramp_status & 0x2AU) != 0U) {
        if (Tmc5130Driver_Write(0x27U, 0U) != HAL_OK ||
            Tmc5130Driver_Write(0x20U, 3U) != HAL_OK) {
            return home_begin_recovery(now, "reference stop command failed");
        }
        g_home_stop_requested = true;
        g_home_confirming = false;
        return APP_ASYNC_BUSY;
    }
    if (now - g_home_last_log_ms >= 500U) {
        LOGI("[Motor] Home polling elapsed=%lu ms RAMP_STAT=0x%08lX",
             (unsigned long)(now - g_home_started_ms),
             (unsigned long)ramp_status);
        g_home_last_log_ms = now;
    }
    if (!g_home_stop_requested) {
        return APP_ASYNC_BUSY;
    }
    /* Never resume motion after an unconfirmed trigger. Keep stopped and
     * require fresh, consecutive observations; the overall timeout applies. */
    if ((ramp_status & 0x402U) != 0x402U) {
        g_home_confirming = false;
        return APP_ASYNC_BUSY;
    }
    if (!g_home_confirming) {
        g_home_confirm_started_ms = HAL_GetTick();
        g_home_confirming = true;
        return APP_ASYNC_BUSY;
    }
    if (HAL_GetTick() - g_home_confirm_started_ms < TMC_HOME_CONFIRM_MS) {
        return APP_ASYNC_BUSY;
    }
    uint32_t actual_position;
    uint32_t target_position;
    if (Tmc5130Driver_Write(0x21U, 0U) != HAL_OK ||
        Tmc5130Driver_Write(0x2DU, 0U) != HAL_OK ||
        Tmc5130Driver_Read(0x21U, &actual_position) != HAL_OK ||
        Tmc5130Driver_Read(0x2DU, &target_position) != HAL_OK ||
        actual_position != 0U || target_position != 0U) {
        Tmc5130Driver_Stop();
        LOGE("[Motor] Home zero write/readback failed");
        return home_begin_recovery(HAL_GetTick(), "zero readback failed");
    }
    Tmc5130Driver_Stop();
    g_home_active = false;
    LOGI("[Motor] Home reference reached elapsed=%lu ms RAMP_STAT=0x%08lX",
         (unsigned long)(now - g_home_started_ms),
         (unsigned long)ramp_status);
    return APP_ASYNC_OK;
}

void TreatmentHw_HomeCancel(void)
{
    if (g_home_active) {
        LOGW("[Motor] Homing cancelled for normal quick resume elapsed=%lu ms",
             (unsigned long)(HAL_GetTick() - g_home_started_ms));
    }
    g_home_active = false;
    g_home_recovering = false;
    Tmc5130Driver_Stop();
}

bool TreatmentHw_PressureStart(float target_mmhg)
{
    const PressureTuningProfile *profile = PressureTuning_ProfileForTarget(target_mmhg);
    g_pressure_fault_attempts = 0U;
    if (!Ads1220Driver_ZeroValid()) {
        return false;
    }
    Tmc5130Driver_Enable(true);
    if (!Tmc5130Driver_Configure()) {
        Tmc5130Driver_Enable(false);
        return false;
    }
    g_pressure_stage = PRESS_STAGE_FAST;
    g_pressure_stage_ms = HAL_GetTick();
    g_pressure_approach_started_ms = g_pressure_stage_ms;
    g_pressure_last_step_ms = g_pressure_stage_ms;
    g_pressure_last_tmc_probe_ms = g_pressure_stage_ms;
    g_pressure_last_sensor_check_ms = g_pressure_stage_ms;
    g_pressure_command = 0.0f;
    g_pressure_recovering = false;
    g_pressure_recovery_attempts = 0U;
    pressure_control_reset();
    pressure_filter_reset();
    g_ads_sample_valid = false;
    g_pressure_active = true;
    LOG_PRESSURE_DIAG("[Pressure] Start target=%ldmmHg stage=fast",
         (long)target_mmhg);
    LOG_PRESSURE_DIAG("[Pressure] P profile kp=%ld", (long)profile->kp);
    return true;
}

static AppFault pressure_step_once(float target_mmhg, float *measured_mmhg)
{
    g_home_pressure_target = target_mmhg;
    const PressureTuningProfile *profile = PressureTuning_ProfileForTarget(target_mmhg);
    Ads1220Result sensor_result;
    int32_t raw;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - g_pressure_last_step_ms;
    uint32_t stage_elapsed_ms = now - g_pressure_stage_ms;
    float unfiltered_pressure;
    float dt_s;
    float error;
    float command;
    uint32_t retract_time_ms;

    if (measured_mmhg == NULL) {
        (void)pressure_set_speed(0.0f);
        return APP_FAULT_PRESSURE_COMM;
    }
    if (g_pressure_recovering) {
        return pressure_recovery_poll(target_mmhg, measured_mmhg);
    }
    if (!g_pressure_active) {
        (void)pressure_set_speed(0.0f);
        return APP_FAULT_PRESSURE_COMM;
    }
    if (now - g_pressure_last_tmc_probe_ms >= TMC_RUNTIME_PROBE_PERIOD_MS) {
        g_pressure_last_tmc_probe_ms = now;
        if (!Tmc5130Driver_Probe()) {
            Tmc5130Driver_Enable(false);
            g_pressure_active = false;
            LOGE("[Motor] TMC5130 disconnected during pressure control");
            return pressure_begin_recovery("identity probe failed");
        }
    }
    if (now - g_pressure_last_sensor_check_ms >= APP_PRESSURE_SENSOR_CHECK_MS) {
        g_pressure_last_sensor_check_ms = now;
        sensor_result = Ads1220Driver_CheckSensor();
        if (sensor_result != ADS1220_RESULT_OK) {
            (void)pressure_set_speed(0.0f);
            g_pressure_active = false;
            return sensor_result == ADS1220_RESULT_SENSOR_SHORT ||
                   sensor_result == ADS1220_RESULT_SENSOR_OPEN ?
                   APP_FAULT_PRESSURE_SENSOR : APP_FAULT_PRESSURE_COMM;
        }
    }
    if (!Ads1220Driver_ReadRaw(&raw)) {
        (void)pressure_set_speed(0.0f);
        return APP_FAULT_PRESSURE_COMM;
    }
    dt_s = elapsed_ms == 0U ? PRESSURE_STEP_DEFAULT_DT_S :
           (float)elapsed_ms / 1000.0f;
    dt_s = clamp_float(dt_s, PRESSURE_STEP_MIN_DT_S,
                       PRESSURE_STEP_MAX_DT_S);
    g_pressure_last_step_ms = now;

    unfiltered_pressure = Ads1220Driver_PressureFromRaw(raw, target_mmhg);
    g_ads_last_raw = raw;
    g_ads_last_unfiltered_pressure_x10 =
        (int32_t)(unfiltered_pressure * 10.0f);
    g_ads_sample_valid = true;
    bool was_overpressure_timing = g_overpressure.timing;
    bool overpressure_confirmed = PressureLimit_Update(
        &g_overpressure, unfiltered_pressure, HAL_GetTick());
    if (unfiltered_pressure >= APP_MAX_SAFE_PRESSURE_MMHG) {
        if (!was_overpressure_timing) {
            LOGW("[Pressure] Over-limit timing started pressure=%ldmmHg confirm=%lums",
                 (long)unfiltered_pressure, (unsigned long)APP_OVER_PRESSURE_CONFIRM_MS);
        }
        if (overpressure_confirmed) {
            g_ads_overpressure_pending = true;
            return APP_FAULT_OVER_PRESSURE;
        }
        /* Do not feed an unconfirmed spike into the display/control filter or
         * the application's backup threshold check. */
        *measured_mmhg = g_pressure_filter_valid ? g_pressure_filtered :
                         APP_MAX_SAFE_PRESSURE_MMHG - 0.1f;
        g_ads_last_pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
        return APP_FAULT_NONE;
    }
    *measured_mmhg = pressure_filter_update(unfiltered_pressure);
    g_ads_last_pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
    if (APP_ENABLE_PRESSURE_DIAGNOSTIC_LOG != 0U &&
        now - g_ads_last_log_ms >= APP_SENSOR_LOG_PERIOD_MS) {
        int32_t pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
        LOG_PRESSURE_DIAG("[ADS1220] raw=%ld zero=%ld pressure=%ld.%01ldmmHg target=%ld stage=%u speed=%ld",
             (long)raw, (long)Ads1220Driver_ZeroRaw(),
             (long)(pressure_x10 / 10), (long)(pressure_x10 % 10),
             (long)target_mmhg, (unsigned)g_pressure_stage,
             (long)g_pressure_command);
        g_ads_last_log_ms = now;
    }

    retract_time_ms = 5000U; /* Maximum release duration, not normal endpoint. */

    /* One deadline for the whole approach, including P correction before
     * reaching the target band. Entering HOLD must not restart it. */
    if (g_pressure_stage != PRESS_STAGE_RETRACT &&
        !g_pressure_hold_timer_started &&
        fabsf(target_mmhg - *measured_mmhg) > 10.0f &&
        now - g_pressure_approach_started_ms >= PRESSURE_APPROACH_TIMEOUT_MS) {
        g_pressure_stage = PRESS_STAGE_RETRACT;
        g_pressure_stage_ms = now;
        pressure_control_reset();
        if (!pressure_set_speed(-profile->retract_speed))
            return pressure_begin_recovery("approach timeout retract failed");
        LOGW("[Pressure] Approach timeout: retract and retry");
        return APP_FAULT_NONE;
    }

    switch (g_pressure_stage) {
    case PRESS_STAGE_FAST:
    case PRESS_STAGE_SLOW:
        error = target_mmhg - *measured_mmhg;
        g_pressure_error = error;
        /* Full speed at low pressure, continuously taper over the profile-defined pressure interval. */
        command = clamp_float(profile->fast_speed * error /
                              fmaxf(target_mmhg * profile->approach_ratio, 1.0f),
                              0.0f, profile->fast_speed);
        if (error <= 10.0f) {
            g_pressure_stage = PRESS_STAGE_HOLD;
            g_pressure_stage_ms = now;
            pressure_control_reset();
            g_pressure_error = error;
            if (fabsf(error) <= 10.0f) {
                g_pressure_hold_timer_started = true;
                g_pressure_hold_started_ms = now;
            }
            command = pressure_control_command(profile, error, dt_s,
                                               g_pressure_hold_timer_started);
        }
        if (!pressure_set_speed(command))
            return pressure_begin_recovery("approach command failed");
        break;
    case PRESS_STAGE_HOLD:
        error = target_mmhg - *measured_mmhg;
        g_pressure_error = error;
        /* Start the one-second hold on first entry into the target band. */
        if (!g_pressure_hold_timer_started &&
            fabsf(error) <= 10.0f) {
            g_pressure_hold_timer_started = true;
            g_pressure_hold_started_ms = now;
            LOG_PRESSURE_DIAG("[Pressure] hold target reached pressure=%ld timer=%lums",
                 (long)*measured_mmhg,
                 (unsigned long)PRESSURE_HOLD_TIME_MS);
        }
        if (g_pressure_hold_timer_started &&
            now - g_pressure_hold_started_ms >= PRESSURE_HOLD_TIME_MS) {
            if (!pressure_set_speed(0.0f))
                return pressure_begin_recovery("hold stop command failed");
            g_pressure_stage = PRESS_STAGE_RETRACT;
            g_pressure_stage_ms = now;
            pressure_control_reset();
            LOG_PRESSURE_DIAG("[Pressure] hold->retract pressure=%ld held=%lums retract_ms=%lu retract_speed=%ld target=%ld",
                 (long)*measured_mmhg,
                 (unsigned long)PRESSURE_HOLD_TIME_MS,
                 (unsigned long)retract_time_ms, (long)profile->retract_speed,
                 (long)target_mmhg);
        } else {
            command = pressure_control_command(profile, error, dt_s,
                                               g_pressure_hold_timer_started);
            if (!pressure_set_speed(command))
                return pressure_begin_recovery("hold PID command failed");
        }
        break;
    case PRESS_STAGE_RETRACT:
        if (unfiltered_pressure < 50.0f) {
            if (!pressure_set_speed(0.0f))
                return pressure_begin_recovery("retract stop command failed");
            g_pressure_stage = PRESS_STAGE_FAST;
            g_pressure_stage_ms = now;
            g_pressure_approach_started_ms = now;
            pressure_control_reset();
            LOG_PRESSURE_DIAG("[Pressure] retract->fast pressure=%ld elapsed=%lums limit=%lums",
                 (long)*measured_mmhg,
                 (unsigned long)stage_elapsed_ms,
                 (unsigned long)retract_time_ms);
        } else {
            if (stage_elapsed_ms >= retract_time_ms) {
                (void)pressure_set_speed(0.0f);
                return APP_FAULT_PRESSURE_SENSOR;
            }
            if (!pressure_set_speed(-profile->retract_speed))
                return pressure_begin_recovery("retract command failed");
        }
        break;
    default:
        return APP_FAULT_PRESSURE_COMM;
    }
    return APP_FAULT_NONE;
}

AppFault TreatmentHw_PressureStep(float target_mmhg, float *measured_mmhg)
{
    AppFault fault;
    bool was_active = g_pressure_active;
    bool was_recovering = g_pressure_recovering;

    if (measured_mmhg == NULL) return APP_FAULT_PRESSURE_COMM;
    *measured_mmhg = g_pressure_filter_valid ? g_pressure_filtered : 0.0f;
    if (g_pressure_fault_attempts != 0U &&
        (int32_t)(HAL_GetTick() - g_pressure_fault_next_ms) < 0) {
        return APP_FAULT_NONE;
    }
    fault = pressure_step_once(target_mmhg, measured_mmhg);
    if (fault == APP_FAULT_PRESSURE_SENSOR || fault == APP_FAULT_PRESSURE_COMM) {
        PressureLimit_Reset(&g_overpressure);
        /* Inhibit motion while pressure is unknown; never keep an old speed
         * running during confirmation. Preserve the operation being retried. */
        Tmc5130Driver_Stop();
        g_pressure_command = 0.0f;
        pressure_control_reset();
        g_ads_sample_valid = false;
        g_pressure_fault_attempts++;
        LOGW("[Pressure] Fault confirmation code=0x%04X attempt=%u/%u",
             (unsigned)fault, (unsigned)g_pressure_fault_attempts,
             (unsigned)APP_PRESSURE_FAULT_CONFIRM_TRIES);
        if (g_pressure_fault_attempts >= APP_PRESSURE_FAULT_CONFIRM_TRIES) {
            return fault;
        }
        g_pressure_active = was_active;
        g_pressure_recovering = was_recovering;
        /* Every retry must pass diagnostics, including restoration of normal
         * ADC configuration, before using a fresh pressure sample. */
        g_pressure_last_sensor_check_ms = HAL_GetTick() - APP_PRESSURE_SENSOR_CHECK_MS;
        g_pressure_fault_next_ms = HAL_GetTick() + APP_PRESSURE_FAULT_RETRY_MS;
        return APP_FAULT_NONE;
    }
    if (fault == APP_FAULT_NONE && g_pressure_fault_attempts != 0U &&
        !g_pressure_recovering && g_pressure_active && g_ads_sample_valid &&
        g_ads_last_unfiltered_pressure_x10 < (int32_t)(APP_MAX_SAFE_PRESSURE_MMHG * 10.0f)) {
        /* The normal control step has already rebuilt the speed command
         * using fresh pressure while the output bridge was disabled. */
        g_pressure_fault_attempts = 0U;
        Tmc5130Driver_Enable(true);
        LOGI("[Pressure] Fault confirmation cleared; pressure control resumed");
    }
    return fault;
}

static AppFault pressure_begin_recovery(const char *reason)
{
    PressureLimit_Reset(&g_overpressure);
    Tmc5130Driver_Enable(false);
    g_pressure_active = false;
    g_pressure_command = 0.0f;
    pressure_control_reset();
    g_pressure_recovering = true;
    g_pressure_recovery_attempts = 0U;
    g_pressure_recovery_next_ms = HAL_GetTick() + APP_TMC_RECOVERY_RETRY_MS;
    LOGW("[Motor] Treatment recovery started: %s", reason);
    return APP_FAULT_NONE;
}

static AppFault pressure_recovery_poll(float target_mmhg, float *measured_mmhg)
{
    Ads1220Result sensor_result;
    int32_t raw;
    float pressure;
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - g_pressure_recovery_next_ms) < 0) {
        *measured_mmhg = g_pressure_filter_valid ? g_pressure_filtered : 0.0f;
        return APP_FAULT_NONE;
    }
    g_pressure_recovery_attempts++;
    Tmc5130Driver_Enable(false);
    if (!Tmc5130Driver_Configure() ||
        !Tmc5130Driver_Probe()) {
        PressureLimit_Reset(&g_overpressure);
        LOGW("[Motor] Treatment recovery configure/probe failed attempt=%u/%u",
             (unsigned)g_pressure_recovery_attempts,
             (unsigned)APP_TMC_TREATMENT_RECOVERY_TRIES);
        if (g_pressure_recovery_attempts >= APP_TMC_TREATMENT_RECOVERY_TRIES) {
            g_pressure_recovering = false;
            return APP_FAULT_MOTOR_COMM;
        }
        g_pressure_recovery_next_ms = HAL_GetTick() + APP_TMC_RECOVERY_RETRY_MS;
        *measured_mmhg = g_pressure_filter_valid ? g_pressure_filtered : 0.0f;
        return APP_FAULT_NONE;
    }
    sensor_result = Ads1220Driver_CheckSensor();
    if (sensor_result != ADS1220_RESULT_OK ||
        !Ads1220Driver_ZeroValid() || !Ads1220Driver_ReadRaw(&raw)) {
        g_pressure_recovering = false;
        LOGE("[Motor] Treatment recovery pressure validation failed sensor=%u zero=%u",
             (unsigned)sensor_result, Ads1220Driver_ZeroValid() ? 1U : 0U);
        return sensor_result == ADS1220_RESULT_SENSOR_SHORT ||
               sensor_result == ADS1220_RESULT_SENSOR_OPEN ?
               APP_FAULT_PRESSURE_SENSOR : APP_FAULT_PRESSURE_COMM;
    }

    pressure = Ads1220Driver_PressureFromRaw(raw, target_mmhg);
    if (!isfinite(pressure)) {
        g_pressure_recovering = false;
        LOGE("[Motor] Treatment recovery rejected nonfinite pressure");
        return APP_FAULT_OVER_PRESSURE;
    }
    bool recovery_overpressure = PressureLimit_Update(&g_overpressure, pressure, HAL_GetTick());
    if (pressure >= APP_MAX_SAFE_PRESSURE_MMHG) {
        /* Keep the bridge disabled while confirming; never resume into high pressure. */
        g_ads_last_raw = raw;
        g_ads_last_unfiltered_pressure_x10 = (int32_t)(pressure * 10.0f);
        if (recovery_overpressure) {
            g_pressure_recovering = false;
            g_ads_overpressure_pending = true;
            return APP_FAULT_OVER_PRESSURE;
        }
        g_pressure_recovery_next_ms = HAL_GetTick() + APP_TMC_RECOVERY_RETRY_MS;
        *measured_mmhg = g_pressure_filter_valid ? g_pressure_filtered : 0.0f;
        return APP_FAULT_NONE;
    }
    pressure_control_reset();
    pressure_filter_reset();
    *measured_mmhg = pressure_filter_update(pressure);
    g_ads_last_raw = raw;
    g_ads_last_unfiltered_pressure_x10 = (int32_t)(pressure * 10.0f);
    g_ads_last_pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
    g_ads_sample_valid = true;
    g_pressure_stage = pressure >= target_mmhg * 0.9f ?
                       PRESS_STAGE_HOLD : PRESS_STAGE_FAST;
    g_pressure_stage_ms = now;
    g_pressure_approach_started_ms = now;
    g_pressure_last_step_ms = now;
    g_pressure_last_tmc_probe_ms = now;
    g_pressure_last_sensor_check_ms = now;

    Tmc5130Driver_Enable(true);
    g_pressure_active = true;
    g_pressure_recovering = false;
    LOGI("[Motor] Treatment recovery resumed stage=%u pressure_x10=%ld attempt=%u",
         (unsigned)g_pressure_stage, (long)(pressure * 10.0f),
         (unsigned)g_pressure_recovery_attempts);
    return APP_FAULT_NONE;
}

bool TreatmentHw_PressureTelemetry(int32_t *raw, float *pressure_mmhg,
                                   uint8_t *stage, bool *active,
                                   int32_t *speed_command,
                                   float *control_error)
{
    if (!g_ads_sample_valid || raw == NULL || pressure_mmhg == NULL ||
        stage == NULL || active == NULL || speed_command == NULL ||
        control_error == NULL) {
        return false;
    }
    *raw = g_ads_last_raw;
    *pressure_mmhg = (float)g_ads_last_pressure_x10 / 10.0f;
    *stage = (uint8_t)g_pressure_stage;
    *active = g_pressure_active;
    *speed_command = (int32_t)g_pressure_command;
    *control_error = g_pressure_error;
    return true;
}

void TreatmentHw_PressureStop(void)
{
    PressureLimit_Reset(&g_overpressure);
    g_pressure_fault_attempts = 0U;
    g_pressure_active = false;
    g_pressure_recovering = false;
    g_pressure_command = 0.0f;
    pressure_control_reset();
    Tmc5130Driver_Stop();
    if (g_ads_overpressure_pending) {
        LOGE("[ADS1220] Overpressure sample: raw=%ld zero=%ld pressure=%ld.%01ldmmHg stage=%u",
             (long)g_ads_last_raw, (long)Ads1220Driver_ZeroRaw(),
             (long)(g_ads_last_unfiltered_pressure_x10 / 10),
             (long)(g_ads_last_unfiltered_pressure_x10 % 10),
             (unsigned)g_pressure_stage);
        g_ads_overpressure_pending = false;
    }
}
