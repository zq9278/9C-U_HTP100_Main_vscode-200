#include "treatment_hw.h"

#include <stdint.h>

#include "ads1220_driver.h"
#include "app_config.h"
#include "app_log.h"
#include "eye_driver.h"
#include "pressure_tuning.h"
#include "tim.h"
#include "tmc5130_driver.h"

typedef enum {
    PRESS_STAGE_FAST = 0,
    PRESS_STAGE_APPROACH,
    PRESS_STAGE_HOLD,
    PRESS_STAGE_RETRACT
} PressureStage;

static bool g_home_active;
static uint32_t g_home_started_ms;
static uint32_t g_home_last_poll_ms;
static uint32_t g_home_last_log_ms;
static uint32_t g_home_spi_errors;

static uint32_t g_ads_last_log_ms;
static int32_t g_ads_last_raw;
static int32_t g_ads_last_pressure_x10;
static bool g_ads_sample_valid;
static bool g_ads_overpressure_pending;
static PressureStage g_pressure_stage;
static uint32_t g_pressure_stage_ms;
static float g_pressure_integral;
static bool g_pressure_active;

static float g_heat_integral;
static float g_heat_previous_error;
static bool g_heat_pwm_started;

void TreatmentHw_Init(void)
{
    Tmc5130Driver_Init();
    TreatmentHw_SafeOutputsOff();
}

void TreatmentHw_SafeOutputsOff(void)
{
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    (void)HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
    g_heat_pwm_started = false;
    Tmc5130Driver_Stop();
}

bool TreatmentHw_HeaterControl(float target_c, float *measured_c)
{
    float error;
    float derivative;
    float pwm;

    if (!EyeDriver_ReadTemperature(measured_c)) {
        TreatmentHw_SafeOutputsOff();
        return false;
    }
    error = target_c - *measured_c;
    if (error <= 0.0f) {
        g_heat_integral = 0.0f;
        pwm = 0.0f;
    } else if (error > 10.0f) {
        pwm = 12.0f * error;
        g_heat_integral = 0.0f;
    } else {
        g_heat_integral += error * 0.15f;
        if (g_heat_integral > 100.0f) {
            g_heat_integral = 100.0f;
        }
        derivative = (error - g_heat_previous_error) / 0.15f;
        pwm = 15.0f * error + 2.0f * g_heat_integral + 0.5f * derivative;
    }
    g_heat_previous_error = error;
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > 254.0f) pwm = 254.0f;
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, (uint32_t)pwm);
    if (!g_heat_pwm_started) {
        g_heat_pwm_started = HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) == HAL_OK;
    }
    return g_heat_pwm_started;
}

bool TreatmentHw_HomeBegin(void)
{
    Tmc5130Driver_Enable(true);
    if (!Tmc5130Driver_Configure() ||
        Tmc5130Driver_Write(0x27U, 0x00010000U) != HAL_OK ||
        Tmc5130Driver_Write(0x20U, 1U) != HAL_OK) {
        LOGE("[Motor] Home begin failed while configuring TMC5130");
        Tmc5130Driver_Enable(false);
        return false;
    }
    g_home_started_ms = HAL_GetTick();
    g_home_last_poll_ms = 0U;
    g_home_last_log_ms = g_home_started_ms;
    g_home_spi_errors = 0U;
    g_home_active = true;
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
    if (now - g_home_last_poll_ms < 100U) {
        return APP_ASYNC_BUSY;
    }
    g_home_last_poll_ms = now;
    if (Tmc5130Driver_Read(0x35U, &ramp_status) != HAL_OK) {
        g_home_spi_errors++;
        if (now - g_home_last_log_ms >= 500U) {
            LOGW("[Motor] Home poll SPI failure elapsed=%lu ms count=%lu",
                 (unsigned long)(now - g_home_started_ms),
                 (unsigned long)g_home_spi_errors);
            g_home_last_log_ms = now;
        }
        return APP_ASYNC_BUSY;
    }
    if (now - g_home_last_log_ms >= 500U) {
        LOGI("[Motor] Home polling elapsed=%lu ms RAMP_STAT=0x%08lX",
             (unsigned long)(now - g_home_started_ms),
             (unsigned long)ramp_status);
        g_home_last_log_ms = now;
    }
    if ((ramp_status & 0x02U) == 0U) {
        return APP_ASYNC_BUSY;
    }

    (void)Tmc5130Driver_Write(0x21U, 0U);
    (void)Tmc5130Driver_Write(0x2DU, 0U);
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
    Tmc5130Driver_Stop();
}

bool TreatmentHw_PressureStart(float target_mmhg)
{
    (void)target_mmhg;
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
    g_pressure_integral = 0.0f;
    g_pressure_active = true;
    return true;
}

bool TreatmentHw_PressureStep(float target_mmhg, float *measured_mmhg)
{
    const PressureTuningProfile *profile = PressureTuning_ProfileForTarget(target_mmhg);
    int32_t raw;
    uint32_t now = HAL_GetTick();
    float error;
    float command;

    if (!g_pressure_active || measured_mmhg == NULL ||
        !Ads1220Driver_ReadRaw(&raw)) {
        (void)Tmc5130Driver_SetSpeed(0);
        return false;
    }
    *measured_mmhg = Ads1220Driver_PressureFromRaw(raw, target_mmhg);
    g_ads_last_raw = raw;
    g_ads_last_pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
    g_ads_sample_valid = true;
    if (*measured_mmhg >= APP_MAX_SAFE_PRESSURE_MMHG) {
        g_ads_overpressure_pending = true;
    }
    if (now - g_ads_last_log_ms >= APP_SENSOR_LOG_PERIOD_MS) {
        int32_t pressure_x10 = (int32_t)(*measured_mmhg * 10.0f);
        LOGI("[ADS1220] raw=%ld zero=%ld pressure=%ld.%01ldmmHg target=%ld stage=%u",
             (long)raw, (long)Ads1220Driver_ZeroRaw(),
             (long)(pressure_x10 / 10), (long)(pressure_x10 % 10),
             (long)target_mmhg, (unsigned)g_pressure_stage);
        g_ads_last_log_ms = now;
    }

    switch (g_pressure_stage) {
    case PRESS_STAGE_FAST:
        if (*measured_mmhg >= profile->hold_threshold) {
            /* A quick resume may start with residual load. Never issue even
             * one fast-forward cycle when pressure is already at hold entry. */
            if (!Tmc5130Driver_SetSpeed(0)) return false;
            g_pressure_stage = PRESS_STAGE_HOLD;
            g_pressure_stage_ms = now;
            g_pressure_integral = 0.0f;
        } else if (*measured_mmhg >= profile->approach_threshold) {
            if (!Tmc5130Driver_SetSpeed((int32_t)profile->approach_speed)) return false;
            g_pressure_stage = PRESS_STAGE_APPROACH;
            g_pressure_stage_ms = now;
        } else {
            if (!Tmc5130Driver_SetSpeed((int32_t)profile->fast_speed)) return false;
        }
        break;
    case PRESS_STAGE_APPROACH:
        if (!Tmc5130Driver_SetSpeed((int32_t)profile->approach_speed)) return false;
        if (*measured_mmhg >= profile->hold_threshold) {
            g_pressure_stage = PRESS_STAGE_HOLD;
            g_pressure_stage_ms = now;
            g_pressure_integral = 0.0f;
        }
        break;
    case PRESS_STAGE_HOLD:
        error = target_mmhg - *measured_mmhg;
        g_pressure_integral += error * 0.02f;
        if (g_pressure_integral > 5000.0f) g_pressure_integral = 5000.0f;
        if (g_pressure_integral < -5000.0f) g_pressure_integral = -5000.0f;
        command = profile->kp * error + profile->ki * g_pressure_integral;
        if (command > 50000.0f) command = 50000.0f;
        if (command < -50000.0f) command = -50000.0f;
        if (!Tmc5130Driver_SetSpeed((int32_t)command)) return false;
        if (now - g_pressure_stage_ms >= profile->hold_ms) {
            g_pressure_stage = PRESS_STAGE_RETRACT;
            g_pressure_stage_ms = now;
        }
        break;
    case PRESS_STAGE_RETRACT:
        if (!Tmc5130Driver_SetSpeed(-(int32_t)profile->retract_speed)) return false;
        if (now - g_pressure_stage_ms >= profile->retract_ms) {
            g_pressure_stage = PRESS_STAGE_APPROACH;
            g_pressure_stage_ms = now;
        }
        break;
    default:
        return false;
    }
    return true;
}

bool TreatmentHw_PressureTelemetry(int32_t *raw, float *pressure_mmhg,
                                   uint8_t *stage, bool *active)
{
    if (!g_ads_sample_valid || raw == NULL || pressure_mmhg == NULL ||
        stage == NULL || active == NULL) {
        return false;
    }
    *raw = g_ads_last_raw;
    *pressure_mmhg = (float)g_ads_last_pressure_x10 / 10.0f;
    *stage = (uint8_t)g_pressure_stage;
    *active = g_pressure_active;
    return true;
}

void TreatmentHw_PressureStop(void)
{
    g_pressure_active = false;
    Tmc5130Driver_Stop();
    if (g_ads_overpressure_pending) {
        LOGE("[ADS1220] Overpressure sample: raw=%ld zero=%ld pressure=%ld.%01ldmmHg stage=%u",
             (long)g_ads_last_raw, (long)Ads1220Driver_ZeroRaw(),
             (long)(g_ads_last_pressure_x10 / 10),
             (long)(g_ads_last_pressure_x10 % 10), (unsigned)g_pressure_stage);
        g_ads_overpressure_pending = false;
    }
}
