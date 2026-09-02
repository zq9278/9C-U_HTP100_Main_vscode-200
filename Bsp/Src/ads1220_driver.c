#include "ads1220_driver.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "app_log.h"
#include "main.h"
#include "pressure_tuning.h"
#include "spi.h"
#include "task.h"

#define ADS_SPI_TIMEOUT_MS  20U
#define ADS_DRDY_TIMEOUT_MS 20U
#define ADS_POWER_SETTLE_MS  200U
#define ADS_RESET_SETTLE_MS  50U
#define ADS_INIT_RETRY_MS    100U
#define ADS_INIT_ATTEMPTS    3U
#define ADS_CONFIG_0         0x3EU
#define ADS_CONFIG_1         0x94U
#define ADS_CONFIG_1_BCS     (ADS_CONFIG_1 | 0x01U)
#define ADS_CONFIG_2         0x98U
#define ADS_CONFIG_3         0x00U
#define ADS_BCS_SETTLE_MS    3U

static int32_t g_zero_raw;
static int32_t g_sensor_diagnostic_raw;
static bool g_zero_valid;
static bool g_ready;

static HAL_StatusTypeDef ads_command(uint8_t command)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, &command, 1U, ADS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return status;
}

static HAL_StatusTypeDef ads_write_register(uint8_t index, uint8_t value)
{
    uint8_t frame[2] = {(uint8_t)(0x40U | (index << 2U)), value};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, frame, sizeof(frame), ADS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return status;
}

static bool ads_verify_registers(void)
{
    static const uint8_t expected[4] = {
        ADS_CONFIG_0, ADS_CONFIG_1, ADS_CONFIG_2, ADS_CONFIG_3
    };
    uint8_t command = 0x23U;
    uint8_t actual[4] = {0U};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, &command, 1U, ADS_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi2, actual, sizeof(actual), ADS_SPI_TIMEOUT_MS);
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (status != HAL_OK || memcmp(actual, expected, sizeof(expected)) != 0) {
        LOGE("[ADS1220] Verify failed status=%u spi_state=%u spi_error=0x%08lX "
             "actual=%02X %02X %02X %02X expected=%02X %02X %02X %02X",
             (unsigned)status, (unsigned)hspi2.State,
             (unsigned long)hspi2.ErrorCode,
             (unsigned)actual[0], (unsigned)actual[1],
             (unsigned)actual[2], (unsigned)actual[3],
             (unsigned)expected[0], (unsigned)expected[1],
             (unsigned)expected[2], (unsigned)expected[3]);
        return false;
    }
    return true;
}

static bool ads_init_attempt(uint8_t attempt)
{
    static const uint8_t values[4] = {
        ADS_CONFIG_0, ADS_CONFIG_1, ADS_CONFIG_2, ADS_CONFIG_3
    };

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (ads_command(0x06U) != HAL_OK) {
        LOGE("[ADS1220] Reset command failed: attempt=%u status=%u error=0x%08lX",
             (unsigned)attempt, (unsigned)hspi2.State,
             (unsigned long)hspi2.ErrorCode);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(ADS_RESET_SETTLE_MS));
    for (uint8_t index = 0U; index < 4U; ++index) {
        if (ads_write_register(index, values[index]) != HAL_OK) {
            LOGE("[ADS1220] Register write failed: attempt=%u reg=%u state=%u error=0x%08lX",
                 (unsigned)attempt, (unsigned)index, (unsigned)hspi2.State,
                 (unsigned long)hspi2.ErrorCode);
            return false;
        }
    }
    if (!ads_verify_registers()) {
        LOGE("[ADS1220] Register readback verification failed: attempt=%u",
             (unsigned)attempt);
        return false;
    }
    if (ads_command(0x08U) != HAL_OK) {
        LOGE("[ADS1220] START command failed: attempt=%u state=%u error=0x%08lX",
             (unsigned)attempt, (unsigned)hspi2.State,
             (unsigned long)hspi2.ErrorCode);
        return false;
    }
    g_ready = true;
    LOGI("[ADS1220] Initialized and registers verified: attempt=%u",
         (unsigned)attempt);
    return true;
}

void Ads1220Driver_PreparePins(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

bool Ads1220Driver_Init(void)
{
    g_ready = false;
    g_zero_valid = false;
    vTaskDelay(pdMS_TO_TICKS(ADS_POWER_SETTLE_MS));
    for (uint8_t attempt = 1U; attempt <= ADS_INIT_ATTEMPTS; ++attempt) {
        if (ads_init_attempt(attempt)) {
            return true;
        }
        if (attempt < ADS_INIT_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(ADS_INIT_RETRY_MS));
        }
    }
    LOGE("[ADS1220] Initialization unavailable after %u attempts",
         (unsigned)ADS_INIT_ATTEMPTS);
    return false;
}

bool Ads1220Driver_RetryInit(void)
{
    if (g_ready) {
        return true;
    }
    return ads_init_attempt(1U);
}

bool Ads1220Driver_Ready(void)
{
    return g_ready;
}

bool Ads1220Driver_ReadRaw(int32_t *raw)
{
    uint32_t started = HAL_GetTick();
    uint8_t command = 0x10U;
    uint8_t bytes[3];
    HAL_StatusTypeDef status;

    if (raw == NULL || !g_ready) {
        return false;
    }
    while (HAL_GPIO_ReadPin(ADS1220_DRDY_GPIO_Port, ADS1220_DRDY_Pin) != GPIO_PIN_RESET) {
        if (HAL_GetTick() - started >= ADS_DRDY_TIMEOUT_MS) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, &command, 1U, ADS_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi2, bytes, sizeof(bytes), ADS_SPI_TIMEOUT_MS);
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return false;
    }

    *raw = ((int32_t)bytes[0] << 16U) | ((int32_t)bytes[1] << 8U) | bytes[2];
    if ((*raw & 0x00800000L) != 0L) {
        *raw |= (int32_t)0xFF000000L;
    }
    return true;
}

Ads1220Result Ads1220Driver_CheckSensor(void)
{
    int32_t diagnostic_raw = 0;
    int64_t magnitude;
    bool sample_ok;
    bool restore_ok;

    if (!g_ready || ads_write_register(1U, ADS_CONFIG_1_BCS) != HAL_OK) {
        return ADS1220_RESULT_COMM;
    }
    /* BCS corrupts precision readings by design. Take one dedicated sample,
     * then always restore the normal configuration before returning. */
    vTaskDelay(pdMS_TO_TICKS(ADS_BCS_SETTLE_MS));
    sample_ok = Ads1220Driver_ReadRaw(&diagnostic_raw);
    restore_ok = ads_write_register(1U, ADS_CONFIG_1) == HAL_OK;
    if (!sample_ok || !restore_ok) {
        return ADS1220_RESULT_COMM;
    }

    magnitude = diagnostic_raw < 0 ? -(int64_t)diagnostic_raw : diagnostic_raw;
    g_sensor_diagnostic_raw = diagnostic_raw;
    if (magnitude <= APP_PRESSURE_SHORT_RAW_LIMIT) {
        LOGE("[ADS1220] Sensor short suspected: bcs_raw=%ld limit=%ld",
             (long)diagnostic_raw, (long)APP_PRESSURE_SHORT_RAW_LIMIT);
        return ADS1220_RESULT_SENSOR_SHORT;
    }
    if (magnitude >= APP_PRESSURE_OPEN_RAW_LIMIT) {
        LOGE("[ADS1220] Sensor open suspected: bcs_raw=%ld limit=%ld",
             (long)diagnostic_raw, (long)APP_PRESSURE_OPEN_RAW_LIMIT);
        return ADS1220_RESULT_SENSOR_OPEN;
    }
    return ADS1220_RESULT_OK;
}

float Ads1220Driver_PressureFromRaw(int32_t raw, float target_mmhg)
{
    const PressureTuningProfile *profile = PressureTuning_ProfileForTarget(target_mmhg);
    float input_voltage = ((float)(-(raw - g_zero_raw)) * 3.3f) / 8388607.0f;
    float load_voltage = input_voltage / 128.0f;
    float weight_g = load_voltage /
                     ((profile->sensitivity_mv_v / 1000.0f) * 3.3f) * 1000.0f;
    float force_n = (weight_g / 1000.0f) * APP_STANDARD_GRAVITY_M_S2;
    float pressure = force_n /
                     (APP_PRESSURE_EFFECTIVE_AREA_MM2 * 1.0e-6f *
                      APP_PASCAL_PER_MMHG) + profile->offset_mmhg;

    return pressure < 0.0f ? 0.0f : pressure;
}

Ads1220Result Ads1220Driver_ZeroCalibrate(void)
{
    int32_t samples[32];
    int64_t sum = 0;
    Ads1220Result sensor_result = Ads1220Driver_CheckSensor();

    if (sensor_result != ADS1220_RESULT_OK) {
        g_zero_valid = false;
        return sensor_result;
    }
    LOGI("[ADS1220] Sensor diagnostic passed: bcs_raw=%ld limits=%ld..%ld",
         (long)g_sensor_diagnostic_raw,
         (long)APP_PRESSURE_SHORT_RAW_LIMIT,
         (long)APP_PRESSURE_OPEN_RAW_LIMIT);

    vTaskDelay(pdMS_TO_TICKS(500U));
    for (uint8_t index = 0U; index < 32U; ++index) {
        if (!Ads1220Driver_ReadRaw(&samples[index])) {
            LOGE("[ADS1220] Zero calibration read failed: sample=%u", (unsigned)index);
            g_zero_valid = false;
            return ADS1220_RESULT_COMM;
        }
        for (uint8_t position = index;
             position > 0U && samples[position] < samples[position - 1U];
             --position) {
            int32_t temporary = samples[position];
            samples[position] = samples[position - 1U];
            samples[position - 1U] = temporary;
        }
    }
    if (Ads1220Driver_PressureFromRaw(samples[27] - samples[4] + g_zero_raw,
                                      350.0f) > 20.0f) {
        LOGE("[ADS1220] Zero calibration unstable: low=%ld high=%ld",
             (long)samples[4], (long)samples[27]);
        g_zero_valid = false;
        return ADS1220_RESULT_ZERO_UNSTABLE;
    }
    for (uint8_t index = 4U; index < 28U; ++index) {
        sum += samples[index];
    }
    g_zero_raw = (int32_t)(sum / 24);
    g_zero_valid = true;
    LOGI("[ADS1220] Zero calibrated: raw=%ld low=%ld high=%ld",
         (long)g_zero_raw, (long)samples[4], (long)samples[27]);
    return ADS1220_RESULT_OK;
}

bool Ads1220Driver_ZeroValid(void)
{
    return g_zero_valid;
}

int32_t Ads1220Driver_ZeroRaw(void)
{
    return g_zero_raw;
}
