#include "eye_driver.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "app_log.h"
#include "i2c.h"
#include "main.h"
#include "product_config.h"
#include "task.h"

#define TMP112_ADDRESS      (0x48U << 1U)
#define EYE_EEPROM_ADDRESS  0xA0U
#define EYE_MARK_ADDRESS    0x01U
#define EYE_SERVICE_ADDRESS 0x03U
#define EYE_SERVICE_MARK    0x0202U

static uint32_t g_recovery_count;
static AppEyeState g_stable_state = APP_EYE_ABSENT;
static AppEyeState g_candidate_state = APP_EYE_ABSENT;
static uint8_t g_candidate_samples;
static uint32_t g_candidate_started_ms;
static uint8_t g_startup_absent_samples;
static bool g_initial_state_confirmed;
static bool g_last_probe_valid;

static bool i2c2_needs_recovery(HAL_StatusTypeDef status, uint32_t error)
{
    return status == HAL_BUSY || status == HAL_TIMEOUT ||
           (error & ~HAL_I2C_ERROR_AF) != HAL_I2C_ERROR_NONE ||
           HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY ||
           __HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET ||
           HAL_GPIO_ReadPin(EYE_SCL_GPIO_Port, EYE_SCL_Pin) == GPIO_PIN_RESET ||
           HAL_GPIO_ReadPin(EYE_SDA_GPIO_Port, EYE_SDA_Pin) == GPIO_PIN_RESET;
}

static bool i2c2_recover(HAL_StatusTypeDef status, uint32_t error)
{
    uint32_t isr = hi2c2.Instance->ISR;
    GPIO_PinState scl_before = HAL_GPIO_ReadPin(EYE_SCL_GPIO_Port, EYE_SCL_Pin);
    GPIO_PinState sda_before = HAL_GPIO_ReadPin(EYE_SDA_GPIO_Port, EYE_SDA_Pin);
    HAL_StatusTypeDef recovery;

    g_recovery_count++;
    LOGW("[I2C2] Recovery #%lu status=%u error=0x%08lX ISR=0x%08lX SCL=%u SDA=%u",
         (unsigned long)g_recovery_count, (unsigned)status,
         (unsigned long)error, (unsigned long)isr,
         scl_before == GPIO_PIN_SET ? 1U : 0U,
         sda_before == GPIO_PIN_SET ? 1U : 0U);
    recovery = I2C2_BusRecover();
    if (recovery != HAL_OK ||
        HAL_GPIO_ReadPin(EYE_SCL_GPIO_Port, EYE_SCL_Pin) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(EYE_SDA_GPIO_Port, EYE_SDA_Pin) == GPIO_PIN_RESET) {
        LOGE("[I2C2] Recovery failed status=%u SCL=%u SDA=%u",
             (unsigned)recovery,
             HAL_GPIO_ReadPin(EYE_SCL_GPIO_Port, EYE_SCL_Pin) == GPIO_PIN_SET ? 1U : 0U,
             HAL_GPIO_ReadPin(EYE_SDA_GPIO_Port, EYE_SDA_Pin) == GPIO_PIN_SET ? 1U : 0U);
        return false;
    }
    LOGI("[I2C2] Recovery complete");
    return true;
}

static HAL_StatusTypeDef i2c2_mem_read(uint16_t device, uint16_t address,
                                       uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
        status = HAL_I2C_Mem_Read(&hi2c2, device, address, I2C_MEMADD_SIZE_8BIT,
                                  data, size, timeout);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        if (i2c2_needs_recovery(status, HAL_I2C_GetError(&hi2c2)) &&
            !i2c2_recover(status, HAL_I2C_GetError(&hi2c2))) {
            return status;
        }
        if (attempt == 0U) {
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
    }
    return status;
}

static HAL_StatusTypeDef i2c2_mem_write(uint16_t device, uint16_t address,
                                        uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
        status = HAL_I2C_Mem_Write(&hi2c2, device, address, I2C_MEMADD_SIZE_8BIT,
                                   data, size, timeout);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        if (i2c2_needs_recovery(status, HAL_I2C_GetError(&hi2c2)) &&
            !i2c2_recover(status, HAL_I2C_GetError(&hi2c2))) {
            return status;
        }
        if (attempt == 0U) {
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
    }
    return status;
}

static void eye_restart_debounce(void)
{
    /* Keep the last confirmed state. A single temperature/EEPROM transaction
     * failure must not masquerade as a physical removal. */
    g_candidate_state = g_stable_state;
    g_candidate_samples = 0U;
    g_candidate_started_ms = 0U;
}

static bool eye_read_u16(uint8_t address, uint16_t *value)
{
    uint8_t bytes[2];

    if (value == NULL ||
        i2c2_mem_read(EYE_EEPROM_ADDRESS, address, bytes, sizeof(bytes), 80U) != HAL_OK) {
        return false;
    }
    *value = ((uint16_t)bytes[0] << 8U) | bytes[1];
    return true;
}

static bool eye_read_raw_state(AppEyeState *state)
{
#if PRODUCT_EYE_FUSE_ENABLED
    uint16_t marker;
#endif
    uint16_t service;

    /* Eye presence is determined by its EEPROM. This keeps a physically
     * inserted eye online when only TMP112 communication has failed, allowing
     * the application to distinguish a real 0x0101 from physical removal. */
    if (state == NULL || !eye_read_u16(EYE_SERVICE_ADDRESS, &service)
#if PRODUCT_EYE_FUSE_ENABLED
        || !eye_read_u16(EYE_MARK_ADDRESS, &marker)
#endif
        ) {
        return false;
    }
#if PRODUCT_EYE_FUSE_ENABLED
    *state = service == EYE_SERVICE_MARK ? APP_EYE_SERVICE :
             marker == 0xFFFFU ? APP_EYE_NEW : APP_EYE_CONSUMED;
#else
    *state = service == EYE_SERVICE_MARK ? APP_EYE_SERVICE : APP_EYE_NEW;
#endif
    return true;
}

void EyeDriver_Init(void)
{
    g_recovery_count = 0U;
    g_stable_state = APP_EYE_ABSENT;
    g_startup_absent_samples = 0U;
    g_initial_state_confirmed = false;
    g_last_probe_valid = false;
    eye_restart_debounce();
}

static bool eye_read_temperature(float *temperature_c)
{
    uint8_t data[2];
    int16_t raw;

    if (temperature_c == NULL ||
        i2c2_mem_read(TMP112_ADDRESS, 0x00U, data, sizeof(data), 50U) != HAL_OK) {
        return false;
    }
    raw = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    raw >>= 4;
    *temperature_c = (float)raw * 0.0625f;
    return true;
}

bool EyeDriver_ReadTemperature(float *temperature_c)
{
    /* Do not cancel an in-progress physical-removal debounce. The application
     * defers 0x0101 long enough for the eye poller to confirm removal. */
    return eye_read_temperature(temperature_c);
}

bool EyeDriver_ReadTemperatureTelemetry(float *temperature_c)
{
    return eye_read_temperature(temperature_c);
}

AppEyeState EyeDriver_ReadState(void)
{
    AppEyeState sample = APP_EYE_ABSENT;
    bool read_valid = eye_read_raw_state(&sample);
    bool state_confirmed = false;

    g_last_probe_valid = read_valid;

    /* At power-up a not-yet-ready eye EEPROM is electrically identical to a
     * removed eye shield. Do not expose that transient as a real state.
     * A valid inserted device still uses the normal five-sample debounce;
     * absence needs a longer run of failed probes only for this first result. */
    if (!g_initial_state_confirmed && !read_valid) {
        g_candidate_state = APP_EYE_ABSENT;
        g_candidate_samples = 0U;
        if (g_startup_absent_samples < 0xFFU) {
            g_startup_absent_samples++;
        }
        if (g_startup_absent_samples >= APP_EYE_STARTUP_ABSENT_SAMPLES) {
            g_stable_state = APP_EYE_ABSENT;
            g_initial_state_confirmed = true;
            LOGI("[Eye I2C] Startup absence confirmed after %u samples",
                 (unsigned)g_startup_absent_samples);
        }
        return g_stable_state;
    }
    if (read_valid) {
        g_startup_absent_samples = 0U;
    }
    if (sample == g_stable_state) {
        g_candidate_state = sample;
        g_candidate_samples = 0U;
        g_candidate_started_ms = 0U;
        g_initial_state_confirmed = true;
        return g_stable_state;
    }
    if (sample != g_candidate_state) {
        g_candidate_state = sample;
        g_candidate_samples = 1U;
        g_candidate_started_ms = HAL_GetTick();
    } else if (g_candidate_samples < 0xFFU) {
        g_candidate_samples++;
    }
    if (sample == APP_EYE_ABSENT) {
        state_confirmed = HAL_GetTick() - g_candidate_started_ms >=
                          APP_EYE_RECONNECT_GRACE_MS;
    } else {
        state_confirmed = g_candidate_samples >= APP_EYE_INSERT_CONFIRM_SAMPLES;
    }
    if (state_confirmed) {
        LOGI("[Eye I2C] Confirmed state=%u after %lums (%u samples)",
             (unsigned)sample,
             (unsigned long)(HAL_GetTick() - g_candidate_started_ms),
             (unsigned)g_candidate_samples);
        g_stable_state = sample;
        g_candidate_samples = 0U;
        g_candidate_started_ms = 0U;
        g_initial_state_confirmed = true;
    }
    return g_stable_state;
}

bool EyeDriver_InitialStateConfirmed(void)
{
    return g_initial_state_confirmed;
}

bool EyeDriver_LastProbeValid(void)
{
    return g_last_probe_valid;
}

bool EyeDriver_MarkConsumed(void)
{
#if PRODUCT_EYE_FUSE_ENABLED
    uint8_t bytes[2] = {0U, 1U};
    uint16_t verify = 0xFFFFU;

    if (i2c2_mem_write(EYE_EEPROM_ADDRESS, EYE_MARK_ADDRESS,
                       bytes, sizeof(bytes), 80U) != HAL_OK) {
        eye_restart_debounce();
        LOGE("[Eye I2C] Failed to write consumed marker");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(6U));
    if (!eye_read_u16(EYE_MARK_ADDRESS, &verify) || verify != 1U) {
        eye_restart_debounce();
        LOGE("[Eye I2C] Consumed marker verify failed value=0x%04X", (unsigned)verify);
        return false;
    }
    g_stable_state = APP_EYE_CONSUMED;
    g_candidate_state = APP_EYE_CONSUMED;
    g_candidate_samples = 0U;
#endif
    return true;
}

void EyeDriver_Erase(void)
{
    uint8_t value = 0xFFU;

    for (uint16_t address = 0U; address < 256U; ++address) {
        if (i2c2_mem_write(EYE_EEPROM_ADDRESS, (uint8_t)address,
                           &value, 1U, 80U) != HAL_OK) {
            LOGE("[Eye I2C] EEPROM erase stopped at address=0x%02X", (unsigned)address);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(6U));
    }
}
