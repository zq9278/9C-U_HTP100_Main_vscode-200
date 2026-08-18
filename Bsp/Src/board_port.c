#include "board_port.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "task.h"
#include "tim.h"
#include "usart.h"

#define TMP112_ADDRESS             (0x48U << 1U)
#define EYE_EEPROM_ADDRESS         0xA0U
#define EYE_MARK_ADDRESS           0x01U
#define EYE_SERVICE_ADDRESS        0x03U
#define EYE_SERVICE_MARK           0x0202U
#define BQ25895_ADDRESS            0xD4U
#define BQ27441_ADDRESS            0xAAU
#define BQ27441_REG_VOLTAGE        0x04U
#define BQ27441_REG_SOC            0x1CU
#define SCREEN_RX_DMA_SIZE         128U
#define SCREEN_RING_SIZE           512U
#define TMC_SPI_TIMEOUT_MS         20U
#define ADS_SPI_TIMEOUT_MS         20U
#define ADS_DRDY_TIMEOUT_MS        20U
#define WS_LED_COUNT               15U
#define WS_BITS_PER_LED            24U
#define WS_RESET_SLOTS             150U
#define WS_BUFFER_SIZE             (WS_LED_COUNT * WS_BITS_PER_LED + WS_RESET_SLOTS)

typedef enum {
    PRESS_STAGE_FAST = 0,
    PRESS_STAGE_APPROACH,
    PRESS_STAGE_HOLD,
    PRESS_STAGE_RETRACT
} PressureStage;

typedef struct {
    float fast_speed;
    float approach_speed;
    float retract_speed;
    float approach_threshold;
    float hold_threshold;
    uint32_t hold_ms;
    uint32_t retract_ms;
    float kp;
    float ki;
} PressureProfile;

static const PressureProfile g_profiles[] = {
    {20000.0f, 4000.0f, 15000.0f, 100.0f, 110.0f, 1000U, 600U, 200.0f, 0.0f},
    {35000.0f, 4000.0f, 15000.0f, 100.0f, 110.5f, 1000U, 600U, 200.0f, 10.0f},
    {50000.0f, 4000.0f, 20000.0f, 100.0f, 110.0f, 1000U, 800U, 200.0f, 10.0f},
    {50000.0f, 4000.0f, 20000.0f, 100.0f, 110.0f, 1000U, 800U, 150.0f, 8.0f},
    {50000.0f, 4000.0f, 20000.0f, 100.0f, 110.0f, 1000U, 1000U, 160.0f, 1.0f}
};

static uint8_t g_uart_dma[SCREEN_RX_DMA_SIZE];
static uint8_t g_uart_ring[SCREEN_RING_SIZE];
static volatile uint16_t g_uart_write;
static volatile uint16_t g_uart_read;
static volatile bool g_button_event;
static volatile bool g_power_loss_event;

static bool g_home_active;
static uint32_t g_home_started_ms;
static uint32_t g_home_last_poll_ms;
static int32_t g_ads_zero_raw;
static bool g_ads_zero_valid;
static PressureStage g_pressure_stage;
static uint32_t g_pressure_stage_ms;
static float g_pressure_integral;
static bool g_pressure_active;

static float g_heat_integral;
static float g_heat_previous_error;
static bool g_heat_pwm_started;

static volatile bool g_ws_ready = true;
static uint16_t g_ws_buffer[WS_BUFFER_SIZE];
static AppLedState g_led_requested = APP_LED_IDLE;
static bool g_led_blink;
static uint32_t g_led_last_ms;
static bool g_led_phase;

static uint16_t g_counters[3];

static void ee_delay_us(uint16_t microseconds)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    (void)HAL_TIM_Base_Start(&htim7);
    while (__HAL_TIM_GET_COUNTER(&htim7) < microseconds) { }
    (void)HAL_TIM_Base_Stop(&htim7);
}

static void ee_sda(bool high)
{
    HAL_GPIO_WritePin(EE_SDA_GPIO_Port, EE_SDA_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ee_scl(bool high)
{
    HAL_GPIO_WritePin(EE_SCL_GPIO_Port, EE_SCL_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ee_start(void)
{
    ee_sda(true); ee_scl(true); ee_delay_us(4U);
    ee_sda(false); ee_delay_us(4U); ee_scl(false);
}

static void ee_stop(void)
{
    ee_scl(false); ee_sda(false); ee_delay_us(2U);
    ee_scl(true); ee_delay_us(4U); ee_sda(true); ee_delay_us(4U);
}

static bool ee_write_bus_byte(uint8_t value)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        ee_sda((value & 0x80U) != 0U);
        value <<= 1U;
        ee_delay_us(2U); ee_scl(true); ee_delay_us(2U); ee_scl(false);
    }
    ee_sda(true); ee_delay_us(1U); ee_scl(true); ee_delay_us(2U);
    bool acknowledged = HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) == GPIO_PIN_RESET;
    ee_scl(false);
    return acknowledged;
}

static uint8_t ee_read_bus_byte(bool acknowledge)
{
    uint8_t value = 0U;
    ee_sda(true);
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        value <<= 1U;
        ee_scl(true); ee_delay_us(2U);
        if (HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) == GPIO_PIN_SET) value |= 1U;
        ee_scl(false); ee_delay_us(2U);
    }
    ee_sda(!acknowledge); ee_scl(true); ee_delay_us(2U); ee_scl(false); ee_sda(true);
    return value;
}

static bool ee_read_byte(uint8_t address, uint8_t *value)
{
    if (value == NULL) return false;
    ee_start();
    if (!ee_write_bus_byte(0xA0U) || !ee_write_bus_byte(address)) { ee_stop(); return false; }
    ee_start();
    if (!ee_write_bus_byte(0xA1U)) { ee_stop(); return false; }
    *value = ee_read_bus_byte(false);
    ee_stop();
    return true;
}

static bool ee_write_byte(uint8_t address, uint8_t value)
{
    ee_start();
    if (!ee_write_bus_byte(0xA0U) || !ee_write_bus_byte(address) ||
        !ee_write_bus_byte(value)) {
        ee_stop();
        return false;
    }
    ee_stop();
    vTaskDelay(pdMS_TO_TICKS(6U));
    return true;
}

static uint16_t ee_read_u16(uint8_t address, uint16_t default_value)
{
    uint8_t high;
    uint8_t low;
    if (!ee_read_byte(address, &high) || !ee_read_byte((uint8_t)(address + 1U), &low) ||
        (high == 0xFFU && low == 0xFFU)) {
        return default_value;
    }
    return (uint16_t)((uint16_t)high << 8U) | low;
}

static bool ee_write_u16(uint8_t address, uint16_t value)
{
    return ee_write_byte(address, (uint8_t)(value >> 8U)) &&
           ee_write_byte((uint8_t)(address + 1U), (uint8_t)value);
}

static uint32_t board_now_ms(void)
{
    return HAL_GetTick();
}

static void tmc_enable(bool enable)
{
    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static HAL_StatusTypeDef tmc_write(uint8_t address, uint32_t value)
{
    uint8_t frame[5] = {
        (uint8_t)(address | 0x80U),
        (uint8_t)(value >> 24U),
        (uint8_t)(value >> 16U),
        (uint8_t)(value >> 8U),
        (uint8_t)value
    };
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi1, frame, sizeof(frame), TMC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    return status;
}

static HAL_StatusTypeDef tmc_read(uint8_t address, uint32_t *value)
{
    uint8_t command[5] = {(uint8_t)(address & 0x7FU), 0U, 0U, 0U, 0U};
    uint8_t response[5] = {0U};
    HAL_StatusTypeDef status;

    if (value == NULL) {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, command, response, sizeof(command), TMC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return status;
    }
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, command, response, sizeof(command), TMC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    if (status == HAL_OK) {
        *value = ((uint32_t)response[1] << 24U) |
                 ((uint32_t)response[2] << 16U) |
                 ((uint32_t)response[3] << 8U) |
                 response[4];
    }
    return status;
}

static bool tmc_configure(void)
{
    static const struct { uint8_t address; uint32_t value; } registers[] = {
        {0x01U, 0x00000001U}, {0x6CU, 0x000300C3U}, {0x10U, 0x00000C04U},
        {0x11U, 0x0000000AU}, {0x00U, 0x00000004U}, {0x13U, 0x000001F4U},
        {0x70U, 0x000701C8U}, {0x25U, 0x00015000U}, {0x24U, 0x00000001U},
        {0x26U, 0x00001001U}, {0x27U, 0x00004000U}, {0x28U, 0x00001FFFU},
        {0x2AU, 0x00008000U}, {0x2BU, 0x0000000AU}, {0x20U, 0x00000000U}
    };

    for (size_t i = 0U; i < sizeof(registers) / sizeof(registers[0]); ++i) {
        if (tmc_write(registers[i].address, registers[i].value) != HAL_OK) {
            return false;
        }
    }
    return true;
}

static bool tmc_set_speed(int32_t speed)
{
    if (speed == 0) {
        return tmc_write(0x27U, 0U) == HAL_OK;
    }
    uint32_t magnitude = speed < 0 ? (uint32_t)(-speed) : (uint32_t)speed;
    uint32_t ramp_mode = speed < 0 ? 1U : 2U;

    return tmc_write(0x20U, ramp_mode) == HAL_OK &&
           tmc_write(0x27U, magnitude) == HAL_OK;
}

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
    static const uint8_t expected[4] = {0x3EU, 0x94U, 0x98U, 0x00U};
    uint8_t command = 0x23U;
    uint8_t actual[4] = {0U};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, &command, 1U, ADS_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi2, actual, sizeof(actual), ADS_SPI_TIMEOUT_MS);
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return status == HAL_OK && memcmp(actual, expected, sizeof(expected)) == 0;
}

static bool ads_init(void)
{
    static const uint8_t values[4] = {0x3EU, 0x94U, 0x98U, 0x00U};
    g_ads_zero_valid = false;
    if (ads_command(0x06U) != HAL_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50U));
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (ads_write_register(i, values[i]) != HAL_OK) {
            return false;
        }
    }
    if (!ads_verify_registers()) {
        return false;
    }
    return ads_command(0x08U) == HAL_OK;
}

static bool ads_read_raw(int32_t *raw)
{
    uint32_t started = HAL_GetTick();
    uint8_t command = 0x10U;
    uint8_t bytes[3];
    HAL_StatusTypeDef status;

    if (raw == NULL) {
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

static float ads_sensitivity(float target)
{
    if (target <= 200.0f) return 0.380f;
    return 0.370f;
}

static float ads_pressure_from_raw(int32_t raw, float target)
{
    float input_voltage = ((float)(-(raw - g_ads_zero_raw)) * 3.3f) / 8388607.0f;
    float load_voltage = input_voltage / 128.0f;
    float weight_g = load_voltage / ((ads_sensitivity(target) / 1000.0f) * 3.3f) * 1000.0f;
    float pressure = (weight_g / 1000.0f) * 9.8f * 80.0f;
    return pressure < 0.0f ? 0.0f : pressure;
}

static bool board_pressure_zero_calibrate(void)
{
    int32_t samples[32];
    int64_t sum = 0;

    vTaskDelay(pdMS_TO_TICKS(500U));
    for (uint8_t i = 0U; i < 32U; ++i) {
        if (!ads_read_raw(&samples[i])) {
            return false;
        }
        for (uint8_t p = i; p > 0U && samples[p] < samples[p - 1U]; --p) {
            int32_t temporary = samples[p];
            samples[p] = samples[p - 1U];
            samples[p - 1U] = temporary;
        }
    }
    if (ads_pressure_from_raw(samples[27] - samples[4] + g_ads_zero_raw, 350.0f) > 20.0f) {
        return false;
    }
    for (uint8_t i = 4U; i < 28U; ++i) {
        sum += samples[i];
    }
    g_ads_zero_raw = (int32_t)(sum / 24);
    g_ads_zero_valid = true;
    return true;
}

static bool tmp112_read(float *temperature)
{
    uint8_t data[2];
    int16_t raw;

    if (temperature == NULL ||
        HAL_I2C_Mem_Read(&hi2c2, TMP112_ADDRESS, 0x00U, I2C_MEMADD_SIZE_8BIT,
                         data, sizeof(data), 50U) != HAL_OK) {
        return false;
    }
    raw = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    raw >>= 4;
    *temperature = (float)raw * 0.0625f;
    return true;
}

static bool eye_read_u16(uint8_t address, uint16_t *value)
{
    uint8_t bytes[2];
    if (value == NULL ||
        HAL_I2C_Mem_Read(&hi2c2, EYE_EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
                         bytes, sizeof(bytes), 80U) != HAL_OK) {
        return false;
    }
    *value = ((uint16_t)bytes[0] << 8U) | bytes[1];
    return true;
}

static AppEyeState board_eye_read_state(void)
{
    uint16_t marker;
    uint16_t service;

    if (HAL_I2C_IsDeviceReady(&hi2c2, TMP112_ADDRESS, 1U, 20U) != HAL_OK) {
        return APP_EYE_ABSENT;
    }
    if (!eye_read_u16(EYE_SERVICE_ADDRESS, &service) ||
        !eye_read_u16(EYE_MARK_ADDRESS, &marker)) {
        return APP_EYE_ABSENT;
    }
    if (service == EYE_SERVICE_MARK) {
        return APP_EYE_SERVICE;
    }
    return marker == 0xFFFFU ? APP_EYE_NEW : APP_EYE_CONSUMED;
}

static bool board_eye_mark_consumed(void)
{
    uint8_t bytes[2] = {0U, 1U};
    uint16_t verify;

    if (HAL_I2C_Mem_Write(&hi2c2, EYE_EEPROM_ADDRESS, EYE_MARK_ADDRESS,
                          I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes), 80U) != HAL_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(6U));
    if (!eye_read_u16(EYE_MARK_ADDRESS, &verify) || verify != 1U) {
        return false;
    }
    (void)ee_write_u16(0xF2U, (uint16_t)(ee_read_u16(0xF2U, 0U) + 1U));
    return true;
}

static void board_safe_outputs_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
    g_heat_pwm_started = false;
    (void)tmc_set_speed(0);
    tmc_enable(false);
}

static bool board_heater_control(float target, float *measured)
{
    float error;
    float derivative;
    float pwm;

    if (!tmp112_read(measured)) {
        board_safe_outputs_off();
        return false;
    }
    error = target - *measured;
    if (error <= 0.0f) {
        g_heat_integral = 0.0f;
        pwm = 0.0f;
    } else if (error > 10.0f) {
        pwm = 12.0f * error;
        g_heat_integral = 0.0f;
    } else {
        g_heat_integral += error * 0.15f;
        if (g_heat_integral > 100.0f) g_heat_integral = 100.0f;
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

static bool board_home_begin(void)
{
    tmc_enable(true);
    if (!tmc_configure() || tmc_write(0x27U, 0x00010000U) != HAL_OK ||
        tmc_write(0x20U, 1U) != HAL_OK) {
        tmc_enable(false);
        return false;
    }
    g_home_started_ms = HAL_GetTick();
    g_home_last_poll_ms = 0U;
    g_home_active = true;
    return true;
}

static AppAsyncResult board_home_poll(void)
{
    uint32_t ramp_status;
    uint32_t now = HAL_GetTick();

    if (!g_home_active) {
        return APP_ASYNC_FAILED;
    }
    if (now - g_home_started_ms >= APP_HOME_TIMEOUT_MS) {
        (void)tmc_set_speed(0);
        tmc_enable(false);
        g_home_active = false;
        return APP_ASYNC_FAILED;
    }
    if (now - g_home_last_poll_ms < 100U) {
        return APP_ASYNC_BUSY;
    }
    g_home_last_poll_ms = now;
    if (tmc_read(0x35U, &ramp_status) != HAL_OK) {
        return APP_ASYNC_BUSY;
    }
    if ((ramp_status & 0x02U) == 0U) {
        return APP_ASYNC_BUSY;
    }
    (void)tmc_write(0x21U, 0U);
    (void)tmc_write(0x2DU, 0U);
    (void)tmc_set_speed(0);
    tmc_enable(false);
    g_home_active = false;
    return APP_ASYNC_OK;
}

static const PressureProfile *pressure_profile(float target)
{
    if (target <= 200.0f) return &g_profiles[0];
    if (target <= 300.0f) return &g_profiles[1];
    if (target <= 400.0f) return &g_profiles[2];
    if (target <= 500.0f) return &g_profiles[3];
    return &g_profiles[4];
}

static bool board_pressure_start(float target)
{
    (void)target;
    if (!g_ads_zero_valid) {
        return false;
    }
    tmc_enable(true);
    if (!tmc_configure()) {
        tmc_enable(false);
        return false;
    }
    g_pressure_stage = PRESS_STAGE_FAST;
    g_pressure_stage_ms = HAL_GetTick();
    g_pressure_integral = 0.0f;
    g_pressure_active = true;
    return true;
}

static bool board_pressure_step(float target, float *measured)
{
    const PressureProfile *profile = pressure_profile(target);
    int32_t raw;
    uint32_t now = HAL_GetTick();
    float error;
    float command;

    if (!g_pressure_active || measured == NULL || !ads_read_raw(&raw)) {
        (void)tmc_set_speed(0);
        return false;
    }
    *measured = ads_pressure_from_raw(raw, target);
    switch (g_pressure_stage) {
    case PRESS_STAGE_FAST:
        if (!tmc_set_speed((int32_t)profile->fast_speed)) return false;
        if (*measured >= profile->approach_threshold) {
            g_pressure_stage = PRESS_STAGE_APPROACH;
            g_pressure_stage_ms = now;
        }
        break;
    case PRESS_STAGE_APPROACH:
        if (!tmc_set_speed((int32_t)profile->approach_speed)) return false;
        if (*measured >= profile->hold_threshold) {
            g_pressure_stage = PRESS_STAGE_HOLD;
            g_pressure_stage_ms = now;
            g_pressure_integral = 0.0f;
        }
        break;
    case PRESS_STAGE_HOLD:
        error = target - *measured;
        g_pressure_integral += error * 0.02f;
        if (g_pressure_integral > 5000.0f) g_pressure_integral = 5000.0f;
        if (g_pressure_integral < -5000.0f) g_pressure_integral = -5000.0f;
        command = profile->kp * error + profile->ki * g_pressure_integral;
        if (command > 50000.0f) command = 50000.0f;
        if (command < -50000.0f) command = -50000.0f;
        if (!tmc_set_speed((int32_t)command)) return false;
        if (now - g_pressure_stage_ms >= profile->hold_ms) {
            g_pressure_stage = PRESS_STAGE_RETRACT;
            g_pressure_stage_ms = now;
        }
        break;
    case PRESS_STAGE_RETRACT:
        if (!tmc_set_speed(-(int32_t)profile->retract_speed)) return false;
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

static void board_pressure_stop(void)
{
    g_pressure_active = false;
    (void)tmc_set_speed(0);
    tmc_enable(false);
}

static bool board_power_read(bool *charging, bool *full, uint16_t *soc, uint16_t *millivolts)
{
    uint8_t charger = 0U;
    uint8_t bytes[2];
    uint8_t charge_state;

    if (charging == NULL || full == NULL || soc == NULL || millivolts == NULL ||
        HAL_I2C_Mem_Read(&hi2c1, BQ25895_ADDRESS, 0x0BU, I2C_MEMADD_SIZE_8BIT,
                         &charger, 1U, 50U) != HAL_OK) {
        return false;
    }
    charge_state = (uint8_t)((charger >> 3U) & 0x03U);
    /* Any valid external input, including charge-done, blocks treatment. */
    *charging = ((charger >> 2U) & 0x01U) != 0U;
    *full = charge_state == 3U;
    if (HAL_I2C_Mem_Read(&hi2c1, BQ27441_ADDRESS, BQ27441_REG_SOC,
                         I2C_MEMADD_SIZE_8BIT, bytes, 2U, 50U) != HAL_OK) {
        return false;
    }
    *soc = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
    if (HAL_I2C_Mem_Read(&hi2c1, BQ27441_ADDRESS, BQ27441_REG_VOLTAGE,
                         I2C_MEMADD_SIZE_8BIT, bytes, 2U, 50U) != HAL_OK) {
        return false;
    }
    *millivolts = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
    return true;
}

static void board_power_latch_off(void)
{
    uint8_t value = 0x64U;
    board_safe_outputs_off();
    (void)HAL_I2C_Mem_Write(&hi2c1, BQ25895_ADDRESS, 0x09U, I2C_MEMADD_SIZE_8BIT,
                            &value, 1U, 50U);
}

static void board_led_set(AppLedState state, bool blink)
{
    g_led_requested = state;
    g_led_blink = blink;
}

static void ws_send_color(uint32_t grb)
{
    if (!g_ws_ready) return;
    for (uint16_t led = 0U; led < WS_LED_COUNT; ++led) {
        for (uint8_t bit = 0U; bit < WS_BITS_PER_LED; ++bit) {
            g_ws_buffer[led * WS_BITS_PER_LED + bit] =
                ((grb << bit) & 0x800000U) != 0U ? 52U : 16U;
        }
    }
    memset(&g_ws_buffer[WS_LED_COUNT * WS_BITS_PER_LED], 0,
           WS_RESET_SLOTS * sizeof(g_ws_buffer[0]));
    g_ws_ready = false;
    if (HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)g_ws_buffer,
                              WS_BUFFER_SIZE) != HAL_OK) {
        g_ws_ready = true;
    }
}

static void board_counter_increment(AppMode mode)
{
    if (mode >= APP_MODE_HEAT && mode <= APP_MODE_AUTO) {
        uint8_t index = (uint8_t)mode - 1U;
        uint8_t address = (uint8_t)(index * 2U);
        g_counters[index]++;
        (void)ee_write_u16(address, g_counters[index]);
    }
}

static void board_screen_boot_sync(void)
{
    uint16_t selected = ee_read_u16(0xFCU, 1U);
    uint8_t base;
    uint16_t temperature;
    uint16_t pressure;
    uint16_t runtime;

    extern void ScreenProtocol_SendU16(uint16_t command, uint16_t value);
    if (selected < 1U || selected > 3U) selected = 1U;
    base = selected == 1U ? 0x08U : selected == 2U ? 0x10U : 0x18U;
    temperature = ee_read_u16(base, 42U);
    pressure = ee_read_u16((uint8_t)(base + 2U),
                           selected == 1U ? 250U : selected == 2U ? 350U : 450U);
    runtime = ee_read_u16((uint8_t)(base + 4U), selected == 1U ? 2U : selected == 2U ? 3U : 4U);

    ScreenProtocol_SendU16(0x00A0U, g_counters[0]);
    ScreenProtocol_SendU16(0x00A1U, g_counters[1]);
    ScreenProtocol_SendU16(0x00A2U, g_counters[2]);
    ScreenProtocol_SendU16(0x00A3U, selected);
    ScreenProtocol_SendU16(0x00ACU, 1U);
    ScreenProtocol_SendU16(0x00A4U, pressure);
    ScreenProtocol_SendU16(0x00A5U, temperature);
    ScreenProtocol_SendU16(0x00A6U, runtime);
    ScreenProtocol_SendU16(0x00A7U, ee_read_u16(0xF8U, 1U));
}

static void board_storage_erase_main(void)
{
    for (uint16_t address = 0U; address < 256U; ++address) {
        (void)ee_write_byte((uint8_t)address, 0xFFU);
    }
    memset(g_counters, 0, sizeof(g_counters));
}

static void board_storage_erase_eye(void)
{
    uint8_t value = 0xFFU;
    for (uint16_t address = 0U; address < 256U; ++address) {
        if (HAL_I2C_Mem_Write(&hi2c2, EYE_EEPROM_ADDRESS, (uint8_t)address,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U, 80U) != HAL_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(6U));
    }
}

static void board_fault_report(AppFault fault)
{
    extern void ScreenProtocol_SendU32(uint16_t command, uint32_t value);
    ScreenProtocol_SendU32(0x2070U, (uint32_t)fault);
}

static void board_screen_float(uint16_t command, float value)
{
    extern void ScreenProtocol_SendFloat(uint16_t command, float value);
    ScreenProtocol_SendFloat(command, value);
}

static void board_screen_u16(uint16_t command, uint16_t value)
{
    extern void ScreenProtocol_SendU16(uint16_t command, uint16_t value);
    ScreenProtocol_SendU16(command, value);
}

static void board_screen_u32(uint16_t command, uint32_t value)
{
    extern void ScreenProtocol_SendU32(uint16_t command, uint32_t value);
    ScreenProtocol_SendU32(command, value);
}

static const AppPort g_port = {
    .now_ms = board_now_ms,
    .safe_outputs_off = board_safe_outputs_off,
    .heater_control = board_heater_control,
    .pressure_control_start = board_pressure_start,
    .pressure_control_step = board_pressure_step,
    .pressure_control_stop = board_pressure_stop,
    .home_begin = board_home_begin,
    .home_poll = board_home_poll,
    .pressure_zero_calibrate = board_pressure_zero_calibrate,
    .eye_read_state = board_eye_read_state,
    .eye_mark_consumed = board_eye_mark_consumed,
    .power_read = board_power_read,
    .power_latch_off = board_power_latch_off,
    .led_set = board_led_set,
    .counter_increment = board_counter_increment,
    .screen_boot_sync = board_screen_boot_sync,
    .storage_read_u16 = ee_read_u16,
    .storage_write_u16 = ee_write_u16,
    .storage_erase_main = board_storage_erase_main,
    .storage_erase_eye = board_storage_erase_eye,
    .screen_float = board_screen_float,
    .screen_u16 = board_screen_u16,
    .screen_u32 = board_screen_u32,
    .fault_report = board_fault_report
};

bool Board_Init(void)
{
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    ee_sda(true);
    ee_scl(true);
    tmc_enable(false);
    board_safe_outputs_off();
    HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(300U));
    HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5U));
    if (!ads_init()) {
        return false;
    }
    g_counters[0] = ee_read_u16(0x00U, 0U);
    g_counters[1] = ee_read_u16(0x02U, 0U);
    g_counters[2] = ee_read_u16(0x04U, 0U);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart_dma, sizeof(g_uart_dma)) != HAL_OK) {
        return false;
    }
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    return true;
}

const AppPort *Board_AppPort(void)
{
    return &g_port;
}

void Board_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t color = 0x222222U;

    if (now - g_led_last_ms < 250U) return;
    g_led_last_ms = now;
    g_led_phase = !g_led_phase;
    switch (g_led_requested) {
    case APP_LED_WORKING: color = 0x222222U; break;
    case APP_LED_WARNING: color = g_led_blink && !g_led_phase ? 0U : 0x222222U; break;
    case APP_LED_CHARGING: color = g_led_phase ? 0x181818U : 0x050505U; break;
    case APP_LED_FULL: color = 0x222222U; break;
    case APP_LED_FAULT: color = 0x808000U; break;
    case APP_LED_IDLE:
    default: color = 0x222222U; break;
    }
    ws_send_color(color);
}

bool Board_ScreenWrite(const uint8_t *data, uint16_t length)
{
    return data != NULL && length != 0U &&
           HAL_UART_Transmit(&huart2, (uint8_t *)data, length, 30U) == HAL_OK;
}

size_t Board_ScreenRead(uint8_t *destination, size_t capacity)
{
    size_t count = 0U;
    taskENTER_CRITICAL();
    while (destination != NULL && count < capacity && g_uart_read != g_uart_write) {
        destination[count++] = g_uart_ring[g_uart_read];
        g_uart_read = (uint16_t)((g_uart_read + 1U) % SCREEN_RING_SIZE);
    }
    taskEXIT_CRITICAL();
    return count;
}

bool Board_TakeButtonEvent(void)
{
    bool value;
    taskENTER_CRITICAL();
    value = g_button_event;
    g_button_event = false;
    taskEXIT_CRITICAL();
    return value;
}

bool Board_TakePowerLossEvent(void)
{
    bool value;
    taskENTER_CRITICAL();
    value = g_power_loss_event;
    g_power_loss_event = false;
    taskEXIT_CRITICAL();
    return value;
}

void Board_UartRxEventFromIsr(uint16_t length)
{
    for (uint16_t i = 0U; i < length && i < SCREEN_RX_DMA_SIZE; ++i) {
        uint16_t next = (uint16_t)((g_uart_write + 1U) % SCREEN_RING_SIZE);
        if (next == g_uart_read) break;
        g_uart_ring[g_uart_write] = g_uart_dma[i];
        g_uart_write = next;
    }
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart_dma, sizeof(g_uart_dma));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void Board_UartErrorFromIsr(void)
{
    (void)HAL_UART_AbortReceive(&huart2);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart_dma, sizeof(g_uart_dma));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void Board_ButtonFromIsr(void) { g_button_event = true; }
void Board_PowerLossFromIsr(void) { g_power_loss_event = true; }

void Board_Ws2812FinishedFromIsr(void)
{
    (void)HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
    g_ws_ready = true;
}
