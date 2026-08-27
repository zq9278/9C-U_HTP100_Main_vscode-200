#include "storage_driver.h"

#include <string.h>

#include "FreeRTOS.h"
#include "main.h"
#include "screen_protocol.h"
#include "task.h"
#include "tim.h"

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
    HAL_GPIO_WritePin(EE_SDA_GPIO_Port, EE_SDA_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ee_scl(bool high)
{
    HAL_GPIO_WritePin(EE_SCL_GPIO_Port, EE_SCL_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    bool acknowledged;

    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        ee_sda((value & 0x80U) != 0U);
        value <<= 1U;
        ee_delay_us(2U); ee_scl(true); ee_delay_us(2U); ee_scl(false);
    }
    ee_sda(true); ee_delay_us(1U); ee_scl(true); ee_delay_us(2U);
    acknowledged = HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) == GPIO_PIN_RESET;
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
        if (HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) == GPIO_PIN_SET) {
            value |= 1U;
        }
        ee_scl(false); ee_delay_us(2U);
    }
    ee_sda(!acknowledge); ee_scl(true); ee_delay_us(2U); ee_scl(false); ee_sda(true);
    return value;
}

static bool ee_read_byte(uint8_t address, uint8_t *value)
{
    if (value == NULL) {
        return false;
    }
    ee_start();
    if (!ee_write_bus_byte(0xA0U) || !ee_write_bus_byte(address)) {
        ee_stop();
        return false;
    }
    ee_start();
    if (!ee_write_bus_byte(0xA1U)) {
        ee_stop();
        return false;
    }
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

void StorageDriver_PreparePins(void)
{
    ee_sda(true);
    ee_scl(true);
}

void StorageDriver_Init(void)
{
    StorageDriver_PreparePins();
    g_counters[0] = StorageDriver_ReadU16(0x00U, 0U);
    g_counters[1] = StorageDriver_ReadU16(0x02U, 0U);
    g_counters[2] = StorageDriver_ReadU16(0x04U, 0U);
}

uint16_t StorageDriver_ReadU16(uint8_t address, uint16_t default_value)
{
    uint8_t high;
    uint8_t low;

    if (!ee_read_byte(address, &high) ||
        !ee_read_byte((uint8_t)(address + 1U), &low) ||
        (high == 0xFFU && low == 0xFFU)) {
        return default_value;
    }
    return (uint16_t)((uint16_t)high << 8U) | low;
}

bool StorageDriver_WriteU16(uint8_t address, uint16_t value)
{
    return ee_write_byte(address, (uint8_t)(value >> 8U)) &&
           ee_write_byte((uint8_t)(address + 1U), (uint8_t)value);
}

bool StorageDriver_ReadBytes(uint8_t address, uint8_t *data, size_t length)
{
    if (data == NULL || length > (size_t)(256U - address)) {
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (!ee_read_byte((uint8_t)(address + index), &data[index])) {
            return false;
        }
    }
    return true;
}

bool StorageDriver_WriteBytes(uint8_t address, const uint8_t *data, size_t length)
{
    if (data == NULL || length > (size_t)(256U - address)) {
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (!ee_write_byte((uint8_t)(address + index), data[index])) {
            return false;
        }
    }
    return true;
}

void StorageDriver_EraseMain(void)
{
    for (uint16_t address = 0U; address < 256U; ++address) {
        (void)ee_write_byte((uint8_t)address, 0xFFU);
    }
    memset(g_counters, 0, sizeof(g_counters));
}

void StorageDriver_IncrementCounter(AppMode mode)
{
    if (mode >= APP_MODE_HEAT && mode <= APP_MODE_AUTO) {
        uint8_t index = (uint8_t)mode - 1U;
        uint8_t address = (uint8_t)(index * 2U);

        g_counters[index]++;
        (void)StorageDriver_WriteU16(address, g_counters[index]);
    }
}

void StorageDriver_IncrementEyeReplacement(void)
{
    uint16_t count = StorageDriver_ReadU16(0xF2U, 0U);
    (void)StorageDriver_WriteU16(0xF2U, (uint16_t)(count + 1U));
}

void StorageDriver_ScreenBootSync(void)
{
    uint16_t selected = StorageDriver_ReadU16(0xFCU, 1U);
    uint8_t base;
    uint16_t temperature;
    uint16_t pressure;
    uint16_t runtime;

    if (selected < 1U || selected > 3U) {
        selected = 1U;
    }
    base = selected == 1U ? 0x08U : selected == 2U ? 0x10U : 0x18U;
    temperature = StorageDriver_ReadU16(base, 42U);
    pressure = StorageDriver_ReadU16((uint8_t)(base + 2U),
                                     selected == 1U ? 250U :
                                     selected == 2U ? 350U : 450U);
    runtime = StorageDriver_ReadU16((uint8_t)(base + 4U),
                                    selected == 1U ? 2U :
                                    selected == 2U ? 3U : 4U);

    ScreenProtocol_SendU16(0x00A0U, g_counters[0]);
    ScreenProtocol_SendU16(0x00A1U, g_counters[1]);
    ScreenProtocol_SendU16(0x00A2U, g_counters[2]);
    ScreenProtocol_SendU16(0x00A3U, selected);
    ScreenProtocol_SendU16(0x00ACU, 1U);
    ScreenProtocol_SendU16(0x00A4U, pressure);
    ScreenProtocol_SendU16(0x00A5U, temperature);
    ScreenProtocol_SendU16(0x00A6U, runtime);
    ScreenProtocol_SendU16(0x00A7U, StorageDriver_ReadU16(0xF8U, 1U));
}
