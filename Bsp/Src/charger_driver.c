#include "charger_driver.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "app_log.h"
#include "bq27441_gauge.h"
#include "i2c.h"
#include "main.h"
#include "task.h"

#define BQ25895_ADDRESS            0xD4U
#define BQ25895_REG_INPUT          0x00U
#define BQ25895_REG_CONTROL1       0x02U
#define BQ25895_REG_CHARGE_CURRENT 0x04U
#define BQ25895_REG_PRECHG_TERM    0x05U
#define BQ25895_REG_CHARGE_VOLTAGE 0x06U
#define BQ25895_REG_TIMER          0x07U
#define BQ25895_REG_STATUS         0x0BU
#define BQ25895_REG_VBUS_ADC       0x11U
#define BQ25895_INPUT_2A           0x26U
#define BQ25895_CONTROL_5V_FIXED   0x20U
#define BQ25895_CHARGE_1984MA      0x1FU
#define BQ25895_PRE128_TERM64      0x10U
#define BQ25895_CHARGE_4208MV      0x5EU
#define BQ25895_TIMER_NO_WATCHDOG  0x8DU

typedef struct {
    uint8_t address;
    uint8_t value;
} Bq25895RegisterSetting;

static const Bq25895RegisterSetting g_5v_2a_settings[] = {
    {BQ25895_REG_CONTROL1,       BQ25895_CONTROL_5V_FIXED},
    {BQ25895_REG_CHARGE_CURRENT, BQ25895_CHARGE_1984MA},
    {BQ25895_REG_PRECHG_TERM,    BQ25895_PRE128_TERM64},
    {BQ25895_REG_CHARGE_VOLTAGE, BQ25895_CHARGE_4208MV},
    {BQ25895_REG_TIMER,          BQ25895_TIMER_NO_WATCHDOG},
    {BQ25895_REG_INPUT,          BQ25895_INPUT_2A}
};

static bool bq25895_write_and_verify(uint8_t address, uint8_t value)
{
    uint8_t readback = 0U;

    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        if (HAL_I2C_Mem_Write(&hi2c1, BQ25895_ADDRESS, address,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U) == HAL_OK &&
            HAL_I2C_Mem_Read(&hi2c1, BQ25895_ADDRESS, address,
                             I2C_MEMADD_SIZE_8BIT, &readback, 1U, 50U) == HAL_OK &&
            readback == value) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    LOGE("[BQ25895] REG%02X write verify failed: want=0x%02X read=0x%02X",
         (unsigned)address, (unsigned)value, (unsigned)readback);
    return false;
}

static bool bq25895_configure_5v_2a(void)
{
    bool configured = true;

    HAL_GPIO_WritePin(CHG_CE_GPIO_Port, CHG_CE_Pin, GPIO_PIN_SET);
    for (size_t index = 0U;
         index < sizeof(g_5v_2a_settings) / sizeof(g_5v_2a_settings[0]);
         ++index) {
        if (!bq25895_write_and_verify(g_5v_2a_settings[index].address,
                                      g_5v_2a_settings[index].value)) {
            configured = false;
        }
    }
    HAL_GPIO_WritePin(CHG_CE_GPIO_Port, CHG_CE_Pin, GPIO_PIN_RESET);

    if (configured) {
        LOGI("[BQ25895] Fixed 5V configured: input=2000mA charge=1984mA no-DPDM");
    } else {
        LOGE("[BQ25895] Fixed 5V configuration incomplete; charger using verified/default limits");
    }
    return configured;
}

bool ChargerDriver_Init(void)
{
    /* Keep the original fail-safe behavior: verified/default charger limits
     * remain active even if one configuration register could not be written. */
    (void)bq25895_configure_5v_2a();
    return Bq27441Gauge_Init();
}

bool ChargerDriver_Read(bool *charging, bool *full, uint16_t *soc,
                        uint16_t *millivolts)
{
    static uint8_t previous_status = 0xFFU;
    static uint8_t previous_vbus = 0xFFU;
    static uint16_t last_valid_soc = 100U;
    static uint16_t last_valid_millivolts;
    uint8_t charger = 0U;
    uint8_t vbus = 0U;
    uint8_t charge_state;
    uint16_t measured_soc;
    uint16_t measured_millivolts;

    if (charging == NULL || full == NULL || soc == NULL || millivolts == NULL ||
        HAL_I2C_Mem_Read(&hi2c1, BQ25895_ADDRESS, BQ25895_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &charger, 1U, 50U) != HAL_OK ||
        HAL_I2C_Mem_Read(&hi2c1, BQ25895_ADDRESS, BQ25895_REG_VBUS_ADC,
                         I2C_MEMADD_SIZE_8BIT, &vbus, 1U, 50U) != HAL_OK) {
        return false;
    }

    charge_state = (uint8_t)((charger >> 3U) & 0x03U);
    *charging = (vbus & 0x80U) != 0U || ((charger >> 2U) & 0x01U) != 0U;
    *full = *charging && charge_state == 3U;
    if (charger != previous_status || vbus != previous_vbus) {
        LOGI("[BQ25895] REG0B=0x%02X REG11=0x%02X VBUS=%u CHRG=%u PG=%u VBUS_GD=%u full=%u",
             (unsigned)charger, (unsigned)vbus,
             (unsigned)(charger >> 5U), (unsigned)charge_state,
             ((charger >> 2U) & 0x01U) != 0U ? 1U : 0U,
             (vbus & 0x80U) != 0U ? 1U : 0U, *full ? 1U : 0U);
        previous_status = charger;
        previous_vbus = vbus;
    }

    if (Bq27441Gauge_Read(&measured_soc, &measured_millivolts)) {
        last_valid_soc = measured_soc;
        last_valid_millivolts = measured_millivolts;
    }
    *soc = last_valid_soc;
    *millivolts = last_valid_millivolts;
    return true;
}

void ChargerDriver_PowerLatchOff(void)
{
    uint8_t value = 0x64U;

    (void)HAL_I2C_Mem_Write(&hi2c1, BQ25895_ADDRESS, 0x09U,
                            I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U);
}
