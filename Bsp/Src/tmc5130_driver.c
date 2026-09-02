#include "tmc5130_driver.h"

#include <stddef.h>

#include "app_log.h"
#include "main.h"
#include "spi.h"

#define TMC_SPI_TIMEOUT_MS 20U
#define TMC_IOIN_ADDRESS   0x04U
#define TMC_VERSION_MASK   0xFF000000UL
#define TMC_VERSION_VALUE  0x11000000UL

static uint32_t g_last_ramp_mode;
static uint32_t g_last_vmax;
static bool g_ramp_mode_valid;
static bool g_vmax_valid;

void Tmc5130Driver_Init(void)
{
    g_ramp_mode_valid = false;
    g_vmax_valid = false;
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    Tmc5130Driver_Enable(false);
}

void Tmc5130Driver_Enable(bool enable)
{
    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

HAL_StatusTypeDef Tmc5130Driver_Write(uint8_t address, uint32_t value)
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
    if (status == HAL_OK) {
        if (address == 0x20U) {
            g_last_ramp_mode = value;
            g_ramp_mode_valid = true;
        } else if (address == 0x27U) {
            g_last_vmax = value;
            g_vmax_valid = true;
        }
    }
    return status;
}

HAL_StatusTypeDef Tmc5130Driver_Read(uint8_t address, uint32_t *value)
{
    uint8_t command[5] = {(uint8_t)(address & 0x7FU), 0U, 0U, 0U, 0U};
    uint8_t response[5] = {0U};
    HAL_StatusTypeDef status;

    if (value == NULL) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, command, response, sizeof(command),
                                     TMC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return status;
    }

    /* TMC5130 read data is returned by the following SPI transaction. */
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, command, response, sizeof(command),
                                     TMC_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TMC_CSN_GPIO_Port, TMC_CSN_Pin, GPIO_PIN_SET);
    if (status == HAL_OK) {
        *value = ((uint32_t)response[1] << 24U) |
                 ((uint32_t)response[2] << 16U) |
                 ((uint32_t)response[3] << 8U) |
                 response[4];
    }
    return status;
}

bool Tmc5130Driver_Probe(void)
{
    uint32_t ioin = 0U;

    if (Tmc5130Driver_Read(TMC_IOIN_ADDRESS, &ioin) != HAL_OK) {
        LOGE("[TMC5130] IOIN read failed state=%u error=0x%08lX",
             (unsigned)hspi1.State, (unsigned long)hspi1.ErrorCode);
        return false;
    }
    /* IOIN.VERSION is fixed at 0x11 for the TMC5130. Unlike a successful
     * STM32 SPI transfer, this proves that the remote device answered. */
    if ((ioin & TMC_VERSION_MASK) != TMC_VERSION_VALUE) {
        LOGE("[TMC5130] Identity mismatch IOIN=0x%08lX expected_version=0x11",
             (unsigned long)ioin);
        return false;
    }
    return true;
}

bool Tmc5130Driver_Configure(void)
{
    static const struct {
        uint8_t address;
        uint32_t value;
    } registers[] = {
        {0x01U, 0x00000001U}, {0x6CU, 0x000300C3U}, {0x10U, 0x00000C04U},
        {0x11U, 0x0000000AU}, {0x00U, 0x00000004U}, {0x13U, 0x000001F4U},
        {0x70U, 0x000701C8U}, {0x25U, 0x00015000U}, {0x24U, 0x00000001U},
        {0x26U, 0x00001001U}, {0x27U, 0x00004000U}, {0x28U, 0x00001FFFU},
        {0x2AU, 0x00008000U}, {0x2BU, 0x0000000AU}, {0x20U, 0x00000000U}
    };

    if (!Tmc5130Driver_Probe()) {
        return false;
    }
    for (size_t index = 0U; index < sizeof(registers) / sizeof(registers[0]); ++index) {
        if (Tmc5130Driver_Write(registers[index].address, registers[index].value) != HAL_OK) {
            return false;
        }
    }
    return true;
}

bool Tmc5130Driver_SetSpeed(int32_t speed)
{
    uint32_t magnitude;
    uint32_t ramp_mode;

    if (speed == 0) {
        if (g_vmax_valid && g_last_vmax == 0U) return true;
        return Tmc5130Driver_Write(0x27U, 0U) == HAL_OK;
    }
    magnitude = speed < 0 ? (uint32_t)(-(int64_t)speed) : (uint32_t)speed;
    ramp_mode = speed < 0 ? 1U : 2U;
    if ((!g_ramp_mode_valid || g_last_ramp_mode != ramp_mode) &&
        Tmc5130Driver_Write(0x20U, ramp_mode) != HAL_OK) {
        return false;
    }
    if ((!g_vmax_valid || g_last_vmax != magnitude) &&
        Tmc5130Driver_Write(0x27U, magnitude) != HAL_OK) {
        return false;
    }
    return true;
}

void Tmc5130Driver_Stop(void)
{
    (void)Tmc5130Driver_SetSpeed(0);
    Tmc5130Driver_Enable(false);
}
