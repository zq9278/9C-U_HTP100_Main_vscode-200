#include "bq27441_gauge.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_log.h"
#include "i2c.h"
#include "task.h"

#define BQ27441_ADDRESS                 0xAAU
#define BQ27441_REG_CONTROL             0x00U
#define BQ27441_REG_VOLTAGE             0x04U
#define BQ27441_REG_FLAGS               0x06U
#define BQ27441_REG_SOC                 0x1CU
#define BQ27441_REG_DESIGN_CAPACITY     0x3CU
#define BQ27441_REG_DATA_CLASS          0x3EU
#define BQ27441_REG_DATA_BLOCK          0x3FU
#define BQ27441_REG_BLOCK_DATA          0x40U
#define BQ27441_REG_BLOCK_CHECKSUM      0x60U
#define BQ27441_REG_BLOCK_CONTROL       0x61U

#define BQ27441_SUBCMD_CONTROL_STATUS   0x0000U
#define BQ27441_SUBCMD_SET_CFGUPDATE    0x0013U
#define BQ27441_SUBCMD_SEAL             0x0020U
#define BQ27441_SUBCMD_SOFT_RESET       0x0042U
#define BQ27441_UNSEAL_KEY              0x8000U

#define BQ27441_FLAG_CFGUPMODE          0x0010U
#define BQ27441_FLAG_ITPOR              0x0020U
#define BQ27441_CONTROL_STATUS_INITCOMP 0x0080U
#define BQ27441_CONTROL_STATUS_SEALED   0x2000U

#define BQ27441_STATE_CLASS             0x52U
#define BQ27441_STATE_BLOCK             0x00U
#define BQ27441_STATE_CAPACITY_MSB      10U
#define BQ27441_STATE_CAPACITY_LSB      11U
#define BQ27441_STATE_ENERGY_MSB        12U
#define BQ27441_STATE_ENERGY_LSB        13U
#define BQ27441_STATE_TERMINATE_MSB     16U
#define BQ27441_STATE_TERMINATE_LSB     17U
#define BQ27441_STATE_TAPER_MSB         27U
#define BQ27441_STATE_TAPER_LSB         28U

#define BQ27441_DESIGN_CAPACITY_MAH     3300U
#define BQ27441_DESIGN_ENERGY_MWH       12210U
#define BQ27441_TERMINATE_VOLTAGE_MV    3000U
#define BQ27441_TAPER_RATE              515U

static HAL_StatusTypeDef gauge_read(uint8_t reg, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        status = HAL_I2C_Mem_Read(&hi2c1, BQ27441_ADDRESS, reg,
                                  I2C_MEMADD_SIZE_8BIT, data, length, 100U);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    return status;
}

static HAL_StatusTypeDef gauge_write(uint8_t reg, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        status = HAL_I2C_Mem_Write(&hi2c1, BQ27441_ADDRESS, reg,
                                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)data,
                                   length, 100U);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    return status;
}

static bool gauge_read_word(uint8_t reg, uint16_t *value)
{
    uint8_t bytes[2];

    if (value == NULL || gauge_read(reg, bytes, sizeof(bytes)) != HAL_OK) {
        return false;
    }
    *value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
    return true;
}

static bool gauge_write_control(uint16_t command)
{
    uint8_t bytes[2] = {
        (uint8_t)(command & 0xFFU),
        (uint8_t)(command >> 8U)
    };
    return gauge_write(BQ27441_REG_CONTROL, bytes, sizeof(bytes)) == HAL_OK;
}

static bool gauge_read_control_status(uint16_t *status)
{
    if (status == NULL ||
        !gauge_write_control(BQ27441_SUBCMD_CONTROL_STATUS)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2U));
    return gauge_read_word(BQ27441_REG_CONTROL, status);
}

static bool gauge_wait_flag(uint16_t mask, bool set, uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    uint16_t flags;

    do {
        if (gauge_read_word(BQ27441_REG_FLAGS, &flags) &&
            (((flags & mask) != 0U) == set)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    } while (HAL_GetTick() - started < timeout_ms);
    return false;
}

static bool gauge_unseal(void)
{
    uint16_t control_status;

    if (!gauge_write_control(BQ27441_UNSEAL_KEY) ||
        !gauge_write_control(BQ27441_UNSEAL_KEY)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));
    if (!gauge_write_control(BQ27441_SUBCMD_CONTROL_STATUS)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2U));
    return gauge_read_word(BQ27441_REG_CONTROL, &control_status) &&
           (control_status & BQ27441_CONTROL_STATUS_SEALED) == 0U;
}

static void gauge_seal(void)
{
    (void)gauge_write_control(BQ27441_SUBCMD_SEAL);
    vTaskDelay(pdMS_TO_TICKS(10U));
}

static bool gauge_select_state_block(void)
{
    uint8_t value = 0U;

    if (gauge_write(BQ27441_REG_BLOCK_CONTROL, &value, 1U) != HAL_OK) {
        return false;
    }
    value = BQ27441_STATE_CLASS;
    if (gauge_write(BQ27441_REG_DATA_CLASS, &value, 1U) != HAL_OK) {
        return false;
    }
    value = BQ27441_STATE_BLOCK;
    if (gauge_write(BQ27441_REG_DATA_BLOCK, &value, 1U) != HAL_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));
    return true;
}

static uint16_t gauge_block_u16(const uint8_t block[32], uint8_t msb, uint8_t lsb)
{
    return ((uint16_t)block[msb] << 8U) | block[lsb];
}

static void gauge_block_set_u16(uint8_t block[32], uint8_t msb, uint8_t lsb,
                                uint16_t value)
{
    block[msb] = (uint8_t)(value >> 8U);
    block[lsb] = (uint8_t)(value & 0xFFU);
}

static bool gauge_block_matches(const uint8_t block[32])
{
    return gauge_block_u16(block, BQ27441_STATE_CAPACITY_MSB,
                           BQ27441_STATE_CAPACITY_LSB) == BQ27441_DESIGN_CAPACITY_MAH &&
           gauge_block_u16(block, BQ27441_STATE_ENERGY_MSB,
                           BQ27441_STATE_ENERGY_LSB) == BQ27441_DESIGN_ENERGY_MWH &&
           gauge_block_u16(block, BQ27441_STATE_TERMINATE_MSB,
                           BQ27441_STATE_TERMINATE_LSB) == BQ27441_TERMINATE_VOLTAGE_MV &&
           gauge_block_u16(block, BQ27441_STATE_TAPER_MSB,
                           BQ27441_STATE_TAPER_LSB) == BQ27441_TAPER_RATE;
}

static uint8_t gauge_block_checksum(const uint8_t block[32])
{
    uint8_t sum = 0U;

    for (uint8_t index = 0U; index < 32U; ++index) {
        sum = (uint8_t)(sum + block[index]);
    }
    return (uint8_t)(0xFFU - sum);
}

static bool gauge_read_state_block(uint8_t block[32])
{
    return gauge_select_state_block() &&
           gauge_read(BQ27441_REG_BLOCK_DATA, block, 32U) == HAL_OK;
}

static bool gauge_program_state_block(uint8_t block[32])
{
    uint8_t verify[32];
    uint8_t checksum;
    uint8_t checksum_read = 0U;

    gauge_block_set_u16(block, BQ27441_STATE_CAPACITY_MSB,
                        BQ27441_STATE_CAPACITY_LSB, BQ27441_DESIGN_CAPACITY_MAH);
    gauge_block_set_u16(block, BQ27441_STATE_ENERGY_MSB,
                        BQ27441_STATE_ENERGY_LSB, BQ27441_DESIGN_ENERGY_MWH);
    gauge_block_set_u16(block, BQ27441_STATE_TERMINATE_MSB,
                        BQ27441_STATE_TERMINATE_LSB, BQ27441_TERMINATE_VOLTAGE_MV);
    gauge_block_set_u16(block, BQ27441_STATE_TAPER_MSB,
                        BQ27441_STATE_TAPER_LSB, BQ27441_TAPER_RATE);
    checksum = gauge_block_checksum(block);

    if (gauge_write(BQ27441_REG_BLOCK_DATA, block, 32U) != HAL_OK ||
        gauge_write(BQ27441_REG_BLOCK_CHECKSUM, &checksum, 1U) != HAL_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));
    if (!gauge_read_state_block(verify) ||
        gauge_read(BQ27441_REG_BLOCK_CHECKSUM, &checksum_read, 1U) != HAL_OK) {
        return false;
    }
    return checksum_read == checksum && memcmp(block, verify, sizeof(verify)) == 0 &&
           gauge_block_matches(verify);
}

bool Bq27441Gauge_Init(void)
{
    uint8_t state_block[32];
    uint16_t flags;
    uint16_t design_capacity;
    bool needs_programming;

    if (!gauge_read_word(BQ27441_REG_FLAGS, &flags) ||
        !gauge_read_word(BQ27441_REG_DESIGN_CAPACITY, &design_capacity)) {
        LOGE("[BQ27441] Not responding on I2C1");
        return false;
    }
    if (!gauge_unseal()) {
        LOGE("[BQ27441] Unseal failed flags=0x%04X capacity=%u",
             (unsigned)flags, (unsigned)design_capacity);
        return false;
    }
    if (!gauge_read_state_block(state_block)) {
        LOGE("[BQ27441] State block read failed");
        gauge_seal();
        return false;
    }

    needs_programming = (flags & BQ27441_FLAG_ITPOR) != 0U ||
                        !gauge_block_matches(state_block);
    if (!needs_programming) {
        gauge_seal();
        LOGI("[BQ27441] Configuration retained: capacity=%u mAh",
             (unsigned)design_capacity);
        return true;
    }

    LOGI("[BQ27441] Programming capacity=%u mAh energy=%u mWh term=%u mV taper=%u ITPOR=%u",
         BQ27441_DESIGN_CAPACITY_MAH, BQ27441_DESIGN_ENERGY_MWH,
         BQ27441_TERMINATE_VOLTAGE_MV, BQ27441_TAPER_RATE,
         (flags & BQ27441_FLAG_ITPOR) != 0U ? 1U : 0U);
    if (!gauge_write_control(BQ27441_SUBCMD_SET_CFGUPDATE) ||
        !gauge_wait_flag(BQ27441_FLAG_CFGUPMODE, true, 2000U) ||
        !gauge_read_state_block(state_block) ||
        !gauge_program_state_block(state_block) ||
        !gauge_write_control(BQ27441_SUBCMD_SOFT_RESET) ||
        !gauge_wait_flag(BQ27441_FLAG_CFGUPMODE, false, 2000U)) {
        LOGE("[BQ27441] Configuration update failed");
        (void)gauge_write_control(BQ27441_SUBCMD_SOFT_RESET);
        gauge_seal();
        return false;
    }
    gauge_seal();
    if (!gauge_read_word(BQ27441_REG_DESIGN_CAPACITY, &design_capacity) ||
        design_capacity != BQ27441_DESIGN_CAPACITY_MAH) {
        LOGE("[BQ27441] Capacity verify failed value=%u", (unsigned)design_capacity);
        return false;
    }
    LOGI("[BQ27441] Configuration verified: capacity=%u mAh", (unsigned)design_capacity);
    return true;
}

bool Bq27441Gauge_Read(uint16_t *soc, uint16_t *millivolts)
{
    static bool initialization_wait_logged;
    static bool initialization_ready_logged;
    uint16_t control_status;
    uint16_t raw_soc;
    uint16_t raw_voltage;

    if (soc == NULL || millivolts == NULL) {
        return false;
    }
    /* After battery insertion/soft reset, SOC can temporarily read as zero.
     * TI defines it as usable only after CONTROL_STATUS.INITCOMP is set. */
    if (!gauge_read_control_status(&control_status) ||
        (control_status & BQ27441_CONTROL_STATUS_INITCOMP) == 0U) {
        if (!initialization_wait_logged) {
            LOGW("[BQ27441] Waiting for measurement initialization");
            initialization_wait_logged = true;
        }
        return false;
    }
    if (!initialization_ready_logged) {
        LOGI("[BQ27441] Measurement initialization complete");
        initialization_ready_logged = true;
    }
    if (
        !gauge_read_word(BQ27441_REG_SOC, &raw_soc) ||
        !gauge_read_word(BQ27441_REG_VOLTAGE, &raw_voltage) ||
        raw_soc > 100U || raw_voltage < 2500U || raw_voltage > 5000U) {
        return false;
    }
    *soc = raw_soc;
    *millivolts = raw_voltage;
    return true;
}
