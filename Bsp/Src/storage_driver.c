#include "storage_driver.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_log.h"
#include "main.h"
#include "product_config.h"
#include "screen_protocol.h"
#include "task.h"
#include "tim.h"

#define STORAGE_SELECTED_PRESET_ADDRESS  0xFCU
#define STORAGE_INIT_FLAG_ADDRESS        0xFEU
#define STORAGE_INIT_FLAG_VALUE          0x4854U
#define STORAGE_PROGRAM_MARKER_ADDRESS   0x0801F800UL
#define STORAGE_PROGRAM_PENDING_MAGIC    UINT64_C(0x4949434652455348)
#define STORAGE_PROGRAM_HANDLED_MAGIC    UINT64_C(0x49494346444F4E45)

static uint16_t g_counters[3];

/* This value is restored whenever the ELF/HEX is downloaded. The linker keeps
 * it in the last, otherwise unused, MCU Flash page. Runtime code reads the
 * address through volatile so the compiler cannot fold the value to a constant. */
__attribute__((used, section(".program_marker"), aligned(8)))
const uint64_t g_storage_program_marker_image = STORAGE_PROGRAM_PENDING_MAGIC;

static bool storage_erase_main(void);
static bool storage_initialize_screen_presets(void);

static bool storage_program_reset_pending(void)
{
    const volatile uint64_t *marker =
        (const volatile uint64_t *)STORAGE_PROGRAM_MARKER_ADDRESS;

    return *marker != STORAGE_PROGRAM_HANDLED_MAGIC;
}

static bool storage_mark_program_reset_handled(void)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_1,
        .Page = (STORAGE_PROGRAM_MARKER_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE,
        .NbPages = 1U
    };
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;

    status = HAL_FLASH_Unlock();
    if (status == HAL_OK) {
        status = HAL_FLASHEx_Erase(&erase, &page_error);
    }
    if (status == HAL_OK) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   STORAGE_PROGRAM_MARKER_ADDRESS,
                                   STORAGE_PROGRAM_HANDLED_MAGIC);
    }
    (void)HAL_FLASH_Lock();

    return status == HAL_OK &&
           *(const volatile uint64_t *)STORAGE_PROGRAM_MARKER_ADDRESS ==
               STORAGE_PROGRAM_HANDLED_MAGIC;
}

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
    bool reset_pending = false;
    bool reset_ok = true;
    bool presets_ok;

    StorageDriver_PreparePins();
#if PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM
    reset_pending = storage_program_reset_pending();
    if (reset_pending) {
        reset_ok = storage_erase_main();
        if (reset_ok) {
            LOGW("[Storage] Firmware download detected: main I2C EEPROM reset");
        } else {
            LOGE("[Storage] Main I2C EEPROM reset failed; retrying next boot");
        }
    }
#endif
    presets_ok = storage_initialize_screen_presets();
#if PRODUCT_MAIN_EEPROM_RESET_AFTER_PROGRAM
    if (reset_pending && reset_ok && presets_ok) {
        if (storage_mark_program_reset_handled()) {
            LOGI("[Storage] Post-download I2C EEPROM reset completed");
        } else {
            LOGE("[Storage] Failed to commit download marker; reset will retry next boot");
        }
    }
#else
    (void)reset_pending;
    (void)reset_ok;
#endif
    (void)presets_ok;
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

static bool storage_write_u16_verified(uint8_t address, uint16_t value)
{
    return StorageDriver_WriteU16(address, value) &&
           StorageDriver_ReadU16(address, (uint16_t)~value) == value;
}

static bool storage_initialize_screen_presets(void)
{
    static const struct {
        uint8_t address;
        uint16_t value;
    } defaults[] = {
        {0x06U, 1U},
        {0x08U, 42U}, {0x0AU, 250U}, {0x0CU, 2U},
        {0x10U, 42U}, {0x12U, 350U}, {0x14U, 2U},
        {0x18U, 42U}, {0x1AU, 450U}, {0x1CU, 2U},
        {STORAGE_SELECTED_PRESET_ADDRESS, 1U}
    };
    uint16_t flag = StorageDriver_ReadU16(STORAGE_INIT_FLAG_ADDRESS, 0xFFFFU);
    uint16_t selected;

    if (flag == STORAGE_INIT_FLAG_VALUE) {
        return true;
    }

    selected = StorageDriver_ReadU16(STORAGE_SELECTED_PRESET_ADDRESS, 0xFFFFU);
    if (selected != 0xFFFFU) {
        /* Upgrade path: an older firmware already has screen-owned settings.
         * Preserve them and only add the new initialization marker. */
        if (storage_write_u16_verified(STORAGE_INIT_FLAG_ADDRESS,
                                       STORAGE_INIT_FLAG_VALUE)) {
            LOGI("[Storage] Existing screen presets preserved; init flag added");
            return true;
        } else {
            LOGE("[Storage] Failed to add screen preset init flag");
            return false;
        }
    }

    for (size_t index = 0U; index < sizeof(defaults) / sizeof(defaults[0]); ++index) {
        if (!storage_write_u16_verified(defaults[index].address,
                                         defaults[index].value)) {
            LOGE("[Storage] New-machine preset initialization failed at 0x%02X",
                 (unsigned)defaults[index].address);
            return false;
        }
    }
    /* Commit the marker last. A power loss before this write causes the full
     * initialization to be retried on the next boot. */
    if (storage_write_u16_verified(STORAGE_INIT_FLAG_ADDRESS,
                                   STORAGE_INIT_FLAG_VALUE)) {
        LOGI("[Storage] New machine initialized: sound=on preset=1 pressures=250/350/450 runtime=2min");
        return true;
    } else {
        LOGE("[Storage] New-machine preset init flag write failed");
        return false;
    }
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

static bool storage_erase_main(void)
{
    bool success = true;

    for (uint16_t address = 0U; address < 256U; ++address) {
        if (!ee_write_byte((uint8_t)address, 0xFFU)) {
            success = false;
        }
    }
    memset(g_counters, 0, sizeof(g_counters));
    return success;
}

void StorageDriver_EraseMain(void)
{
    (void)storage_erase_main();
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
    uint16_t selected = StorageDriver_ReadU16(STORAGE_SELECTED_PRESET_ADDRESS, 1U);
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
    runtime = StorageDriver_ReadU16((uint8_t)(base + 4U), 2U);

    ScreenProtocol_SendU16(0x00A0U, g_counters[0]);
    ScreenProtocol_SendU16(0x00A1U, g_counters[1]);
    ScreenProtocol_SendU16(0x00A2U, g_counters[2]);
    ScreenProtocol_SendU16(0x00A3U, selected);
    ScreenProtocol_SendU16(0x00ACU, 1U);
    ScreenProtocol_SendU16(0x00A4U, temperature);
    ScreenProtocol_SendU16(0x00A5U, pressure);
    ScreenProtocol_SendU16(0x00A6U, runtime);
    ScreenProtocol_SendU16(0x00A7U, StorageDriver_ReadU16(0xF8U, 1U));
}
