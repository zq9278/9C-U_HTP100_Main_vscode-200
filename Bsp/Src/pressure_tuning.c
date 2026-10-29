#include "pressure_tuning.h"

#include <math.h>
#include <string.h>

#include "app_log.h"
#include "storage_driver.h"

#define TUNING_STORAGE_ADDRESS       0x20U
/* Temporary tuning switch: 1 uses the code table directly for control and
 * queries, and rejects host profile writes/saves. Set to 0 to restore
 * EEPROM loading and normal RAM/host tuning. */
#define PRESSURE_JLINK_TUNING_ENABLE 1U
#define TUNING_USE_CODE_DEFAULTS     (PRESSURE_JLINK_TUNING_ENABLE == 0U)
const volatile uint32_t g_pressure_jlink_tuning_enabled = PRESSURE_JLINK_TUNING_ENABLE;
#define TUNING_STORAGE_MAGIC_0       0x50U
#define TUNING_STORAGE_MAGIC_1       0x54U
#define TUNING_STORAGE_VERSION       12U
#define TUNING_STORAGE_HEADER_BYTES  4U
#define TUNING_STORAGE_PROFILE_BYTES 8U
#define TUNING_STORAGE_CRC_BYTES     2U
#define TUNING_STORAGE_TOTAL_BYTES   \
    (TUNING_STORAGE_HEADER_BYTES + \
     PRESSURE_TUNING_PROFILE_COUNT * TUNING_STORAGE_PROFILE_BYTES + \
     TUNING_STORAGE_CRC_BYTES)

/* Used whenever external storage is blank, corrupt or has another version. */

// 速度为驱动器速度指令值；保压速度还受内部3000上限约束，实际上限=min(表中值,3000)。
// ??????(mV/V)?????(mmHg)????????????????????Kp????????
// ???????????0.3????????????30%??????
static const PressureTuningProfile g_defaults[PRESSURE_TUNING_PROFILE_COUNT] = {
// 当前有效字段：快速推进速度上限、快速回退速度、比例增益Kp、减速区间比例。
// 当前字段：灵敏度(mV/V)、压力偏移(mmHg)、推进速度上限、回退速度、Kp、减速区间比例。
//灵敏度、压力偏移、快速推进速度上限、保压双向补偿速度上限、快速回退速度、比例增益Kp。
    // {0.380f, 0.0f, 20000.0f,        6000.0f,            15000.0f,   50.0f},
    // {0.370f, 0.0f, 30000.0f,        9000.0f,            30000.0f,   600.0f},
    // {0.370f, 0.0f, 40000.0f,        9000.0f,            30000.0f,   150.0f},
    // {0.370f, 0.0f, 45000.0f,        9000.0f,            30000.0f,   150.0f},
    // {0.370f, 0.0f, 45000.0f,        9000.0f,            30000.0f,   100.0f}

    { 20000.0f,  40000.0f,   30.0f, 0.9f},
    { 40000.0f,  65000.0f,   1.0f,  0.5f},
    { 40000.0f,  65000.0f,   1.0f,  0.8f},
    { 45000.0f,  65000.0f,   1.0f,  0.8f},
    { 45000.0f,  65000.0f,   1.0f,  0.9f}
};

static PressureTuningProfile g_profiles[PRESSURE_TUNING_PROFILE_COUNT];

static uint16_t tuning_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                                     (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static bool profile_valid(const PressureTuningProfile *profile)
{
    return profile != NULL &&
           isfinite(profile->fast_speed) &&
           isfinite(profile->retract_speed) &&
           profile->fast_speed >= 0.0f && profile->fast_speed <= 65535.0f &&
           profile->retract_speed >= 0.0f && profile->retract_speed <= 65535.0f &&
           isfinite(profile->kp) && profile->kp >= 0.0f && profile->kp <= 1000.0f &&
           isfinite(profile->approach_ratio) &&
           profile->approach_ratio >= 0.01f && profile->approach_ratio <= 1.0f;
}

static void encode_profile(uint8_t *data, const PressureTuningProfile *profile)
{
    write_u16_le(&data[0], (uint16_t)lroundf(profile->fast_speed));
    write_u16_le(&data[2], (uint16_t)lroundf(profile->retract_speed));
    write_u16_le(&data[6], (uint16_t)lroundf(profile->approach_ratio * 10000.0f));
    write_u16_le(&data[4], (uint16_t)lroundf(profile->kp * 10.0f));
}

static void decode_profile(PressureTuningProfile *profile, const uint8_t *data)
{
    profile->fast_speed = (float)read_u16_le(&data[0]);
    profile->retract_speed = (float)read_u16_le(&data[2]);
    profile->approach_ratio = (float)read_u16_le(&data[6]) / 10000.0f;
    profile->kp = (float)read_u16_le(&data[4]) / 10.0f;
}

static bool load_saved_profiles(void)
{
    uint8_t record[TUNING_STORAGE_TOTAL_BYTES];
    PressureTuningProfile loaded[PRESSURE_TUNING_PROFILE_COUNT];
    const size_t crc_offset = TUNING_STORAGE_TOTAL_BYTES - TUNING_STORAGE_CRC_BYTES;

    if (!StorageDriver_ReadBytes(TUNING_STORAGE_ADDRESS, record, sizeof(record)) ||
        record[0] != TUNING_STORAGE_MAGIC_0 ||
        record[1] != TUNING_STORAGE_MAGIC_1 ||
        record[2] != TUNING_STORAGE_VERSION ||
        record[3] != PRESSURE_TUNING_PROFILE_COUNT ||
        read_u16_le(&record[crc_offset]) != tuning_crc16(record, crc_offset)) {
        return false;
    }
    for (uint8_t index = 0U; index < PRESSURE_TUNING_PROFILE_COUNT; ++index) {
        decode_profile(&loaded[index],
                       &record[TUNING_STORAGE_HEADER_BYTES +
                               index * TUNING_STORAGE_PROFILE_BYTES]);
        if (!profile_valid(&loaded[index])) {
            return false;
        }
    }
    memcpy(g_profiles, loaded, sizeof(g_profiles));
    return true;
}

void PressureTuning_ResetDefaults(void)
{
    memcpy(g_profiles, g_defaults, sizeof(g_profiles));
}

void PressureTuning_Init(void)
{
    PressureTuning_ResetDefaults();
    if (TUNING_USE_CODE_DEFAULTS != 0U) {
        LOGW("[Tuning] DEBUG: CODE TABLE LOCKED; EEPROM/host profile overrides disabled");
        return;
    }
    if (g_pressure_jlink_tuning_enabled != 0U) return;
    if (load_saved_profiles()) {
        LOGI("[Tuning] Pressure profiles loaded from external EEPROM");
    } else {
        LOGW("[Tuning] External profile invalid or blank; compiled defaults loaded");
    }
}

const PressureTuningProfile *PressureTuning_ProfileForTarget(float target_mmhg)
{
    const PressureTuningProfile *profiles =
        TUNING_USE_CODE_DEFAULTS != 0U ? g_defaults : g_profiles;
    if (target_mmhg <= 200.0f) return &profiles[0];
    if (target_mmhg <= 300.0f) return &profiles[1];
    if (target_mmhg <= 400.0f) return &profiles[2];
    if (target_mmhg <= 500.0f) return &profiles[3];
    return &profiles[4];
}

bool PressureTuning_Get(uint8_t index, PressureTuningProfile *profile)
{
    if (index >= PRESSURE_TUNING_PROFILE_COUNT || profile == NULL) {
        return false;
    }
    *profile = TUNING_USE_CODE_DEFAULTS != 0U ? g_defaults[index] : g_profiles[index];
    return true;
}

bool PressureTuning_Set(uint8_t index, const PressureTuningProfile *profile)
{
    if (TUNING_USE_CODE_DEFAULTS != 0U) {
        LOGW("[Tuning] Host profile update rejected: CODE TABLE LOCKED");
        return false;
    }
    if (index >= PRESSURE_TUNING_PROFILE_COUNT || !profile_valid(profile)) {
        return false;
    }
    g_profiles[index] = *profile;
    return true;
}

bool PressureTuning_Save(void)
{
    if (PRESSURE_JLINK_TUNING_ENABLE != 0U) return false;
    if (TUNING_USE_CODE_DEFAULTS != 0U) {
        LOGW("[Tuning] Profile save rejected: CODE TABLE LOCKED");
        return false;
    }
    uint8_t record[TUNING_STORAGE_TOTAL_BYTES] = {0U};
    uint8_t verify[TUNING_STORAGE_TOTAL_BYTES];
    const uint8_t invalid_magic[2] = {0U, 0U};
    const size_t crc_offset = TUNING_STORAGE_TOTAL_BYTES - TUNING_STORAGE_CRC_BYTES;

    record[0] = TUNING_STORAGE_MAGIC_0;
    record[1] = TUNING_STORAGE_MAGIC_1;
    record[2] = TUNING_STORAGE_VERSION;
    record[3] = PRESSURE_TUNING_PROFILE_COUNT;
    for (uint8_t index = 0U; index < PRESSURE_TUNING_PROFILE_COUNT; ++index) {
        if (!profile_valid(&g_profiles[index])) {
            return false;
        }
        encode_profile(&record[TUNING_STORAGE_HEADER_BYTES +
                               index * TUNING_STORAGE_PROFILE_BYTES],
                       &g_profiles[index]);
    }
    write_u16_le(&record[crc_offset], tuning_crc16(record, crc_offset));

    /* Invalidate first and publish the magic last. A reset during the slow
     * byte-write sequence can therefore only produce an invalid record. */
    if (!StorageDriver_WriteBytes(TUNING_STORAGE_ADDRESS, invalid_magic,
                                  sizeof(invalid_magic)) ||
        !StorageDriver_WriteBytes((uint8_t)(TUNING_STORAGE_ADDRESS + 2U),
                                  &record[2], sizeof(record) - 2U) ||
        !StorageDriver_WriteBytes(TUNING_STORAGE_ADDRESS, record, 2U) ||
        !StorageDriver_ReadBytes(TUNING_STORAGE_ADDRESS, verify, sizeof(verify)) ||
        memcmp(record, verify, sizeof(record)) != 0) {
        LOGE("[Tuning] External EEPROM profile save or readback failed");
        return false;
    }
    LOGI("[Tuning] Pressure profiles saved to external EEPROM");
    return true;
}
