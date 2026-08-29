#include "pressure_tuning.h"

#include <math.h>
#include <string.h>

#include "app_log.h"
#include "storage_driver.h"

#define TUNING_STORAGE_ADDRESS       0x20U
#define TUNING_STORAGE_MAGIC_0       0x50U
#define TUNING_STORAGE_MAGIC_1       0x54U
#define TUNING_STORAGE_VERSION       8U
#define TUNING_STORAGE_HEADER_BYTES  4U
#define TUNING_STORAGE_PROFILE_BYTES 20U
#define TUNING_STORAGE_CRC_BYTES     2U
#define TUNING_STORAGE_TOTAL_BYTES   \
    (TUNING_STORAGE_HEADER_BYTES + \
     PRESSURE_TUNING_PROFILE_COUNT * TUNING_STORAGE_PROFILE_BYTES + \
     TUNING_STORAGE_CRC_BYTES)

/* Used whenever external storage is blank, corrupt or has another version. */
static const PressureTuningProfile g_defaults[PRESSURE_TUNING_PROFILE_COUNT] = {
    {0.380f, 0.0f, 20000.0f, 6000.0f, 15000.0f, 30.0f, 50.0f, 500U, 50.0f, 0.0f},
    {0.370f, 0.0f, 30000.0f, 9000.0f, 18000.0f, 90.0f, 92.0f, 600U, 150.0f, 0.0f},
    {0.370f, 0.0f, 40000.0f, 9000.0f, 20000.0f, 65.0f, 70.0f, 800U, 150.0f, 0.0f},
    {0.370f, 0.0f, 45000.0f, 9000.0f, 20000.0f, 55.0f, 60.0f, 800U, 100.0f, 0.0f},
    {0.370f, 0.0f, 45000.0f, 9000.0f, 20000.0f, 50.0f, 50.0f, 1000U, 100.0f, 0.0f}
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
           isfinite(profile->sensitivity_mv_v) &&
           isfinite(profile->offset_mmhg) &&
           isfinite(profile->fast_speed) &&
           isfinite(profile->approach_speed) &&
           isfinite(profile->retract_speed) &&
           isfinite(profile->speed_switch_percent) &&
           isfinite(profile->hold_switch_percent) &&
           isfinite(profile->kp) && isfinite(profile->ki) &&
           profile->sensitivity_mv_v >= 0.05f && profile->sensitivity_mv_v <= 5.0f &&
           profile->offset_mmhg >= -200.0f && profile->offset_mmhg <= 200.0f &&
           profile->fast_speed >= 0.0f && profile->fast_speed <= 65535.0f &&
           profile->approach_speed >= 0.0f && profile->approach_speed <= 65535.0f &&
           profile->retract_speed >= 0.0f && profile->retract_speed <= 65535.0f &&
           profile->speed_switch_percent >= 5.0f &&
           profile->speed_switch_percent <= 100.0f &&
           profile->hold_switch_percent >= profile->speed_switch_percent &&
           profile->hold_switch_percent <= 100.0f &&
           profile->retract_ms <= 10000U &&
           profile->kp >= 0.0f && profile->kp <= 1000.0f &&
           profile->ki >= 0.0f && profile->ki <= 100.0f;
}

static void encode_profile(uint8_t *data, const PressureTuningProfile *profile)
{
    write_u16_le(&data[0], (uint16_t)lroundf(profile->sensitivity_mv_v * 10000.0f));
    write_u16_le(&data[2], (uint16_t)(int16_t)lroundf(profile->offset_mmhg * 10.0f));
    write_u16_le(&data[4], (uint16_t)lroundf(profile->fast_speed));
    write_u16_le(&data[6], (uint16_t)lroundf(profile->approach_speed));
    write_u16_le(&data[8], (uint16_t)lroundf(profile->retract_speed));
    write_u16_le(&data[10], (uint16_t)lroundf(profile->speed_switch_percent * 10.0f));
    write_u16_le(&data[12], (uint16_t)lroundf(profile->hold_switch_percent * 10.0f));
    write_u16_le(&data[14], (uint16_t)profile->retract_ms);
    write_u16_le(&data[16], (uint16_t)lroundf(profile->kp * 10.0f));
    write_u16_le(&data[18], (uint16_t)lroundf(profile->ki * 10.0f));
}

static void decode_profile(PressureTuningProfile *profile, const uint8_t *data)
{
    profile->sensitivity_mv_v = (float)read_u16_le(&data[0]) / 10000.0f;
    profile->offset_mmhg = (float)(int16_t)read_u16_le(&data[2]) / 10.0f;
    profile->fast_speed = (float)read_u16_le(&data[4]);
    profile->approach_speed = (float)read_u16_le(&data[6]);
    profile->retract_speed = (float)read_u16_le(&data[8]);
    profile->speed_switch_percent =
        (float)read_u16_le(&data[10]) / 10.0f;
    profile->hold_switch_percent =
        (float)read_u16_le(&data[12]) / 10.0f;
    profile->retract_ms = read_u16_le(&data[14]);
    profile->kp = (float)read_u16_le(&data[16]) / 10.0f;
    profile->ki = (float)read_u16_le(&data[18]) / 10.0f;
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
    if (load_saved_profiles()) {
        LOGI("[Tuning] Pressure profiles loaded from external EEPROM");
    } else {
        LOGW("[Tuning] External profile invalid or blank; compiled defaults loaded");
    }
}

const PressureTuningProfile *PressureTuning_ProfileForTarget(float target_mmhg)
{
    if (target_mmhg <= 200.0f) return &g_profiles[0];
    if (target_mmhg <= 300.0f) return &g_profiles[1];
    if (target_mmhg <= 400.0f) return &g_profiles[2];
    if (target_mmhg <= 500.0f) return &g_profiles[3];
    return &g_profiles[4];
}

bool PressureTuning_Get(uint8_t index, PressureTuningProfile *profile)
{
    if (index >= PRESSURE_TUNING_PROFILE_COUNT || profile == NULL) {
        return false;
    }
    *profile = g_profiles[index];
    return true;
}

bool PressureTuning_Set(uint8_t index, const PressureTuningProfile *profile)
{
    if (index >= PRESSURE_TUNING_PROFILE_COUNT || !profile_valid(profile)) {
        return false;
    }
    g_profiles[index] = *profile;
    return true;
}

bool PressureTuning_Save(void)
{
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
