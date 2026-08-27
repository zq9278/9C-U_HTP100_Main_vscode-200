#include "tuning_protocol.h"

#include <math.h>
#include <string.h>

#include "ads1220_driver.h"
#include "app_config.h"
#include "app_controller.h"
#include "app_log.h"
#include "main.h"
#include "pressure_tuning.h"
#include "treatment_hw.h"

#define TUNING_HEADER_0         0x7AU
#define TUNING_HEADER_1         0xA7U
#define TUNING_PROTOCOL_VERSION 1U
#define TUNING_TAIL_0           0x0DU
#define TUNING_TAIL_1           0x0AU
#define TUNING_FIXED_BYTES      10U
#define TUNING_MAX_PAYLOAD      64U
#define TUNING_RX_STREAM_SIZE   160U

#define CMD_HELLO               0x01U
#define CMD_GET_ALL             0x10U
#define CMD_GET_PROFILE         0x11U
#define CMD_SET_PROFILE         0x12U
#define CMD_SAVE                0x13U
#define CMD_DEFAULTS            0x14U
#define CMD_SET_FIELD           0x15U
#define CMD_SET_TARGET          0x20U
#define CMD_PREPARE             0x21U
#define CMD_START               0x22U
#define CMD_STOP                0x23U
#define CMD_TELEMETRY_CONTROL   0x24U
#define CMD_GET_STATUS          0x25U
#define CMD_DEBUG_CONTROL       0x30U

#define RSP_INFO                0x81U
#define RSP_PROFILE             0x90U
#define RSP_ACK                 0x91U
#define RSP_TELEMETRY           0xA0U
#define RSP_HOME_EVENT          0xA1U

#define STATUS_OK               0U
#define STATUS_BAD_PAYLOAD      1U
#define STATUS_OUT_OF_RANGE     2U
#define STATUS_REJECTED         3U
#define STATUS_SAVE_FAILED      4U
#define STATUS_NOT_IDLE         5U
#define STATUS_CHARGING         6U
#define STATUS_FAULT_ACTIVE     7U
#define STATUS_HOME_INVALID     8U
#define STATUS_EYE_INVALID      9U
#define STATUS_ZERO_INVALID     10U

#define PROFILE_FLOAT_COUNT     11U
#define PROFILE_PAYLOAD_BYTES   (1U + PROFILE_FLOAT_COUNT * 4U)
#define TELEMETRY_MIN_PERIOD_MS 100U
#define TELEMETRY_MAX_PERIOD_MS 2000U
#define DEBUG_MODE_TIMEOUT_MS   15000U

static TuningWriteFn g_write;
static uint8_t g_rx_stream[TUNING_RX_STREAM_SIZE];
static size_t g_rx_length;
static uint16_t g_telemetry_period_ms;
static uint32_t g_last_telemetry_ms;
static uint8_t g_telemetry_sequence;
static bool g_state_seen;
static AppState g_previous_state;
static uint32_t g_debug_last_keepalive_ms;

static uint16_t crc16(const uint8_t *data, size_t length)
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

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void write_float(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

static float read_float(const uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static bool send_frame(uint8_t command, uint8_t sequence,
                       const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[TUNING_FIXED_BYTES + TUNING_MAX_PAYLOAD];
    size_t crc_offset;
    uint16_t crc;

    if (payload_length > TUNING_MAX_PAYLOAD || g_write == NULL) {
        return false;
    }
    frame[0] = TUNING_HEADER_0;
    frame[1] = TUNING_HEADER_1;
    frame[2] = TUNING_PROTOCOL_VERSION;
    frame[3] = command;
    frame[4] = sequence;
    frame[5] = payload_length;
    if (payload_length != 0U && payload != NULL) {
        memcpy(&frame[6], payload, payload_length);
    }
    crc_offset = 6U + payload_length;
    crc = crc16(frame, crc_offset);
    write_u16(&frame[crc_offset], crc);
    frame[crc_offset + 2U] = TUNING_TAIL_0;
    frame[crc_offset + 3U] = TUNING_TAIL_1;
    return g_write(frame, (uint16_t)(crc_offset + 4U));
}

static void send_ack(uint8_t sequence, uint8_t request, uint8_t status)
{
    uint8_t payload[2] = {request, status};
    (void)send_frame(RSP_ACK, sequence, payload, sizeof(payload));
}

static void profile_to_payload(uint8_t *payload, uint8_t index,
                               const PressureTuningProfile *profile)
{
    payload[0] = index;
    write_float(&payload[1], profile->sensitivity_mv_v);
    write_float(&payload[5], profile->offset_mmhg);
    write_float(&payload[9], profile->fast_speed);
    write_float(&payload[13], profile->approach_speed);
    write_float(&payload[17], profile->retract_speed);
    write_float(&payload[21], profile->approach_threshold);
    write_float(&payload[25], profile->hold_threshold);
    write_float(&payload[29], (float)profile->hold_ms);
    write_float(&payload[33], (float)profile->retract_ms);
    write_float(&payload[37], profile->kp);
    write_float(&payload[41], profile->ki);
}

static bool payload_to_profile(PressureTuningProfile *profile, const uint8_t *payload)
{
    float hold_ms = read_float(&payload[29]);
    float retract_ms = read_float(&payload[33]);

    if (!isfinite(hold_ms) || !isfinite(retract_ms) ||
        hold_ms < 0.0f || hold_ms > 10000.0f ||
        retract_ms < 0.0f || retract_ms > 10000.0f) {
        return false;
    }
    profile->sensitivity_mv_v = read_float(&payload[1]);
    profile->offset_mmhg = read_float(&payload[5]);
    profile->fast_speed = read_float(&payload[9]);
    profile->approach_speed = read_float(&payload[13]);
    profile->retract_speed = read_float(&payload[17]);
    profile->approach_threshold = read_float(&payload[21]);
    profile->hold_threshold = read_float(&payload[25]);
    profile->hold_ms = (uint32_t)hold_ms;
    profile->retract_ms = (uint32_t)retract_ms;
    profile->kp = read_float(&payload[37]);
    profile->ki = read_float(&payload[41]);
    return true;
}

static void send_profile(uint8_t sequence, uint8_t index)
{
    PressureTuningProfile profile;
    uint8_t payload[PROFILE_PAYLOAD_BYTES];

    if (!PressureTuning_Get(index, &profile)) {
        send_ack(sequence, CMD_GET_PROFILE, STATUS_OUT_OF_RANGE);
        return;
    }
    profile_to_payload(payload, index, &profile);
    (void)send_frame(RSP_PROFILE, sequence, payload, sizeof(payload));
}

static void send_telemetry(uint8_t sequence)
{
    const AppSnapshot *status = AppController_Status();
    uint8_t payload[36];
    int32_t raw = 0;
    float pressure = 0.0f;
    uint8_t stage = 0xFFU;
    bool active = false;

    if (!TreatmentHw_PressureTelemetry(&raw, &pressure, &stage, &active) || !active) {
        if (Ads1220Driver_Ready() && Ads1220Driver_ZeroValid() &&
            Ads1220Driver_ReadRaw(&raw)) {
            pressure = Ads1220Driver_PressureFromRaw(raw,
                                                      status->settings.pressure_mmhg);
        }
    }
    write_u32(&payload[0], HAL_GetTick());
    write_float(&payload[4], status->settings.pressure_mmhg);
    write_float(&payload[8], pressure);
    write_u32(&payload[12], (uint32_t)raw);
    write_u32(&payload[16], (uint32_t)Ads1220Driver_ZeroRaw());
    payload[20] = stage;
    payload[21] = active ? 1U : 0U;
    payload[22] = (uint8_t)status->state;
    write_u16(&payload[23], (uint16_t)status->fault);
    payload[25] = status->charging ? 1U : 0U;
    payload[26] = status->pressure_zero_valid ? 1U : 0U;
    payload[27] = (uint8_t)status->eye;
    payload[28] = status->home_valid ? 1U : 0U;
    payload[29] = (uint8_t)status->stop_reason;
    payload[30] = status->charge_full ? 1U : 0U;
    payload[31] = AppController_DebugMode() ? 1U : 0U;
    write_u16(&payload[32], status->battery_soc);
    write_u16(&payload[34], status->battery_mv);
    (void)send_frame(RSP_TELEMETRY, sequence, payload, sizeof(payload));
}

static uint8_t prepare_rejection_status(void)
{
    const AppSnapshot *status = AppController_Status();
    bool quick_candidate = status->state == APP_STATE_HOMING &&
                           (status->stop_reason == APP_STOP_USER ||
                            status->stop_reason == APP_STOP_NATURAL);

    if (status->state != APP_STATE_IDLE && !quick_candidate) return STATUS_NOT_IDLE;
    if (status->charging && !AppController_DebugMode()) return STATUS_CHARGING;
    if (status->fault != APP_FAULT_NONE) return STATUS_FAULT_ACTIVE;
    if (!status->home_valid && !quick_candidate) return STATUS_HOME_INVALID;
    if (status->eye != APP_EYE_NEW && status->eye != APP_EYE_IN_USE &&
        status->eye != APP_EYE_SERVICE) return STATUS_EYE_INVALID;
    if (!status->pressure_zero_valid && !quick_candidate) return STATUS_ZERO_INVALID;
    return STATUS_OK;
}

static void send_home_event(bool success)
{
    const AppSnapshot *status = AppController_Status();
    uint8_t payload[9];

    write_u32(&payload[0], HAL_GetTick());
    payload[4] = success ? 1U : 0U;
    payload[5] = (uint8_t)status->stop_reason;
    write_u16(&payload[6], (uint16_t)status->fault);
    payload[8] = status->pressure_zero_valid ? 1U : 0U;
    (void)send_frame(RSP_HOME_EVENT, g_telemetry_sequence++, payload, sizeof(payload));
}

static bool state_allows_storage(void)
{
    /* A full byte-wise EEPROM write takes about 0.7 s. Keep it out of all
     * control states so heater, pressure and homing loops cannot be delayed. */
    return AppController_Status()->state == APP_STATE_IDLE;
}

static bool set_profile_field(uint8_t profile_index, uint8_t field_index,
                              float value)
{
    PressureTuningProfile profile;

    if (!isfinite(value) || !PressureTuning_Get(profile_index, &profile)) {
        return false;
    }
    switch (field_index) {
    case 0U: profile.sensitivity_mv_v = value; break;
    case 1U: profile.offset_mmhg = value; break;
    case 2U: profile.fast_speed = value; break;
    case 3U: profile.approach_speed = value; break;
    case 4U: profile.retract_speed = value; break;
    case 5U: profile.approach_threshold = value; break;
    case 6U: profile.hold_threshold = value; break;
    case 7U:
        if (value < 0.0f || value > 10000.0f) return false;
        profile.hold_ms = (uint32_t)value;
        break;
    case 8U:
        if (value < 0.0f || value > 10000.0f) return false;
        profile.retract_ms = (uint32_t)value;
        break;
    case 9U: profile.kp = value; break;
    case 10U: profile.ki = value; break;
    default: return false;
    }
    return PressureTuning_Set(profile_index, &profile);
}

static void handle_frame(uint8_t command, uint8_t sequence,
                         const uint8_t *payload, uint8_t payload_length)
{
    if (AppController_DebugMode()) {
        g_debug_last_keepalive_ms = HAL_GetTick();
    }
    switch (command) {
    case CMD_HELLO: {
        uint8_t info[6];
        write_u32(&info[0], APP_SOFTWARE_VERSION);
        info[4] = TUNING_PROTOCOL_VERSION;
        info[5] = PRESSURE_TUNING_PROFILE_COUNT;
        (void)send_frame(RSP_INFO, sequence, info, sizeof(info));
        break;
    }
    case CMD_GET_ALL:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
            break;
        }
        for (uint8_t index = 0U; index < PRESSURE_TUNING_PROFILE_COUNT; ++index) {
            send_profile(sequence, index);
        }
        break;
    case CMD_GET_PROFILE:
        if (payload_length != 1U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            send_profile(sequence, payload[0]);
        }
        break;
    case CMD_SET_PROFILE: {
        PressureTuningProfile profile;
        if (payload_length != PROFILE_PAYLOAD_BYTES) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
            break;
        }
        if (!payload_to_profile(&profile, payload)) {
            send_ack(sequence, command, STATUS_OUT_OF_RANGE);
            break;
        }
        if (!PressureTuning_Set(payload[0], &profile)) {
            send_ack(sequence, command, STATUS_OUT_OF_RANGE);
            break;
        }
        LOGI("[Tuning] RAM profile updated index=%u", (unsigned)payload[0]);
        send_ack(sequence, command, STATUS_OK);
        break;
    }
    case CMD_SET_FIELD:
        if (payload_length != 6U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else if (!set_profile_field(payload[0], payload[1],
                                      read_float(&payload[2]))) {
            send_ack(sequence, command, STATUS_OUT_OF_RANGE);
        } else {
            LOGI("[Tuning] RAM field updated profile=%u field=%u",
                 (unsigned)payload[0], (unsigned)payload[1]);
            send_ack(sequence, command, STATUS_OK);
        }
        break;
    case CMD_SAVE:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else if (!state_allows_storage()) {
            send_ack(sequence, command, STATUS_REJECTED);
        } else {
            send_ack(sequence, command,
                     PressureTuning_Save() ? STATUS_OK : STATUS_SAVE_FAILED);
        }
        break;
    case CMD_DEFAULTS:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else if (!state_allows_storage()) {
            send_ack(sequence, command, STATUS_REJECTED);
        } else {
            PressureTuning_ResetDefaults();
            send_ack(sequence, command, STATUS_OK);
        }
        break;
    case CMD_SET_TARGET:
        if (payload_length != 4U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            float target = read_float(payload);
            if (!isfinite(target) || target < 1.0f ||
                target > APP_MAX_SAFE_PRESSURE_MMHG) {
                send_ack(sequence, command, STATUS_OUT_OF_RANGE);
            } else {
                AppController_SetPressure(target);
                send_ack(sequence, command, STATUS_OK);
            }
        }
        break;
    case CMD_PREPARE:
        if (payload_length != 4U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            float target = read_float(payload);
            if (!isfinite(target) || target < 1.0f ||
                target > APP_MAX_SAFE_PRESSURE_MMHG) {
                send_ack(sequence, command, STATUS_OUT_OF_RANGE);
            } else {
                uint8_t status = prepare_rejection_status();
                if (status == STATUS_OK &&
                    !AppController_Prepare(APP_MODE_PRESSURE, target)) {
                    status = STATUS_REJECTED;
                }
                send_ack(sequence, command, status);
            }
        }
        break;
    case CMD_START:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            send_ack(sequence, command,
                     AppController_Start() ? STATUS_OK : STATUS_REJECTED);
        }
        break;
    case CMD_STOP:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            AppController_Stop(APP_STOP_USER);
            send_ack(sequence, command, STATUS_OK);
        }
        break;
    case CMD_TELEMETRY_CONTROL:
        if (payload_length != 2U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            uint16_t period = read_u16(payload);
            if (period != 0U &&
                (period < TELEMETRY_MIN_PERIOD_MS || period > TELEMETRY_MAX_PERIOD_MS)) {
                send_ack(sequence, command, STATUS_OUT_OF_RANGE);
            } else {
                g_telemetry_period_ms = period;
                g_last_telemetry_ms = HAL_GetTick();
                send_ack(sequence, command, STATUS_OK);
            }
        }
        break;
    case CMD_GET_STATUS:
        if (payload_length != 0U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            send_telemetry(sequence);
        }
        break;
    case CMD_DEBUG_CONTROL:
        if (payload_length != 1U || payload[0] > 1U) {
            send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        } else {
            AppController_SetDebugMode(payload[0] != 0U);
            g_debug_last_keepalive_ms = HAL_GetTick();
            send_ack(sequence, command, STATUS_OK);
        }
        break;
    default:
        send_ack(sequence, command, STATUS_BAD_PAYLOAD);
        break;
    }
}

void TuningProtocol_Init(TuningWriteFn write_fn)
{
    g_write = write_fn;
    g_rx_length = 0U;
    g_telemetry_period_ms = 0U;
    g_last_telemetry_ms = 0U;
    g_telemetry_sequence = 0U;
    g_state_seen = false;
    g_previous_state = APP_STATE_BOOT;
    g_debug_last_keepalive_ms = 0U;
}

void TuningProtocol_Feed(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }
    if (length > TUNING_RX_STREAM_SIZE - g_rx_length) {
        g_rx_length = 0U;
        if (length > TUNING_RX_STREAM_SIZE) {
            data += length - TUNING_RX_STREAM_SIZE;
            length = TUNING_RX_STREAM_SIZE;
        }
    }
    memcpy(&g_rx_stream[g_rx_length], data, length);
    g_rx_length += length;

    while (g_rx_length >= TUNING_FIXED_BYTES) {
        size_t start = 0U;
        size_t frame_length;
        size_t crc_offset;
        uint16_t received_crc;

        while (start + 1U < g_rx_length &&
               !(g_rx_stream[start] == TUNING_HEADER_0 &&
                 g_rx_stream[start + 1U] == TUNING_HEADER_1)) {
            ++start;
        }
        if (start > 0U) {
            memmove(g_rx_stream, &g_rx_stream[start], g_rx_length - start);
            g_rx_length -= start;
        }
        if (g_rx_length < TUNING_FIXED_BYTES) {
            return;
        }
        if (g_rx_stream[2] != TUNING_PROTOCOL_VERSION ||
            g_rx_stream[5] > TUNING_MAX_PAYLOAD) {
            memmove(g_rx_stream, &g_rx_stream[1], --g_rx_length);
            continue;
        }
        frame_length = TUNING_FIXED_BYTES + g_rx_stream[5];
        if (g_rx_length < frame_length) {
            return;
        }
        crc_offset = 6U + g_rx_stream[5];
        received_crc = read_u16(&g_rx_stream[crc_offset]);
        if (g_rx_stream[frame_length - 2U] == TUNING_TAIL_0 &&
            g_rx_stream[frame_length - 1U] == TUNING_TAIL_1 &&
            received_crc == crc16(g_rx_stream, crc_offset)) {
            handle_frame(g_rx_stream[3], g_rx_stream[4], &g_rx_stream[6],
                         g_rx_stream[5]);
        }
        memmove(g_rx_stream, &g_rx_stream[frame_length], g_rx_length - frame_length);
        g_rx_length -= frame_length;
    }
}

void TuningProtocol_Tick(void)
{
    uint32_t now = HAL_GetTick();
    const AppSnapshot *status = AppController_Status();

    if (AppController_DebugMode() &&
        now - g_debug_last_keepalive_ms >= DEBUG_MODE_TIMEOUT_MS) {
        AppController_SetDebugMode(false);
        LOGW("[Debug] Test mode disabled by communication timeout");
    }

    if (!g_state_seen) {
        g_previous_state = status->state;
        g_state_seen = true;
    } else if (g_previous_state == APP_STATE_HOMING &&
               status->state != APP_STATE_HOMING) {
        /* PREHEAT/READY with home_valid=false is the intentional normal-stop
         * quick-resume path, not a failed home operation. */
        if (!((status->state == APP_STATE_PREHEAT || status->state == APP_STATE_READY) &&
              !status->home_valid && status->fault == APP_FAULT_NONE)) {
            send_home_event(status->home_valid && status->state != APP_STATE_FAULT);
        }
        g_previous_state = status->state;
    } else {
        g_previous_state = status->state;
    }

    if (g_telemetry_period_ms != 0U &&
        now - g_last_telemetry_ms >= g_telemetry_period_ms) {
        g_last_telemetry_ms = now;
        send_telemetry(g_telemetry_sequence++);
    }
}
