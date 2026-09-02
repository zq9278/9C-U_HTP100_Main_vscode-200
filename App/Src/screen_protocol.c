#include "screen_protocol.h"

#include <string.h>

#include "app_config.h"
#include "app_controller.h"
#include "app_log.h"
#include "product_config.h"
#include "storage_driver.h"

#define WORK_HEADER_0      0x5AU
#define WORK_HEADER_1      0xA5U
#define PREP_HEADER_0      0x6AU
#define PREP_HEADER_1      0xA6U
#define FRAME_TAIL         0xFFU
#define WORK_FRAME_SIZE    13U
#define PREP_FRAME_SIZE    11U
#define RX_STREAM_SIZE     256U

static ScreenWriteFn g_write;
static uint8_t g_rx_stream[RX_STREAM_SIZE];
static size_t g_rx_length;
static uint16_t g_edit_preset;

static uint8_t preset_base(uint16_t preset)
{
    return preset == 1U ? 0x08U : preset == 2U ? 0x10U : 0x18U;
}

static void send_preset(uint16_t preset, bool edit_response)
{
    uint8_t base;
    uint16_t temperature;
    uint16_t pressure;
    uint16_t runtime;

    if (preset == 0U) {
        temperature = 42U; pressure = 150U; runtime = 1U;
    } else {
        if (preset > 3U) return;
        base = preset_base(preset);
        temperature = AppController_StorageRead(base, 42U);
        pressure = AppController_StorageRead((uint8_t)(base + 2U),
                                             preset == 1U ? 250U : preset == 2U ? 350U : 450U);
        runtime = AppController_StorageRead((uint8_t)(base + 4U), 2U);
    }
    if (edit_response) {
        ScreenProtocol_SendU16(0x00A9U, pressure);
        ScreenProtocol_SendU16(0x00A8U, temperature);
    } else {
        ScreenProtocol_SendU16(0x00A4U, temperature);
        ScreenProtocol_SendU16(0x00A5U, pressure);
    }
    ScreenProtocol_SendU16(edit_response ? 0x00AAU : 0x00A6U, runtime);
}

uint16_t ScreenProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                                     (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint16_t read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8U);
}

static void write_u16_le(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
}

static uint16_t frame_command(const uint8_t *frame)
{
    return (uint16_t)((uint16_t)frame[3] << 8U) | frame[4];
}

static void handle_work_frame(const uint8_t *frame)
{
    uint16_t command = frame_command(frame);
    float value = 0.0f;

    memcpy(&value, &frame[5], sizeof(value));
    LOGI("[Screen RX] cmd=0x%04X value_x100=%ld", command,
         (long)(value * 100.0f));
    switch (command) {
    case 0x8900U:
        AppController_Stop(APP_STOP_NATURAL);
        break;
    case 0x1041U:
        (void)AppController_Prepare(APP_MODE_HEAT, 0.0f);
        break;
    case 0x1005U:
        (void)AppController_Prepare(APP_MODE_PRESSURE, value);
        break;
    case 0x1037U:
        (void)AppController_Prepare(APP_MODE_AUTO, value);
        break;
    case 0x1040U:
    case 0x1006U:
    case 0x1036U:
        (void)AppController_Start();
        break;
    case 0x1030U:
    case 0x1034U:
    case 0x1038U:
        AppController_Stop(APP_STOP_USER);
        break;
    case 0x1050U:
        /* Language is deliberately the first response. The screen caches it
         * before LVGL creates its first visible frame. */
        ScreenProtocol_SendU16(0x00ABU, AppController_StorageRead(0x06U, 1U));
        ScreenProtocol_SendU16(0x00ADU, 1U);
        AppController_ScreenBoot();
        ScreenProtocol_SendU32(0x2060U, PRODUCT_VERSION_NUMBER);
        break;
    case 0x1051U:
    case 0x1052U:
        break;
    case 0x1053U:
        AppController_SetTemperature(value);
        break;
    case 0x1054U:
        AppController_SetPressure(value);
        break;
    case 0x1056U:
        ScreenProtocol_SendU16(0x00B0U, AppController_StorageRead(0xF2U, 0U));
        break;
    default:
        break;
    }
}

static void handle_prepare_frame(const uint8_t *frame)
{
    uint16_t command = frame_command(frame);
    uint16_t value = read_u16_le(&frame[5]);

    LOGI("[Screen RX] prepare cmd=0x%04X value=%u", command, (unsigned)value);
    switch (command) {
    case 0x1042U:
        g_edit_preset = value;
        send_preset(value, true);
        break;
    case 0x1044U:
        if (value <= 3U) {
            (void)AppController_StorageWrite(0xFCU, value);
            send_preset(value, false);
        }
        break;
    case 0x1039U:
        if (g_edit_preset >= 1U && g_edit_preset <= 3U) {
            (void)AppController_StorageWrite(preset_base(g_edit_preset), value);
        }
        break;
    case 0x1040U:
        if (g_edit_preset >= 1U && g_edit_preset <= 3U) {
            (void)AppController_StorageWrite((uint8_t)(preset_base(g_edit_preset) + 2U), value);
        }
        break;
    case 0x1041U:
        AppController_SetRuntime(value);
        if (g_edit_preset >= 1U && g_edit_preset <= 3U) {
            (void)AppController_StorageWrite((uint8_t)(preset_base(g_edit_preset) + 4U), value);
        }
        break;
    case 0x1043U:
        (void)AppController_StorageWrite(0xF8U, value);
        break;
    case 0x1046U:
        AppController_StorageEraseMain();
        break;
    case 0x1047U:
        AppController_StorageEraseEye();
        break;
    case 0x1060U:
        if (value <= 1U) {
            (void)AppController_StorageWrite(0x06U, value);
        }
        ScreenProtocol_SendU16(0x00ABU, AppController_StorageRead(0x06U, 1U));
        break;
    default:
        /* Preset persistence is handled by the storage service, not the
         * treatment state machine. Unknown legacy commands remain harmless. */
        break;
    }
}

static bool frame_is_valid(const uint8_t *frame, size_t length)
{
    uint16_t received_crc;
    uint16_t expected_crc;
    size_t crc_offset;

    if (length != WORK_FRAME_SIZE && length != PREP_FRAME_SIZE) {
        return false;
    }
    if (frame[2] != length || frame[length - 2U] != FRAME_TAIL ||
        frame[length - 1U] != FRAME_TAIL) {
        return false;
    }
    crc_offset = length - 4U;
    received_crc = read_u16_le(&frame[crc_offset]);
    expected_crc = ScreenProtocol_Crc16(frame, crc_offset);
    return received_crc == expected_crc;
}

void ScreenProtocol_Init(ScreenWriteFn write_fn)
{
    g_write = write_fn;
    g_rx_length = 0U;
    g_edit_preset = 0U;
}

bool ScreenProtocol_EarlyBootReply(const uint8_t *data, size_t length)
{
    if (data == NULL || length < WORK_FRAME_SIZE) {
        return false;
    }
    for (size_t offset = 0U; offset + WORK_FRAME_SIZE <= length; ++offset) {
        const uint8_t *frame = &data[offset];

        if (frame[0] == WORK_HEADER_0 && frame[1] == WORK_HEADER_1 &&
            frame_is_valid(frame, WORK_FRAME_SIZE) &&
            frame_command(frame) == 0x1050U) {
            /* This path runs before Board_Init/AppController_Init. Only the
             * persisted language is safe and necessary at this point. Do not
             * acknowledge yet: the normal retry performs the full boot sync. */
            ScreenProtocol_SendU16(0x00ABU,
                                   StorageDriver_ReadU16(0x06U, 1U));
            return true;
        }
    }
    return false;
}

void ScreenProtocol_Feed(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }
    if (length > RX_STREAM_SIZE - g_rx_length) {
        g_rx_length = 0U;
        if (length > RX_STREAM_SIZE) {
            data += length - RX_STREAM_SIZE;
            length = RX_STREAM_SIZE;
        }
    }
    memcpy(&g_rx_stream[g_rx_length], data, length);
    g_rx_length += length;

    while (g_rx_length >= 3U) {
        size_t start = 0U;
        size_t frame_size;

        while (start + 1U < g_rx_length &&
               !((g_rx_stream[start] == WORK_HEADER_0 && g_rx_stream[start + 1U] == WORK_HEADER_1) ||
                 (g_rx_stream[start] == PREP_HEADER_0 && g_rx_stream[start + 1U] == PREP_HEADER_1))) {
            ++start;
        }
        if (start > 0U) {
            memmove(g_rx_stream, &g_rx_stream[start], g_rx_length - start);
            g_rx_length -= start;
        }
        if (g_rx_length < 3U) {
            return;
        }
        frame_size = g_rx_stream[2];
        if (frame_size != WORK_FRAME_SIZE && frame_size != PREP_FRAME_SIZE) {
            memmove(g_rx_stream, &g_rx_stream[1], --g_rx_length);
            continue;
        }
        if (g_rx_length < frame_size) {
            return;
        }
        if (frame_is_valid(g_rx_stream, frame_size)) {
            if (g_rx_stream[0] == WORK_HEADER_0) {
                handle_work_frame(g_rx_stream);
            } else {
                handle_prepare_frame(g_rx_stream);
            }
        }
        memmove(g_rx_stream, &g_rx_stream[frame_size], g_rx_length - frame_size);
        g_rx_length -= frame_size;
    }
}

void ScreenProtocol_SendFloat(uint16_t command, float value)
{
    uint8_t frame[WORK_FRAME_SIZE] = {WORK_HEADER_0, WORK_HEADER_1, WORK_FRAME_SIZE};
    uint16_t crc;

    frame[3] = (uint8_t)(command >> 8U);
    frame[4] = (uint8_t)command;
    memcpy(&frame[5], &value, sizeof(value));
    crc = ScreenProtocol_Crc16(frame, WORK_FRAME_SIZE - 4U);
    write_u16_le(&frame[9], crc);
    frame[11] = FRAME_TAIL;
    frame[12] = FRAME_TAIL;
    if (g_write != NULL) {
        (void)g_write(frame, sizeof(frame));
    }
}

void ScreenProtocol_SendU16(uint16_t command, uint16_t value)
{
    uint8_t frame[PREP_FRAME_SIZE] = {PREP_HEADER_0, PREP_HEADER_1, PREP_FRAME_SIZE};
    uint16_t crc;

    frame[3] = (uint8_t)(command >> 8U);
    frame[4] = (uint8_t)command;
    write_u16_le(&frame[5], value);
    crc = ScreenProtocol_Crc16(frame, PREP_FRAME_SIZE - 4U);
    write_u16_le(&frame[7], crc);
    frame[9] = FRAME_TAIL;
    frame[10] = FRAME_TAIL;
    if (g_write != NULL) {
        (void)g_write(frame, sizeof(frame));
    }
}

void ScreenProtocol_SendU32(uint16_t command, uint32_t value)
{
    uint8_t frame[WORK_FRAME_SIZE] = {WORK_HEADER_0, WORK_HEADER_1, WORK_FRAME_SIZE};
    uint16_t crc;

    frame[3] = (uint8_t)(command >> 8U);
    frame[4] = (uint8_t)command;
    memcpy(&frame[5], &value, sizeof(value));
    crc = ScreenProtocol_Crc16(frame, WORK_FRAME_SIZE - 4U);
    write_u16_le(&frame[9], crc);
    frame[11] = FRAME_TAIL;
    frame[12] = FRAME_TAIL;
    if (g_write != NULL) {
        (void)g_write(frame, sizeof(frame));
    }
}
