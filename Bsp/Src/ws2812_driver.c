#include "ws2812_driver.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_log.h"
#include "main.h"
#include "task.h"
#include "tim.h"

#define WS_LED_COUNT        15U
#define WS_BITS_PER_LED     24U
#define WS_RESET_SLOTS      150U
#define WS_BUFFER_SIZE      (WS_LED_COUNT * WS_BITS_PER_LED + WS_RESET_SLOTS)
#define WS_BREATH_PERIOD_MS 6000U
#define WS_BREATH_UPDATE_MS 40U
#define WS_BREATH_MIN_LEVEL 4U
#define WS_BREATH_MAX_LEVEL 50U

static volatile bool g_ready = true;
static uint16_t g_buffer[WS_BUFFER_SIZE];
static AppLedState g_requested = APP_LED_IDLE;
static bool g_blink;
static uint32_t g_last_ms;
static uint32_t g_breath_started_ms;
static bool g_phase;
static bool g_update_pending = true;
static bool g_fault_flash_active;
static uint8_t g_fault_flash_half_cycles;

static bool ws_send_color(uint32_t grb)
{
    if (!g_ready) {
        return false;
    }
    for (uint16_t led = 0U; led < WS_LED_COUNT; ++led) {
        for (uint8_t bit = 0U; bit < WS_BITS_PER_LED; ++bit) {
            g_buffer[led * WS_BITS_PER_LED + bit] =
                ((grb << bit) & 0x800000U) != 0U ? 52U : 16U;
        }
    }
    memset(&g_buffer[WS_LED_COUNT * WS_BITS_PER_LED], 0,
           WS_RESET_SLOTS * sizeof(g_buffer[0]));
    g_ready = false;
    if (HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)g_buffer,
                              WS_BUFFER_SIZE) != HAL_OK) {
        g_ready = true;
        return false;
    }
    return true;
}

void Ws2812Driver_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0U);
    HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(300U));
    HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5U));
    (void)ws_send_color(0x000000U);
}

void Ws2812Driver_Set(AppLedState state, bool blink)
{
    AppLedState previous = g_requested;
    bool previous_blink = g_blink;

    if (state == APP_LED_CHARGING || state == APP_LED_FULL) {
        g_fault_flash_active = false;
    } else if (state == APP_LED_FAULT && g_requested != APP_LED_FAULT &&
               !g_fault_flash_active) {
        g_fault_flash_active = true;
        g_fault_flash_half_cycles = 0U;
        g_phase = true;
        g_last_ms = HAL_GetTick() - 500U;
    }
    g_requested = state;
    g_blink = blink;
    if (state != previous || blink != previous_blink) {
        LOGI("[LED] state=%u blink=%u", (unsigned)state, blink ? 1U : 0U);
        g_update_pending = true;
        g_last_ms = HAL_GetTick() - WS_BREATH_UPDATE_MS;
        if (state == APP_LED_CHARGING) {
            g_breath_started_ms = HAL_GetTick();
        }
    }
}

void Ws2812Driver_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t color = 0x222222U;

    if (g_fault_flash_active) {
        if (now - g_last_ms < 500U) {
            return;
        }
        g_last_ms = now;
        color = g_phase ? 0x808000U : 0U;
        g_fault_flash_half_cycles++;
        if (g_fault_flash_half_cycles >= 6U) {
            g_fault_flash_active = false;
            color = 0x222222U;
        } else {
            g_phase = !g_phase;
        }
        (void)ws_send_color(color);
        return;
    }
    if (g_requested == APP_LED_CHARGING) {
        uint32_t elapsed;
        uint32_t half_period = WS_BREATH_PERIOD_MS / 2U;
        uint32_t triangle;
        uint32_t level;

        if (now - g_last_ms < WS_BREATH_UPDATE_MS) {
            return;
        }
        g_last_ms = now;
        elapsed = (now - g_breath_started_ms) % WS_BREATH_PERIOD_MS;
        triangle = elapsed < half_period ? elapsed : WS_BREATH_PERIOD_MS - elapsed;
        level = WS_BREATH_MIN_LEVEL +
                triangle * (WS_BREATH_MAX_LEVEL - WS_BREATH_MIN_LEVEL) / half_period;
        color = level * 0x010101U;
        (void)ws_send_color(color);
        return;
    }
    if (g_requested == APP_LED_WARNING && g_blink) {
        if (now - g_last_ms < 250U) {
            return;
        }
        g_last_ms = now;
        g_phase = !g_phase;
    } else if (!g_update_pending) {
        return;
    }

    switch (g_requested) {
    case APP_LED_WORKING:  color = 0x222222U; break;
    case APP_LED_WARNING:  color = g_blink && !g_phase ? 0U : 0x808000U; break;
    case APP_LED_CHARGING: color = 0x020202U; break;
    case APP_LED_FULL:     color = 0x222222U; break;
    case APP_LED_FAULT:    color = 0x222222U; break;
    case APP_LED_IDLE:
    default:               color = 0x222222U; break;
    }
    if (ws_send_color(color)) {
        g_update_pending = false;
    }
}

void Ws2812Driver_FinishedFromIsr(void)
{
    (void)HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
    g_ready = true;
}
