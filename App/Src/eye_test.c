#include "eye_test.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_log.h"
#include "product_config.h"

typedef enum {
    EYE_TEST_WAIT_INSERT = 0,
    EYE_TEST_HEATING,
    EYE_TEST_WAIT_REMOVE
} EyeTestState;

typedef struct {
    const AppPort *port;
    EyeTestState state;
    uint32_t started_ms;
    uint32_t last_heat_ms;
    uint32_t last_sample_ms;
    float previous_temperature_c;
    bool previous_temperature_valid;
    uint8_t consecutive_rises;
} EyeTestContext;

static EyeTestContext g_test;

static uint32_t test_now_ms(void)
{
    return g_test.port != NULL && g_test.port->now_ms != NULL ?
           g_test.port->now_ms() : 0U;
}

static void test_outputs_off(void)
{
    if (g_test.port != NULL && g_test.port->safe_outputs_off != NULL) {
        g_test.port->safe_outputs_off();
    }
}

static void test_set_led(AppLedState state)
{
    if (g_test.port != NULL && g_test.port->led_set != NULL) {
        g_test.port->led_set(state, false);
    }
}

static void test_wait_for_removal(AppLedState led)
{
    test_outputs_off();
    g_test.state = EYE_TEST_WAIT_REMOVE;
    test_set_led(led);
}

void EyeTest_Init(const AppPort *port)
{
    g_test.port = port;
    g_test.state = EYE_TEST_WAIT_INSERT;
    g_test.started_ms = 0U;
    g_test.last_heat_ms = 0U;
    g_test.last_sample_ms = 0U;
    g_test.previous_temperature_c = 0.0f;
    g_test.previous_temperature_valid = false;
    g_test.consecutive_rises = 0U;
    test_outputs_off();
    test_set_led(APP_LED_IDLE);
    LOGI("[Eye Test] Ready: insert I2C eye shield power=100%% rises=%u",
         (unsigned)PRODUCT_I2C_EYE_TEST_REQUIRED_RISES);
}

void EyeTest_SetEyeState(AppEyeState eye)
{
    uint32_t now = test_now_ms();

    if (eye == APP_EYE_ABSENT) {
        if (g_test.state != EYE_TEST_WAIT_INSERT) {
            test_outputs_off();
            g_test.state = EYE_TEST_WAIT_INSERT;
            g_test.previous_temperature_valid = false;
            g_test.consecutive_rises = 0U;
            test_set_led(APP_LED_IDLE);
            LOGI("[Eye Test] Eye removed; ready for next unit");
        }
        return;
    }
    if (g_test.state != EYE_TEST_WAIT_INSERT) {
        return;
    }

    g_test.state = EYE_TEST_HEATING;
    g_test.started_ms = now;
    g_test.last_heat_ms = now - APP_HEAT_CONTROL_PERIOD_MS;
    g_test.last_sample_ms = now;
    g_test.previous_temperature_valid = false;
    g_test.consecutive_rises = 0U;
    test_set_led(APP_LED_TEST_RUNNING);
    LOGI("[Eye Test] Eye inserted state=%u; heater started",
         (unsigned)eye);
}

void EyeTest_Tick(void)
{
    uint32_t now;
    float temperature_c;

    if (g_test.state != EYE_TEST_HEATING || g_test.port == NULL ||
        g_test.port->heater_test_max == NULL) {
        return;
    }
    now = test_now_ms();
    if (now - g_test.started_ms >= PRODUCT_I2C_EYE_TEST_TIMEOUT_MS) {
        LOGE("[Eye Test] FAIL: temperature did not rise three times in %u ms; remove eye",
             (unsigned)PRODUCT_I2C_EYE_TEST_TIMEOUT_MS);
        test_wait_for_removal(APP_LED_TEST_FAIL);
        return;
    }
    if (now - g_test.last_heat_ms < APP_HEAT_CONTROL_PERIOD_MS) {
        return;
    }
    g_test.last_heat_ms = now;
    if (!g_test.port->heater_test_max(&temperature_c)) {
        LOGE("[Eye Test] Temperature read or heater control failed; remove eye");
        test_wait_for_removal(APP_LED_TEST_FAIL);
        return;
    }
    if (temperature_c > PRODUCT_I2C_EYE_TEST_PASS_TEMP_C) {
        LOGI("[Eye Test] PASS: temperature_x100=%ld exceeded threshold_x100=%ld; LED green",
             (long)(temperature_c * 100.0f),
             (long)(PRODUCT_I2C_EYE_TEST_PASS_TEMP_C * 100.0f));
        test_wait_for_removal(APP_LED_TEST_PASS);
        return;
    }

    if (!g_test.previous_temperature_valid) {
        g_test.previous_temperature_c = temperature_c;
        g_test.previous_temperature_valid = true;
        g_test.last_sample_ms = now;
        LOGI("[Eye Test] Baseline temperature_x100=%ld",
             (long)(temperature_c * 100.0f));
        return;
    }
    if (now - g_test.last_sample_ms < PRODUCT_I2C_EYE_TEST_SAMPLE_PERIOD_MS) {
        return;
    }
    g_test.last_sample_ms = now;
    if (temperature_c > g_test.previous_temperature_c) {
        g_test.consecutive_rises++;
    } else {
        g_test.consecutive_rises = 0U;
    }
    LOGI("[Eye Test] temperature_x100=%ld previous_x100=%ld rises=%u/%u",
         (long)(temperature_c * 100.0f),
         (long)(g_test.previous_temperature_c * 100.0f),
         (unsigned)g_test.consecutive_rises,
         (unsigned)PRODUCT_I2C_EYE_TEST_REQUIRED_RISES);
    g_test.previous_temperature_c = temperature_c;

    if (g_test.consecutive_rises >= PRODUCT_I2C_EYE_TEST_REQUIRED_RISES) {
        LOGI("[Eye Test] PASS: three consecutive temperature rises; LED green");
        test_wait_for_removal(APP_LED_TEST_PASS);
    }
}

void EyeTest_Abort(void)
{
    if (g_test.state == EYE_TEST_HEATING) {
        LOGE("[Eye Test] Aborted by power-loss event; remove eye");
        test_wait_for_removal(APP_LED_TEST_FAIL);
    } else {
        test_outputs_off();
    }
}
