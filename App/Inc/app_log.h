#ifndef APP_LOG_H
#define APP_LOG_H

#include <stddef.h>
#include "app_config.h"

void AppLog_Init(void);
void AppLog_Write(const char *level, const char *format, ...);
void AppLog_WriteRaw(const char *data, size_t length);

#define LOGI(...) AppLog_Write("INFO", __VA_ARGS__)
#define LOGW(...) AppLog_Write("WARN", __VA_ARGS__)
#define LOGE(...) AppLog_Write("ERROR", __VA_ARGS__)

/* Compile-time constant removes diagnostic calls and argument evaluation. */
#define LOG_PRESSURE_DIAG(...) do { \
    if (APP_ENABLE_PRESSURE_DIAGNOSTIC_LOG != 0U) { LOGI(__VA_ARGS__); } \
} while (0)

/* Formatted UART output is deliberately disabled in interrupt context. */
#define LOG_ISR(...) ((void)0)

#endif
