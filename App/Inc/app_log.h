#ifndef APP_LOG_H
#define APP_LOG_H

#include <stddef.h>

void AppLog_Init(void);
void AppLog_Write(const char *level, const char *format, ...);
void AppLog_WriteRaw(const char *data, size_t length);

#define LOGI(...) AppLog_Write("INFO", __VA_ARGS__)
#define LOGW(...) AppLog_Write("WARN", __VA_ARGS__)
#define LOGE(...) AppLog_Write("ERROR", __VA_ARGS__)

/* Formatted UART output is deliberately disabled in interrupt context. */
#define LOG_ISR(...) ((void)0)

#endif
