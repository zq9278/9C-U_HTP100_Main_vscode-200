#include "FreeRTOS.h"
#include "task.h"

volatile const char *g_assert_file;
volatile int g_assert_line;
volatile TaskHandle_t g_stack_overflow_task;
volatile const char *g_stack_overflow_name;
volatile uint32_t g_hardfault_pc;
volatile uint32_t g_hardfault_lr;

void vAssertCalled(const char *file, int line)
{
    g_assert_file = file;
    g_assert_line = line;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    g_stack_overflow_task = task;
    g_stack_overflow_name = task_name;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
