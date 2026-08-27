#include "screen_uart.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "usart.h"

#define SCREEN_RX_DMA_SIZE 128U
#define SCREEN_RING_SIZE   512U

static uint8_t g_dma[SCREEN_RX_DMA_SIZE];
static uint8_t g_ring[SCREEN_RING_SIZE];
static volatile uint16_t g_write;
static volatile uint16_t g_read;

static void screen_uart_start_receive(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma, sizeof(g_dma));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

bool ScreenUart_Init(void)
{
    g_write = 0U;
    g_read = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma, sizeof(g_dma)) != HAL_OK) {
        return false;
    }
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    return true;
}

void ScreenUart_SetPower(bool enabled)
{
    /* LCD_PW is an open-drain, active-low screen power enable. */
    HAL_GPIO_WritePin(LCD_PW_GPIO_Port, LCD_PW_Pin,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool ScreenUart_Write(const uint8_t *data, uint16_t length)
{
    return data != NULL && length != 0U &&
           HAL_UART_Transmit(&huart2, (uint8_t *)data, length, 30U) == HAL_OK;
}

size_t ScreenUart_Read(uint8_t *destination, size_t capacity)
{
    size_t count = 0U;

    taskENTER_CRITICAL();
    while (destination != NULL && count < capacity && g_read != g_write) {
        destination[count++] = g_ring[g_read];
        g_read = (uint16_t)((g_read + 1U) % SCREEN_RING_SIZE);
    }
    taskEXIT_CRITICAL();
    return count;
}

void ScreenUart_RxEventFromIsr(uint16_t length)
{
    for (uint16_t index = 0U; index < length && index < SCREEN_RX_DMA_SIZE; ++index) {
        uint16_t next = (uint16_t)((g_write + 1U) % SCREEN_RING_SIZE);
        if (next == g_read) {
            break;
        }
        g_ring[g_write] = g_dma[index];
        g_write = next;
    }
    screen_uart_start_receive();
}

void ScreenUart_ErrorFromIsr(void)
{
    (void)HAL_UART_AbortReceive(&huart2);
    screen_uart_start_receive();
}
