#include "ELRS_DMA.h"

#include "elrs_crsf.h"

#define ELRS_DMA_BUFFER_LEN 256u

static UART_HandleTypeDef *elrs_uart;
static uint8_t elrs_rx_buffer[ELRS_DMA_BUFFER_LEN];
static uint16_t elrs_last_position;
static volatile uint16_t elrs_pending_position;
static volatile uint8_t elrs_data_pending;
static volatile uint8_t elrs_restart_pending;

static HAL_StatusTypeDef ELRS_DMA_StartReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(elrs_uart,
                                          elrs_rx_buffer,
                                          ELRS_DMA_BUFFER_LEN);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(elrs_uart->hdmarx, DMA_IT_HT);
    }

    return status;
}

static void ELRS_DMA_ProcessNewBytes(uint16_t position)
{
    if (position > elrs_last_position)
    {
        for (uint16_t i = elrs_last_position; i < position; i++)
        {
            ELRS_CRSF_UART_RxCallback(elrs_rx_buffer[i]);
        }
    }
    else if (position < elrs_last_position)
    {
        for (uint16_t i = elrs_last_position; i < ELRS_DMA_BUFFER_LEN; i++)
        {
            ELRS_CRSF_UART_RxCallback(elrs_rx_buffer[i]);
        }

        for (uint16_t i = 0u; i < position; i++)
        {
            ELRS_CRSF_UART_RxCallback(elrs_rx_buffer[i]);
        }
    }

    elrs_last_position = (position == ELRS_DMA_BUFFER_LEN) ? 0u : position;
}

HAL_StatusTypeDef ELRS_DMA_Start(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    elrs_uart = huart;
    elrs_last_position = 0u;
    elrs_pending_position = 0u;
    elrs_data_pending = 0u;
    elrs_restart_pending = 0u;

    return ELRS_DMA_StartReceive();
}

void ELRS_DMA_UpdateSize(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_UART_RxEventTypeTypeDef event_type;

    if (huart != elrs_uart || size == 0u || size > ELRS_DMA_BUFFER_LEN)
    {
        return;
    }

    event_type = HAL_UARTEx_GetRxEventType(huart);
    if (event_type != HAL_UART_RXEVENT_IDLE && event_type != HAL_UART_RXEVENT_TC)
    {
        return;
    }

    elrs_pending_position = size;
    elrs_data_pending = 1u;
}

void ELRS_DMA_Process(void)
{
    uint16_t position;
    uint32_t primask;

    if (elrs_uart == NULL)
    {
        return;
    }

    if (elrs_restart_pending != 0u)
    {
        (void)HAL_UART_AbortReceive(elrs_uart);
        ELRS_CRSF_Init();
        elrs_last_position = 0u;
        elrs_pending_position = 0u;
        elrs_data_pending = 0u;
        if (ELRS_DMA_StartReceive() == HAL_OK)
        {
            elrs_restart_pending = 0u;
        }
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (elrs_data_pending == 0u)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }
        return;
    }
    position = elrs_pending_position;
    elrs_data_pending = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }

    ELRS_DMA_ProcessNewBytes(position);
}

void ELRS_DMA_HandleError(UART_HandleTypeDef *huart)
{
    if (huart == elrs_uart)
    {
        elrs_restart_pending = 1u;
    }
}
