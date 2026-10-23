#include "Binary_Protocol.h"

#include <string.h>

enum
{
    BINARY_FRAME_QUEUE_SIZE = 4U,
    BINARY_INTER_BYTE_TIMEOUT_MS = 100U,
    BINARY_DMA_RX_BUFFER_SIZE = 256U,
    BINARY_DMA_EVENT_QUEUE_SIZE = 4U
};

typedef enum
{
    RX_WAIT_HEADER_1 = 0,
    RX_WAIT_HEADER_2,
    RX_ADDRESS,
    RX_COMMAND,
    RX_LENGTH_LOW,
    RX_LENGTH_HIGH,
    RX_PAYLOAD,
    RX_CRC_LOW,
    RX_CRC_HIGH
} BinaryRxState;

static UART_HandleTypeDef *binary_uart = NULL;
static uint8_t rx_dma_buffer[BINARY_DMA_RX_BUFFER_SIZE];
static volatile BinaryRxState rx_state = RX_WAIT_HEADER_1;
static BinaryProtocolFrame rx_working_frame;
static BinaryProtocolFrame rx_frame_queue[BINARY_FRAME_QUEUE_SIZE];
static volatile uint16_t rx_payload_index = 0U;
/* C 板这份和 SC09ServoTest 只差这一处：原来初值是 0xFFFF。BinaryProtocol_Init → ResetParser 会设成 0xFFFF，
   收到地址字节时也从 0xFFFF 重新算，所以初值用不到；改成 0 是为了让它进 .bss 而不是 .data，
   否则 .data 变大会把 .bss 里所有变量（README 里的 VOFA 地址）整体往后挤 4 字节。 */
static volatile uint16_t rx_calculated_crc = 0U;
static volatile uint16_t rx_received_crc = 0U;
static volatile uint8_t rx_queue_read = 0U;
static volatile uint8_t rx_queue_write = 0U;
static volatile uint8_t rx_queue_count = 0U;
static volatile uint32_t rx_last_byte_tick = 0U;
static uint16_t rx_dma_last_position = 0U;
static uint8_t rx_dma_boundary_processed = 0U;
static uint16_t rx_dma_event_position[BINARY_DMA_EVENT_QUEUE_SIZE];
static uint8_t rx_dma_event_is_full[BINARY_DMA_EVENT_QUEUE_SIZE];
static volatile uint8_t rx_dma_event_read = 0U;
static volatile uint8_t rx_dma_event_write = 0U;
static volatile uint8_t rx_dma_event_count = 0U;
static volatile uint8_t rx_dma_restart_pending = 0U;

static uint16_t BinaryProtocol_CRC16Update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= byte;
    for (bit = 0U; bit < 8U; bit++)
    {
        if ((crc & 0x0001U) != 0U)
        {
            crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
        }
        else
        {
            crc >>= 1U;
        }
    }
    return crc;
}

static void BinaryProtocol_ResetParser(void)
{
    rx_state = RX_WAIT_HEADER_1;
    rx_payload_index = 0U;
    rx_calculated_crc = 0xFFFFU;
    rx_received_crc = 0U;
}

static uint8_t BinaryProtocol_IsLocalAddress(uint8_t address)
{
    return (address == BINARY_PROTOCOL_DEVICE_ADDRESS) ||
           (address == BINARY_PROTOCOL_BROADCAST_ADDRESS);
}

static HAL_StatusTypeDef BinaryProtocol_StartReceive(void)
{
    HAL_StatusTypeDef status;

    if (binary_uart == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(binary_uart,
                                          rx_dma_buffer,
                                          BINARY_DMA_RX_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(binary_uart->hdmarx, DMA_IT_HT);
    }

    return status;
}

static void BinaryProtocol_ProcessDmaBytes(uint16_t position, uint8_t is_full)
{
    uint16_t index;

    /* TC and IDLE can report the same 256-byte boundary. Consume it once. */
    if ((is_full == 0U) && (position == BINARY_DMA_RX_BUFFER_SIZE) &&
        (rx_dma_last_position == 0U) && (rx_dma_boundary_processed != 0U))
    {
        return;
    }

    if (is_full != 0U)
    {
        for (index = rx_dma_last_position; index < BINARY_DMA_RX_BUFFER_SIZE; index++)
        {
            (void)BinaryProtocol_InputByte(rx_dma_buffer[index]);
        }
    }
    else if (position > rx_dma_last_position)
    {
        for (index = rx_dma_last_position; index < position; index++)
        {
            (void)BinaryProtocol_InputByte(rx_dma_buffer[index]);
        }
    }
    else if (position < rx_dma_last_position)
    {
        for (index = rx_dma_last_position; index < BINARY_DMA_RX_BUFFER_SIZE; index++)
        {
            (void)BinaryProtocol_InputByte(rx_dma_buffer[index]);
        }

        for (index = 0U; index < position; index++)
        {
            (void)BinaryProtocol_InputByte(rx_dma_buffer[index]);
        }
    }

    rx_dma_last_position = (is_full != 0U) ? 0U : position;
    rx_dma_boundary_processed = (is_full != 0U) || (position == BINARY_DMA_RX_BUFFER_SIZE);
}

void BinaryProtocol_Init(UART_HandleTypeDef *huart)
{
    binary_uart = huart;
    rx_queue_read = 0U;
    rx_queue_write = 0U;
    rx_queue_count = 0U;
    rx_last_byte_tick = 0U;
    rx_dma_last_position = 0U;
    rx_dma_boundary_processed = 0U;
    rx_dma_event_read = 0U;
    rx_dma_event_write = 0U;
    rx_dma_event_count = 0U;
    rx_dma_restart_pending = 0U;
    memset(&rx_working_frame, 0, sizeof(rx_working_frame));
    memset(rx_frame_queue, 0, sizeof(rx_frame_queue));
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
    memset(rx_dma_event_position, 0, sizeof(rx_dma_event_position));
    memset(rx_dma_event_is_full, 0, sizeof(rx_dma_event_is_full));
    BinaryProtocol_ResetParser();
    rx_dma_restart_pending = (BinaryProtocol_StartReceive() != HAL_OK);
}

void BinaryProtocol_Task(void)
{
    uint16_t position;
    uint8_t is_full;
    uint32_t primask;

    if (binary_uart == NULL)
    {
        return;
    }

    if (rx_dma_restart_pending != 0U)
    {
        (void)HAL_UART_AbortReceive(binary_uart);
        BinaryProtocol_ResetParser();
        rx_dma_last_position = 0U;
        rx_dma_boundary_processed = 0U;
        rx_dma_event_read = 0U;
        rx_dma_event_write = 0U;
        rx_dma_event_count = 0U;
        if (BinaryProtocol_StartReceive() == HAL_OK)
        {
            rx_dma_restart_pending = 0U;
        }
        return;
    }

    for (;;)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        if (rx_dma_event_count == 0U)
        {
            if (primask == 0U)
            {
                __enable_irq();
            }
            break;
        }

        position = rx_dma_event_position[rx_dma_event_read];
        is_full = rx_dma_event_is_full[rx_dma_event_read];
        rx_dma_event_read = (uint8_t)((rx_dma_event_read + 1U) % BINARY_DMA_EVENT_QUEUE_SIZE);
        rx_dma_event_count--;
        if (primask == 0U)
        {
            __enable_irq();
        }
        BinaryProtocol_ProcessDmaBytes(position, is_full);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((rx_state != RX_WAIT_HEADER_1) &&
        ((HAL_GetTick() - rx_last_byte_tick) > BINARY_INTER_BYTE_TIMEOUT_MS))
    {
        BinaryProtocol_ResetParser();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void BinaryProtocol_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_UART_RxEventTypeTypeDef event_type;

    if ((binary_uart == NULL) || (huart != binary_uart) ||
        (size == 0U) || (size > BINARY_DMA_RX_BUFFER_SIZE))
    {
        return;
    }

    event_type = HAL_UARTEx_GetRxEventType(huart);
    if ((event_type != HAL_UART_RXEVENT_IDLE) &&
        (event_type != HAL_UART_RXEVENT_TC))
    {
        return;
    }

    if (rx_dma_event_count >= BINARY_DMA_EVENT_QUEUE_SIZE)
    {
        rx_dma_restart_pending = 1U;
        return;
    }

    rx_dma_event_position[rx_dma_event_write] = size;
    rx_dma_event_is_full[rx_dma_event_write] = (event_type == HAL_UART_RXEVENT_TC) ? 1U : 0U;
    rx_dma_event_write = (uint8_t)((rx_dma_event_write + 1U) % BINARY_DMA_EVENT_QUEUE_SIZE);
    rx_dma_event_count++;
}

void BinaryProtocol_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((binary_uart == NULL) || (huart != binary_uart))
    {
        return;
    }

    rx_dma_restart_pending = 1U;
}

uint8_t BinaryProtocol_ReadFrame(BinaryProtocolFrame *frame)
{
    uint32_t primask;
    if (frame == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (rx_queue_count == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *frame = rx_frame_queue[rx_queue_read];
    rx_queue_read = (uint8_t)((rx_queue_read + 1U) % BINARY_FRAME_QUEUE_SIZE);
    rx_queue_count--;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void BinaryProtocol_ResetReceiver(void)
{
    BinaryProtocol_ResetParser();
}

uint8_t BinaryProtocol_InputByte(uint8_t byte)
{
    if (binary_uart == NULL)
    {
        return 0U;
    }

    if (rx_state == RX_WAIT_HEADER_1)
    {
        if (byte != BINARY_PROTOCOL_HEADER_1)
        {
            return 0U;
        }
        rx_state = RX_WAIT_HEADER_2;
        rx_last_byte_tick = HAL_GetTick();
        return 1U;
    }

    if (rx_state == RX_WAIT_HEADER_2)
    {
        rx_last_byte_tick = HAL_GetTick();
        if (byte == BINARY_PROTOCOL_HEADER_2)
        {
            rx_state = RX_ADDRESS;
            return 1U;
        }

        BinaryProtocol_ResetParser();
        if (byte == BINARY_PROTOCOL_HEADER_1)
        {
            rx_state = RX_WAIT_HEADER_2;
            return 1U;
        }
        return 0U;
    }

    rx_last_byte_tick = HAL_GetTick();

    switch (rx_state)
    {
    case RX_ADDRESS:
        rx_working_frame.address = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(0xFFFFU, byte);
        rx_state = RX_COMMAND;
        break;

    case RX_COMMAND:
        rx_working_frame.command = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        rx_state = RX_LENGTH_LOW;
        break;

    case RX_LENGTH_LOW:
        rx_working_frame.length = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        rx_state = RX_LENGTH_HIGH;
        break;

    case RX_LENGTH_HIGH:
        rx_working_frame.length |= (uint16_t)((uint16_t)byte << 8U);
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        if (rx_working_frame.length > BINARY_PROTOCOL_MAX_PAYLOAD)
        {
            BinaryProtocol_ResetParser();
        }
        else if (rx_working_frame.length == 0U)
        {
            rx_state = RX_CRC_LOW;
        }
        else
        {
            rx_payload_index = 0U;
            rx_state = RX_PAYLOAD;
        }
        break;

    case RX_PAYLOAD:
        rx_working_frame.payload[rx_payload_index++] = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        if (rx_payload_index >= rx_working_frame.length)
        {
            rx_state = RX_CRC_LOW;
        }
        break;

    case RX_CRC_LOW:
        rx_received_crc = byte;
        rx_state = RX_CRC_HIGH;
        break;

    case RX_CRC_HIGH:
        rx_received_crc |= (uint16_t)((uint16_t)byte << 8U);
        if ((rx_received_crc == rx_calculated_crc) &&
            BinaryProtocol_IsLocalAddress(rx_working_frame.address) &&
            (rx_queue_count < BINARY_FRAME_QUEUE_SIZE))
        {
            rx_frame_queue[rx_queue_write] = rx_working_frame;
            rx_queue_write = (uint8_t)((rx_queue_write + 1U) % BINARY_FRAME_QUEUE_SIZE);
            rx_queue_count++;
        }
        BinaryProtocol_ResetParser();
        break;

    default:
        BinaryProtocol_ResetParser();
        break;
    }

    return 1U;
}

uint16_t BinaryProtocol_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    if ((data == NULL) && (length > 0U))
    {
        return 0U;
    }

    for (index = 0U; index < length; index++)
    {
        crc = BinaryProtocol_CRC16Update(crc, data[index]);
    }
    return crc;
}

HAL_StatusTypeDef BinaryProtocol_SendFrame(uint8_t address,
                                           uint8_t command,
                                           const uint8_t *payload,
                                           uint16_t payload_length)
{
    uint8_t frame[2U + 1U + 1U + 2U + BINARY_PROTOCOL_MAX_PAYLOAD + 2U];
    uint16_t crc;
    uint16_t frame_length;

    if ((binary_uart == NULL) ||
        (payload_length > BINARY_PROTOCOL_MAX_PAYLOAD) ||
        ((payload == NULL) && (payload_length > 0U)))
    {
        return HAL_ERROR;
    }

    frame[0] = BINARY_PROTOCOL_HEADER_1;
    frame[1] = BINARY_PROTOCOL_HEADER_2;
    frame[2] = address;
    frame[3] = command;
    frame[4] = (uint8_t)(payload_length & 0xFFU);
    frame[5] = (uint8_t)((payload_length >> 8U) & 0xFFU);
    if (payload_length > 0U)
    {
        memcpy(&frame[6], payload, payload_length);
    }

    crc = BinaryProtocol_CRC16(&frame[2], (uint16_t)(4U + payload_length));
    frame[6U + payload_length] = (uint8_t)(crc & 0xFFU);
    frame[7U + payload_length] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame_length = (uint16_t)(8U + payload_length);

    return HAL_UART_Transmit(binary_uart, frame, frame_length, 100U);
}
