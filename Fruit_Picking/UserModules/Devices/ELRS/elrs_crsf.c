#include "elrs_crsf.h"

#include "stm32f4xx_hal.h"
#include <string.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER    0xC8u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_MAX_FRAME_SIZE                64u
#define CRSF_MIN_LENGTH                    2u
#define CRSF_MAX_LENGTH                    62u
#define CRSF_RC_PAYLOAD_SIZE               22u
#define CRSF_CRC_POLY                      0xD5u

typedef enum
{
    CRSF_STATE_WAIT_ADDRESS = 0,
    CRSF_STATE_WAIT_LENGTH,
    CRSF_STATE_READ_FRAME
} CRSF_ParseState;

static volatile uint16_t elrs_channels[ELRS_CRSF_CHANNEL_COUNT];
static volatile uint32_t elrs_last_update_ms;
static volatile uint32_t crsf_valid_frame_count;
static volatile uint32_t crsf_crc_error_count;
static uint8_t crsf_frame[CRSF_MAX_FRAME_SIZE];
static uint8_t crsf_len;
static uint8_t crsf_pos;
static CRSF_ParseState crsf_state;

static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;

    while (len-- != 0u)
    {
        crc ^= *data++;
        for (uint8_t i = 0u; i < 8u; i++)
        {
            crc = ((crc & 0x80u) != 0u) ?
                  (uint8_t)((crc << 1u) ^ CRSF_CRC_POLY) :
                  (uint8_t)(crc << 1u);
        }
    }

    return crc;
}

static uint16_t CRSF_GetBits11(const uint8_t *payload, uint8_t channel)
{
    uint16_t bit_index = (uint16_t)channel * 11u;
    uint8_t byte_index = (uint8_t)(bit_index >> 3u);
    uint8_t bit_offset = (uint8_t)(bit_index & 0x07u);
    uint32_t value = payload[byte_index];

    if ((byte_index + 1u) < CRSF_RC_PAYLOAD_SIZE)
    {
        value |= (uint32_t)payload[byte_index + 1u] << 8u;
    }
    if ((byte_index + 2u) < CRSF_RC_PAYLOAD_SIZE)
    {
        value |= (uint32_t)payload[byte_index + 2u] << 16u;
    }

    return (uint16_t)((value >> bit_offset) & 0x07FFu);
}

static void CRSF_ParseChannels(const uint8_t *payload)
{
    uint16_t parsed[ELRS_CRSF_CHANNEL_COUNT];
    uint32_t primask;

    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        parsed[i] = CRSF_GetBits11(payload, i);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        elrs_channels[i] = parsed[i];
    }
    elrs_last_update_ms = HAL_GetTick();
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void CRSF_ProcessFrame(void)
{
    uint8_t type;
    uint8_t payload_len;
    uint8_t crc;
    uint8_t calc_crc;

    if (crsf_len < CRSF_MIN_LENGTH)
    {
        return;
    }

    type = crsf_frame[2];
    payload_len = (uint8_t)(crsf_len - 2u);
    crc = crsf_frame[crsf_len + 1u];
    calc_crc = CRSF_CalcCrc(&crsf_frame[2], (uint8_t)(crsf_len - 1u));
    if (crc != calc_crc)
    {
        crsf_crc_error_count++;
        return;
    }

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
        payload_len == CRSF_RC_PAYLOAD_SIZE)
    {
        CRSF_ParseChannels(&crsf_frame[3]);
        crsf_valid_frame_count++;
    }
}

void ELRS_CRSF_Init(void)
{
    crsf_state = CRSF_STATE_WAIT_ADDRESS;
    crsf_len = 0u;
    crsf_pos = 0u;
    memset(crsf_frame, 0, sizeof(crsf_frame));

    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        elrs_channels[i] = ELRS_CRSF_VALUE_MID;
    }
    elrs_last_update_ms = 0u;
    crsf_valid_frame_count = 0u;
    crsf_crc_error_count = 0u;
}

void ELRS_CRSF_UART_RxCallback(uint8_t byte)
{
    switch (crsf_state)
    {
    case CRSF_STATE_WAIT_ADDRESS:
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            crsf_frame[0] = byte;
            crsf_state = CRSF_STATE_WAIT_LENGTH;
        }
        break;

    case CRSF_STATE_WAIT_LENGTH:
        if (byte >= CRSF_MIN_LENGTH && byte <= CRSF_MAX_LENGTH)
        {
            crsf_len = byte;
            crsf_pos = 0u;
            crsf_frame[1] = byte;
            crsf_state = CRSF_STATE_READ_FRAME;
        }
        else
        {
            crsf_state = CRSF_STATE_WAIT_ADDRESS;
        }
        break;

    case CRSF_STATE_READ_FRAME:
        crsf_frame[2u + crsf_pos] = byte;
        crsf_pos++;
        if (crsf_pos >= crsf_len)
        {
            CRSF_ProcessFrame();
            crsf_state = CRSF_STATE_WAIT_ADDRESS;
        }
        break;

    default:
        crsf_state = CRSF_STATE_WAIT_ADDRESS;
        break;
    }
}

uint8_t ELRS_CRSF_IsOnline(void)
{
    uint32_t last = elrs_last_update_ms;

    if (last == 0u)
    {
        return 0u;
    }

    return ((HAL_GetTick() - last) <= ELRS_CRSF_ONLINE_TIMEOUT_MS) ? 1u : 0u;
}

uint16_t ELRS_CRSF_GetChannel(uint8_t ch)
{
    uint16_t value;
    uint32_t primask;

    if (ch < 1u || ch > ELRS_CRSF_CHANNEL_COUNT)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    value = elrs_channels[ch - 1u];
    if (primask == 0u)
    {
        __enable_irq();
    }

    return value;
}

void ELRS_CRSF_GetChannels(uint16_t ch[ELRS_CRSF_CHANNEL_COUNT])
{
    uint32_t primask;

    if (ch == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0u; i < ELRS_CRSF_CHANNEL_COUNT; i++)
    {
        ch[i] = elrs_channels[i];
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
}

uint32_t ELRS_CRSF_GetLastUpdateTime(void)
{
    return elrs_last_update_ms;
}

uint32_t ELRS_CRSF_GetValidFrameCount(void)
{
    return crsf_valid_frame_count;
}

uint32_t ELRS_CRSF_GetCrcErrorCount(void)
{
    return crsf_crc_error_count;
}

uint16_t ELRS_CRSF_RawToUs(uint16_t raw)
{
    if (raw <= ELRS_CRSF_VALUE_MIN)
    {
        return 1000u;
    }
    if (raw >= ELRS_CRSF_VALUE_MAX)
    {
        return 2000u;
    }

    return (uint16_t)(1000u + (((uint32_t)(raw - ELRS_CRSF_VALUE_MIN) * 1000u) /
                              (ELRS_CRSF_VALUE_MAX - ELRS_CRSF_VALUE_MIN)));
}

float ELRS_CRSF_RawToNormalized(uint16_t raw)
{
    if (raw <= ELRS_CRSF_VALUE_MIN)
    {
        return -1.0f;
    }
    if (raw >= ELRS_CRSF_VALUE_MAX)
    {
        return 1.0f;
    }

    /* 以实际中点 992 为零点，分别缩放正负半轴，保证中点严格为 0。 */
    if (raw < ELRS_CRSF_VALUE_MID)
    {
        return -(float)(ELRS_CRSF_VALUE_MID - raw) /
               (float)(ELRS_CRSF_VALUE_MID - ELRS_CRSF_VALUE_MIN);
    }

    return (float)(raw - ELRS_CRSF_VALUE_MID) /
           (float)(ELRS_CRSF_VALUE_MAX - ELRS_CRSF_VALUE_MID);
}
