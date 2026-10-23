#include "RS05_codec.h"

#include <string.h>

void RS05_Codec_WriteU16BE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

void RS05_Codec_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void RS05_Codec_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

uint16_t RS05_Codec_ReadU16BE(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

uint16_t RS05_Codec_ReadU16LE(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

uint32_t RS05_Codec_ReadU32LE(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

void RS05_Codec_WriteFloatLE(uint8_t *data, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
    data[2] = (uint8_t)(raw >> 16);
    data[3] = (uint8_t)(raw >> 24);
}

float RS05_Codec_ReadFloatLE(const uint8_t *data)
{
    uint32_t raw = RS05_Codec_ReadU32LE(data);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

uint16_t RS05_Codec_FloatToU16(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        value = minimum;
    }
    else if (value > maximum)
    {
        value = maximum;
    }

    return (uint16_t)((value - minimum) * 65535.0f / (maximum - minimum));
}

float RS05_Codec_U16ToFloat(uint16_t value, float minimum, float maximum)
{
    return ((float)value * (maximum - minimum) / 65535.0f) + minimum;
}
