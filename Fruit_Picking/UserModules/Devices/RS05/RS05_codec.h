#ifndef RS05_CODEC_H
#define RS05_CODEC_H

#include <stdint.h>

void RS05_Codec_WriteU16BE(uint8_t *data, uint16_t value);
void RS05_Codec_WriteU16LE(uint8_t *data, uint16_t value);
void RS05_Codec_WriteU32LE(uint8_t *data, uint32_t value);
uint16_t RS05_Codec_ReadU16BE(const uint8_t *data);
uint16_t RS05_Codec_ReadU16LE(const uint8_t *data);
uint32_t RS05_Codec_ReadU32LE(const uint8_t *data);
void RS05_Codec_WriteFloatLE(uint8_t *data, float value);
float RS05_Codec_ReadFloatLE(const uint8_t *data);
uint16_t RS05_Codec_FloatToU16(float value, float minimum, float maximum);
float RS05_Codec_U16ToFloat(uint16_t value, float minimum, float maximum);

#endif /* RS05_CODEC_H */
