#ifndef ELRS_CRSF_H
#define ELRS_CRSF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_CRSF_CHANNEL_COUNT       16u
#define ELRS_CRSF_ONLINE_TIMEOUT_MS  500u

#define ELRS_CRSF_VALUE_MIN          172u
#define ELRS_CRSF_VALUE_MID          992u
#define ELRS_CRSF_VALUE_MAX          1811u

void ELRS_CRSF_Init(void);
void ELRS_CRSF_UART_RxCallback(uint8_t byte);

uint8_t ELRS_CRSF_IsOnline(void);
uint16_t ELRS_CRSF_GetChannel(uint8_t ch);
void ELRS_CRSF_GetChannels(uint16_t ch[ELRS_CRSF_CHANNEL_COUNT]);
uint32_t ELRS_CRSF_GetLastUpdateTime(void);
uint32_t ELRS_CRSF_GetValidFrameCount(void);
uint32_t ELRS_CRSF_GetCrcErrorCount(void);

uint16_t ELRS_CRSF_RawToUs(uint16_t raw);
float ELRS_CRSF_RawToNormalized(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif
