#ifndef ELRS_DMA_H
#define ELRS_DMA_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef ELRS_DMA_Start(UART_HandleTypeDef *huart);
void ELRS_DMA_UpdateSize(UART_HandleTypeDef *huart, uint16_t size);
void ELRS_DMA_Process(void);
void ELRS_DMA_HandleError(UART_HandleTypeDef *huart);

#endif
