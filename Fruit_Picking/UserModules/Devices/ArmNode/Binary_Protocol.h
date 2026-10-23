#ifndef HOST_COMPUTER_BINARY_PROTOCOL_H
#define HOST_COMPUTER_BINARY_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧格式：AA 55 地址 命令 长度低 长度高 数据... CRC低 CRC高。 */
#define BINARY_PROTOCOL_HEADER_1 0xAAU
#define BINARY_PROTOCOL_HEADER_2 0x55U
#define BINARY_PROTOCOL_DEVICE_ADDRESS 0x01U
#define BINARY_PROTOCOL_BROADCAST_ADDRESS 0xFFU
#define BINARY_PROTOCOL_MAX_PAYLOAD 64U

/* 完整且 CRC 正确的通用协议帧。命令和数据内容由上层自行定义。 */
typedef struct
{
    uint8_t address;
    uint8_t command;
    uint16_t length;
    uint8_t payload[BINARY_PROTOCOL_MAX_PAYLOAD];
} BinaryProtocolFrame;

/* 初始化协议接收器，并启动 UART 循环 DMA 接收。 */
void BinaryProtocol_Init(UART_HandleTypeDef *huart);

/* 在主循环调用，处理 DMA 新数据并用于超时丢弃未接收完整的帧。 */
void BinaryProtocol_Task(void);

/* 由 HAL_UARTEx_RxEventCallback 和 HAL_UART_ErrorCallback 直接调用。 */
void BinaryProtocol_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void BinaryProtocol_ErrorCallback(UART_HandleTypeDef *huart);

/* 取出一帧已接收且地址匹配的完整帧；队列为空时返回 0。 */
uint8_t BinaryProtocol_ReadFrame(BinaryProtocolFrame *frame);

/* 丢弃当前未接收完整的帧，重新等待帧头。 */
void BinaryProtocol_ResetReceiver(void);

/* 向状态机送入一个字节；循环 DMA 数据在主循环中批量送入。 */
uint8_t BinaryProtocol_InputByte(uint8_t byte);

/* 计算 CRC-16/MODBUS，初值 0xFFFF，多项式 0xA001。 */
uint16_t BinaryProtocol_CRC16(const uint8_t *data, uint16_t length);

/* 发送一帧通用二进制数据。 */
HAL_StatusTypeDef BinaryProtocol_SendFrame(uint8_t address,
                                           uint8_t command,
                                           const uint8_t *payload,
                                           uint16_t payload_length);

#ifdef __cplusplus
}
#endif

#endif
