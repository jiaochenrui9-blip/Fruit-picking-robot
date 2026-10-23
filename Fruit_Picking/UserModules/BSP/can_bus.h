#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "main.h"

/*
 * 过滤器组分配：F407的28个过滤器组由CAN1、CAN2共用，SlaveStartFilterBank = 14，
 * 即CAN1用0~13，CAN2用14~27。
 *   大疆电机（标准帧0x200~0x20F）：CAN1用组0，CAN2用组14（dji_motor_manager.c）
 *   RS05私有协议（扩展帧）       ：CAN1用组1，CAN2用组15（RS05_app.c）
 * 所有过滤器都进FIFO0。同一路CAN上挂了多种设备时（如CAN2上的3508和RS05），
 * 在HAL_CAN_RxFifo0MsgPendingCallback里只读一次FIFO，再把这一帧交给该路上的
 * 每个驱动，各驱动按IDE只处理自己的帧。
 */

/* 配置一个可接收4个标准帧ID的16位列表过滤器。 */
HAL_StatusTypeDef CAN_Bus_ConfigStdIdList(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          const uint16_t std_ids[4]);

/*
 * 配置32位掩码过滤器。
 * 接收条件为：(标准ID & std_id_mask) == (std_id & std_id_mask)。
 */
HAL_StatusTypeDef CAN_Bus_ConfigStdIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint16_t std_id,
                                          uint16_t std_id_mask);

/*
 * 配置32位掩码过滤器，只接收扩展数据帧。
 * 接收条件为：(扩展ID & ext_id_mask) == (ext_id & ext_id_mask)；
 * ext_id_mask为0时接收所有扩展数据帧。
 */
HAL_StatusTypeDef CAN_Bus_ConfigExtIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint32_t ext_id,
                                          uint32_t ext_id_mask);

/* 启动CAN（已经启动过则跳过），并开启FIFO0有新消息中断。 */
HAL_StatusTypeDef CAN_Bus_StartRxFifo0(CAN_HandleTypeDef *hcan);

/* 发送一帧11位标准ID的数据帧。 */
HAL_StatusTypeDef CAN_Bus_SendStdData(CAN_HandleTypeDef *hcan,
                                      uint16_t std_id,
                                      const uint8_t *data,
                                      uint8_t length);

/* 发送一帧29位扩展ID的数据帧。 */
HAL_StatusTypeDef CAN_Bus_SendExtData(CAN_HandleTypeDef *hcan,
                                      uint32_t ext_id,
                                      const uint8_t *data,
                                      uint8_t length);

/* 从接收FIFO0读取一帧CAN数据。 */
HAL_StatusTypeDef CAN_Bus_ReadFifo0(CAN_HandleTypeDef *hcan,
                                    CAN_RxHeaderTypeDef *header,
                                    uint8_t data[8]);

#endif /* CAN_BUS_H */
