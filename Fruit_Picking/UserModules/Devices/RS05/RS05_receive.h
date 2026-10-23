#ifndef RS05_RECEIVE_H
#define RS05_RECEIVE_H

#include "RS05_app.h"

/* 第一层按通信类型分发，第二层按参数 index 分发。 */
void RS05_ParseFeedback(RS05_MotorTypedef *motor,
                        uint32_t ext_id,
                        const uint8_t data[8]);
void RS05_ParseParameterReply(RS05_MotorTypedef *motor,
                              uint32_t exd_id,
                              const uint8_t data[8]);
void RS05_ProcessFrame(RS05_ManagerTypedef *manager,
                       uint32_t ext_id,
                       const uint8_t data[8]);
/*
 * 处理一帧已经从FIFO0读出来的CAN数据：只认扩展数据帧，其余（如CAN2上3508的标准帧）直接忽略。
 * 在HAL_CAN_RxFifo0MsgPendingCallback里用CAN_Bus_ReadFifo0读一帧，再交给这里和同一路的其他驱动。
 */
void RS05_ProcessRxFrame(RS05_ManagerTypedef *manager,
                         const CAN_RxHeaderTypeDef *header,
                         const uint8_t data[8]);

RS05_ParameterCache *RS05_FindParameterCache(RS05_MotorTypedef *motor,
                                              uint16_t index);

#endif /* RS05_RECEIVE_H */
