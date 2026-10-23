#ifndef DJI_MOTOR_MANAGER_H
#define DJI_MOTOR_MANAGER_H

#include "dji_motor.h"
#include "can_bus.h"

#define DJI_MOTOR_CONTROL_ID_1_4   0x200U
#define DJI_MOTOR_CONTROL_ID_5_8   0x1FFU
#define DJI_MOTOR_FEEDBACK_ID_FIRST 0x201U
#define DJI_MOTOR_FEEDBACK_ID_LAST  0x208U

#define DJI_MOTOR_FRAME_CONTROL     4U
static const uint16_t DJI_Motor_Frame_Control[DJI_MOTOR_FRAME_CONTROL] ={
    0x200,
    0x1FF,
    0x1FE,
    0x2FE
};

typedef struct
{
    CAN_HandleTypeDef *hcan;
    DJI_Motor_t *motors[DJI_MOTOR_COUNT];
    uint8_t registered_count;
} DJI_MotorManager_t;

HAL_StatusTypeDef DJI_MotorManager_Init(DJI_MotorManager_t *manager,
                                        CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef DJI_MotorManager_Register(DJI_MotorManager_t *manager,
                                            DJI_Motor_t *motor,
                                            uint8_t esc_id,
                                            uint16_t feedback_id,
                                            uint16_t tx_id,
                                            uint8_t tx_slot);

HAL_StatusTypeDef DJI_MotorManager_Start(DJI_MotorManager_t *manager);

HAL_StatusTypeDef DJI_MotorManager_SendAll(DJI_MotorManager_t *manager);

/*
 * 处理一帧已经从FIFO0读出来的CAN数据：只认标准数据帧、且反馈ID已注册的帧，其余直接忽略。
 * 在HAL_CAN_RxFifo0MsgPendingCallback里每次用CAN_Bus_ReadFifo0读一帧即可：只要FIFO里
 * 还有数据，中断就会一直挂着，处理完这一帧退出后会马上再进中断（但FIFO不要溢出）。
 * 同一路CAN上还有别的设备（如CAN2上的RS05）时，把读出来的同一帧也交给它们的驱动。
 */
void DJI_MotorManager_ProcessRxFrame(DJI_MotorManager_t *manager,
                                     const CAN_RxHeaderTypeDef *header,
                                     const uint8_t data[8]);

#endif /* DJI_MOTOR_MANAGER_H */
