//
// Created by game on 2026/7/16.
//

#ifndef RS05_RS05_APP_H
#define RS05_RS05_APP_H

#include "can_bus.h"
#include "RS05_parameter.h"
#include "RS05_Command.h"
typedef uint8_t RS05_FaultFlags;
typedef enum
{
    RS05_FAULT_NONE                  = 0U,

    /* 扩展 ID bit16，对应 fault_flags bit0 */
    RS05_FAULT_UNDERVOLTAGE          = (1U << 0),

    /* 扩展 ID bit17，对应 fault_flags bit1 */
    RS05_FAULT_PHASE_OVERCURRENT     = (1U << 1),

    /* 扩展 ID bit18，对应 fault_flags bit2 */
    RS05_FAULT_OVERTEMPERATURE       = (1U << 2),

    /* 扩展 ID bit19，对应 fault_flags bit3 */
    RS05_FAULT_MAGNETIC_ENCODER      = (1U << 3),

    /* 扩展 ID bit20，对应 fault_flags bit4 */
    RS05_FAULT_STALL_OVERLOAD        = (1U << 4),

     /*暂时保留*/
    RS05_FAULT_RESERVED_BIT5         = (1U << 5)

} RS05_FaultFlag;

typedef struct
{
    uint8_t motor_id;

    /* 以下状态由 CAN 接收中断写入，应用层可在主循环中读取。 */
     float position_rad;
     float velocity_rad_s;
     float torque_nm;
     float temperature_c;

     uint8_t state;
     RS05_FaultFlags fault_flags;

     uint32_t last_feedback_tick;

    /* Type 17 参数读取响应数组，最多保存10个参数的最新数值，协议中的浮点值采用小端字节序。 */
    RS05_ParameterCache parameters[RS05_PARAMETER_CACHE_COUNT];

} RS05_MotorTypedef;


typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t master_id;
    RS05_MotorTypedef *motors[RS05_MOTOR_MAX_COUNT];
    uint8_t num_motors;
} RS05_ManagerTypedef;

uint32_t RS05_BuildExtId(uint8_t comm_type,
                         uint16_t data2,
                         uint8_t motor_id);
HAL_StatusTypeDef RS05_SendData(CAN_HandleTypeDef *hcan,
                                uint8_t comm_type,
                                const uint8_t *data,
                                uint16_t data2,
                                uint8_t motor_id);
HAL_StatusTypeDef RS05_Enable(CAN_HandleTypeDef *hcan,
                              uint8_t master_id,
                              uint8_t motor_id);
HAL_StatusTypeDef RS05_MotionControl(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     float position_rad,
                                     float velocity_rad_s,
                                     float kp,
                                     float kd,
                                     float torque_nm);
HAL_StatusTypeDef RS05_Stop(CAN_HandleTypeDef *hcan,
                            uint8_t master_id,
                            uint8_t motor_id);
HAL_StatusTypeDef RS05_SetMechanicalZero(RS05_ManagerTypedef *manager,
                                          RS05_MotorTypedef *motor);
HAL_StatusTypeDef RS05_SetMotionMode(CAN_HandleTypeDef *hcan,
                                     uint8_t master_id,
                                     uint8_t motor_id);
HAL_StatusTypeDef RS05_Command_ParaRead(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index);
HAL_StatusTypeDef RS05_WriteParameterU8(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index,
                                        uint8_t value);
HAL_StatusTypeDef RS05_WriteParameterU16(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint16_t value);
HAL_StatusTypeDef RS05_WriteParameterU32(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint32_t value);
HAL_StatusTypeDef RS05_WriteParameterFloat(RS05_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint16_t index,
                                           float value);
HAL_StatusTypeDef RS05_SetMode(RS05_ManagerTypedef *manager,
                               RS05_MotorTypedef *motor,
                               uint8_t mode);
HAL_StatusTypeDef RS05_EnterMotionMode(RS05_ManagerTypedef *manager,
                                       RS05_MotorTypedef *motor);
HAL_StatusTypeDef RS05_SetMotionTarget(RS05_ManagerTypedef *manager,
                                       RS05_MotorTypedef *motor,
                                       float position_rad,
                                       float velocity_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_ff_nm);
HAL_StatusTypeDef RS05_EnterSpeedMode(RS05_ManagerTypedef *manager,
                                      RS05_MotorTypedef *motor,
                                      float acceleration_rad_s2,
                                      float current_limit_a);
HAL_StatusTypeDef RS05_SetSpeedTarget(RS05_ManagerTypedef *manager,
                                      RS05_MotorTypedef *motor,
                                      float target_speed_rad_s);
HAL_StatusTypeDef RS05_EnterCurrentMode(RS05_ManagerTypedef *manager,
                                        RS05_MotorTypedef *motor);
HAL_StatusTypeDef RS05_SetCurrentTarget(RS05_ManagerTypedef *manager,
                                        RS05_MotorTypedef *motor,
                                        float target_current_a);
HAL_StatusTypeDef RS05_EnterPPMode(RS05_ManagerTypedef *manager,
                                   RS05_MotorTypedef *motor,
                                   float max_velocity_rad_s,
                                   float acceleration_rad_s2,
                                   float deceleration_rad_s2,
                                   float current_limit_a);
HAL_StatusTypeDef RS05_SetPPTarget(RS05_ManagerTypedef *manager,
                                   RS05_MotorTypedef *motor,
                                   float target_position_rad);
HAL_StatusTypeDef RS05_EnterCSPMode(RS05_ManagerTypedef *manager,
                                    RS05_MotorTypedef *motor,
                                    float speed_limit_rad_s,
                                    float current_limit_a);
HAL_StatusTypeDef RS05_SetCSPTarget(RS05_ManagerTypedef *manager,
                                    RS05_MotorTypedef *motor,
                                    float target_position_rad);

void RS05_Manager_Init(RS05_ManagerTypedef *manager,
                       CAN_HandleTypeDef *hcan,
                       uint8_t master_id);
HAL_StatusTypeDef RS05_Manager_RegisterMotor(RS05_ManagerTypedef *manager,
                                              RS05_MotorTypedef *motor);
/* 配置扩展帧过滤器并启动CAN（该路CAN已被别的驱动启动过也没关系）。 */
HAL_StatusTypeDef RS05_Manager_Start(RS05_ManagerTypedef *manager);
uint8_t RS05_IsOnline(const RS05_MotorTypedef *motor);
uint8_t RS05_CheckIfFault(const RS05_MotorTypedef *motor);
HAL_StatusTypeDef RS05_SetActiveReport(RS05_ManagerTypedef *manager,
                                       uint8_t motor_id,
                                       uint8_t enable);
/* 类型 22：把之前用类型 18 写的参数存进电机 flash（类型 18 本身掉电就丢）。 */
HAL_StatusTypeDef RS05_SaveParameters(RS05_ManagerTypedef *manager,
                                      uint8_t motor_id);
#endif // RS05_RS05_APP_H
