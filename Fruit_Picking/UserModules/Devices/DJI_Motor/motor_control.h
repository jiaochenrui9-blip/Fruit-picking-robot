#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "dji_motor.h"

HAL_StatusTypeDef MotorControl_Init(DJI_Motor_t *motor, float current_limit,
                                    int8_t motor_direction,
                                    float speed_kp, float speed_ki, float speed_kd,
                                    float pos_kp,float pos_ki,float pos_kd);

void MotorControl_SetTargetDegree(DJI_Motor_t *motor, float target_degree);

void MotorControl_SetTargetSpeed(DJI_Motor_t *motor, int16_t target_speed);

/* 改速度环积分项对输出的贡献上限（协议数值，正负对称）。MotorControl_Init 默认是 1000，
   要更大的（比如底盘上坡）在 Init 之后调一次；SetMode / 失能只清积分，不会把上限改回去。 */
void MotorControl_SetSpeedIntegralLimit(DJI_Motor_t *motor, float limit);

void MotorControl_Update(DJI_Motor_t *motor, uint32_t now_tick);
void MotorControl_SetMode(DJI_Motor_t *motor, MotorControlMode_e mode);

#endif /* MOTOR_CONTROL_H */
