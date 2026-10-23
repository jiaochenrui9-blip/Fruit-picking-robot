#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "dji_motor.h"

HAL_StatusTypeDef MotorControl_Init(DJI_Motor_t *motor, float current_limit,
                                    int8_t motor_direction,
                                    float speed_kp, float speed_ki, float speed_kd,
                                    float pos_kp,float pos_ki,float pos_kd);

void MotorControl_SetTargetDegree(DJI_Motor_t *motor, float target_degree);

void MotorControl_SetTargetSpeed(DJI_Motor_t *motor, int16_t target_speed);

void MotorControl_Update(DJI_Motor_t *motor, uint32_t now_tick);
void MotorControl_SetMode(DJI_Motor_t *motor, MotorControlMode_e mode);

#endif /* MOTOR_CONTROL_H */
