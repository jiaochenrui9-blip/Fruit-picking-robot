//
// Created by game on 2026/8/5.
//

#ifndef INC_3508_DJI_GM6020_H
#define INC_3508_DJI_GM6020_H

#include "dji_motor_manager.h"
#include "dji_motor.h"
#define DJI_GM6020_MAX_CURRENT_LIMIT 16384

HAL_StatusTypeDef DJI_GM6020_Register(DJI_MotorManager_t* manager,DJI_Motor_t *motor,uint8_t motor_id,uint16_t tx_id,
                                    uint8_t tx_slot);
#endif //INC_3508_DJI_GM6020_H
