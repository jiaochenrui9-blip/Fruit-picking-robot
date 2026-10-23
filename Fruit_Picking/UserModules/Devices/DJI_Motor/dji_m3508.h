#ifndef DJI_M3508_H
#define DJI_M3508_H

#include "dji_motor_manager.h"

#define DJI_M3508_REDUCTION_RATIO       (3591.0f / 187.0f)
#define DJI_M3508_CURRENT_LIMIT          16384

HAL_StatusTypeDef DJI_M3508_Register(DJI_MotorManager_t *manager,
                                     DJI_Motor_t *motor,
                                     uint8_t motor_id,
                                     uint16_t tx_id,
                                     uint8_t tx_slot);

#endif /* DJI_M3508_H */
