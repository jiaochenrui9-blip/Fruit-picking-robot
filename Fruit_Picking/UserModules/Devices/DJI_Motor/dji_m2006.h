#ifndef DJI_M2006_H
#define DJI_M2006_H

#include "dji_motor_manager.h"

#define DJI_M2006_REDUCTION_RATIO       36.0f
#define DJI_M2006_CURRENT_LIMIT          10000

HAL_StatusTypeDef DJI_M2006_Register(DJI_MotorManager_t *manager,DJI_Motor_t *motor,uint8_t motor_id,uint16_t tx_id,
                                               uint8_t tx_slot);

#endif /* DJI_M2006_H */
