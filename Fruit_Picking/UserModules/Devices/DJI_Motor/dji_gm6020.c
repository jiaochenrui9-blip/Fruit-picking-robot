//
// Created by game on 2026/8/5.
//
#include "dji_gm6020.h"
HAL_StatusTypeDef DJI_GM6020_Register(DJI_MotorManager_t* manager,DJI_Motor_t *motor,uint8_t motor_id,uint16_t tx_id,
                                    uint8_t tx_slot)
{
    HAL_StatusTypeDef status;

    if ((motor_id == 0U) || (motor_id > 7U) ||
        ((motor_id <= 4U) &&
         ((tx_id != 0x1FEU) || (tx_slot != (motor_id - 1U)))) ||
        ((motor_id >= 5U) &&
         ((tx_id != 0x2FEU) || (tx_slot != (motor_id - 5U)))))
    {
        return HAL_ERROR;
    }

    status = DJI_MotorManager_Register(manager, motor, motor_id,
                                       0x204U + motor_id,
                                       tx_id, tx_slot);

    if (status != HAL_OK)
    {
        return status;
    }

    motor->type = DJI_MOTOR_GM6020;
    motor->current_limit = DJI_GM6020_MAX_CURRENT_LIMIT;
    motor->reduction_ratio = 1;
    motor->mode = MOTOR_MODE_DISABLE;
    return HAL_OK;
}
