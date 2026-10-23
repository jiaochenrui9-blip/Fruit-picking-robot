#include "dji_m3508.h"

HAL_StatusTypeDef DJI_M3508_Register(DJI_MotorManager_t *manager,
                                     DJI_Motor_t *motor,
                                     uint8_t motor_id,
                                     uint16_t tx_id,
                                     uint8_t tx_slot)
{
    HAL_StatusTypeDef status;

    if (((motor_id <= 4U) &&
         ((tx_id != DJI_MOTOR_CONTROL_ID_1_4) ||
          (tx_slot != (motor_id - 1U)))) ||
        ((motor_id >= 5U) &&
         ((tx_id != DJI_MOTOR_CONTROL_ID_5_8) ||
          (tx_slot != (motor_id - 5U)))))
    {
        return HAL_ERROR;
    }

    status = DJI_MotorManager_Register(manager, motor, motor_id,
                                       0x200U + motor_id,
                                       tx_id, tx_slot);
    if (status != HAL_OK)
    {
        return status;
    }

    motor->type = DJI_MOTOR_M3508;
    motor->mode = MOTOR_MODE_DISABLE;
    motor->reduction_ratio = DJI_M3508_REDUCTION_RATIO;
    motor->current_limit = DJI_M3508_CURRENT_LIMIT;
    return HAL_OK;
}
