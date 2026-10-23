#include "dji_motor.h"

HAL_StatusTypeDef DJI_Motor_SetCurrent(DJI_Motor_t *motor,
                                       int16_t target_current)
{
    if ((motor == NULL) || (motor->esc_id == 0U) ||
        (motor->current_limit <= 0) ||
        (target_current < -motor->current_limit) ||
        (target_current > motor->current_limit))
    {
        return HAL_ERROR;
    }

    motor->target_current = target_current;
    motor->output_current = target_current;
    return HAL_OK;
}

uint8_t DJI_Motor_GetFeedback(DJI_Motor_t *motor,
                              DJI_MotorFeedback_t *feedback)
{
    uint32_t primask;

    if ((motor == NULL) || (feedback == NULL))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (motor->feedback_updated == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    feedback->angle = motor->encoder;
    feedback->speed_rpm = motor->speed_rpm;
    feedback->current = motor->feedback_current;
    feedback->temperature = motor->temperature;
    feedback->total_encoder = motor->total_encoder;
    motor->feedback_updated = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

uint8_t DJI_Motor_IsOnline(const DJI_Motor_t *motor,
                           uint32_t now_tick,
                           uint32_t timeout_ms)
{
    if ((motor == NULL) || (motor->feedback_count == 0U))
    {
        return 0U;
    }

    return ((now_tick - motor->last_feedback_tick) <= timeout_ms) ? 1U : 0U;
}
