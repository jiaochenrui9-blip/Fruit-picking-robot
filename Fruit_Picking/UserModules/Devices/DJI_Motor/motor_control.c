#include "motor_control.h"

/* 当前位置-速度串级控制参数。 */
#define MOTOR_CONTROL_POSITION_PERIOD_MS          5U
#define MOTOR_CONTROL_OFFLINE_TIMEOUT_MS        100U
#define MOTOR_CONTROL_SPEED_PERIOD_S          0.001f
#define MOTOR_CONTROL_POSITION_PERIOD_S       0.005f

#define MOTOR_CONTROL_ENCODER_COUNTS_PER_REV 8192.0f

#define MOTOR_CONTROL_POSITION_KP              15.0f
#define MOTOR_CONTROL_POSITION_KI               0.0f
#define MOTOR_CONTROL_POSITION_KD               0.0f
#define MOTOR_CONTROL_POSITION_SPEED_LIMIT     300.0f
#define MOTOR_CONTROL_POSITION_DEADBAND_DEG      0.2f

#define MOTOR_CONTROL_SPEED_KP                   3.0f
#define MOTOR_CONTROL_SPEED_KI                   0.02f
#define MOTOR_CONTROL_SPEED_KD                   0.0f
#define SPEED_PID_I_OUTPUT_LIMIT              1000.0f
#define POSITION_PID_I_OUTPUT_LIMIT           3000.0f

static float MotorControl_EncoderToOutputDegree(const DJI_Motor_t *motor,
                                                int32_t encoder_delta)
{
    return ((float)encoder_delta * 360.0f) /
           (MOTOR_CONTROL_ENCODER_COUNTS_PER_REV * motor->reduction_ratio);
}

HAL_StatusTypeDef MotorControl_Init(DJI_Motor_t *motor, float current_limit,
                                    int8_t motor_direction,
                                    float speed_kp, float speed_ki, float speed_kd,
                                    float pos_kp, float pos_ki, float pos_kd)
{
    if ((motor == NULL) || (motor->esc_id == 0U) ||
        (motor->reduction_ratio <= 0.0f) ||
        ((motor_direction != 1) && (motor_direction != -1)))
    {
        return HAL_ERROR;
    }

    motor->current_limit = (int16_t)current_limit;
    motor->motor_direction = motor_direction;

    PID_Init(&motor->speed.pid,
             speed_kp,
             speed_ki,
             speed_kd,
             -current_limit,
             current_limit,
             -SPEED_PID_I_OUTPUT_LIMIT,
             SPEED_PID_I_OUTPUT_LIMIT);

    PID_Init(&motor->position.pid,
             pos_kp,
             pos_ki,
             pos_kd,
             -MOTOR_CONTROL_POSITION_SPEED_LIMIT,
             MOTOR_CONTROL_POSITION_SPEED_LIMIT,
             -POSITION_PID_I_OUTPUT_LIMIT,
             POSITION_PID_I_OUTPUT_LIMIT);

    motor->speed.target_rpm = 0.0f;
    motor->position.target_deg = 0.0f;
    PID_SetTarget(&motor->position.pid, motor->position.target_deg);
    PID_SetTarget(&motor->speed.pid, motor->speed.target_rpm);
    return DJI_Motor_SetCurrent(motor, 0);
}

void MotorControl_SetTargetDegree(DJI_Motor_t *motor, float target_degree)
{
    if (motor != NULL)
    {
        motor->position.target_deg = target_degree;
        PID_SetTarget(&motor->position.pid, target_degree);
    }
}

static void Motor_Offline_Update(DJI_Motor_t* motor)
{
    if (motor == NULL)
    {
        return;
    }

    PID_Reset(&motor->speed.pid);
    PID_Reset(&motor->position.pid);

    motor->output_current = 0;
}
void MotorControl_SetTargetSpeed(DJI_Motor_t *motor, int16_t target_speed)
{
    if  (motor == NULL )
    {
        return;
    }
    motor->speed.target_rpm = target_speed;
}

void MotorControl_SetMode(DJI_Motor_t *motor, MotorControlMode_e mode)
{
    if (motor == NULL )
    {
        return;
    }
    PID_Reset(&motor->speed.pid);
    PID_Reset(&motor->position.pid);
    motor->position.target_deg = 0;
    motor->speed.target_rpm = 0;
    motor->output_current = 0;
    motor->target_current = 0;

    motor->mode = mode;
}

static void MotorControl_Disable_Update(DJI_Motor_t *motor)
{
    PID_Reset(&motor->speed.pid);
    PID_Reset(&motor->position.pid);
    motor->speed.target_rpm = 0.0f;
    motor->output_current = 0;
}

static void MotorControl_Current_Update(DJI_Motor_t *motor)
{
    if (motor->target_current > motor->current_limit)
    {
        motor->target_current = motor->current_limit;
    }
    if (motor->target_current < -motor->current_limit)
    {
        motor->target_current = -motor->current_limit;
    }
    motor->output_current = motor->target_current;
}

static void MotorControl_Speed_Update(DJI_Motor_t *motor)
{
    PID_SetTarget(&motor->speed.pid, motor->speed.target_rpm);
    motor->output_current = (int16_t)PID_Calculate(
        &motor->speed.pid, (float)motor->speed_rpm,
        MOTOR_CONTROL_SPEED_PERIOD_S);
}

static void MotorControl_Position_Update(DJI_Motor_t *motor,
                                         uint32_t now_tick)
{
    float now_degree;
    float position_error;

    if (motor->position.reference_ready == 0U)
    {
        motor->position.zero_encoder = motor->total_encoder;
        motor->position.reference_ready = 1U;
        motor->position.last_update_tick = now_tick;
    }

    if ((now_tick - motor->position.last_update_tick) >=
        MOTOR_CONTROL_POSITION_PERIOD_MS)
    {
        now_degree = MotorControl_EncoderToOutputDegree(
            motor,
            motor->total_encoder - motor->position.zero_encoder);
        PID_SetTarget(&motor->position.pid,
                      motor->position.target_deg);
        position_error = motor->position.target_deg - now_degree;

        if ((position_error <= MOTOR_CONTROL_POSITION_DEADBAND_DEG) &&
            (position_error >= -MOTOR_CONTROL_POSITION_DEADBAND_DEG))
        {
            motor->speed.target_rpm = 0.0f;
            PID_Reset(&motor->position.pid);
        }
        else
        {
            motor->speed.target_rpm = PID_Calculate(
                &motor->position.pid, now_degree,
                MOTOR_CONTROL_POSITION_PERIOD_S);
        }

        motor->position.last_update_tick = now_tick;
    }

    PID_SetTarget(&motor->speed.pid, motor->speed.target_rpm);
    motor->output_current = (int16_t)PID_Calculate(
        &motor->speed.pid, (float)motor->speed_rpm,
        MOTOR_CONTROL_SPEED_PERIOD_S);
}

void MotorControl_Update(DJI_Motor_t *motor, uint32_t now_tick)
{
    if (motor == NULL)
    {
        return;
    }

    if (DJI_Motor_IsOnline(motor, now_tick,
                           MOTOR_CONTROL_OFFLINE_TIMEOUT_MS) == 0U)
    {
        Motor_Offline_Update(motor);
        return;
    }

    switch (motor->mode)
    {
        case MOTOR_MODE_POSITION:
            MotorControl_Position_Update(motor, now_tick);
            break;

        case MOTOR_MODE_SPEED:
            MotorControl_Speed_Update(motor);
            break;

        case MOTOR_MODE_CURRENT:
            MotorControl_Current_Update(motor);
            break;

        case MOTOR_MODE_DISABLE:
        default:
            MotorControl_Disable_Update(motor);
            break;
    }
}
