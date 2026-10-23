#include "Chassis_Control.h"

#include "can.h"
#include "dji_m3508.h"
#include "motor_control.h"

#define CHASSIS_MOTOR_COUNT        4U
#define CHASSIS_CURRENT_LIMIT   3000.0f   /* 电流指令限幅（协议数值，非安培），试调值 */

/* 速度环 Kp / Ki / Kd，下标 0~3 对应 C620 ID1~4（右前、右后、左后、左前），保留各 ID 的实测试调值。 */
static const float chassis_speed_pid[CHASSIS_MOTOR_COUNT][3] =
{
    {5.0f, 20.0f, 0.0f},
    {8.0f, 20.0f, 0.0f},
    {5.0f, 20.0f, 0.0f},
    {5.0f, 20.0f, 0.0f}
};

DJI_MotorManager_t chassis_motor_manager;
DJI_Motor_t chassis_motors[CHASSIS_MOTOR_COUNT];
volatile HAL_StatusTypeDef chassis_can_tx_status = HAL_OK;

HAL_StatusTypeDef Chassis_Control_Init(void)
{
    if (DJI_MotorManager_Init(&chassis_motor_manager, &hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        DJI_Motor_t *motor = &chassis_motors[i];

        /* 电调 ID = 下标 + 1，都用 0x200 控制帧，槽位 = ID - 1。 */
        if (DJI_M3508_Register(&chassis_motor_manager, motor,
                               (uint8_t)(i + 1U), DJI_MOTOR_CONTROL_ID_1_4,
                               i) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* 方向都是 1（2026-10-21 实测）；位置环不用，参数给 0。 */
        if (MotorControl_Init(motor, CHASSIS_CURRENT_LIMIT, 1,
                              chassis_speed_pid[i][0],
                              chassis_speed_pid[i][1],
                              chassis_speed_pid[i][2],
                              0.0f, 0.0f, 0.0f) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* 先失能，遥控使能后由 TIM8 切到速度模式。 */
        MotorControl_SetMode(motor, MOTOR_MODE_DISABLE);
    }

    return DJI_MotorManager_Start(&chassis_motor_manager);
}

/* 底盘：遥控在线 + 使能才驱动；其余（离线 / 锁车）一律失能。 */
void Chassis_Control_Update(const Chassis_Command_t *command, uint8_t enabled)
{
    const uint32_t now_tick = HAL_GetTick();
    Chassis_Speed_t speed = {0};
    MotorControlMode_e desired_mode;

    if (enabled != 0U)
    {
        desired_mode = MOTOR_MODE_SPEED;
        Omni_Remote_Update(&speed, command);
    }
    else
    {
        desired_mode = MOTOR_MODE_DISABLE;
    }

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        DJI_Motor_t *motor = &chassis_motors[i];

        /* SetMode 会清 PID 和目标，只在模式真正切换时调用；
           每周期调用会把速度环积分清掉，退化成纯比例。 */
        if (motor->mode != desired_mode)
        {
            MotorControl_SetMode(motor, desired_mode);
        }
        if (desired_mode == MOTOR_MODE_SPEED)
        {
            MotorControl_SetTargetSpeed(motor, (int16_t)speed.wheel_rpm[i]);
        }

        MotorControl_Update(motor, now_tick);
    }

    /* 四台电机合并成一帧 0x200 发送；未收到反馈或掉线时驱动输出 0。 */
    chassis_can_tx_status = DJI_MotorManager_SendAll(&chassis_motor_manager);
}
