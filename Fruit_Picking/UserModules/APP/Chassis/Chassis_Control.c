#include "Chassis_Control.h"

#include <math.h>

#include "can.h"
#include "dji_m3508.h"
#include "motor_control.h"

#define CHASSIS_MOTOR_COUNT        4U
/* 电流指令限幅（协议数值，±16384 ↔ ±20 A）：8000 ≈ 9.8 A，接近 M3508 额定电流 10 A（额定 / 最大连续转矩 3 N·m）。
   2026-10-25 为了上坡从 3000（≈ 3.7 A，约 1.1 N·m）提上来。 */
#define CHASSIS_CURRENT_LIMIT   8000.0f
/* 速度环积分项最多出这么多（协议数值）：坡上要一直出力，靠积分把转速稳住，不然全靠比例项、转速会被拖下来。
   驱动默认 1000（≈ 1.2 A，升降还在用），底盘单独放到 6000（≈ 7.3 A）。 */
#define CHASSIS_SPEED_I_LIMIT   6000.0f
/* 指令加速度限制：vx / vy / wz（−1~1，已乘档位）每秒最多变这么多，0 → 中档推满（0.65）约 0.33 s。
   电流限幅放大以后，不加这个，平地起步 / 刹车的电流会是原来的两倍左右（机械臂高，容易晃、打滑）。 */
#define CHASSIS_COMMAND_RAMP_PER_S   1.0f
#define CHASSIS_COMMAND_RAMP_STEP   (CHASSIS_COMMAND_RAMP_PER_S * 0.001f)   /* TIM8 1 kHz，每拍最多变这么多 */

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
/* 限过加速度的指令（只用 vx / vy / wz），在 TIM8 里一拍一拍追发布出来的 chassis_command；失能时清零。 */
static Chassis_Command_t chassis_ramped_command;

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
        MotorControl_SetSpeedIntegralLimit(motor, CHASSIS_SPEED_I_LIMIT);

        /* 先失能，遥控使能后由 TIM8 切到速度模式。 */
        MotorControl_SetMode(motor, MOTOR_MODE_DISABLE);
    }

    return DJI_MotorManager_Start(&chassis_motor_manager);
}

/* 指令往目标走一步：vx / vy / wz 的变化按同一个比例缩小，变化最大的那个这一拍最多变 CHASSIS_COMMAND_RAMP_STEP。
   三个量同时到达目标，从停着起步时合成的运动方向一开始就是摇杆的方向。 */
static void Chassis_Control_Ramp(Chassis_Command_t *ramped, const Chassis_Command_t *target)
{
    float dvx = target->vx - ramped->vx;
    float dvy = target->vy - ramped->vy;
    float dwz = target->wz - ramped->wz;
    float max_abs = fabsf(dvx);

    if (fabsf(dvy) > max_abs)
    {
        max_abs = fabsf(dvy);
    }
    if (fabsf(dwz) > max_abs)
    {
        max_abs = fabsf(dwz);
    }
    if (max_abs > CHASSIS_COMMAND_RAMP_STEP)
    {
        const float scale = CHASSIS_COMMAND_RAMP_STEP / max_abs;

        dvx *= scale;
        dvy *= scale;
        dwz *= scale;
    }
    ramped->vx += dvx;
    ramped->vy += dvy;
    ramped->wz += dwz;
}

/* 底盘：遥控在线 + 使能才驱动；其余（离线 / 锁车）一律失能。解算用的是限过加速度的指令。 */
void Chassis_Control_Update(const Chassis_Command_t *command, uint8_t enabled)
{
    const uint32_t now_tick = HAL_GetTick();
    Chassis_Speed_t speed = {0};
    MotorControlMode_e desired_mode;

    if (enabled != 0U)
    {
        desired_mode = MOTOR_MODE_SPEED;
        Chassis_Control_Ramp(&chassis_ramped_command, command);
        Omni_Remote_Update(&speed, &chassis_ramped_command);
    }
    else
    {
        desired_mode = MOTOR_MODE_DISABLE;
        /* 失能时电机不出力，指令也清零：重新使能从 0 开始加速。 */
        chassis_ramped_command.vx = 0.0f;
        chassis_ramped_command.vy = 0.0f;
        chassis_ramped_command.wz = 0.0f;
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
