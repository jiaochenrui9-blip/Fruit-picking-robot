#include "Arm_Lift.h"

#include "can.h"
#include "dji_m3508.h"
#include "motor_control.h"

/* 升降 3508（CAN2，C620 ID1）：SB 拨上 / 中 / 下 = +LIFT_SPEED_RPM / 0 / -LIFT_SPEED_RPM。 */
/* 电流指令限幅（协议数值，±16384 ↔ ±20 A）：8000 ≈ 9.8 A，接近 M3508 额定电流 10 A。
   2026-10-25 用户要求调大（之前是上板试调的 6000，约 7.3 A）。还没有软限位，拨着 SB 顶到头会一直出满这么多。 */
#define LIFT_CURRENT_LIMIT      8000.0f
#define LIFT_SPEED_KP             15.0f   /* 速度环，上板试调值 */
#define LIFT_SPEED_KI             30.0f
#define LIFT_SPEED_KD              0.0f
#define LIFT_SPEED_RPM          1000      /* 转子转速；输出轴约 1000 / 19.2 ≈ 52 rpm */

/* CAN2 机械臂上的大疆电机：现在只有升降 3508。 */
DJI_MotorManager_t arm_motor_manager;
DJI_Motor_t lift_motor;
volatile HAL_StatusTypeDef arm_can_tx_status = HAL_OK;

HAL_StatusTypeDef Arm_Lift_Init(void)
{
    if (DJI_MotorManager_Init(&arm_motor_manager, &hcan2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* C620 ID1：0x200 控制帧槽位 0，反馈 0x201。ID 只要同一路总线上不重复，和 CAN1 底盘的 ID1 不冲突。 */
    if (DJI_M3508_Register(&arm_motor_manager, &lift_motor,
                           1U, DJI_MOTOR_CONTROL_ID_1_4, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 方向 -1：+rpm 往上（2026-10-22 实测）；位置环不用，参数给 0。 */
    if (MotorControl_Init(&lift_motor, LIFT_CURRENT_LIMIT, -1,
                          LIFT_SPEED_KP, LIFT_SPEED_KI, LIFT_SPEED_KD,
                          0.0f, 0.0f, 0.0f) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 先失能，遥控使能后由 TIM8 切到速度模式。 */
    MotorControl_SetMode(&lift_motor, MOTOR_MODE_DISABLE);

    /* 配 CAN2 的过滤器组 14，启动 CAN2 并打开 FIFO0 接收中断。 */
    return DJI_MotorManager_Start(&arm_motor_manager);
}

/*
 * 升降：遥控在线 + 使能 → 速度模式，目标 = 升降方向 × LIFT_SPEED_RPM
 * （SB 在中位、或使能后还没回过中位时方向为 0，原地停住）；
 * 其余（离线 / 锁车）失能，机构失能后自己能停住。
 */
void Arm_Lift_Update(int8_t lift_dir, uint8_t enabled)
{
    const uint32_t now_tick = HAL_GetTick();
    MotorControlMode_e desired_mode;

    if (enabled != 0U)
    {
        desired_mode = MOTOR_MODE_SPEED;
    }
    else
    {
        desired_mode = MOTOR_MODE_DISABLE;
    }

    /* 和底盘一样，只在模式真正切换时调 SetMode。 */
    if (lift_motor.mode != desired_mode)
    {
        MotorControl_SetMode(&lift_motor, desired_mode);
    }
    if (desired_mode == MOTOR_MODE_SPEED)
    {
        MotorControl_SetTargetSpeed(&lift_motor,
                                    (int16_t)(lift_dir * LIFT_SPEED_RPM));
    }

    MotorControl_Update(&lift_motor, now_tick);

    /* CAN2 上发一帧 0x200（升降在槽位 0）；没收到反馈或掉线时驱动输出 0。 */
    arm_can_tx_status = DJI_MotorManager_SendAll(&arm_motor_manager);
}
