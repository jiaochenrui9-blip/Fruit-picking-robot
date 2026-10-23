#ifndef ARM_LIFT_H
#define ARM_LIFT_H

#include <stdint.h>

#include "dji_motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 升降：M3508 + C620（CAN2，ID1：0x200 槽位 0，反馈 0x201），速度模式。 */

/* CAN2 机械臂上的大疆电机管理器（现在只有升降 3508）：main.c 的 CAN 接收回调要用。 */
extern DJI_MotorManager_t arm_motor_manager;

/* 注册升降电机、配 CAN2 过滤器组 14 并启动 CAN2；电机先失能。 */
HAL_StatusTypeDef Arm_Lift_Init(void);

/* 在 TIM8 1kHz 里调用：enabled（遥控在线 + SA 使能）为 1 → 速度模式，目标 = lift_dir × LIFT_SPEED_RPM；
   为 0 → 失能（机构失能后自己能停住）。最后 CAN2 发一帧 0x200。 */
void Arm_Lift_Update(int8_t lift_dir, uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
