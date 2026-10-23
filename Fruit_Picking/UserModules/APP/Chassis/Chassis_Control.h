#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#include "Chassis_Command.h"
#include "dji_motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 底盘：4 台 M3508 + C620（CAN1，ID1~4 = 右前、右后、左后、左前），全向轮解算 → 速度环 → CAN1 电流帧。 */

/* CAN1 底盘电机管理器：main.c 的 CAN 接收回调要用。 */
extern DJI_MotorManager_t chassis_motor_manager;

/* 注册四台电机、配 CAN1 过滤器组 0 并启动；电机先失能，遥控使能后由 TIM8 切到速度模式。 */
HAL_StatusTypeDef Chassis_Control_Init(void);

/* 在 TIM8 1kHz 里调用（速度环 dt 写死 1 ms）：enabled（遥控在线 + SA 使能）为 1 → 按 command 解算 → 速度环；
   为 0 → 失能。最后四台合并发一帧 0x200。 */
void Chassis_Control_Update(const Chassis_Command_t *command, uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
