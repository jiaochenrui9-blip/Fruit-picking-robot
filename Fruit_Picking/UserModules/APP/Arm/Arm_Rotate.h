#ifndef ARM_ROTATE_H
#define ARM_ROTATE_H

#include <stdint.h>

#include "RS05_receive.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 旋转：RS05（CAN2，私有协议，电机 ID 1，主机 0xFD），CSP 位置模式点动：往果框上方 / 往正前方走，或者停在当前位置，
   两端就是软限位。还管：在线 / 重启跟踪、zero_sta 自动检查、每 100 ms 读 mechPos。CAN2 的帧都在 TIM8 里发。 */

/* CAN2 上的 RS05（旋转）：main.c 的 CAN 接收回调要用，和 arm_motor_manager 收同一帧，各自按标准帧 / 扩展帧挑。 */
extern RS05_ManagerTypedef rs05_manager;

/* 注册 RS05、配 CAN2 过滤器组 15，发一次停止（让它处在失能状态）。要在 Arm_Lift_Init 启动 CAN2 之后调。 */
HAL_StatusTypeDef Arm_Rotate_Init(void);

/* 在 TIM8 1kHz 里调用，每拍最多发一帧 RS05 的帧：rotate_dir 是 +1 往果框上方 / 0 停 / -1 往正前方（SC），
   enabled 是遥控在线 + SA 使能。能不能使能、使能流程、重发目标、zero_sta 检查、读 mechPos 都在里面。 */
void Arm_Rotate_Update(int8_t rotate_dir, uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
