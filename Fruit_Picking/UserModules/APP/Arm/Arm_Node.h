#ifndef ARM_NODE_H
#define ARM_NODE_H

#include <stdint.h>

#include "Arm_Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 机械臂 STM32 节点（C:\Clion\SC09ServoTest）：C 板 USART6 → 节点 USART2，命令见 ArmNode_Command.h。
   节点管伸缩步进（PD42S1）和夹爪舵机（SC09）。收发都在主循环里，不进 TIM8。 */

/* 启动 USART6 循环 DMA 接收 + 空闲中断。开机不给节点发任何命令。 */
void Arm_Node_Init(void);

/* 在主循环里、遥控逻辑算完之后调用：cmd 是 Arm_Remote 算好的伸缩 / 夹爪指令，
   enabled 是主循环这份整车状态（在线 + SA）。指令变了才发帧，两帧至少隔 50 ms，阻塞发送约 0.7 ms。 */
void Arm_Node_Update(const Arm_NodeCommand_t *cmd, uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
