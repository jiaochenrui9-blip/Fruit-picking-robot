#ifndef BAREMETAL_H
#define BAREMETAL_H

#include "main.h"
/* 三个机构的头文件：main.c 的 CAN 接收回调要用里面的电机管理器
   （chassis_motor_manager、arm_motor_manager、rs05_manager），main.c 只 include 本文件。 */
#include "Arm_Lift.h"
#include "Arm_Rotate.h"
#include "Chassis_Control.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 裸机结构，沿用 C:\Project\Chassis_BareMetal：没有调度器，只有一条 1kHz 的控制节拍和一个主循环。
 * 本模块只做调度，各机构的逻辑在各自的文件里（APP/Chassis/Chassis_Control、APP/Arm/Arm_Lift / Arm_Rotate / Arm_Node）。
 *
 *   main() while(1) ────────  BareMetal_MainLoop()
 *        ├─ ELRS_DMA_Process：解析 CRSF（中断里只记 DMA 写到哪）
 *        ├─ Chassis_Remote_Update：在线 + SA（CH5）→ state，摇杆 → vx / vy / wz（固定低档）
 *        ├─ Arm_Remote_Update：SB（CH6）→ 升降方向（使能后回过一次中位才生效），SD（CH8）→ 旋转到正前方 / 果框；
 *        │    SC（CH7）→ 伸缩方向（同样回过中位才生效），左摇杆上下（CH1）→ 夹爪开 / 合（使能后杆换过一次区才算）
 *        │    → 关中断，chassis_command、arm_command 一起整条拷贝（发布）
 *        └─ Arm_Node_Update：伸缩 / 夹爪指令变了就经 USART6 给机械臂 STM32 节点发命令帧（阻塞发送，
 *             两帧至少隔 50 ms）；伸缩、夹爪不经过 TIM8
 *
 *   TIM8 (1kHz, 优先级 5)  ── BareMetal_ControlTick()：先判一次遥控在线 + SA 使能，再按顺序调
 *        ├─ Chassis_Control_Update：使能 → 全向轮解算 → 速度环；否则失能 → CAN1 电流帧
 *        ├─ Arm_Lift_Update：使能 → 速度环，目标 = 升降方向 × LIFT_SPEED_RPM；否则失能 → CAN2 电流帧
 *        └─ Arm_Rotate_Update：遥控在线 + SA、RS05 在线、zero_sta 已是 1、位置在行程内 → CSP 使能，
 *           目标按 SD（使能后先停在原地，拨一下 SD 才动）；否则停止。另外每 100 ms 读 mechPos → CAN2 扩展帧
 *
 * CAN2 的帧都从 TIM8 里发（初始化时除外），主循环不发：两边同时调 HAL_CAN_AddTxMessage 会抢同一个邮箱。
 *
 * 中断里只读发布出来的 chassis_command / arm_command，不碰遥控模块自己那份：
 * Chassis_Remote_Update 是逐字段写的，中途被打断就会读到半新半旧的一条。
 * 机构模块不读遥控：要用的指令和"是否使能"都由这里当参数传进去。
 * 所有原生中断（HAL 回调）都写在 Core/Src/main.c 里，只负责转交给对应驱动或本模块。
 */

/* 电机、遥控全部初始化完成后，才放开 TIM8 的 1kHz 控制节拍。 */
HAL_StatusTypeDef BareMetal_Init(void);

/* 在 TIM8 周期中断里调用：判遥控 → 底盘、升降、旋转各自控制 → 发送。 */
void BareMetal_ControlTick(void);

/* 在 main 的 while(1) 里调用：ELRS 解析 + 遥控逻辑 + 发布指令 + 给机械臂节点发命令。 */
void BareMetal_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif
