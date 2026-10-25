#ifndef ARM_REMOTE_H
#define ARM_REMOTE_H

#include <stdint.h>

#include "Chassis_Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 机械臂遥控（2026-10-24 按机械那边的新映射改）：
   SA（CH5，两档）整车使能，顺带果篮：使能关、锁定开、掉线不动（SA 由 Chassis_Remote 读，这里用它算好的整车状态）；
   SB（CH6，三档）升降；SC（CH7，三档）旋转 RS05 往两端走 / 停；SD（CH8，两档）拨上夹爪合（合到位后剪线）、拨下夹爪开；
   左摇杆上下（CH1，不回中）步进：推上前进 / 中间停 / 拉下后退。底盘只用 CH2 / CH3 / CH4。 */
#define ARM_LIFT_CHANNEL          6u
#define ARM_ROTATE_CHANNEL        7u
#define ARM_GRIPPER_CHANNEL       8u
#define ARM_EXTEND_CHANNEL        1u

/* 拨杆使用归一化值：拨上接近 -1，中位接近 0，拨下接近 +1。 */
#define ARM_SWITCH_THRESHOLD      0.50f

/* SC 拨上（左）时 RS05 往哪边转：-1 往正前方，+1 往果框上方（rotate_dir 的正负见 Arm_Command_t）；拨下自动相反。
   上板发现左右反了，把它改成 1。 */
#define ARM_ROTATE_SC_UP_DIR      (-1)

/* 左摇杆上下 → 步进（摇杆推上读数为正，和拨杆相反）：> +0.5 前进、< -0.5 后退、绝对值 < 0.3 停，
   0.3 ~ 0.5 之间保持原来的区（回差：杆停在门限附近也不会在走 / 停之间来回跳，每跳一次就要给节点发一帧）。 */
#define ARM_EXTEND_MOVE_THRESHOLD 0.50f
#define ARM_EXTEND_STOP_THRESHOLD 0.30f

/* 夹爪 gripper 的取值 */
#define ARM_GRIPPER_HOLD          0u   /* 不发：使能后 SD 还没拨过 */
#define ARM_GRIPPER_OPEN          1u
#define ARM_GRIPPER_CLOSE         2u

/* SD 读数 > +门限（拨下）→ ARM_GRIPPER_POSITIVE，< -门限（拨上）→ ARM_GRIPPER_NEGATIVE，中间那段保持原来的区。
   现在是拨上合、拨下开；反了就把这两个对调。 */
#define ARM_GRIPPER_THRESHOLD     0.50f
#define ARM_GRIPPER_POSITIVE      ARM_GRIPPER_OPEN
#define ARM_GRIPPER_NEGATIVE      ARM_GRIPPER_CLOSE

/* 果篮 basket 的取值 */
#define ARM_BASKET_HOLD           0u   /* 不发：遥控掉线，保持原样 */
#define ARM_BASKET_OPEN           1u   /* SA 锁定 */
#define ARM_BASKET_CLOSE          2u   /* SA 使能 */

typedef struct
{
    int8_t lift_dir;        /* SB：拨上 +1 往上 / 中位 0 停 / 拨下 -1 往下 */
    int8_t rotate_dir;      /* SC：+1 往果框上方（RS05 正方向）/ 0 停 / -1 往正前方；拨上是哪个看 ARM_ROTATE_SC_UP_DIR */
} Arm_Command_t;

/* 发给 STM32 节点的那部分：发送也在主循环里（Arm_Node_Update），不经过 TIM8，所以单独一份、不进 Arm_Command_t。 */
typedef struct
{
    int8_t extend_dir;      /* 左摇杆：推上 +1 前进 / 中间 0 停 / 拉下 -1 后退（使能后回过中间才生效） */
    uint8_t gripper;        /* SD：ARM_GRIPPER_HOLD / OPEN / CLOSE（使能后 SD 拨过一次才不是 HOLD）；剪线跟着它，见 Arm_Node.c */
    uint8_t basket;         /* SA：ARM_BASKET_HOLD / OPEN / CLOSE */
} Arm_NodeCommand_t;

/* State 传整车使能状态（在线 + SA），即 Chassis_Remote_Update 算出来的 state。 */
void Arm_Remote_Update(Chassis_State_t State);
const Arm_Command_t *Arm_Remote_GetCommand(void);
const Arm_NodeCommand_t *Arm_Remote_GetNodeCommand(void);

#ifdef __cplusplus
}
#endif

#endif
