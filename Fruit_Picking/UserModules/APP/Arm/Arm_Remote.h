#ifndef ARM_REMOTE_H
#define ARM_REMOTE_H

#include <stdint.h>

#include "Chassis_Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 机械臂拨杆：SB（CH6，三档）控制升降，SD（CH8，两档）控制旋转 RS05，SC（CH7，三档）控制伸缩步进；
   左摇杆上下控制夹爪。底盘只用了 CH2 / CH3 / CH4，剩下没用的摇杆通道是 CH1，按左摇杆上下处理（上板核对）。 */
#define ARM_LIFT_CHANNEL          6u
#define ARM_ROTATE_CHANNEL        8u
#define ARM_EXTEND_CHANNEL        7u
#define ARM_GRIPPER_CHANNEL       1u

/* 拨杆使用归一化值：拨上接近 -1，中位接近 0，拨下接近 +1。 */
#define ARM_SWITCH_THRESHOLD      0.50f

/* 夹爪 gripper 的取值 */
#define ARM_GRIPPER_HOLD          0u   /* 不发：使能后左摇杆还没换过区 */
#define ARM_GRIPPER_OPEN          1u
#define ARM_GRIPPER_CLOSE         2u

/* 左摇杆读数 > +门限 → ARM_GRIPPER_POSITIVE，< -门限 → ARM_GRIPPER_NEGATIVE，中间那段保持原来的区（回差）。
   推上读数为正时就是推上打开、拉下闭合；上板发现反了，把这两个对调。 */
#define ARM_GRIPPER_THRESHOLD     0.50f
#define ARM_GRIPPER_POSITIVE      ARM_GRIPPER_OPEN
#define ARM_GRIPPER_NEGATIVE      ARM_GRIPPER_CLOSE

typedef struct
{
    int8_t lift_dir;        /* SB：拨上 +1 往上 / 中位 0 停 / 拨下 -1 往下 */
    uint8_t rotate_basket;  /* SD：拨上 0 到正前方 / 拨下 1 到果框上方 */
} Arm_Command_t;

/* 发给 STM32 节点的那部分：发送也在主循环里（BareMetal_NodeUpdate），不经过 TIM8，所以单独一份、不进 Arm_Command_t。 */
typedef struct
{
    int8_t extend_dir;      /* SC：拨上 +1 / 中位 0 停 / 拨下 -1（使能后回过中位才生效） */
    uint8_t gripper;        /* 左摇杆上下：ARM_GRIPPER_HOLD / OPEN / CLOSE（使能后杆换过一次区才不是 HOLD） */
} Arm_NodeCommand_t;

/* State 传整车使能状态（在线 + SA），即 Chassis_Remote_Update 算出来的 state。 */
void Arm_Remote_Update(Chassis_State_t State);
const Arm_Command_t *Arm_Remote_GetCommand(void);
const Arm_NodeCommand_t *Arm_Remote_GetNodeCommand(void);

#ifdef __cplusplus
}
#endif

#endif
