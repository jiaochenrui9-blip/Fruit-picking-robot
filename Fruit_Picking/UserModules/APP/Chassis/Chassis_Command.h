#ifndef CHASSIS_COMMAND_H
#define CHASSIS_COMMAND_H

#include "Chassis_Remote.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_NUM   4

/* 速度结构体：传给更新函数，wheel_rpm 自动更新。
 * 全向轮顺序（当前底盘）：0 右前、1 右后、2 左后、3 左前，对应 ID1~4。
 * 麦轮函数仍用原顺序：0 前左、1 后左、2 后右、3 前右。
 */
typedef struct
{
    float wheel_rpm[WHEEL_NUM];   // 各轮转子目标转速 rpm，乘电机方向 dir 之前的逻辑方向
} Chassis_Speed_t;

/* 更新函数读传进来的指令 command 的 vx、vy、wz（-1~1；vx 向右、vy 向前、wz 顺时针为正），
 * 自己不读遥控；遥控控制时 command 传 Chassis_Remote_GetCommand()。
 * 各底盘的参数写在各自的 .c 里。两种底盘用法相同，换底盘只换函数名。
 */
void Mecanum_Remote_Update(Chassis_Speed_t *speed, const Chassis_Command_t *command);
void Omni_Remote_Update(Chassis_Speed_t *speed, const Chassis_Command_t *command);

#ifdef __cplusplus
}
#endif

#endif
