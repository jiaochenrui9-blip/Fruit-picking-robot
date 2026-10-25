#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WZ_CHANNEL        2u
#define CHASSIS_VY_CHANNEL        3u
#define CHASSIS_VX_CHANNEL        4u
#define CHASSIS_SA_CHANNEL        5u      /* SA：整车使能；SB（CH6）已改给升降，见 Arm_Remote.h */

/* 摇杆和拨杆均使用归一化值，范围为 -1.0f ~ 1.0f。 */
#define CHASSIS_RC_DEADZONE       0.08f
#define CHASSIS_SWITCH_THRESHOLD  0.50f
#define CHASSIS_COMMAND_MIN      (-1.00f)
#define CHASSIS_COMMAND_MAX       1.00f
/* 底盘现在固定中档（2026-10-25 起，之前是低档）；没有开关切档，高 / 低档暂时不用。 */
#define CHASSIS_HIGH_LIMIT        1.00f
#define CHASSIS_MEDIUM_LIMIT      0.65f
#define CHASSIS_LOW_LIMIT         0.35f

typedef enum
{
    CHASSIS_STATE_OFFLINE = 0,
    CHASSIS_STATE_LOCKED,
    CHASSIS_STATE_ENABLED
} Chassis_State_t;

typedef enum
{
    CHASSIS_GEAR_LOW = 0,
    CHASSIS_GEAR_MEDIUM,
    CHASSIS_GEAR_HIGH
} Chassis_Gear_t;

typedef struct
{
    Chassis_State_t state;
    Chassis_Gear_t gear;
    float speed_limit; /* 当前档位的最大输出比例 */
    float vx;          /* C4：右摇杆左右，底盘横向速度 */
    float vy;          /* C3：右摇杆上下，底盘纵向速度 */
    float wz;          /* C2：左摇杆左右，底盘旋转速度 */
} Chassis_Command_t;

void Chassis_Remote_Update(void);
const Chassis_Command_t *Chassis_Remote_GetCommand(void);

#ifdef __cplusplus
}
#endif

#endif
