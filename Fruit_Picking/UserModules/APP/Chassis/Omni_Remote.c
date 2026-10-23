#include "Chassis_Command.h"

#include <math.h>

#define CHASSIS_R      0.20f      // 轮心到底盘中心距离 m，实测
#define WHEEL_RADIUS   0.076f     // 全向轮半径 m，实测
#define M3508_RATIO    19.0f      // 减速比（精确值 3591/187≈19.2），C620 反馈的是转子转速
#define RPM_MAX        8000.0f    // 转子转速上限，按需调整
#define VXY_MAX        1.0f       // 平移速度上限 m/s（高速档摇杆推满）
#define WZ_MAX         3.0f       // 旋转角速度上限 rad/s（高速档摇杆推满，约每秒半圈），按需调整

/* ID1~4 从右前开始顺时针：右前、右后、左后、左前。
 * 下表是驱动方向角（不是轮子位置角），从右（vx 方向）逆时针量；
 * 逻辑正转沿顺时针切向，推动车体顺时针转。
 */
static const float wheel_theta[WHEEL_NUM] = {
    5.497787f,  // 315° ID1 右前（位置角 45°）
    3.926991f,  // 225° ID2 右后（位置角 315°）
    2.356194f,  // 135° ID3 左后（位置角 225°）
    0.785398f   // 45°  ID4 左前（位置角 135°）
};

// 指令 vx、vy、wz（-1~1）→ 车体速度（m/s、rad/s）→ 各轮线速度 → 转子目标转速 (rpm)
// 输出为乘 dir 之前的"逻辑方向"
void Omni_Remote_Update(Chassis_Speed_t *speed, const Chassis_Command_t *command)
{
    const float k = 60.0f / (2.0f * 3.1415926f) / WHEEL_RADIUS * M3508_RATIO;   // m/s → 转子 rpm
    float vx = command->vx * VXY_MAX;   // 向右 m/s
    float vy = command->vy * VXY_MAX;   // 向前 m/s
    float wz = command->wz * WZ_MAX;    // 顺时针 rad/s
    float max_abs = 0.0f;

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        float v = cosf(wheel_theta[i]) * vx      // 平移速度投影到驱动方向
                + sinf(wheel_theta[i]) * vy
                + CHASSIS_R * wz;                // 旋转带来的轮速 = 半径 × 角速度
        speed->wheel_rpm[i] = v * k;
        if (fabsf(speed->wheel_rpm[i]) > max_abs) max_abs = fabsf(speed->wheel_rpm[i]);
    }

    // 超过转速上限时整体等比例缩放，保持合成运动方向不变
    if (max_abs > RPM_MAX)
    {
        float scale = RPM_MAX / max_abs;
        for (int i = 0; i < WHEEL_NUM; i++) speed->wheel_rpm[i] *= scale;
    }
}
