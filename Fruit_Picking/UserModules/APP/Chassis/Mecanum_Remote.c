//
// Created by game on 2026/9/27.
//

#include "Chassis_Command.h"

#include <math.h>

#define CHASSIS_LX     0.20f      // 轮心到底盘中心的前后距离 m，实测
#define CHASSIS_LY     0.20f      // 轮心到底盘中心的左右距离 m，实测
#define WHEEL_RADIUS   0.076f     // 麦轮半径 m，实测
#define M3508_RATIO    19.0f      // 减速比（精确值 3591/187≈19.2），C620 反馈的是转子转速
#define RPM_MAX        8000.0f    // 转子转速上限，按需调整
#define VXY_MAX        1.0f       // 平移速度上限 m/s（高速档摇杆推满）
#define WZ_MAX         3.0f       // 旋转角速度上限 rad/s（高速档摇杆推满，约每秒半圈），按需调整

// X 型安装（从上往下看辊子连成 X 形）
// 指令 vx、vy、wz（-1~1）→ 车体速度（m/s、rad/s）→ 各轮线速度 → 转子目标转速 (rpm)
// 输出为乘 dir 之前的"逻辑方向"，正值表示该轮向前滚动
void Mecanum_Remote_Update(Chassis_Speed_t *speed, const Chassis_Command_t *command)
{
    const float k = 60.0f / (2.0f * 3.1415926f) / WHEEL_RADIUS * M3508_RATIO;   // m/s → 转子 rpm
    float vx = command->vx * VXY_MAX;                               // 向右 m/s
    float vy = command->vy * VXY_MAX;                               // 向前 m/s
    float w  = command->wz * WZ_MAX * (CHASSIS_LX + CHASSIS_LY);    // 旋转带来的轮速 m/s
    float max_abs = 0.0f;

    speed->wheel_rpm[0] = (vy + vx + w) * k;   // 前左
    speed->wheel_rpm[1] = (vy - vx + w) * k;   // 后左
    speed->wheel_rpm[2] = (vy + vx - w) * k;   // 后右
    speed->wheel_rpm[3] = (vy - vx - w) * k;   // 前右

    for (int i = 0; i < WHEEL_NUM; i++)
    {
        if (fabsf(speed->wheel_rpm[i]) > max_abs) max_abs = fabsf(speed->wheel_rpm[i]);
    }

    // 超过转速上限时整体等比例缩放，保持合成运动方向不变
    if (max_abs > RPM_MAX)
    {
        float scale = RPM_MAX / max_abs;
        for (int i = 0; i < WHEEL_NUM; i++) speed->wheel_rpm[i] *= scale;
    }
}
