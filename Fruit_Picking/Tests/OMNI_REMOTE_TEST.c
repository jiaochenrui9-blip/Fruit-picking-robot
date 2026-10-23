/* 在工程根目录用主机 gcc 编译运行，不需要板子：
 * gcc -std=c11 -I UserModules/APP/Chassis Tests/OMNI_REMOTE_TEST.c -lm -o omni_test
 */
#include "../UserModules/APP/Chassis/Omni_Remote.c"   // 直接包含 .c，测试里能用它的参数宏

#include <assert.h>
#include <stdio.h>

#define K (60.0f / (2.0f * 3.1415926f) / WHEEL_RADIUS * M3508_RATIO)   // 1 m/s 对应的转子 rpm

static Chassis_Command_t Remote;   // 假指令

static void Check(float vx, float vy, float wz, const float expect[WHEEL_NUM])
{
    Chassis_Speed_t speed;

    Remote.vx = vx;
    Remote.vy = vy;
    Remote.wz = wz;
    Omni_Remote_Update(&speed, &Remote);
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        assert(fabsf(speed.wheel_rpm[i] - expect[i]) < 0.01f);
    }
}

int main(void)
{
    const float s = 0.70710678f * 0.1f * VXY_MAX * K;   // 摇杆 0.1 平移，投影到 45° 方向
    const float w = CHASSIS_R * 0.1f * WZ_MAX * K;      // 摇杆 0.1 旋转

    // ID1~4 顺序：右前、右后、左后、左前；逻辑正值沿顺时针切向。
    Check(0, 0.1f, 0, (float[]){-s, -s,  s,  s});         // 前进
    Check(0, -0.1f, 0, (float[]){ s,  s, -s, -s});        // 后退
    Check(0.1f, 0, 0, (float[]){ s, -s, -s,  s});         // 右移
    Check(-0.1f, 0, 0, (float[]){-s,  s,  s, -s});        // 左移
    Check(0, 0, 0.1f, (float[]){ w,  w,  w,  w});         // 顺时针
    Check(0, 0, -0.1f, (float[]){-w, -w, -w, -w});        // 逆时针
    Check(0.1f, 0.1f, 0.1f,
          (float[]){w, w - 2*s, w, w + 2*s});            // 右前平移叠加顺时针
    Check(0, 0, 0,    (float[]){ 0,  0,  0,  0});
    Check(10, 10, 0,  (float[]){0, -RPM_MAX, 0, RPM_MAX}); // 右前：右后/左前驱动，超过上限等比例缩小
    puts("PASS: omni");
    return 0;
}
