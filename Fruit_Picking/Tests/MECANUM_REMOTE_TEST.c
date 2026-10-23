/* 在工程根目录用主机 gcc 编译运行，不需要板子：
 * gcc -std=c11 -I UserModules/APP/Chassis Tests/MECANUM_REMOTE_TEST.c -lm -o mecanum_test
 */
#include "../UserModules/APP/Chassis/Mecanum_Remote.c"   // 直接包含 .c，测试里能用它的参数宏

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
    Mecanum_Remote_Update(&speed, &Remote);
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        assert(fabsf(speed.wheel_rpm[i] - expect[i]) < 0.01f);
    }
}

int main(void)
{
    const float v = 0.1f * VXY_MAX * K;                                // 摇杆 0.1 平移
    const float w = 0.1f * WZ_MAX * (CHASSIS_LX + CHASSIS_LY) * K;     // 摇杆 0.1 旋转

    // 顺序：前左、后左、后右、前右
    Check(0, 0.1f, 0, (float[]){ v,  v,  v,  v});            // 前进：四轮都向前
    Check(0.1f, 0, 0, (float[]){ v, -v,  v, -v});            // 右移
    Check(0, 0, 0.1f, (float[]){ w,  w, -w, -w});            // 顺时针：左轮向前、右轮向后
    Check(0, 0, 0,    (float[]){ 0,  0,  0,  0});
    Check(10, 10, 0,  (float[]){RPM_MAX, 0, RPM_MAX, 0});    // 超过 RPM_MAX：等比例缩小
    puts("PASS: mecanum");
    return 0;
}
