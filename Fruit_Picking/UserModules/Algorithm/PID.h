#ifndef PID_H
#define PID_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 位置式PID控制器。
 * 调用PID_Calculate时传入本次控制周期dt。
 */
typedef struct
{
    float Kp;            /* 比例系数。 */
    float Ki;            /* 积分系数。 */
    float Kd;            /* 微分系数。 */

    float target;        /* 目标值。 */
    float actual;        /* 本次实际值。 */
    float last_actual;

    float error0;        /* 当前误差。 */
    float error1;        /* 上一次误差。 */

    float integral;      /* 误差累计值。 */
    float out;           /* PID最终输出。 */

    float outMax;        /* 输出上限。 */
    float outMin;        /* 输出下限。 */

    float integralMax;   /* 积分累计上限。 */
    float integralMin;   /* 积分累计下限。 */

    uint8_t initialized;
} PID_t;

/* 初始化PID参数、限幅值，并清空历史状态。 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float outMin,
              float outMax,
              float integralMin,
              float integralMax);

/* 修改PID目标值。 */
void PID_SetTarget(PID_t *pid, float target);

/* 输入本次实际值，计算并返回经过限幅的PID输出。 */
float PID_Calculate(PID_t *pid, float actual, float dt);

/* 清除误差、积分和输出，PID参数及目标值保持不变。 */
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
