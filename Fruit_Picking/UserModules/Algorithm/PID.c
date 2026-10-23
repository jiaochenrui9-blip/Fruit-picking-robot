#include "PID.h"

#include <stdbool.h>
#include <stddef.h>

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float outMin,
              float outMax,
              float integralMin,
              float integralMax)
{

    pid->initialized = 0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->last_actual = 0.0f;

    pid->error0 = 0.0f;
    pid->error1 = 0.0f;

    pid->integral = 0.0f;
    pid->out = 0.0f;

    pid->outMin = outMin;
    pid->outMax = outMax;

    pid->integralMin = integralMin;
    pid->integralMax = integralMax;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

float PID_Calculate(PID_t *pid, float actual, float dt)
{
    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    if (pid->initialized == 0U)
    {
        pid->last_actual = actual;
        pid->error0 = pid->target - actual;
        pid->error1 = pid->error0;
        pid->initialized = 1U;
    }

    /* 更新反馈 */
    pid->actual = actual;

    /* 更新误差 */
    pid->error1 = pid->error0;
    pid->error0 = pid->target - pid->actual;

    /* P */
    float P_term =
        pid->Kp * pid->error0;

    /*
     * D：对测量值微分
     * actual增加 → D为负 → 提供阻尼
     * 卸掉微分冲击
     */
    float D_term =
        -pid->Kd *
        (pid->actual - pid->last_actual) / dt;

    /*
     * 梯形积分候选值
     */
    float integral_candidate =
        pid->integral
        + 0.5f *
          (pid->error0 + pid->error1) * dt;

    /*
     * 计算候选积分输出
     */
    float I_candidate =
        pid->Ki * integral_candidate;

    /*
     * 限制I项对最终输出的贡献
     */
    if (I_candidate > pid->integralMax)
    {
        I_candidate = pid->integralMax;

        if (pid->Ki != 0.0f)
        {
            integral_candidate =
                I_candidate / pid->Ki;
        }
    }
    else if (I_candidate < pid->integralMin)
    {
        I_candidate = pid->integralMin;

        if (pid->Ki != 0.0f)
        {
            integral_candidate =
                I_candidate / pid->Ki;
        }
    }

    /*
     * 假设允许本次积分后的输出
     */
    float output_candidate =
        P_term
        + I_candidate
        + D_term;

    /*
     * 条件积分抗饱和：
     *
     * 未饱和               → 积分
     * 上饱和且误差<0      → 积分，可帮助退出饱和
     * 下饱和且误差>0      → 积分，可帮助退出饱和
     */
    if (((output_candidate <= pid->outMax) &&
         (output_candidate >= pid->outMin)) ||

        ((output_candidate > pid->outMax) &&
         (pid->error0 < 0.0f)) ||

        ((output_candidate < pid->outMin) &&
         (pid->error0 > 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    /*
     * 使用真正保存下来的积分状态
     */
    float I_term =
        pid->Ki * pid->integral;

    /*
     * 最终PID输出
     */
    pid->out =
        P_term
        + I_term
        + D_term;

    /* 输出限幅 */
    if (pid->out > pid->outMax)
    {
        pid->out = pid->outMax;
    }
    else if (pid->out < pid->outMin)
    {
        pid->out = pid->outMin;
    }

    /* 保存反馈供下一次D项使用 */
    pid->last_actual = pid->actual;

    return pid->out;
}

void PID_Reset(PID_t *pid)
{
    pid->initialized = 0U;
    pid->actual = 0.0f;
    pid->last_actual = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}
