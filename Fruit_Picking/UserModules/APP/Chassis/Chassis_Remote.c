#include "Chassis_Remote.h"

#include "elrs_crsf.h"
#include <stddef.h>

/* SB 改给升降以后（2026-10-22）没有档位开关了，底盘固定一个档，gear / speed_limit 不再变：
   2026-10-25 起固定中档（0.65，之前是低档 0.35）。 */
static Chassis_Command_t ChassisCommand =
{
    .state = CHASSIS_STATE_OFFLINE,
    .gear = CHASSIS_GEAR_MEDIUM,
    .speed_limit = CHASSIS_MEDIUM_LIMIT,
    .vx = 0.0f,
    .vy = 0.0f,
    .wz = 0.0f
};

static float Chassis_Remote_Abs(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Chassis_Remote_Clamp(float Value, float Minimum, float Maximum)
{
    if (Value < Minimum)
    {
        return Minimum;
    }
    if (Value > Maximum)
    {
        return Maximum;
    }
    return Value;
}

static float Chassis_Remote_ApplyDeadzone(float Value)
{
    float Magnitude;

    Value = Chassis_Remote_Clamp(Value,
                                 CHASSIS_COMMAND_MIN,
                                 CHASSIS_COMMAND_MAX);
    Magnitude = Chassis_Remote_Abs(Value);
    if (Magnitude <= CHASSIS_RC_DEADZONE)
    {
        return 0.0f;
    }

    /* 消除死区后重新映射，保证输出连续且摇杆推满仍为 1.0。 */
    Magnitude = (Magnitude - CHASSIS_RC_DEADZONE) /
                (CHASSIS_COMMAND_MAX - CHASSIS_RC_DEADZONE);

    return (Value >= 0.0f) ? Magnitude : -Magnitude;
}

static void Chassis_Remote_Stop(void)
{
    ChassisCommand.vx = 0.0f;
    ChassisCommand.vy = 0.0f;
    ChassisCommand.wz = 0.0f;
}

void Chassis_Remote_Update(void)
{
    uint16_t Channels[ELRS_CRSF_CHANNEL_COUNT];
    float SaValue;

    if (ELRS_CRSF_IsOnline() == 0u)
    {
        ChassisCommand.state = CHASSIS_STATE_OFFLINE;
        Chassis_Remote_Stop();
        return;
    }

    /* 一次性复制同一帧的通道，避免三个运动轴来自不同的更新时刻。 */
    ELRS_CRSF_GetChannels(Channels);
    SaValue = ELRS_CRSF_RawToNormalized(Channels[CHASSIS_SA_CHANNEL - 1u]);

    /* SA 向上接近 -1 为使能，其余位置一律锁车。 */
    if (SaValue >= -CHASSIS_SWITCH_THRESHOLD)
    {
        ChassisCommand.state = CHASSIS_STATE_LOCKED;
        Chassis_Remote_Stop();
        return;
    }

    ChassisCommand.state = CHASSIS_STATE_ENABLED;
    ChassisCommand.vx = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToNormalized(Channels[CHASSIS_VX_CHANNEL - 1u]));
    ChassisCommand.vy = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToNormalized(Channels[CHASSIS_VY_CHANNEL - 1u]));
    ChassisCommand.wz = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToNormalized(Channels[CHASSIS_WZ_CHANNEL - 1u]));

    /* 档位只改变最大幅值（现在固定中档），最终结果始终限制在 -1.0 ~ 1.0。 */
    ChassisCommand.vx = Chassis_Remote_Clamp(
        ChassisCommand.vx * ChassisCommand.speed_limit,
        CHASSIS_COMMAND_MIN, CHASSIS_COMMAND_MAX);
    ChassisCommand.vy = Chassis_Remote_Clamp(
        ChassisCommand.vy * ChassisCommand.speed_limit,
        CHASSIS_COMMAND_MIN, CHASSIS_COMMAND_MAX);
    ChassisCommand.wz = Chassis_Remote_Clamp(
        ChassisCommand.wz * ChassisCommand.speed_limit,
        CHASSIS_COMMAND_MIN, CHASSIS_COMMAND_MAX);
}

const Chassis_Command_t *Chassis_Remote_GetCommand(void)
{
    return &ChassisCommand;
}
