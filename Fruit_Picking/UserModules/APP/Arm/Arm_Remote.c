#include "Arm_Remote.h"

#include "elrs_crsf.h"

static Arm_Command_t ArmCommand =
{
    .lift_dir = 0,
    .rotate_basket = 0u
};

/* 拨杆不会自己回中：每次进入使能后，SB 要先回过一次中位才生效，防止一使能升降就动。 */
static uint8_t LiftArmed = 0u;

static Arm_NodeCommand_t NodeCommand =
{
    .extend_dir = 0,
    .gripper = ARM_GRIPPER_HOLD
};

/* 伸缩（SC）和升降一样：每次进入使能后，SC 要先回过一次中位才生效。 */
static uint8_t ExtendArmed = 0u;

/* 夹爪（左摇杆上下，不回中）：GripperZone 是杆现在在哪个区（带回差）；
   使能那一刻记下当时的区（GripperBaselineTaken / GripperZoneAtEnable），杆换到别的区以后（GripperArmed）才跟着发，
   和 SD 控 RS05 一样，防止一使能夹爪就按杆的位置动。 */
static uint8_t GripperZone = ARM_GRIPPER_HOLD;
static uint8_t GripperZoneAtEnable = ARM_GRIPPER_HOLD;
static uint8_t GripperBaselineTaken = 0u;
static uint8_t GripperArmed = 0u;

/* 三档拨杆 → 方向：拨上（读数为负）为 +1，拨下为 -1，中位为 0。 */
static int8_t Arm_Remote_SwitchToDir(float Value)
{
    if (Value < -ARM_SWITCH_THRESHOLD)
    {
        return 1;
    }
    if (Value > ARM_SWITCH_THRESHOLD)
    {
        return -1;
    }
    return 0;
}

/* 左摇杆 → 夹爪：过门限才换区，门限之间保持原来的区，杆停在门限附近也不会来回发。 */
static void Arm_Remote_UpdateGripperZone(float Value)
{
    if (Value > ARM_GRIPPER_THRESHOLD)
    {
        GripperZone = ARM_GRIPPER_POSITIVE;
    }
    else if (Value < -ARM_GRIPPER_THRESHOLD)
    {
        GripperZone = ARM_GRIPPER_NEGATIVE;
    }
}

void Arm_Remote_Update(Chassis_State_t State)
{
    uint16_t Channels[ELRS_CRSF_CHANNEL_COUNT];
    int8_t LiftDir;
    int8_t ExtendDir;

    /* 掉线、锁定：停，并且要求重新回中（上电、重连、重新拨 SA 使能都会经过这里）。 */
    if (State != CHASSIS_STATE_ENABLED)
    {
        LiftArmed = 0u;
        ArmCommand.lift_dir = 0;
        ArmCommand.rotate_basket = 0u;

        /* 伸缩停（节点那边由 BareMetal_NodeUpdate 发停止）；夹爪不发，保持原样，免得掉线时松开。 */
        ExtendArmed = 0u;
        GripperBaselineTaken = 0u;
        GripperArmed = 0u;
        NodeCommand.extend_dir = 0;
        NodeCommand.gripper = ARM_GRIPPER_HOLD;
        return;
    }

    ELRS_CRSF_GetChannels(Channels);
    LiftDir = Arm_Remote_SwitchToDir(
        ELRS_CRSF_RawToNormalized(Channels[ARM_LIFT_CHANNEL - 1u]));

    if (LiftDir == 0)
    {
        LiftArmed = 1u;
    }
    ArmCommand.lift_dir = (LiftArmed != 0u) ? LiftDir : 0;

    /* SD 两档：拨下（读数为正）到果框上方，其余到正前方。
       使能那一刻不跟着 SD 走（先停在原地，拨一下 SD 才动），这个在 BareMetal 里管，因为要看 RS05 什么时候真正使能。 */
    ArmCommand.rotate_basket =
        (ELRS_CRSF_RawToNormalized(Channels[ARM_ROTATE_CHANNEL - 1u]) > ARM_SWITCH_THRESHOLD) ? 1u : 0u;

    /* SC 三档：拨上 +1、中位 0、拨下 -1，回过中位才生效。 */
    ExtendDir = Arm_Remote_SwitchToDir(
        ELRS_CRSF_RawToNormalized(Channels[ARM_EXTEND_CHANNEL - 1u]));
    if (ExtendDir == 0)
    {
        ExtendArmed = 1u;
    }
    NodeCommand.extend_dir = (ExtendArmed != 0u) ? ExtendDir : 0;

    /* 左摇杆：先按这一帧更新区，再和使能那一刻的区比。 */
    Arm_Remote_UpdateGripperZone(
        ELRS_CRSF_RawToNormalized(Channels[ARM_GRIPPER_CHANNEL - 1u]));
    if (GripperBaselineTaken == 0u)
    {
        GripperZoneAtEnable = GripperZone;
        GripperBaselineTaken = 1u;
    }
    else if (GripperZone != GripperZoneAtEnable)
    {
        GripperArmed = 1u;
    }
    NodeCommand.gripper = (GripperArmed != 0u) ? GripperZone : ARM_GRIPPER_HOLD;
}

const Arm_Command_t *Arm_Remote_GetCommand(void)
{
    return &ArmCommand;
}

const Arm_NodeCommand_t *Arm_Remote_GetNodeCommand(void)
{
    return &NodeCommand;
}
