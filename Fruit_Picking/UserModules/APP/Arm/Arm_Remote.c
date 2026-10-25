#include "Arm_Remote.h"

#include "elrs_crsf.h"

static Arm_Command_t ArmCommand =
{
    .lift_dir = 0,
    .rotate_dir = 0
};

/* 拨杆不会自己回中：每次进入使能后，SB 要先回过一次中位才生效，防止一使能升降就动。 */
static uint8_t LiftArmed = 0u;

static Arm_NodeCommand_t NodeCommand =
{
    .extend_dir = 0,
    .gripper = ARM_GRIPPER_HOLD,
    .basket = ARM_BASKET_HOLD
};

/* 步进（左摇杆上下，不回中）：ExtendZone 是杆现在在哪个区（+1 / 0 / -1，带回差）；
   和升降一样，每次进入使能后杆要先回过一次中间（ExtendZone == 0）才生效。 */
static uint8_t ExtendArmed = 0u;
static int8_t ExtendZone = 0;

/* 夹爪（SD）：GripperZone 是 SD 现在在哪个区；使能那一刻记下当时的区（GripperBaselineTaken / GripperZoneAtEnable），
   SD 拨到另一边以后（GripperArmed）才跟着发，防止一使能夹爪就按 SD 的位置动（合上还会带着剪线）。 */
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

/* 左摇杆 → 步进的区：过了走的门限才进 +1 / -1，回到停的门限以内才回 0，两个门限之间保持原来的区。 */
static void Arm_Remote_UpdateExtendZone(float Value)
{
    if (Value > ARM_EXTEND_MOVE_THRESHOLD)
    {
        ExtendZone = 1;
    }
    else if (Value < -ARM_EXTEND_MOVE_THRESHOLD)
    {
        ExtendZone = -1;
    }
    else if ((Value < ARM_EXTEND_STOP_THRESHOLD) && (Value > -ARM_EXTEND_STOP_THRESHOLD))
    {
        ExtendZone = 0;
    }
}

/* SD → 夹爪：过门限才换区，门限之间保持原来的区。 */
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

    /* 果篮跟着 SA：使能关、锁定开；掉线时不发、保持原样（免得断链把果子倒出来）。
       要放在下面"不使能就清零返回"之前：锁定时果篮也要动。 */
    if (State == CHASSIS_STATE_ENABLED)
    {
        NodeCommand.basket = ARM_BASKET_CLOSE;
    }
    else if (State == CHASSIS_STATE_LOCKED)
    {
        NodeCommand.basket = ARM_BASKET_OPEN;
    }
    else
    {
        NodeCommand.basket = ARM_BASKET_HOLD;
    }

    /* 掉线、锁定：停，并且要求重新回中（上电、重连、重新拨 SA 使能都会经过这里）。 */
    if (State != CHASSIS_STATE_ENABLED)
    {
        LiftArmed = 0u;
        ArmCommand.lift_dir = 0;
        ArmCommand.rotate_dir = 0;

        /* 步进停、剪线停（节点那边由 Arm_Node_Update 发停止）；夹爪不发，保持原样，免得掉线时松开。 */
        ExtendArmed = 0u;
        ExtendZone = 0;
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

    /* SC 三档：往两端走 / 停。使能那一刻不跟着 SC 走（先停在原地，SC 回过中位才动），这个在 Arm_Rotate 里管，
       因为要看 RS05 什么时候真正使能（它可能比遥控晚，也可能重启以后重新使能）。 */
    ArmCommand.rotate_dir = (int8_t)(ARM_ROTATE_SC_UP_DIR *
        Arm_Remote_SwitchToDir(ELRS_CRSF_RawToNormalized(Channels[ARM_ROTATE_CHANNEL - 1u])));

    /* 左摇杆：先按这一帧更新区，回过中间才生效。 */
    Arm_Remote_UpdateExtendZone(
        ELRS_CRSF_RawToNormalized(Channels[ARM_EXTEND_CHANNEL - 1u]));
    if (ExtendZone == 0)
    {
        ExtendArmed = 1u;
    }
    NodeCommand.extend_dir = (ExtendArmed != 0u) ? ExtendZone : 0;

    /* SD：先按这一帧更新区，再和使能那一刻的区比。 */
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
