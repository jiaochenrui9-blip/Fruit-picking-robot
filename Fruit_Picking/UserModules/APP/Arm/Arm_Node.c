#include "Arm_Node.h"

#include "ArmNode_Command.h"
#include "Binary_Protocol.h"
#include "usart.h"

/* 机械臂 STM32 节点（C 板 USART6 → 节点 USART2，命令见 ArmNode_Command.h）：伸缩步进 + 夹爪舵机。
   节点每收到一条命令就阻塞执行、等驱动器回包（舵机不回话时要等约 100 ms），收帧事件队列只有 4 格，
   所以只在指令变化时发，两帧之间至少隔 ARM_NODE_SEND_INTERVAL_MS。 */
#define ARM_NODE_SEND_INTERVAL_MS     50U
/* SC 拨上（extend_dir = +1）→ 节点正转，拨下 → 反转。正转是伸还是缩要上板看，反了就对调这两个。 */
#define ARM_NODE_EXTEND_UP_COMMAND    ARM_NODE_CMD_STEPPER_FORWARD
#define ARM_NODE_EXTEND_DOWN_COMMAND  ARM_NODE_CMD_STEPPER_REVERSE

/* 机械臂 STM32 节点（USART6），收发都在主循环里。调试器里看：
   arm_node_last_command 最近一次发出去的命令（ArmNode_Command.h 里的编号），arm_node_send_count 一共发出去几帧，
   arm_node_tx_status 最近一次发送的结果，arm_node_rx_count 节点发回来的合法帧数（节点现在不回帧，应该一直是 0）。 */
volatile uint8_t arm_node_last_command;
volatile uint32_t arm_node_send_count;
volatile HAL_StatusTypeDef arm_node_tx_status = HAL_OK;
volatile uint32_t arm_node_rx_count;
/* 上次发给节点的伸缩方向（0 停 / +1 / -1；节点上电不会自己转，开机当成已经停着）、夹爪（0 还没发过 / 1 打开 / 2 闭合）；
   要不要补发停止、上次补发停止以后转过没有、上一圈是不是使能、上次发送的时刻（只在主循环里用）。 */
static int8_t arm_node_extend_sent;
static uint8_t arm_node_gripper_sent;
static uint8_t arm_node_stop_pending;
static uint8_t arm_node_extend_moved;
static uint8_t arm_node_was_enabled;
static uint32_t arm_node_last_send_tick;

/* USART6 循环 DMA 接收 + 空闲中断（启动失败时协议层在主循环里重试）；发送在 Arm_Node_Update 里。 */
void Arm_Node_Init(void)
{
    BinaryProtocol_Init(&huart6);
}

/* 伸缩方向 → 节点命令。 */
static uint8_t Arm_Node_ExtendCommand(int8_t dir)
{
    if (dir > 0)
    {
        return ARM_NODE_EXTEND_UP_COMMAND;
    }
    if (dir < 0)
    {
        return ARM_NODE_EXTEND_DOWN_COMMAND;
    }
    return ARM_NODE_CMD_STEPPER_STOP;
}

/*
 * 机械臂节点（主循环，在 BareMetal_RemoteUpdate 之后由 BareMetal_MainLoop 调用）：按 Arm_Remote 算好的伸缩 / 夹爪指令给节点发命令帧。
 * 只在指令和上次发的不一样时发，一次一帧、两帧至少隔 ARM_NODE_SEND_INTERVAL_MS，步进优先；没发出去下个间隔再试。
 * 掉线 / 锁定时 Arm_Remote 把伸缩清成 0，这里就会发停止；另外使能 → 失能那一下，只要上次补发停止以后转过，
 * 就不管上次发的是什么再补发一次停止，防止之前那帧停止丢了（节点不回 ACK，C 板不知道它收没收到）。
 * 节点没有掉线超时：C 板死机或者不发了，步进会一直按最后一条命令走。
 */
void Arm_Node_Update(const Arm_NodeCommand_t *cmd, uint8_t enabled)
{
    const uint32_t now_tick = HAL_GetTick();
    BinaryProtocolFrame frame;
    int8_t extend_dir = arm_node_extend_sent;
    uint8_t command;
    HAL_StatusTypeDef status;

    /* 接收：节点现在不回帧，收到的只计数。 */
    BinaryProtocol_Task();
    while (BinaryProtocol_ReadFrame(&frame) != 0U)
    {
        arm_node_rx_count++;
    }

    if ((arm_node_was_enabled != 0U) && (enabled == 0U) && (arm_node_extend_moved != 0U))
    {
        arm_node_stop_pending = 1U;
    }
    arm_node_was_enabled = enabled;

    if ((now_tick - arm_node_last_send_tick) < ARM_NODE_SEND_INTERVAL_MS)
    {
        return;
    }

    if (arm_node_stop_pending != 0U)
    {
        extend_dir = 0;
        command = ARM_NODE_CMD_STEPPER_STOP;
    }
    else if (cmd->extend_dir != arm_node_extend_sent)
    {
        extend_dir = cmd->extend_dir;
        command = Arm_Node_ExtendCommand(extend_dir);
    }
    else if ((cmd->gripper != ARM_GRIPPER_HOLD) && (cmd->gripper != arm_node_gripper_sent))
    {
        command = (cmd->gripper == ARM_GRIPPER_OPEN) ? ARM_NODE_CMD_SERVO_OPEN : ARM_NODE_CMD_SERVO_CLOSE;
    }
    else
    {
        return;
    }

    /* 8 字节，115200 下约 0.7 ms，阻塞发送（只能在主循环里调，不能放进 TIM8）。 */
    status = BinaryProtocol_SendFrame(ARM_NODE_ADDRESS, command, NULL, 0U);
    arm_node_last_send_tick = now_tick;
    arm_node_tx_status = status;
    if (status != HAL_OK)
    {
        return;
    }

    arm_node_last_command = command;
    arm_node_send_count++;
    if ((command == ARM_NODE_CMD_SERVO_OPEN) || (command == ARM_NODE_CMD_SERVO_CLOSE))
    {
        arm_node_gripper_sent = cmd->gripper;
        return;
    }

    arm_node_extend_sent = extend_dir;
    if (extend_dir != 0)
    {
        arm_node_extend_moved = 1U;
    }
    else if (arm_node_stop_pending != 0U)
    {
        arm_node_stop_pending = 0U;
        arm_node_extend_moved = 0U;
    }
}
