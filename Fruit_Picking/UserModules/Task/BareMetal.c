#include "BareMetal.h"

#include "Arm_Node.h"
#include "Arm_Remote.h"
#include "ELRS_DMA.h"
#include "elrs_crsf.h"
#include "tim.h"
#include "usart.h"

/* 主循环发布、TIM8 读取的底盘指令：主循环只在关中断时整条写入。 */
Chassis_Command_t chassis_command;
/* 主循环发布、TIM8 读取的机械臂指令：和 chassis_command 在同一次关中断里整条写入。 */
Arm_Command_t arm_command;

/*
 * 主循环每圈都跑：解析 ELRS → 遥控逻辑 → 发布指令。
 * ELRS_DMA_Process 没有新数据时立刻返回；串口出错后的重启也在这里做，失败了下一圈接着试。
 * Chassis_Remote_Update 每圈都判一次在线，掉线马上就能反映到指令里。
 */
static void BareMetal_RemoteUpdate(void)
{
    uint32_t primask;

    ELRS_DMA_Process();
    Chassis_Remote_Update();
    /* 整车使能（在线 + SA）由 Chassis_Remote 算，机械臂拨杆只在使能时才算数。 */
    Arm_Remote_Update(Chassis_Remote_GetCommand()->state);

    /* 整条指令一次性发布给中断：Chassis_Remote_Update 是逐字段写的，
       中断直接读它那份会读到半新半旧的一条（新的 vx 配旧的 vy）。
       机械臂指令放在同一次关中断里，中断不会看到新的使能状态配上旧的升降方向。 */
    primask = __get_PRIMASK();
    __disable_irq();
    chassis_command = *Chassis_Remote_GetCommand();
    arm_command = *Arm_Remote_GetCommand();
    __set_PRIMASK(primask);
}

/*
 * 遥控在线 + SA 在使能位：底盘、升降、旋转都按这一个条件。
 * 在线要在这里直接判：主循环一旦卡住，发布的指令会停在最后一次，
 * 而 ELRS 的时间戳也不再更新，这里照样能判出掉线。
 */
static uint8_t BareMetal_IsEnabled(void)
{
    return ((ELRS_CRSF_IsOnline() != 0U) &&
            (chassis_command.state == CHASSIS_STATE_ENABLED)) ? 1U : 0U;
}

HAL_StatusTypeDef BareMetal_Init(void)
{
    if (Chassis_Control_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (Arm_Lift_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (Arm_Rotate_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ELRS：USART1 DMA 循环接收 + 空闲中断。 */
    ELRS_CRSF_Init();
    if (ELRS_DMA_Start(&huart1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 机械臂节点：USART6 循环 DMA 接收 + 空闲中断（启动失败时协议层在主循环里重试）；发送在主循环。
       开机不给节点发任何命令，第一帧要等遥控使能后拨了 SC 或左摇杆。 */
    Arm_Node_Init();

    /* 全部就绪后才放开 1kHz 控制节拍。 */
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    return HAL_TIM_Base_Start_IT(&htim8);
}

/* TIM8 1kHz 中断里只做电机控制：底盘（解算 → 速度环 → CAN1），升降（速度环 → CAN2），
   旋转（判能不能使能 → CSP 目标 → CAN2）。
   遥控在线 + 使能这里判一次，三个机构用同一个结果：TIM8 执行期间 uwTick 不会变（SysTick 优先级更低，打断不了它），
   发布的指令也不会变（主循环跑不了），和以前每个机构各判一次结果一样。 */
void BareMetal_ControlTick(void)
{
    const uint8_t enabled = BareMetal_IsEnabled();

    Chassis_Control_Update(&chassis_command, enabled);
    Arm_Lift_Update(arm_command.lift_dir, enabled);
    Arm_Rotate_Update(arm_command.rotate_basket, enabled);
}

/* 主循环：ELRS 解析和遥控逻辑、给机械臂节点发命令，不参与 1kHz 控制，随便被中断打断。 */
void BareMetal_MainLoop(void)
{
    BareMetal_RemoteUpdate();
    /* 节点用主循环这份整车状态（Chassis_Remote 刚按在线 + SA 算好的）。 */
    Arm_Node_Update(Arm_Remote_GetNodeCommand(),
                    (Chassis_Remote_GetCommand()->state == CHASSIS_STATE_ENABLED) ? 1U : 0U);
}
