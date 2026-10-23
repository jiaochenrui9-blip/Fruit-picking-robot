#include "Arm_Rotate.h"

#include "RS05_app.h"
#include "can.h"

/* 旋转 RS05（CAN2，私有协议，ID 1）：CSP 位置模式，SD 拨上到正前方、拨下到果框上方。
   位置都是 zero_sta = 1 下的读数（RS05 输出轴，rad）。 */
#define ROTATE_FRONT_RAD          (-2.460f)   /* 正前方：2026-10-22 实测 3.823 − 2π */
#define ROTATE_BASKET_RAD           2.426f    /* 果框上方：实测 8.709 − 2π */
/* 能使能的位置：两个目标再往外各放这么多。不能放大：没用到的那段（约 1.4 rad）上电读数会折到另一头，
   放得越大越分不清"果框再往外"和"正前方再往里"（0.2 rad 时，要推出果框 1.2 rad 以上才会认错）。 */
#define ROTATE_RANGE_MARGIN_RAD     0.20f
#define ROTATE_SPEED_LIMIT_RAD_S    1.0f      /* CSP 限速（RS05 输出轴），先慢，走完全程约 5 s */
#define ROTATE_CURRENT_LIMIT_A      2.4f      /* CSP 限流：RS05 额定相电流 */
#define ROTATE_TARGET_PERIOD_MS     20U       /* 运行时每 20 ms 重发一次目标，防丢帧 */
#define ROTATE_TARGET_TICK           5U       /* 重发目标在每 20 ms 的第 5 拍，避开读 mechPos（第 0 拍）和参数帧（第 50 拍） */
#define ROTATE_MEASURE_PERIOD_MS   100U   /* 每隔多久读一次 mechPos */
#define ROTATE_PARAM_TICK           50U   /* 每 100 ms 的第 50 拍发参数帧（读 / 写 zero_sta），和读 mechPos 错开 */
#define ROTATE_ZERO_STA_READ_EVERY  10U   /* 判断完以后，每 10 个 100 ms 读一次 zero_sta */
#define ROTATE_ONLINE_TIMEOUT_MS   300U   /* 这么久没收到 mechPos 的回复，就当 RS05 不在线 */
#define ROTATE_REBOOT_OFFLINE_MS  1000U   /* 掉线这么久再回来，当成 RS05 重新上过电 */

/* RS05 控制状态 rotate_state */
enum
{
    ROTATE_OFF = 0,     /* 失能（复位模式），只读位置 */
    ROTATE_STARTING,    /* 正在发使能流程，一拍一帧 */
    ROTATE_ON,          /* CSP 运行中，每 20 ms 重发目标 */
    ROTATE_STOPPING     /* 发停止，发成功了回到 OFF */
};

/* 不能使能的原因 rotate_block（第一个不满足的条件） */
enum
{
    ROTATE_BLOCK_NONE = 0,   /* 可以使能 */
    ROTATE_BLOCK_REMOTE,     /* 遥控掉线或 SA 锁定 */
    ROTATE_BLOCK_OFFLINE,    /* RS05 不在线 */
    ROTATE_BLOCK_ZERO_STA,   /* 这次 RS05 上电时 zero_sta 还不是 1 */
    ROTATE_BLOCK_RANGE       /* 位置不在行程（加余量）范围里 */
};

/* CAN2 上的 RS05：现在只有旋转一台。 */
RS05_ManagerTypedef rs05_manager;
RS05_MotorTypedef rotate_motor;
/* 测量用，暂停后在调试器里看：最近一次读到的 mechPos（负载端多圈角度，rad）和一共读到了几次。 */
volatile float rotate_mech_pos_rad;
volatile uint32_t rotate_mech_pos_count;
volatile HAL_StatusTypeDef rotate_can_tx_status = HAL_OK;
/* zero_sta（0x7029，上电位置的范围：0 → 0~2π，1 → −π~π），上电后自动检查，不是 1 就写 1 并保存。
   rotate_zero_sta：最近一次读到的值。
   rotate_zero_sta_state：0 还没读到；1 这次 RS05 上电时就是 1（已生效）；
                          2 这次上电时不是 1，已自动写入保存，要把 RS05 断电重启一次才生效。
   rotate_zero_sta_write_count：一共自动写了几次（正常只有第一次；每次重启都涨，说明没存上）。 */
volatile uint8_t rotate_zero_sta;
volatile uint8_t rotate_zero_sta_state;
volatile uint32_t rotate_zero_sta_write_count;
/* RS05 控制（调试器里看）：rotate_state 0 失能 / 1 正在使能 / 2 运行中 / 3 正在停；
   rotate_block 0 可以使能 / 1 遥控没使能 / 2 RS05 不在线 / 3 zero_sta 还不是 1 / 4 位置不在行程范围；
   rotate_target_rad 现在发给 RS05 的目标。 */
volatile uint8_t rotate_state;
volatile uint8_t rotate_block;
volatile float rotate_target_rad;
/* RS05 在不在线、什么时候掉的线、这次上电读过 zero_sta 没有（只在 TIM8 里用）。 */
static uint8_t rotate_was_online;
static uint32_t rotate_offline_since;
static uint8_t rotate_zero_sta_read_sent;

HAL_StatusTypeDef Arm_Rotate_Init(void)
{
    RS05_Manager_Init(&rs05_manager, &hcan2, RS05_MASTER_ID);

    /* 电机 ID 1（用户确认过），注册前要先填好。 */
    rotate_motor.motor_id = RS05_MOTOR_ID;
    if (RS05_Manager_RegisterMotor(&rs05_manager, &rotate_motor) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配 CAN2 的过滤器组 15（只收扩展帧）；CAN2 已经在升降那边启动过，这里只会再开一次接收中断。 */
    if (RS05_Manager_Start(&rs05_manager) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 发一次停止（类型 4），保证它处在失能（复位）状态，比如 C 板复位时它还在跑。
       和 C 板一起上电时 RS05 还没启动好（实测开机约 1.4 s 内不回话），这帧多半没回复；判断在线看 rotate_mech_pos_count。
       发不出去也不影响底盘和升降，所以不当成初始化失败。 */
    (void)RS05_Stop(rs05_manager.hcan, rs05_manager.master_id, rotate_motor.motor_id);
    return HAL_OK;
}

/* RS05 在线：最近 ROTATE_ONLINE_TIMEOUT_MS 内回过 mechPos。 */
static uint8_t Arm_Rotate_Online(uint32_t now_tick)
{
    const RS05_ParameterCache *mech_pos =
        RS05_FindParameterCache(&rotate_motor, RS05_PARAMETER_MECH_POS);

    return ((mech_pos != NULL) &&
            ((now_tick - mech_pos->last_update_tick) < ROTATE_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
}

/*
 * 每拍最先做：跟踪 RS05 在不在线。掉线 1 s 以上又回来，当成它重新上过电，这次上电的 zero_sta 要重新读。
 * 必须在控制之前做：不然 RS05 一回来，控制还拿着上一次上电的 zero_sta 结论就去使能了。
 */
static void Arm_Rotate_TrackOnline(uint32_t now_tick)
{
    if (Arm_Rotate_Online(now_tick) == 0U)
    {
        if (rotate_was_online != 0U)
        {
            rotate_was_online = 0U;
            rotate_offline_since = now_tick;
        }
        return;
    }
    if (rotate_was_online == 0U)
    {
        rotate_was_online = 1U;
        if ((now_tick - rotate_offline_since) >= ROTATE_REBOOT_OFFLINE_MS)
        {
            rotate_zero_sta_state = 0U;
            rotate_zero_sta_read_sent = 0U;
        }
    }
}

/*
 * 参数帧（每 100 ms 的第 50 拍，最多一帧）：zero_sta 自动检查。
 *   RS05 上线后（或者掉线 1 s 以上又回来，当成它重新上过电）先读 zero_sta，读到的第一个值
 *   就是这次上电用的范围 → rotate_zero_sta_state 1（是 1）或 2（不是 1）。
 *   任何时候读到不是 1：下一个参数时刻写 1（类型 18），再下一个保存（类型 22）。写完 RAM 里就是 1，
 *   再读不会重复写；但这次上电已经按旧范围算好了位置，要 RS05 断电重启一次才生效。
 *   判断完以后每 1 s 读一次，rotate_zero_sta 显示最新的值。
 */
static void Arm_Rotate_Param(void)
{
    static uint32_t round_count;
    static uint32_t last_reply_tick;
    static uint8_t write_step;   /* 0 不用写，1 该发写入，2 该发保存 */
    const RS05_ParameterCache *zero_sta;

    /* 1. 先处理上一次读 zero_sta 的回复（掉线时也处理，免得漏掉）。 */
    zero_sta = RS05_FindParameterCache(&rotate_motor, RS05_PARAMETER_ZERO_STA);
    if ((zero_sta != NULL) && (zero_sta->last_update_tick != last_reply_tick))
    {
        last_reply_tick = zero_sta->last_update_tick;
        rotate_zero_sta = zero_sta->value.u8;
        if ((rotate_zero_sta_state == 0U) && (rotate_zero_sta_read_sent != 0U))
        {
            rotate_zero_sta_state = (rotate_zero_sta == 1U) ? 1U : 2U;
        }
        if ((rotate_zero_sta != 1U) && (write_step == 0U))
        {
            write_step = 1U;
        }
    }

    /* 2. 不在线就什么都不发（在线 / 重启在 Arm_Rotate_TrackOnline 里每拍跟踪）。 */
    if (rotate_was_online == 0U)
    {
        return;
    }

    /* 3. 写入、保存优先；发成功了才往下走，邮箱满就下一轮再发。 */
    if (write_step == 1U)
    {
        rotate_can_tx_status = RS05_WriteParameterU8(&rs05_manager,
                                                     rotate_motor.motor_id,
                                                     RS05_PARAMETER_ZERO_STA, 1U);
        if (rotate_can_tx_status == HAL_OK)
        {
            write_step = 2U;
        }
        return;
    }
    if (write_step == 2U)
    {
        rotate_can_tx_status = RS05_SaveParameters(&rs05_manager,
                                                   rotate_motor.motor_id);
        if (rotate_can_tx_status == HAL_OK)
        {
            write_step = 0U;
            rotate_zero_sta_write_count++;
        }
        return;
    }

    /* 4. 读：还没判断出这次上电的值时每 100 ms 读一次，判断完以后每 1 s 读一次。 */
    if ((rotate_zero_sta_state == 0U) || (round_count == 0U))
    {
        rotate_can_tx_status = RS05_Command_ParaRead(&rs05_manager,
                                                     rotate_motor.motor_id,
                                                     RS05_PARAMETER_ZERO_STA);
        if (rotate_can_tx_status == HAL_OK)
        {
            rotate_zero_sta_read_sent = 1U;
        }
    }
    round_count++;
    if (round_count >= ROTATE_ZERO_STA_READ_EVERY)
    {
        round_count = 0U;
    }
}

/* 最新的 mechPos：直接取参数缓存（CAN 接收中断一收到回复就更新），没读到过返回 0。 */
static uint8_t Arm_Rotate_MechPos(float *pos)
{
    const RS05_ParameterCache *mech_pos =
        RS05_FindParameterCache(&rotate_motor, RS05_PARAMETER_MECH_POS);

    if (mech_pos == NULL)
    {
        return 0U;
    }
    *pos = mech_pos->value.f32;
    return 1U;
}

/* 现在能不能让 RS05 使能：能返回 ROTATE_BLOCK_NONE，不能返回第一个不满足的条件。 */
static uint8_t Arm_Rotate_Block(uint32_t now_tick, uint8_t enabled)
{
    float pos;

    if (enabled == 0U)
    {
        return ROTATE_BLOCK_REMOTE;
    }
    if ((Arm_Rotate_Online(now_tick) == 0U) || (Arm_Rotate_MechPos(&pos) == 0U))
    {
        return ROTATE_BLOCK_OFFLINE;
    }
    /* 这次 RS05 上电时 zero_sta 不是 1：读数和两个目标不在同一个范围里。 */
    if (rotate_zero_sta_state != 1U)
    {
        return ROTATE_BLOCK_ZERO_STA;
    }
    /* 位置不在行程（加余量）里就不敢动：比如写完 zero_sta 只重刷了 C 板、RS05 没断过电，用的还是旧范围。 */
    if ((pos < (ROTATE_FRONT_RAD - ROTATE_RANGE_MARGIN_RAD)) ||
        (pos > (ROTATE_BASKET_RAD + ROTATE_RANGE_MARGIN_RAD)))
    {
        return ROTATE_BLOCK_RANGE;
    }
    return ROTATE_BLOCK_NONE;
}

/*
 * RS05 控制（每拍最多发一帧，发了返回 1）：
 *   OFF → 可以使能时：记下当前位置，发使能流程，一拍一帧，发不出去下一拍重发：
 *     停止 → run_mode = 5（CSP）→ 限流 → 限速 → 目标 → 使能 → 限流 → 限速 → 目标（使能前写的参数万一没收下，使能后再写一遍）
 *   → ON：每 20 ms 重发一次目标。
 *   不能使能了（遥控锁定 / 掉线、RS05 掉线、位置不对）→ 发停止 → OFF，失能后机械臂可以手推。
 * 目标：使能后先停在原地（使能那一刻的位置），SD 拨一下（和使能那一刻不一样了）以后才按 SD：
 *   拨上正前方，拨下果框上方。这样上电、重连、重新拨 SA 使能时机械臂都不会自己转。
 */
static uint8_t Arm_Rotate_Control(uint32_t now_tick, uint32_t tick,
                                       uint8_t rotate_basket, uint8_t enabled)
{
    static uint8_t step;
    static float hold_rad;
    static uint8_t start_basket;
    static uint8_t following;
    const uint8_t block = Arm_Rotate_Block(now_tick, enabled);
    const uint8_t id = rotate_motor.motor_id;
    HAL_StatusTypeDef status;

    rotate_block = block;

    if ((block != ROTATE_BLOCK_NONE) &&
        ((rotate_state == ROTATE_STARTING) || (rotate_state == ROTATE_ON)))
    {
        rotate_state = ROTATE_STOPPING;
    }
    if ((block == ROTATE_BLOCK_NONE) && (rotate_state == ROTATE_OFF))
    {
        (void)Arm_Rotate_MechPos(&hold_rad);
        start_basket = rotate_basket;
        following = 0U;
        step = 0U;
        rotate_state = ROTATE_STARTING;
    }

    if ((rotate_state == ROTATE_STARTING) || (rotate_state == ROTATE_ON))
    {
        if ((following == 0U) && (rotate_basket != start_basket))
        {
            following = 1U;
        }
        if (following == 0U)
        {
            rotate_target_rad = hold_rad;
        }
        else
        {
            rotate_target_rad = (rotate_basket != 0U) ? ROTATE_BASKET_RAD : ROTATE_FRONT_RAD;
        }
    }

    switch (rotate_state)
    {
        case ROTATE_STOPPING:
            status = RS05_Stop(rs05_manager.hcan, rs05_manager.master_id, id);
            rotate_can_tx_status = status;
            if (status == HAL_OK)
            {
                rotate_state = ROTATE_OFF;
            }
            return 1U;

        case ROTATE_STARTING:
            switch (step)
            {
                case 0U:   /* 换模式前要先停 */
                    status = RS05_Stop(rs05_manager.hcan, rs05_manager.master_id, id);
                    break;
                case 1U:
                    status = RS05_WriteParameterU8(&rs05_manager, id, RS05_PARAMETER_RUN_MODE,
                                                   RS05_RUN_MODE_POSITION_CSP);
                    break;
                case 2U:
                case 6U:
                    status = RS05_WriteParameterFloat(&rs05_manager, id, RS05_PARAMETER_CURRENT_LIMIT,
                                                      ROTATE_CURRENT_LIMIT_A);
                    break;
                case 3U:
                case 7U:
                    status = RS05_WriteParameterFloat(&rs05_manager, id, RS05_PARAMETER_CSP_SPEED_LIMIT,
                                                      ROTATE_SPEED_LIMIT_RAD_S);
                    break;
                case 5U:
                    status = RS05_Enable(rs05_manager.hcan, rs05_manager.master_id, id);
                    break;
                default:   /* 4、8：目标 */
                    status = RS05_WriteParameterFloat(&rs05_manager, id, RS05_PARAMETER_POSITION_REF,
                                                      rotate_target_rad);
                    break;
            }
            rotate_can_tx_status = status;
            if (status == HAL_OK)
            {
                step++;
                if (step > 8U)
                {
                    rotate_state = ROTATE_ON;
                }
            }
            return 1U;

        case ROTATE_ON:
            if ((tick % ROTATE_TARGET_PERIOD_MS) != ROTATE_TARGET_TICK)
            {
                return 0U;
            }
            rotate_can_tx_status = RS05_WriteParameterFloat(&rs05_manager, id, RS05_PARAMETER_POSITION_REF,
                                                            rotate_target_rad);
            return 1U;

        case ROTATE_OFF:
        default:
            return 0U;
    }
}

/*
 * 旋转 RS05（TIM8，每拍最多一帧 RS05 的帧）：先看控制要不要发（使能流程 / 停止 / 每 20 ms 重发目标），
 * 不发的话第 50 拍发参数帧（zero_sta 自动检查），第 0 拍读 mechPos（在不在线也靠它判断）。
 * 都放在 TIM8 里发，不放主循环：CAN2 的帧都从这里发，主循环和中断同时调
 * HAL_CAN_AddTxMessage 会选中同一个空邮箱，主循环那一帧就丢了。
 */
void Arm_Rotate_Update(uint8_t rotate_basket, uint8_t enabled)
{
    static uint32_t tick;
    static uint32_t last_reply_tick;
    const uint32_t now_tick = HAL_GetTick();
    const RS05_ParameterCache *mech_pos;

    tick++;
    if (tick >= ROTATE_MEASURE_PERIOD_MS)
    {
        tick = 0U;
    }

    Arm_Rotate_TrackOnline(now_tick);
    if (Arm_Rotate_Control(now_tick, tick, rotate_basket, enabled) != 0U)
    {
        return;
    }
    if (tick == ROTATE_PARAM_TICK)
    {
        Arm_Rotate_Param();
        return;
    }
    if (tick != 0U)
    {
        return;
    }

    /* 先取上一次请求的回复（CAN 接收中断写进参数缓存），再发下一次请求。 */
    mech_pos = RS05_FindParameterCache(&rotate_motor, RS05_PARAMETER_MECH_POS);
    if ((mech_pos != NULL) && (mech_pos->last_update_tick != last_reply_tick))
    {
        last_reply_tick = mech_pos->last_update_tick;
        rotate_mech_pos_rad = mech_pos->value.f32;
        rotate_mech_pos_count++;
    }

    rotate_can_tx_status = RS05_Command_ParaRead(&rs05_manager,
                                                 rotate_motor.motor_id,
                                                 RS05_PARAMETER_MECH_POS);
}
