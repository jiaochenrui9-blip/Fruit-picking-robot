# 摘果机器人固件

主控：RoboMaster C 型开发板（STM32F407），CMake + ARM GCC。

## 当前：ELRS 遥控控制底盘（全向轮，速度环）

- 底盘：CAN1，1 Mbps；M3508 + C620，电调 ID 1~4 从右前开始顺时针 = 右前、右后、左后、左前（`chassis_motors[0..3]`，反馈 `0x201..0x204`，四台合并发送一帧 `0x200`）。
- 全向轮驱动方向角按 ID1~4 为 315°、225°、135°、45°（从右边逆时针量）；逻辑正轮速沿顺时针切向。轮子位置角与驱动方向角相差 90°，详见 `UserModules/APP/Chassis/Omni_Remote.md`。麦轮函数仍保留原编号，切换底盘类型时需匹配电调编号。
- 遥控：ELRS 接收机 CRSF 输出接 USART1（4-pin 口，420000 8N1），DMA 循环接收 + 空闲中断。
- 结构沿用 `C:\Project\Chassis_BareMetal`：调度在 `UserModules/Task/BareMetal.c`（2026-10-23 起只做调度；各机构拆到 `APP/Chassis/Chassis_Control`、`APP/Arm/Arm_Lift`、`Arm_Rotate`、`Arm_Node`，机构不读遥控，指令和"是否使能"由 BareMetal 当参数传进去）。`main.c` 只调 `BareMetal_Init()`、`BareMetal_MainLoop()`，HAL 回调只做转发：
  - USART1 空闲中断 / DMA 满一圈：`ELRS_DMA_UpdateSize` 只记 DMA 写到了哪；串口出错：`ELRS_DMA_HandleError` 只做标记。
  - 主循环（`BareMetal_MainLoop`）：`ELRS_DMA_Process()` 解析 CRSF → `Chassis_Remote_Update()`（在线判断 + SA 使能 + 摇杆死区，固定中档）→ `Arm_Remote_Update(state)`（2026-10-24 起：SB → 升降方向，SC → 旋转方向，左摇杆上下 → 步进方向，SD → 夹爪，SA（整车状态）→ 果篮）→ 关中断，把 `chassis_command`、`arm_command` 一起整条拷贝（发布）→ `Arm_Node_Update()`：步进 / 夹爪 / 剪线 / 果篮指令变了就经 USART6 给机械臂 STM32 节点发命令帧（见下面"USART6：机械臂 STM32 节点"）。
  - TIM8 1 kHz（`BareMetal_ControlTick`）：先判一次遥控在线（`ELRS_CRSF_IsOnline()`）且 `chassis_command.state == ENABLED`（`BareMetal_IsEnabled`），结果当参数传给三个机构，按顺序调：`Chassis_Control_Update`（全向轮解算 `Omni_Remote_Update` → 速度环 → CAN1 一帧 `0x200`）、`Arm_Lift_Update`（升降速度环 → CAN2 `0x200`，见下面"CAN2：升降 3508"）、`Arm_Rotate_Update`（RS05，见下面"CAN2：旋转 RS05"）；不使能时都失能。
  - 中断只读发布出来的 `chassis_command` / `arm_command`，不碰遥控模块自己那份（`Chassis_Remote` 那份是逐字段写的）。在线在中断里直接判，主循环卡住、发布的指令停在最后一次时照样能停。
- 状态：
  - 上电后还没收到帧，或超过 `ELRS_CRSF_ONLINE_TIMEOUT_MS`（现在 500 ms）没收到有效通道帧 → OFFLINE；CH5 不在使能位 → LOCKED。这两种情况四个电机都失能（电流 0）。
  - CH5 拨到使能位 → ENABLED，电机切到速度模式。只在模式变化时调 `MotorControl_SetMode`（它会清 PID 和目标）。
  - 没做"重新上线要先拨锁定再拨使能"：摇杆会回中，开关在使能位时上线底盘也不会自己跑。不回中的拨杆 / 摇杆单独做了保护：升降的 SB、步进的左摇杆上下"回过中位才生效"，旋转的 SC"这次 RS05 使能后回过中位才动"，夹爪的 SD"使能后拨一次才动"，见下面各节。
- 通道（遥控器 BETAFPV，SA / SD 两档，SB / SC 三档，拨上读数为负）：CH2 = wz，CH3 = vy，CH4 = vx，CH5 = SA 使能 + 果篮（拨上使能 + 关果篮 / 拨下锁定 + 开果篮，掉线时果篮不动），CH6 = SB 升降（拨上 / 中 / 下 = 往上 / 停 / 往下），CH7 = SC 旋转 RS05（拨上 / 中 / 下 = 往正前方 / 停 / 往果框上方，左右反了改 `ARM_ROTATE_SC_UP_DIR`），CH8 = SD 夹爪（拨上合，合到位后剪线一直往复 / 拨下开，剪线先回张开位），CH1 = 左摇杆上下 → 步进（推上前进 / 中间停 / 拉下后退；10-23 读板确认 CH1 是不回中的左摇杆上下，拉下为负）。这套映射是 2026-10-24 按机械那边的要求改的。
- 底盘速度固定中档（`speed_limit` = 0.65，2026-10-25 起；之前是低档 0.35）：SB 改给升降以后（2026-10-22），没有档位开关了。
- 上坡（2026-10-25，用户实测坡上不去）：
  - 电流指令限幅 `CHASSIS_CURRENT_LIMIT` 3000 → 8000（协议数值，±16384 ↔ ±20 A；8000 ≈ 9.8 A，接近 M3508 手册的额定电流 10 A / 额定（最大连续）转矩 3 N·m；原来 3000 ≈ 3.7 A，约 1.1 N·m）。C620 最大持续电流 20 A。
  - 速度环积分项上限：驱动默认 `SPEED_PID_I_OUTPUT_LIMIT` = 1000（≈ 1.2 A，升降还在用，没动），底盘用新加的 `MotorControl_SetSpeedIntegralLimit` 单独放到 `CHASSIS_SPEED_I_LIMIT` = 6000（≈ 7.3 A）。坡上要一直出力，原来积分最多补 1.2 A，剩下全靠比例项，转速会被拖下来。
  - 指令加速度限制 `CHASSIS_COMMAND_RAMP_PER_S` = 1.0（`Chassis_Control.c`，本次发布参数）：TIM8 里 vx / vy / wz 每拍最多变 0.001，三个量按同一比例缩，方向不变；0 → 中档推满约 0.65 s。限幅放大以后不加这个，平地起步 / 刹车的电流会是原来的两倍左右；嫌肉就把这个数改大。锁车 / 掉线照旧马上失能（不出力，坡上会溜）。
  - 还是上不去的话：先看坡上 `chassis_motors[i].output_current` 是不是顶到 ±8000（顶到了 = 电流不够，可以再往上放，M3508 堵转扭矩 4.5 N·m ≈ 15 A ≈ 12288，长时间不要超过额定）；没顶到但轮子在空转 = 打滑，软件帮不了。
- 解算参数（`UserModules/APP/Chassis/Omni_Remote.c`）还是旧底盘的实测值：轮心到中心 0.20 m、轮半径 0.076 m、平移最大 1 m/s、旋转最大 3 rad/s、转子转速上限 8000 rpm，要按新底盘改。
- 速度环 PID 在 `Chassis_Control.c` 的 `chassis_speed_pid` 表里，沿用试调值：ID1/3/4 Kp=5、ID2 Kp=8，Ki 都是 20，Kd=0；电流指令限幅 `CHASSIS_CURRENT_LIMIT` ±3000（协议数值，非安培）。
- 沿用驱动的反馈超时保护：未收到反馈或反馈超过 100 ms 未更新，输出电流清零。

### 上板测试

1. 车架空。上电后先不开遥控器：`chassis_command.state` 应为 OFFLINE（0），四个电机的 `mode` 为 `MOTOR_MODE_DISABLE`。
2. 打开遥控器，暂停看：`crsf_valid_frame_count` 在涨（每秒涨的数接近包率），`crsf_crc_error_count` 基本不动，`elrs_channels[]` 摇杆 172~1811、中位 992；挨个拨开关核对通道。这几个都是 `elrs_crsf.c` 里的 static 变量，Watches 里看不到就写 `'elrs_crsf.c'::elrs_channels`。
3. SB 放中位，把 SA（CH5）拨到使能：state 变 ENABLED，电机切到速度模式。右摇杆推满向前时，`chassis_motors[0..3].speed.target_rpm` 按 ID1~4 应为右前 −、右后 −、左后 +、左前 +（固定中档，约 0.33 s 内涨到约 ±1097 转子 rpm，拨 SB 不影响）；右移应为 `+ − − +`，顺时针应为 `+ + + +`。先核对电调编号和每轮实际驱动方向，再验证通道方向。
4. 关掉遥控器：超过 500 ms 后电机失能，state 变 OFFLINE。
5. 暂停时单片机不发 CAN 帧，C620 收不到指令时会怎样手册没写：测试时车要架空，暂停尽量短。

### 调试变量地址（VOFA）

按 2026-10-22 BareMetal 版 Debug 构建出的 ELF。全局变量从 `main.c` 挪进了 `BareMetal.c`，地址和之前不一样了；后续重新编译仍应以当次 ELF 为准。2026-10-22 加了 CAN2 升降、加了 `Arm_Remote` 以后都重新核对过，下面这几个地址没变；2026-10-23 加机械臂节点以后也核对过，没变。2026-10-23 把 `BareMetal.c` 按机构拆开以后又核对过：四台电机的地址没变（`Chassis_Control.c` 在 CMakeLists 里排在原来 `BareMetal.c` 的位置），只有 `chassis_command` 挪了。2026-10-24 遥控映射改版以后再核对：四台电机没变，`chassis_command` 往后挪了 12 字节（`Arm_Node.c` 多了几个变量）。2026-10-25 底盘加了加速度限制（`Chassis_Control.c` 多了一个 20 字节的变量）：四台电机还是没变，**它们以外的地址（本节的 `chassis_command`，以及升降、RS05、节点三节）都往后挪了 20 字节（0x14）**，下面已按新 ELF 用 gdb 重算。升降的地址见下一节。

- 四台 `speed_rpm`（`int16_t`，实际转子转速）：`0x2000042E`、`0x200004EE`、`0x200005AE`、`0x2000066E`
- 四台 `speed.target_rpm`（`float`，目标转子转速）：`0x2000043C`、`0x200004FC`、`0x200005BC`、`0x2000067C`
- 速度环 Kp（`float`）：`0x20000440`、`0x20000500`、`0x200005C0`、`0x20000680`，Ki / Kd 各顺延 4 / 8 字节
- `chassis_command`：`0x20000974` 起（2026-10-24 改版以后；10-23 拆分后是 0x20000954，更早是 0x20000718），`state` 在 +0（1 字节），vx / vy / wz（`float`）在 +8 / +12 / +16

## CAN2：升降 3508（SB 拨杆控制，速度模式）

- CAN2，1 Mbps；M3508 + C620，电调 ID 1：`0x200` 控制帧槽位 0，反馈 `0x201`。ID 只要求同一路总线内不重复，和 CAN1 底盘的 ID1 不冲突。CAN2 用过滤器组 14（CAN1 用 0~13，CAN2 用 14~27）。
- `main.c` 的 CAN 接收回调先读一帧，再按 CAN1 / CAN2 交给各自的管理器（`chassis_motor_manager` / `arm_motor_manager`）。
- 遥控（主循环，`UserModules/APP/Arm/Arm_Remote.c`）：
  - 只在整车使能（在线 + SA）时看 SB（CH6）：拨上 → `lift_dir` = +1，中位 → 0，拨下 → −1；门限是归一化 0.5。
  - 回过中位才生效：掉线或锁定时清掉 `LiftArmed`；重新使能（上电、遥控重连、重新拨 SA）以后，SB 要先在中位出现一次，之后才按档位走。没回中之前 `lift_dir` 一直是 0，升降原地停住。
  - 算完和底盘指令在同一次关中断里发布到 `arm_command`。
- 控制（TIM8，`Arm_Lift_Update`，在 `APP/Arm/Arm_Lift.c`）：遥控在线 + SA 使能 → 速度模式，目标 = `arm_command.lift_dir × LIFT_SPEED_RPM`；否则失能（机构失能后自己能停住）。和底盘一样只在模式变化时调 `MotorControl_SetMode`。
- 参数（`Arm_Lift.c`）：`LIFT_SPEED_RPM` = 1000（转子 rpm，输出轴约 52 rpm）；Kp = 15、Ki = 30、Kd = 0（2026-10-22 上板试调）；电流指令限幅 `LIFT_CURRENT_LIMIT` ±8000（约 9.8 A，接近 M3508 额定电流 10 A；2026-10-25 用户要求从 6000（约 7.3 A）调大）；`motor_direction` = −1（实测 + = 往上）。速度环积分项上限还是驱动默认的 1000（约 1.2 A），带重物往上时转速掉得多的话可以像底盘一样单独放大。
- 要注意：
  - **还没有软限位**：拨着 SB 冲到头会顶住，速度环 Kp × 1000 = 15000 超过限幅，输出顶满 8000（约 9.8 A，2026-10-25 起；之前 6000）。到头别一直拨着，行程测完再加软限位。
  - ELRS 串口出错重启时，驱动会把它当成掉线（通道、时间戳清零）：底盘会停一下，升降还要 SB 重新回中才能再动。上板时偶尔遇到"升降突然不动、回中再拨就好"，可结合串口错误计数核对。
  - 往上走时如果稳态转速明显低于目标，可能是速度环积分上限 `SPEED_PID_I_OUTPUT_LIMIT` = 1000（约 1.2 A，`motor_control.c`）扛不住重力；这个宏底盘也在用。
  - CAN2 启动失败（比如总线一直是显性电平）时 `BareMetal_Init` 返回错误、进 `Error_Handler`，底盘也跟着不动。

### 上板测试（升降）

1. 机构停在中间（上下都有行程）。上电前遥控器 SA 拨锁定、SB 放中位。
2. 上电：`lift_motor.mode` 为 `MOTOR_MODE_DISABLE`（0），升降不动；`lift_motor.feedback_count` 在涨，`arm_can_tx_status` 为 HAL_OK（0）。
3. SA 拨使能：`mode` 变 `MOTOR_MODE_SPEED`（2），`speed.target_rpm` = 0，升降原地停住。
4. SB 拨上 → `arm_command.lift_dir` = 1、目标 +1000，往上走；拨下 → −1、−1000，往下走；回中 → 0，停。现在没有软限位，离头远一点就回中。
5. 回中保护：SB 拨上的状态下 SA 拨锁定 → 失能；SA 再拨回使能 → 不动（`lift_dir` = 0），SB 回一次中位再拨上才动。关遥控再开同理。
6. 关遥控：超过 500 ms 后底盘和升降都失能。
7. 排查：`arm_can_tx_status` 一直是 HAL_BUSY（2）→ CAN2 上没有节点应答（查线、分电板电源、终端电阻）；发送正常但 `feedback_count` 不涨 → C620 的 ID 可能不是 1（绿灯每秒闪 N 下 = ID N）；拨了 SB 但 `lift_dir` 一直是 0 → 看 `LiftArmed` 是不是 0，是的话先回中。

### 调试变量地址（升降）

2026-10-23 把 `BareMetal.c` 按机构拆开以后，这一组地址整体变了，下面是按新 ELF 用 gdb 算的；以后重新编译仍以当次 ELF 为准。2026-10-24 遥控映射改版以后又算过：电机这几个没变，只有 `arm_command.lift_dir`、`LiftArmed` 挪了。2026-10-25 以后这一节全部往后挪了 20 字节（见上一节），下面是新值。

速度、电流、编码器都已经乘过 `motor_direction`（−1），+ = 往上；实际发给 C620 的电流是 `output_current` 取反。

- `lift_motor.speed_rpm`（`int16_t`，实际转子转速）：`0x2000076E`
- `lift_motor.speed.target_rpm`（`float`，目标转子转速）：`0x2000077C`
- 速度环 Kp / Ki / Kd（`float`）：`0x20000780` / `0x20000784` / `0x20000788`
- `lift_motor.output_current`（`int16_t`，速度环输出的电流指令）：`0x2000080A`
- `lift_motor.feedback_current`（`int16_t`，C620 反馈的实际电流）：`0x20000770`
- `lift_motor.total_encoder`（`int32_t`，收到第一帧反馈时清 0，转子每圈 8192）：`0x20000778`
- `lift_motor.mode`（1 字节，0 失能 / 2 速度）：`0x20000759`
- `arm_can_tx_status`（1 字节）：`0x20000818`
- `arm_command.lift_dir`（`int8_t`，发布出去的升降方向）：`0x20000988`（2026-10-24 改版以后；之前 0x20000968）
- `LiftArmed`（`Arm_Remote.c` 里的 static，1 = SB 已经回过中位）：`0x2000098E`（2026-10-24 改版以后；之前 0x2000096E），Watches 里写 `'Arm_Remote.c'::LiftArmed`

## CAN2：旋转 RS05（SC 点动：拨上往正前方、拨下往果框上方、中位停，CSP 位置模式）

- RS05 走私有协议（扩展帧，1 Mbps），电机 ID 1，主机 ID 0xFD；CAN2 用过滤器组 15（扩展帧全收，是哪台电机由驱动按 ID 判断）。
- 初始化（`Arm_Rotate_Init`，在 `APP/Arm/Arm_Rotate.c`）：`RS05_Manager_Init` → 填 `motor_id` = 1 再注册 → `RS05_Manager_Start`（配过滤器；CAN2 已被升降启动过，只再开一次接收中断）→ 发一次停止（类型 4），保证处在失能（复位）状态。没回不算初始化失败，底盘和升降照常。和 C 板一起上电时 RS05 启动得慢（2026-10-22 实测：开机后约 1.4 s 内的读请求都没回复），这帧停止多半没回复，`rotate_motor.last_feedback_tick` 会是 0，不代表 RS05 不在线；判断在线看 `rotate_mech_pos_count` 在不在涨。
- `main.c` 的 CAN 接收回调：CAN2 的同一帧交给大疆和 RS05 两个驱动，大疆只认标准帧，RS05 只认扩展帧。
- 遥控（主循环，`Arm_Remote.c`，2026-10-24 起）：使能时 SC（CH7）拨上 → `arm_command.rotate_dir` = −1（往正前方），中位 → 0（停），拨下 → +1（往果框上方）；拨上对应哪边由 `ARM_ROTATE_SC_UP_DIR` 定，左右反了改它。锁定 / 掉线时为 0。
- TIM8（`Arm_Rotate_Update`）每拍最多发一帧 RS05 的帧，按顺序看：
  1. `Arm_Rotate_TrackOnline`：跟踪 RS05 在不在线（300 ms 内回过 mechPos）。掉线 1 s 以上又回来，当成 RS05 重新上过电，`rotate_zero_sta_state` 清 0、重新读。这一步必须在控制之前，不然 RS05 一回来，控制会拿着上一次上电的 zero_sta 结论去使能（主机模拟里抓到过）。
  2. 控制（`Arm_Rotate_Control`），见下面。
  3. 第 50 拍：参数帧（zero_sta 自动检查，见下一节）。
  4. 第 0 拍：读 `mechPos`（参数 `0x7019`，负载端多圈角度，rad），存进 `rotate_mech_pos_rad`、`rotate_mech_pos_count` 加 1。用 mechPos、不用类型 2 里的位置：类型 2 的位置在 ±4π 里循环。
  - 全部放在 TIM8 里发：主循环和中断同时调 `HAL_CAN_AddTxMessage` 会选中同一个空邮箱，主循环那一帧会丢。
- 控制（`Arm_Rotate_Control`）：
  - 能使能的条件（`rotate_block` 显示第一个不满足的）：遥控在线 + SA 使能（1）、RS05 在线（2）、这次 RS05 上电时 zero_sta 已经是 1（3）、位置在 [正前方 − 0.2, 果框上方 + 0.2] rad 里（4）。
  - 使能流程（照旧驱动 `RS05_EnterCSPMode` 的顺序，一拍一帧，发不出去下一拍原地重发，不跳步）：停止 → run_mode = 5 → 限流 2.4 A → 限速 1.0 rad/s → 目标 → 使能 → 限流 → 限速 → 目标。使能后把限流、限速、目标再写一遍，防使能前写的没被收下（手册写法是使能后再写；电机默认限流可能很大）。
  - 运行中（`rotate_state` = 2）每 20 ms 重发一次目标。
  - 目标（点动，2026-10-24 起）：`rotate_dir` = −1 → 目标直接给正前方 `ROTATE_FRONT_RAD` = −2.460，+1 → 给果框上方 `ROTATE_BASKET_RAD` = 2.426（都是 zero_sta = 1 下的读数），速度由 CSP 限速管；0 → 目标 = 回中那一刻的位置（只在回中那一下记一次，之后一直停在那里，不跟着读数走，否则抱不住）。两端就是软限位：目标永远在 [−2.460, 2.426] 里，这段经过 0、不经过 ±π；跳变点 ±π 在没用到的约 75° 里（离两端 0.68 / 0.72 rad），电机自己转不过去。
  - 回中时取哪个位置：优先用类型 2 应答里的位置 `rotate_motor.position_rad`（运行中每 20 ms 写一次目标都有应答，最多旧约 20 ms）；它超过 50 ms 没更新、或者和 mechPos 差 0.3 rad 以上（万一两种读数对不上），就退回用 mechPos（每 100 ms 读一次，按 1 rad/s 停下时会往回退最多约 0.1 rad）。两种读数一不一样要上板确认（见下面"上板测试"）。
  - 使能那一刻先停在原地（当时的位置），这次使能后 SC 回过一次中位才按 SC 走。这条放在 `Arm_Rotate` 里、不放 `Arm_Remote`：RS05 可能比遥控晚使能（等 zero_sta、位置检查），也可能自己重启后重新使能，要按它真正使能的那一刻算。所以上电、遥控重连、重新拨 SA 使能、RS05 重启以后，SC 正好拨在一边，机械臂也不会自己转。
  - 条件不满足了（SA 锁定、遥控掉线、RS05 掉线、位置不对）→ 发停止 → 失能（复位模式），机械臂可以手推。
  - 余量为什么只给 0.2 rad：没用到的那段（约 1.4 rad）上电读数会折到另一头，余量越大越分不清"果框再往外"和"正前方再往里"；0.2 时要推出果框 1.2 rad（RS05 输出）以上才会被误认成正前方。
- 参数（`Arm_Rotate.c` 开头）：`ROTATE_SPEED_LIMIT_RAD_S` 1.0（RS05 输出轴，走完全程约 5 s，先慢）、`ROTATE_CURRENT_LIMIT_A` 2.4（RS05 额定相电流）、`ROTATE_TARGET_PERIOD_MS` 20、`ROTATE_FEEDBACK_FRESH_MS` 50 / `ROTATE_FEEDBACK_MATCH_RAD` 0.3（回中取位置用）。正前方、果框上方是手推量的（现在也是点动的两端），差几度就推到准确位置读一下 `rotate_mech_pos_rad` 改这两个宏。
- 还没做：和升降的联锁（转之前先把升降、伸缩收好，先靠操作习惯）；RS05 的 CAN 超时（canTimeout = 0：C 板死机时 RS05 按最后的目标走，点动时最后的目标可能是行程一端，会走到那一端停下，不会出行程）。

### zero_sta：上电后自动检查，不是 1 就写 1（2026-10-22 用户决定）

- zero_sta 是 RS05 上电时报位置用的"窗口"。上电时它只知道输出轴在一圈里的角度，不知道转过几圈，只能在一个 2π 宽的窗口里报：0 → 0~2π（跳变点在 0 / 2π），1 → −π~π（跳变点在 ±π）。通电以后连续数圈，读数可以超出窗口，只有上电那一刻折回窗口。
- 为什么要改：实测正前方约 3.8、果框上方 8.709，行程跨过了 2π，同一个果框位置上电读 2.426、从正前方推过去读 8.709，目标没法写死。改成 1 以后跳变点 ±π 落在机械臂不去的那约 75° 里，行程上哪里上电读数都连续：正前方约 −2.46，果框上方约 2.43，跳变点离两头约 39° / 41°（RS05 输出）。
- 为什么要类型 22：类型 18 写的值掉电就丢，而 zero_sta 正是上电那一刻才起作用，所以要保存进电机 flash。
- 为什么不是"每次上电直接设"：zero_sta 存在电机里，设一次一直有效（像 C620 设 ID）；RS05 比 C 板晚约 1.4 s 才能通信，C 板能发命令时它已经按旧值算好了上电位置，这次改了要下次上电才生效；每次都写 flash 没必要，写的瞬间断电还可能写坏参数。所以做成"读一下，不是 1 才写"，正常只有第一次真的写。
- 程序怎么做（`Arm_Rotate_Param`）：
  - RS05 上线后（或者掉线 1 s 以上又回来，当成它重新上过电）先读 zero_sta，读到的第一个值就是这次上电用的范围：`rotate_zero_sta_state` = 1（是 1，已生效）或 2（不是 1）。
  - 任何时候读到不是 1：下一个参数时刻写 1（类型 18），再下一个保存（类型 22），`rotate_zero_sta_write_count` 加 1。写完 RAM 里就是 1，再读不会重复写。
  - 判断完以后每 1 s 读一次，`rotate_zero_sta` 显示最新的值。全程不使能。
- 第一次上电会看到：`rotate_zero_sta_state` = 2、`rotate_zero_sta_write_count` = 1、`rotate_zero_sta` 读回 1 → 把电池那一路断电再上电（C 板不跟着断电也没关系）→ `rotate_zero_sta_state` 变 1，正前方读约 −2.46，果框上方约 2.43。
- 注意：
  - 写完以后只重刷 C 板、RS05 没断电时，`rotate_zero_sta_state` 会显示 1，但 RS05 这次上电用的还是旧范围。所以第一次写完一定要把 RS05 断电重启一次；以后做使能时，除了 state = 1，还要检查位置在行程范围内才放行。
  - `rotate_zero_sta_write_count` 每次重启都涨 → 说明没存进 flash，告诉我。

### 上板测试（SC 点动旋转）

1. 先把机械臂摆在正前方，旋转范围里别放东西，手放在电池开关上。烧录这一版。
2. 第一次上电：zero_sta 还是出厂的 0 → 程序自动写 1 并保存，`rotate_zero_sta_state` = 2，`rotate_block` = 3，RS05 不会使能。把电池那一路断电再上电 → `rotate_zero_sta_state` 变 1。
3. SC 放中位，SA 拨使能：`rotate_block` = 0，`rotate_state` 变 2（运行中），机械臂停在原地不动，`rotate_target_rad` ≈ 当前位置（约 −2.46）。
4. SC 拨下：`arm_command.rotate_dir` = 1、`rotate_target_rad` 变 2.426，机械臂往果框上方转（走完全程约 5 s）；中途回中：`rotate_target_rad` 变成当时的位置，机械臂停住、不往回退；拨上：往正前方走。拨着不放会正好停在两端。左右反了（比如拨上往果框转）马上拨 SA 锁定，然后把 `Arm_Remote.h` 的 `ARM_ROTATE_SC_UP_DIR` 改成 1。回中停住后对比 `rotate_motor.position_rad` 和 `rotate_mech_pos_rad`：两个应该差不多（0.1 rad 以内）；差得多告诉我（程序会自动退回用 mechPos，但最好弄清楚）。
5. SA 拨锁定：`rotate_state` 变 0，RS05 失能，可以手推。SC 拨在一边时再拨使能：停在原地，SC 回一次中位才动。
6. 不动的话看 `rotate_block`：1 遥控没使能 / 2 RS05 不在线 / 3 zero_sta 还不是 1（断电重启 RS05）/ 4 位置不在行程里（推回正前方和果框之间）。

### 上板测量（找位置区间）

1. 断电时把机械臂摆到正前方，再上电。暂停看：`rotate_mech_pos_count` 在涨（每秒约 10），记下 `rotate_mech_pos_rad` = p_front。（`rotate_motor.last_feedback_tick` 是 0 正常，见上面。）
2. 继续运行，用手把机械臂推到果框上方（RS05 是失能的），暂停记 p_basket。两头如果有机械极限，也推过去记一下，好留余量。来回 2~3 次看重复性。
3. Δ = p_basket − p_front（带符号）。目标怎么给取决于第 4 步的上电位置验证结果，需结合前述 zero_sta 设置与旋转目标参数确认。
4. 验证上电位置是不是绝对的：① 不动机械臂，断电再上电，读数应该不变；② 断电时把机械臂推到量过的另一个位置（比如果框上方），再上电，看读数是不是和通电时在那里量到的一样。2026-10-22 实测：正前方上电 3.741，推到果框上方 8.709（Δ = +4.968 rad）；在果框上方断电重启读 2.426（= 8.709 − 2π）；断电时推回正前方再上电读 3.823 → 上电位置在一圈以内是绝对的，跳变点问题用 zero_sta = 1 解决（见上面）。
5. 排查：
   - `rotate_mech_pos_count` 不涨 → RS05 没回话：查 ID 是不是 1、CAN2 线、RS05 供电；
   - `rotate_can_tx_status` 一直是 HAL_BUSY（2）→ CAN2 上没有节点应答；
   - 计数在涨，但手推时 `rotate_mech_pos_rad` 不变 → 失能状态下可能不更新位置，告诉我，换别的读法。
6. 暂停时 C 板不发 CAN 帧。RS05 本来就是失能的；测之前先把 SA 拨到锁定，免得暂停时升降或底盘在动。

### 调试变量地址（RS05）

按 2026-10-23 拆分以后的 Debug 构建（这一组地址整体变了；2026-10-24 改版以后核对过，除了 `arm_command.rotate_dir` 都没变；2026-10-25 以后全部往后挪了 20 字节，下面是新值）；Watches 里直接写变量名也行。

- `rotate_mech_pos_rad`（`float`，最近一次读到的 mechPos，rad）：`0x2000090C`
- `rotate_mech_pos_count`（`uint32_t`，一共读到了几次）：`0x20000910`
- `rotate_can_tx_status`（1 字节，最近一次读请求的发送结果）：`0x20000914`
- `rotate_motor.position_rad`（`float`，类型 2 应答里的位置，±4π 循环；运行中每 20 ms 写目标都有应答会更新它，SC 回中时用它当停住的位置；失能时只有停止帧的回复会更新）：`0x20000854`
- `rotate_motor.state` / `fault_flags`（各 1 字节，类型 2 里的模式状态 0 复位 / 1 标定 / 2 运行，故障位）：`0x20000864` / `0x20000865`
- `rotate_motor.last_feedback_tick`（`uint32_t`，最近一次收到类型 2 的时刻）：`0x20000868`
- `rotate_zero_sta`（1 字节，最近一次读到的 zero_sta）：`0x20000915`
- `rotate_zero_sta_state`（1 字节，0 还没读到 / 1 这次上电已经是 1 / 2 不是 1、已自动写好、要重启 RS05）：`0x20000916`
- `rotate_zero_sta_write_count`（`uint32_t`，一共自动写了几次）：`0x20000918`
- `rotate_state`（1 字节，0 失能 / 1 正在使能 / 2 运行中 / 3 正在停）：`0x2000091C`
- `rotate_block`（1 字节，0 可以使能 / 1 遥控没使能 / 2 RS05 不在线 / 3 zero_sta 还不是 1 / 4 位置不在行程里）：`0x2000091D`
- `rotate_target_rad`（`float`，现在发给 RS05 的目标）：`0x20000920`
- `arm_command.rotate_dir`（`int8_t`，发布出去的 SC：−1 往正前方 / 0 停 / +1 往果框上方）：`0x20000989`（2026-10-24 改版以后；之前是 `rotate_basket` 0x20000969）

## USART6：机械臂 STM32 节点（左摇杆上下控步进，SD 控夹爪 + 剪线，SA 带果篮）

- 节点固件是 `C:\Clion\SC09ServoTest`（STM32F103C8T6）：USART2 收 C 板的命令，USART3 经 RS-485 控 PD42S1（伸缩步进），USART1 控 SC09 舵机（夹爪），TIM1_CH1（PA8）出 PWM 给 LD-2015 剪线舵机，TIM1_CH4（PA11）出 PWM 给果篮舵机（2026-10-24 加的）。C 板按它的协议发，九条命令都不带数据（`UserModules/Devices/ArmNode/ArmNode_Command.h`，和节点 `node_app.h` 的 `Node_Command` 一一对应）：

  | 命令 | 动作 | 完整帧 |
  |---|---|---|
  | 0x01 | 夹爪打开（舵机 270） | `AA 55 01 01 00 00 50 18` |
  | 0x02 | 夹爪闭合（舵机 380，2026-10-25 起；之前 360） | `AA 55 01 02 00 00 A0 18` |
  | 0x03 | 步进正转 20 rpm | `AA 55 01 03 00 00 F1 D8` |
  | 0x04 | 步进停止（节点里 0xFC 刹车 + 0xFB 清状态） | `AA 55 01 04 00 00 40 19` |
  | 0x05 | 步进反转 20 rpm | `AA 55 01 05 00 00 11 D9` |
  | 0x06 | 剪线开始（LD-2015 在 500 / 2500 µs 之间一直往复，每端停 2 s） | `AA 55 01 06 00 00 E1 D9` |
  | 0x07 | 剪线停止（回到剪刀张开的位置停住） | `AA 55 01 07 00 00 B0 19` |
  | 0x08 | 果篮打开 | `AA 55 01 08 00 00 80 1A` |
  | 0x09 | 果篮关闭 | `AA 55 01 09 00 00 D1 DA` |

- 接线：C 板 3-pin 口（USART6，线序 GND / TXD / RXD；115200 8N1，.ioc 早就是这样配的，这次没改）↔ 节点 PA3（RX）/ PA2（TX），TX、RX 交叉，共地。3-pin 口没有 5V，节点要自己供电。节点现在不回帧，只接 C 板 TXD → 节点 PA3 和 GND 也能用。
- 帧层：`UserModules/Devices/ArmNode/Binary_Protocol.c/h`，拷自 SC09ServoTest（和 Host_Computer_ReadyForApp、旧工程 Chassis_BareMetal 是同一套），只改了一处：`rx_calculated_crc` 的初值 0xFFFF → 0（`BinaryProtocol_Init` 会重新设成 0xFFFF，行为不变），不改的话它进 `.data`，会把本文件记的所有调试地址整体往后挤 4 字节。用法和旧工程一样：`Arm_Node_Init` 里 `BinaryProtocol_Init(&huart6)`（由 `BareMetal_Init` 调）（循环 DMA 接收 + 空闲中断），`main.c` 的两个 UART 回调按 Instance 转给它，主循环里 `BinaryProtocol_Task()`。
- 遥控（主循环，`Arm_Remote.c` 算出 `Arm_NodeCommand_t`；只在主循环里用，不经过 TIM8，所以没放进 `arm_command`）：
  - 左摇杆上下（CH1，不回中）→ 步进 `extend_dir`：读数 > +0.5 → +1（前进），< −0.5 → −1（后退），绝对值 < 0.3 → 0（停），0.3 ~ 0.5 之间保持原来的区（回差：杆停在门限附近不会在走 / 停之间来回跳，每跳一次就要发一帧）。使能后杆要先回过一次中间才生效（`ExtendArmed`）：开机时油门杆一般在最下，一使能不会后退。门限是 `Arm_Remote.h` 的 `ARM_EXTEND_MOVE_THRESHOLD` / `ARM_EXTEND_STOP_THRESHOLD`。杆不回中，松手不会停，要推回中间。
  - SD（CH8）→ 夹爪 `gripper`：拨上合、拨下开（2026-10-24 用户确认）。使能那一刻记下 SD 在哪边，拨到另一边以后才发（`GripperArmed`）：一使能不会自己夹，更不会自己剪。反了对调 `ARM_GRIPPER_POSITIVE` / `ARM_GRIPPER_NEGATIVE`。
  - SA → 果篮 `basket`：使能 → 关，锁定 → 开，掉线 → 不发（保持原样，免得断链把果子倒出来）。要注意：SA 也是锁车开关，每次锁车（包括紧急锁车）果篮都会开；上电时 SA 在锁定位，遥控一连上果篮就会开。
- 发送（主循环，`Arm_Node_Update`，在 `APP/Arm/Arm_Node.c`）：
  - 只在指令和上次发的不一样时发，一次一帧，两帧之间至少隔 50 ms（`ARM_NODE_SEND_INTERVAL_MS`）。按这个顺序挑：步进补发停止 → 剪线补发停止 → 步进 → 剪线停止 → 夹爪 → 剪线开始 → 果篮。原因：节点每收到一条就在主循环里执行，步进、夹爪要阻塞等驱动器回包（舵机不回话时约 100 ms），收帧事件队列只有 4 格。
  - 步进：+1 → 0x03 正转，−1 → 0x05 反转，0 → 0x04 停止（`ARM_NODE_EXTEND_UP_COMMAND` / `ARM_NODE_EXTEND_DOWN_COMMAND`；正转是伸还是缩上板看，反了对调）。
  - 夹爪 + 剪线：SD 拨上 → 0x02 闭合，发出去 700 ms 后（`ARM_NODE_CUT_DELAY_MS`：节点里夹爪舵机运动时间 500 ms + 余量）→ 0x06 剪线开始；SD 拨下 → 先 0x07 剪线停止，50 ms 后 0x01 打开（先收剪刀再松夹爪）。合上不到 700 ms 就拨开：不剪。
  - 果篮：0x08 打开 / 0x09 关闭。
  - 锁定 / 掉线：步进清 0 → 发停止，剪线发停止，夹爪不发（保持原样，免得掉线时松开），锁定时再发果篮打开。另外使能 → 失能那一下，只要上次补发以后转过 / 剪过，就不管上次发的是什么再补一帧停止：节点不回 ACK，防之前那帧停止丢了。重新使能以后 SD 要拨一下才会再夹 / 剪。
  - 开机不发帧；遥控连上后先按 SA 发一帧果篮开 / 关。节点上电步进不转，两个 PWM 舵机都不出脉冲（等第一条命令）。
  - 阻塞发送：8 字节在 115200 下约 0.7 ms，只在主循环里调，TIM8 里不碰串口。发送失败（`arm_node_tx_status` 不是 0）下个间隔重发。
- 节点那边 2026-10-24 改的（细节见节点 README）：
  - 剪线（LD-2015）原来上电 1 s 后就一直自己往复、不受 C 板控制，现在改成命令控制：上电不出脉冲，0x06 开始往复（第一下走离张开位远的那一端，先合剪刀），0x07 回到张开位停住。张开位 `LD2015_CUT_OPEN_US` 现在是占位值 1500，量好以后改节点的 `node_config.h`；往复两端 500 / 2500、每端停 2 s 沿用原来的。
  - 果篮：PA11 / TIM1_CH4，开 / 关脉宽 `BASKET_OPEN_US` / `BASKET_CLOSE_US` = 1000 / 2500（2026-10-25 用户上板纠正：开 1000、关 2500）。舵机单独供电、和节点共地。
  - 节点 .ioc：TIM1 原来那几行是手加的、不全（缺 `SH.S_TIM1_CH1` 和内部时钟源），按它重新生成会把 TIM1 整个删掉；这次补全并加了 CH4，用 CubeMX 6.15 重新生成，顺带删掉了 `stm32f1xx_it.c` USER CODE 里手写、现在会重复定义的 DMA1_Channel6 / USART2 中断函数。
- 要注意（都是节点那边的，C 板这边管不了）：
  - 节点的停止：原来只发 `PD42S1_Stop()`（0xFC 立即停止 / 刹车），PD42S1 手册 4.4.13 写着刹停后要及时清除电机状态，否则电机会严重发烫。2026-10-23 已在节点里补上：0xFC 回包通过后再发 `PD42S1_ClearState()`（0xFB）。手册没写清完状态以后电机是什么样（停着，还是接着按之前的速度转），上板要确认：停下后不会自己再转、放一会儿不烫。
  - 节点没有掉线超时：C 板死机、被调试器暂停、复位，或者 USART6 线断了，步进和剪线都会一直按最后一条命令动。C 板复位以后也不会自动补停止（开机当成已经停着），这时要把左摇杆推出去再回中、SD 拨一下，才会重新发。
  - 节点不回 ACK：`arm_node_tx_status` = 0 只说明 C 板发出去了，节点收没收到、电机动没动，要看节点的 `g_node_app`。

### 上板测试（节点）

1. 先单独确认节点：用串口助手按上面的表发 HEX：0x01 / 0x02 夹爪动；0x03 / 0x05 步进转、0x04 停住不再转、放一会儿摸电机不烫；0x06 剪线开始往复、0x07 回到一个位置停住；0x08 / 0x09 果篮开（1000）/ 关（2500）。节点的 `g_node_app.last_command_result` = 1（是 4 就是有一条没回包或 PWM 没起来）。节点上电时剪线、果篮都不应该动。
2. 断电接线：C 板 3-pin 口 TXD → 节点 PA3、RXD ← 节点 PA2、GND 共地；剪线舵机信号接 PA8、果篮舵机信号接 PA11，舵机电源单独供、和节点共地。C 板和节点都烧这一版。
3. 遥控开、SA 锁定、左摇杆在最下、SD 拨下（开），上电：只发一帧果篮打开（`arm_node_last_command` = 8，`arm_node_send_count` = 1）。
4. SA 拨使能：只发果篮关闭（9）。左摇杆在最下时步进不动，推回中间也不发。
5. 左摇杆推上 → 3（正转 / 前进）；回中 → 4；拉下 → 5；回中 → 4。记下正转是伸还是缩，反了对调那两个宏。杆停在 0.3 ~ 0.5 之间不会变。
6. SD 拨上 → 2（闭合），约 0.7 s 后 → 6（剪线开始），剪线舵机一直往复；SD 拨下 → 7（剪线停止，回张开位）→ 1（打开）。
7. 剪线中 SA 锁定 → 发 4（步进转过的话）、7、8，夹爪保持合着；再使能 → 只发 9，不剪；SD 拨下再拨上才重新夹 / 剪。关遥控同理（约 500 ms 后发 7，果篮不动）。
8. 排查：`arm_node_send_count` 在涨但节点没反应 → 看节点的 `g_node_app.host_frame_count` 涨不涨，不涨查 TX / RX 是否交叉、是否共地；`arm_node_tx_status` 不是 0 → C 板这边串口没发出去。

### 调试变量地址（节点）

按 2026-10-25 的 Debug 构建（2026-10-24 改版时 `arm_node_*` 没变、`Arm_Remote.c` 的 static 挪了；2026-10-25 底盘加了加速度限制以后，这一节全部往后挪了 20 字节）。

- `arm_node_last_command`（1 字节，最近一次发出去的命令 1~9）：`0x2000094C`
- `arm_node_send_count`（`uint32_t`，一共发出去几帧）：`0x20000950`
- `arm_node_tx_status`（1 字节，最近一次发送的结果，0 = HAL_OK）：`0x20000954`
- `arm_node_rx_count`（`uint32_t`，节点发回来的合法帧数，现在应该一直是 0）：`0x20000958`
- `NodeCommand`（`Arm_Remote.c` 里的 static：`extend_dir` 在 +0（`int8_t`），`gripper` 在 +1（0 不发 / 1 打开 / 2 闭合），`basket` 在 +2（0 不发 / 1 打开 / 2 关闭））：`0x20000990`，Watches 里写 `'Arm_Remote.c'::NodeCommand`
- `ExtendArmed`（1 = 左摇杆已经回过中间）：`0x20000993`；`ExtendZone`（`int8_t`，左摇杆现在在哪个区 +1 / 0 / −1）：`0x20000994`；`GripperZone`（0 还没读到 / 1 开 / 2 合）：`0x20000995`；`GripperArmed`（1 = 使能后 SD 拨过）：`0x20000998`
- 剪线 / 果篮上次发的（`Arm_Node.c` 里的 static）：`arm_node_cut_sent`（1 = 在剪）`0x20000968`、`arm_node_basket_sent`（1 开 / 2 关）`0x20000970`，Watches 里写 `'Arm_Node.c'::arm_node_cut_sent`

## 构建

将 ARM GCC、CMake、Ninja 加入 PATH，在本目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

固件输出：`build/Debug/Fruit_Picking.elf`。编译及主机模拟验证不能代替烧录后的 CAN 反馈与实际转动测试。

## 记录

2026-10-21 初版验证：Debug 全量构建通过，0 警告；提取实际 `main.c` 初始化与回调、联同真实电机驱动/PID 进行主机模拟，四电机注册、反馈分发、正负速度电流槽位、电流限幅、超时清零和发送状态检查均通过。用户随后确认实机电机可正常运动、方向正确。

2026-10-21 独立初始化版：四个 ID 的初始化分别展开，保留当前 PID 5/20/0 和初始目标 500 rpm。Debug 构建与实际初始化/回调主机模拟通过；经 CMSIS-DAP 烧录，OpenOCD 返回 `Verified OK`，复位运行后读取到注册数 4、四台反馈计数持续增加、发送状态 HAL_OK。一次瞬时转子速度读数（ID1~4）为 491/466/493/510 rpm，仅代表该次采样，不代表完成 PID 整定。

2026-10-21 四电机自动切速版：仅修改代码并生成 ELF，未烧录或操作硬件。编译通过；提取实际初始化、主循环和回调进行主机模拟，四台每 10 秒切换、边界时刻、计时回绕及保留手动目标检查均通过。（已被遥控版取代。）

2026-10-21 遥控版（遥控逻辑放在 TIM8 里）：写在 main.c 里，没上板。第二天按用户要求改成了下面的 BareMetal 结构。

2026-10-22 BareMetal 版：只改了代码，未烧录、未上板测试。在临时目录用 CLion 自带 CMake / Ninja + arm-none-eabi-gcc 做全量构建：Debug 0 警告（FLASH 34588 B，RAM 3376 B），Release 0 警告。主机测试 OMNI / MECANUM 通过。主机模拟（假 HAL + 真 CAN / 大疆 / PID / ELRS 驱动 + 真 `Chassis_Remote` / `Omni_Remote` + 真 `BareMetal.c`）通过：
- 没开遥控：发布 OFFLINE，失能，发出的 `0x200` 电流全 0；
- 拨使能 + 中速档推满前进：发布 ENABLED，四台切速度模式，目标 1097 / 1097 / −1097 / −1097 转子 rpm，电流 ±3000（限幅）；
- 连续保持使能：速度环积分持续累加，说明没有每拍调 `SetMode`；
- 主循环卡住不跑：发布的指令仍是 ENABLED，最后一帧后 500 ms 仍在线、501 ms 中断自己判掉线并失能，电流全 0；
- 拨锁定：LOCKED，失能；再拨使能恢复速度模式；
- DMA 环形缓冲连续收 20 帧（520 字节，缓冲 256 字节）全部解析成功，没有 CRC 错误；所有关中断都恢复了。

2026-10-22 全向轮编号调整：按用户确认，电调 ID1~4 改为右前、右后、左后、左前（顺时针）；驱动方向角为 315°、225°、135°、45°，保留原来的逻辑正转和各 ID 的 PID / 电机方向配置。Debug 构建通过，无警告；主机 OMNI 测试验证前后、左右、双向旋转、平移叠加旋转、停止及超限比例缩放，MECANUM 回归测试通过。未烧录，未验证本次改动后的实际轮子运动。
修改前的六个文件已备份并逐文件校验 SHA-256：`C:\Project\Fruit-picking robot\backups\wheel-layout-20261022-115515`。

2026-10-22 CAN2 升降 3508：只改了代码，未烧录、未上板。`BareMetal.c` 加了 CAN2 的大疆电机管理器 `arm_motor_manager` 和 `lift_motor`（C620 ID1，速度模式，目标 300 转子 rpm，PID 5 / 20 / 0，限幅 3000，方向 1），TIM8 里跑速度环、CAN2 发 `0x200`；`main.c` 的 CAN 接收回调改成先读帧，再按 CAN1 / CAN2 分发。在临时目录全量构建：Debug 0 警告（FLASH 34852 B，RAM 3616 B），Release 0 警告；底盘几个变量的地址没变。主机测试 OMNI / MECANUM 通过。主机模拟（假 HAL + 真 CAN / 大疆 / PID / ELRS 驱动 + 真 `Chassis_Remote` / `Omni_Remote` + 真 `BareMetal.c` + 从 `main.c` 原样抽出来的 CAN 接收回调）通过：
- CAN2 过滤器组 14、起始组 14，只收标准数据帧 `0x200~0x20F`；CAN1、CAN2 各启动一次，CAN2 打开了 FIFO0 接收中断；
- 没收到反馈时 CAN2 照样每拍发 `0x200`，电流 0；
- 两路的 `0x201` 不串；CAN2 上的扩展帧和没注册的 `0x202` 被读走、不处理；FIFO 里一次 3 帧也能读空；
- 第一拍（转速 0）电流 1506；接近目标时积分连续累加；堵转 2 s 电流停在 2500；反馈断了 100 ms 还照常、101 ms 输出 0，恢复后接着转；
- 手动改目标（模拟暂停后 F2 改值）不会被改回 300；
- 遥控使能、推满前进时底盘目标为 −1097 / −1097 / 1097 / 1097，升降不受影响；遥控掉线时底盘失能，升降照转；
- CAN2 邮箱满只会让 `arm_can_tx_status` 变成 HAL_BUSY，CAN1 照发。
- 反向检查：把回调换回改之前的版本（遇到 CAN2 直接 return），模拟里 CAN2 的接收中断一直重进，被断言抓到。
修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\can2-lift-20261022-121529`（带 SHA-256）。

2026-10-22 升降接遥控（SB）：只改了代码，未烧录、未上板。新增 `UserModules/APP/Arm/Arm_Remote.c/h`（SB → 升降方向，使能后回过中位才生效）；`BareMetal.c` 主循环把 `chassis_command`、`arm_command` 放在同一次关中断里发布，TIM8 里底盘和升降共用 `BareMetal_IsEnabled()`（遥控在线 + SA），升降使能 → 速度模式、目标 = 方向 × `LIFT_SPEED_RPM`，否则失能；`BareMetal_LiftInit` 改成先失能，`LIFT_TEST_RPM` 改名 `LIFT_SPEED_RPM`，顺手改掉了用户调参后已经过时的几行注释；`Chassis_Remote` 去掉 SB 档位，底盘固定低档。软限位按用户要求先不做。在临时目录全量构建：Debug 0 警告（FLASH 35044 B，RAM 3624 B），Release 0 警告；原有变量地址都没变。主机测试 OMNI / MECANUM 通过。主机模拟（假 HAL + 真驱动 + 真 `Chassis_Remote` / `Omni_Remote` / `Arm_Remote` + 真 `BareMetal.c` + 从 `main.c` 抽出的 CAN 接收回调）通过：
- 上电、遥控没开：升降失能，CAN2 电流 0；
- SA 使能 + SB 中位：速度模式、目标 0；SB 拨上 / 下 / 回中：目标 +1000 / −1000 / 0，速度环输出 ±5000，发到 CAN2 的电流符号相反（方向 −1）；SB 在 −0.36 附近算中位；
- SB 在上时拨 SA 锁定：失能；再使能：目标 0，回中一次再拨上才 +1000；
- 遥控掉线（主循环照跑）：失能；重连时 SB 在上：目标 0，回中后才生效；
- 主循环卡死：发布的方向停在 +1，最后一帧后 500 ms 还在转，501 ms TIM8 自己失能；
- 底盘推满前进，SB 在上 / 中 / 下时底盘目标都是 −590 / −590 / 590 / 590（固定低档）；所有关中断都恢复了。
- 反向检查：去掉"回中才生效"，模拟在"SA 重新使能时 SB 在上"那一步被断言抓到；升降只看发布的 state、不在 TIM8 里判在线，模拟在"主循环卡死 501 ms"那一步被抓到。
修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\arm-remote-20261022-133323`（带 SHA-256）。

2026-10-22 RS05 注册 + 测量代码：只改了代码，未烧录、未上板。`BareMetal.c` 加了 `BareMetal_RotateInit`（CAN2、ID 1、主机 0xFD，过滤器组 15，初始化发一次停止）和 TIM8 里的 `BareMetal_RotateMeasure`（每 100 ms 读一次 `mechPos` 0x7019，拷到 `rotate_mech_pos_rad`，RS05 一直失能）；`main.c` 的 CAN2 接收也交给 `RS05_ProcessRxFrame`；`RS05_Command.h` 加了 `RS05_PARAMETER_MECH_POS`；顺手把 `LIFT_CURRENT_LIMIT` 注释改成约 7.3 A（用户改成 6000 以后）。在临时目录全量构建：Debug 0 警告（FLASH 37860 B，RAM 3880 B），Release 0 警告；底盘、升降的地址没变，只有 `LiftArmed` 挪到 0x20000925。主机测试 OMNI / MECANUM 通过；升降遥控模拟（按用户的新参数 6000 / 15）通过。RS05 主机模拟（假 HAL + 真驱动含 RS05 + 真 `BareMetal.c` + `main.c` 的 CAN 回调）通过：
- 注册：CAN2、主机 0xFD、一台 ID 1；CAN2 过滤器组 14（大疆标准帧）、组 15（扩展帧全收），CAN2 只启动一次；
- 初始化只发了一帧类型 4 停止（0x0400FD01，数据全 0）；RS05 回的类型 2 能解析（复位模式、30.0 ℃），升降不受影响，升降的标准帧也不会进 RS05；
- TIM8 前 99 拍不发 RS05 帧，第 100 拍发类型 17 读 0x7019（0x1100FD01，数据 19 70 00 …），每拍的升降 0x200 照发；
- 回复 1.2345 / −3.5 / 15.0（大于 4π 也照收）在下一个 100 ms 拷到 `rotate_mech_pos_rad`，计数 1 / 2 / 3；某一轮没回复时值和计数都不变；带错误码的回复不收；
- CAN2 邮箱满时这次读请求返回 HAL_BUSY，下一轮照常；全程发给 RS05 的只有 1 帧停止和若干帧读参数，没有使能；中断路径里没调过 `HAL_Delay`。
- 反向检查：回调里不交给 RS05（改之前的写法），模拟在"RS05 回复停止帧"那一步就被断言抓到。
修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\rs05-measure-20261022-150320`（带 SHA-256）。

2026-10-22 zero_sta 一次性写入：只改了代码，未烧录。`RS05_Command.h` 加了 `RS05_COMM_TYPE_SAVE_PARAMETER`（0x16）、`RS05_PARAMETER_ZERO_STA`（0x7029）；`RS05_app.c/h` 加了 `RS05_SaveParameters`（类型 22，数据 01~08）；`BareMetal.c` 的 TIM8 里每 100 ms 的第 50 拍发参数帧：平时每 1 s 读一次 zero_sta，`rotate_zero_sta_write` 置 1 且 RS05 在线时发写入、下一轮发保存。在临时目录全量构建：Debug 0 警告（FLASH 38468 B，RAM 3896 B），Release 0 警告；原有地址没变，`LiftArmed` 挪到 0x20000935。主机测试 OMNI / MECANUM、升降遥控模拟通过。zero_sta 主机模拟通过：第 50 拍读 zero_sta、第 100 拍读 mechPos；离线时请求挂着不发；在线时下一个参数时刻发写入（0x1200FD01，29 70 00 00 01 00 00 00）、再下一个发保存（0x1600FD01，01~08），之后只读不写，读回 1；邮箱满时这次不算、下一轮重发；全程只有停止 / 读 / 写 0x7029 = 1 / 保存，没有使能。反向检查：去掉"在线才发"，模拟在离线那一步被抓到。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\zero-sta-20261022-180700`（带 SHA-256）。

2026-10-22 zero_sta 改成自动：按用户要求去掉手动触发（`rotate_zero_sta_write`、`rotate_zero_sta_count`），改成 `BareMetal_RotateParam` 自动检查：RS05 上线（或掉线 1 s 以上再回来）后读 zero_sta，第一个值决定 `rotate_zero_sta_state`（1 已生效 / 2 不是 1），读到不是 1 就写 1 + 保存（`rotate_zero_sta_write_count` 加 1），写完 RAM 是 1 不再重复写。只改了代码，未烧录。在临时目录全量构建：Debug 0 警告（FLASH 38636 B，RAM 3904 B），Release 0 警告；原有地址没变，新增 `rotate_zero_sta_state` 0x2000091A、`rotate_zero_sta_write_count` 0x2000091C，`LiftArmed` 挪到 0x20000939。主机模拟（假 RS05 区分 RAM 和 flash 里的 zero_sta）通过：RS05 晚 1.5 s 上线前不发参数帧；读到出厂值 0 → 状态 2，写 1 + 保存各一次，读回 1，之后每 1 s 读一次、不再写；掉线不到 1 s 状态不变；断电重启后读到 1 → 状态 1；设置丢了会重写；写入那一拍邮箱满下一轮重发；全程只有停止 / 读 / 写 0x7029 = 1 / 保存，没有使能。反向检查：去掉重启判断、或者每次读都写，模拟分别在"重启后状态变 1""只写一次"那一步被抓到。升降遥控模拟、OMNI / MECANUM 通过。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\zero-sta-auto-20261022-182850`（带 SHA-256）。

2026-10-22 RS05 遥控控制（SD）：只改了代码，未烧录（当时板上还是 17:32 的测量版，zero_sta 也还是出厂 0）。`Arm_Remote` 加了 SD（CH8）→ `rotate_basket`；`BareMetal.c` 的 TIM8 里加了 RS05 的 CSP 控制：能使能的条件（遥控 + SA、RS05 在线、这次上电 zero_sta 已是 1、位置在行程 ±0.2 rad 里）、一拍一帧的使能流程（停止 → run_mode 5 → 限流 → 限速 → 目标 → 使能 → 限流 → 限速 → 目标）、每 20 ms 重发目标、使能后先停在原地拨一下 SD 才动、条件不满足就停止。在线 / 重启的跟踪挪到每拍最前面（`BareMetal_RotateTrackOnline`）。在临时目录全量构建：Debug 0 警告（FLASH 39764 B，RAM 3928 B），Release 0 警告；原有地址没变，新增 `rotate_state` 0x20000920、`rotate_block` 0x20000921、`rotate_target_rad` 0x20000924，`LiftArmed` 挪到 0x20000952。主机模拟（真遥控帧 + 假 RS05：按收到的帧改模式 / 参数 / zero_sta 的 RAM 和 flash，使能后按限速走）通过：第一次上电 zero_sta 为 0 时自动写入、不使能；断电重启后读数变 −2.4598、自动使能且帧顺序正确、停在原地；每 20 ms 重发目标；SD 拨下约 5 s 到果框 2.426、拨上回正前方；SA 锁定发停止后不再发目标；SD 在下时重新使能停在原地，拨上再拨下才去果框；遥控关掉、RS05 掉线都会停；推到果框外 0.9 rad（读数折成 −2.957）不使能；邮箱时满时空时使能流程不跳步；RS05 重启后先重读 zero_sta 再使能、只使能一次；没有运控 / 设零帧，中断里没 HAL_Delay。模拟中途发现两个问题并改掉：余量 0.6 rad 太大（果框外 0.9 会被认成正前方附近 → 改 0.2）；重启判断在控制之后（会先拿旧结论使能再停 → 挪到每拍最前面）。反向检查：把在线跟踪挪回控制后面、或去掉"拨一下 SD 才动"，模拟分别被抓到。升降遥控模拟、zero_sta 自动检查模拟、OMNI / MECANUM 通过。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\rs05-remote-20261022-185149`（带 SHA-256）。

2026-10-23 机械臂 STM32 节点（SC 伸缩、左摇杆夹爪）：只改了代码，未烧录（板子当时没上电，OpenOCD 连不上）。按用户要求照 `C:\Clion\SC09ServoTest` 的协议发：新增 `UserModules/Devices/ArmNode/`（`Binary_Protocol.c/h` 拷自节点工程，只把 `rx_calculated_crc` 初值改成 0；`ArmNode_Command.h` 五条命令）；`Arm_Remote` 加了 SC（CH7）→ 伸缩方向（回过中位才生效）、左摇杆上下（CH1）→ 夹爪（使能后换过区才发，带回差），放在单独的 `Arm_NodeCommand_t` 里；`BareMetal.c` 的 `BareMetal_Init` 启动 USART6 接收，主循环加 `BareMetal_NodeUpdate`（变化才发、一次一帧、间隔 ≥ 50 ms、步进优先、失能时补发停止）；`main.c` 的两个 UART 回调加 USART6 分支；CMakeLists 把协议文件排在最后。在临时目录全量构建：Debug 0 警告（FLASH 43560 B，RAM 4600 B），Release 0 警告（FLASH 25056 B，RAM 4592 B）；和改之前的 ELF 逐个比过变量地址，只有 `Arm_Remote.c` 里的 `LiftArmed` 等和 `BareMetal.c` 函数里的 static 挪了（拷来的原文件 `rx_calculated_crc` 初值 0xFFFF 会进 `.data`、把所有地址挤后 4 字节，所以改成 0）。主机模拟（假 HAL + 真驱动 + 真 `Chassis_Remote` / `Arm_Remote` / `BareMetal.c` / `Binary_Protocol.c` + `main.c` 原样抽出的 CAN / UART 回调）通过：
- 上电、遥控没开、SA 锁定、刚使能（SC 中位、左摇杆在下）都不发帧；
- SC 上 / 中 / 下 / 中 → 正转、停止、反转、停止各一帧，字节和节点 `BINARY_PROTOCOL.md` 表里的完全一样（CRC 对得上）；左摇杆推上 / 中间 / 推上 / 拉下 → 只发打开、闭合各一帧；在门限两边来回晃各只发一次；
- SC 和左摇杆同一帧一起变：先发步进，50 ms 后发夹爪；SC 每 20 ms 来回拨：帧间隔照样 ≥ 50 ms，最后和拨杆一致；
- 正转中 SA 锁定：只发一帧停止、夹爪不发；锁定时 SC 留在上、左摇杆推上，再使能不发，SC 回中再拨上才转；转过、回中发过停止以后锁定，再补一帧停止，没转过的锁定不补；
- 正转中关遥控：约 500 ms 后发停止，重连不动；发送失败 50 ms 后重发；节点回的合法帧能收到、CRC 错的丢掉，USART6 出错后主循环重启接收，ELRS 不受影响；TIM8 里从没调过 `HAL_UART_Transmit`，关中断都恢复了，CAN1 / CAN2 每拍照发。
- 反向检查：去掉发送间隔、去掉 SC 回中、去掉补发停止、去掉夹爪的使能基准、`main.c` 不转 USART6 回调，模拟分别在对应那一步被断言抓到。
- 回归：RS05 遥控控制模拟、升降遥控模拟（遥控 CH1 / CH7 在中位，全程没往节点发一帧）、OMNI / MECANUM 都通过。
修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\arm-node-20261023-005108`（带 SHA-256）。模拟源码在 `%TEMP%\fp_node_sim`（临时目录，可能被清理）。

2026-10-23 节点停止补清状态：按用户要求只改了 `C:\Clion\SC09ServoTest` 的停止命令——`PD42S1_Stop()`（0xFC）回包通过后再调 `PD42S1_ClearState()`（0xFB），节点的主机测试、README、BINARY_PROTOCOL.md 跟着改了（测试：停止 = 两帧，0xFB 整帧对上手册例子 `C5 01 FB C1 5C`，清状态失败报驱动错误）。节点 ARM Debug / Release 0 警告，主机测试通过。C 板这边代码没变，只改了 `ArmNode_Command.h` 的注释和本文件的说明。未烧录。改之前的文件备份在 `C:\Project\Fruit-picking robot\backups\node-clearstate-20261023-012623`。

2026-10-23 拆分 BareMetal.c：按用户要求只搬家、不改逻辑。`BareMetal.c` 798 行 → 103 行，只留调度（各模块初始化顺序、遥控解析 + 关中断发布、`BareMetal_IsEnabled`、TIM8 节拍、主循环）；新增 `APP/Chassis/Chassis_Control.c/h`（底盘四台电机）、`APP/Arm/Arm_Lift.c/h`（升降）、`APP/Arm/Arm_Rotate.c/h`（RS05：在线跟踪、zero_sta、读 mechPos、CSP 控制）、`APP/Arm/Arm_Node.c/h`（机械臂节点）。机构不读遥控：TIM8 里 `BareMetal_IsEnabled()` 判一次，和指令一起当参数传进去（TIM8 执行期间 uwTick 和发布的指令都不会变，和原来每个机构各判一次等价）。函数改名：`BareMetal_MotorInit/MotorUpdate` → `Chassis_Control_Init/Update`，`BareMetal_LiftInit/LiftUpdate` → `Arm_Lift_Init/Update`，`BareMetal_Rotate*` → `Arm_Rotate_*`，`BareMetal_NodeUpdate` → `Arm_Node_Update`（新增 `Arm_Node_Init`）；变量名都没变。`main.c` 没改（`BareMetal.h` 改成 include 三个机构的头文件，CAN 回调要用的电机管理器从那里来）。在临时目录全量构建：Debug 0 警告（FLASH 43668 B，RAM 4600 B，FLASH 比拆分前多 108 B 是传参），Release 0 警告（FLASH 25144 B）。核对：
- 机器码：10 个签名没变的函数（三个 Init、`Arm_Rotate_Online` / `TrackOnline` / `Param` / `MechPos`、`Arm_Node_ExtendCommand`、`BareMetal_RemoteUpdate`、`BareMetal_IsEnabled`）和拆分前逐条指令相同（地址归一化后）；6 个改了签名的函数在源码层逐行比过，只差函数名 / 参数、`BareMetal_IsEnabled()` → `enabled`、`arm_command.xxx` / `chassis_command` → 参数这几处。
- 行为：节点发帧、RS05 遥控控制、升降遥控三套主机模拟在拆分前后各跑一遍，全部 PASS，打印的输出逐行一致（RS05 收到停止 8 / 使能 4 / 写参数 1184 / 读 393 / 保存 1，节点 32 次发送，CAN1 21520 帧 / CAN2 21736 帧）；在新文件上做的三个反向检查（去掉节点补发停止、去掉 RS05 使能后先停在原地、升降不判使能）都在对应那一步被抓到；OMNI / MECANUM 通过。
- 地址：底盘四台电机（`chassis_motors`、`chassis_motor_manager`）和驱动层变量没变；`chassis_command`、升降、RS05、节点、`Arm_Remote.c` 的变量都挪了，上面的地址表已按新 ELF 用 gdb 重算。
未烧录。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\split-baremetal-20261023-161301`（带 SHA-256）。模拟源码在 `%TEMP%\fp_node_sim`（`build.sh` 已加上四个新文件；`build_presplit.sh` 是拆分前的）。

2026-10-24 遥控映射改版（机械那边的要求）：SA = 使能 + 果篮（锁定开 / 使能关 / 掉线不动），SB 升降不变，SC = RS05 点动（拨上往正前方 / 中位停 / 拨下往果框上方），SD = 夹爪（拨上合、合上 700 ms 后剪线一直往复 / 拨下先停剪线再开），左摇杆上下（CH1）= 步进（推上前进 / 中间停 / 拉下后退，带回差、使能后回过中间才生效）。C 板改了 `Arm_Remote.c/h`（新映射、步进三区回差、果篮跟 SA，`rotate_basket` → `rotate_dir`）、`Arm_Rotate.c/h`（CSP 点动：两端就是软限位，回中停在当时的位置，停住的位置优先用类型 2 应答、对不上退回 mechPos；"使能后先停在原地"改成"这次使能后 SC 回过中位才动"，仍放在 Arm_Rotate 里）、`Arm_Node.c/h`（剪线开始 / 停止、果篮开 / 关、发送顺序、失能补发剪线停止）、`ArmNode_Command.h`（加 0x06~0x09）、`BareMetal.c/h`（一行参数和注释）。RS05 没改成速度模式：使能流程差不多长、位置保护一个都省不掉，C 板死机 / 调试器暂停时速度模式会一直转，CSP 最多走到行程一端。节点（`C:\Clion\SC09ServoTest`）：LD-2015 剪线从"上电自己往复"改成命令控制（`ld2015_auto.c/h` → `ld2015_cut.c/h`），新增果篮 `basket.c/h`（PA11 / TIM1_CH4），`node_app` 加四条命令；.ioc 里 TIM1 原来不全（按它生成会删掉 TIM1），补全 CH1、加 CH4 后用 CubeMX 6.15 重新生成，删掉 `stm32f1xx_it.c` USER CODE 里会重复定义的两个手写中断函数。剪刀张开位、果篮开 / 关脉宽都是占位值，等用户给。
- 构建（临时目录全量）：C 板 Debug 0 警告（FLASH 44344 B，RAM 4616 B），Release 0 警告（FLASH 25524 B）；节点 Debug 0 警告（FLASH 22972 B，RAM 3288 B），Release 0 警告（FLASH 12308 B）。
- 节点主机测试（GCC 13.1，`-Werror`）三组都过：九条帧的字节、剪线往复 / 回张开位 / 上电不出脉冲、果篮只动 CH4、PWM 起不来报驱动错误。
- C 板主机模拟：新写的节点模拟 14 个场景（果篮跟 SA、步进回差和回中、合上 700 ms 后剪、拨开先停剪线、锁定 / 掉线补发停止、重新使能不自己剪、发送失败重发等）、RS05 点动模拟（回中后往回退 0.015 rad；类型 2 故意偏 1 rad 时退回 mechPos、往回退 0.06 rad；受控运行全程在 −2.460 ~ 2.426 之间；使能 / RS05 重启时 SC 拨在一边不动）、升降模拟（结果和以前一样）、OMNI / MECANUM 都 PASS。反向检查 13 个（C 板 10 个：去回差、去回中、锁定不开果篮、去 700 ms、先开夹爪后停剪线、去补发剪线停止、RS05 去回中保护、不比对类型 2、只用 mechPos、回中不更新停住的位置；节点 3 个：上电就往复、停止不回张开位、果篮写错通道）全部被抓到。
- 地址：底盘四台电机、升降、RS05、`arm_node_*` 没变；`chassis_command` → 0x20000960，`arm_command` → 0x20000974（`rotate_dir` 在 +1），`Arm_Remote.c` 的 static 挪了，上面已按新 ELF 用 gdb 重算。
未烧录。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\remap-20261024-224528`（含 SC09ServoTest 的副本）。模拟源码在 `%TEMP%\fp_remap_sim`（`build.sh` 要把 CLion 的 `mingw/bin` 放在 PATH 最前面）。

2026-10-25 上板以后：用户说其余功能都正常，三处要改。① 底盘上坡上不去：固定档位低档 0.35 → 中档 0.65；电流指令限幅 3000 → 8000（≈ 9.8 A，接近 M3508 额定 10 A / 3 N·m，手册核过）；速度环积分项上限底盘单独 1000 → 6000（`motor_control.c/h` 新加 `MotorControl_SetSpeedIntegralLimit`，只新增函数，升降仍是默认 1000）；`Chassis_Control.c` 加了指令加速度限制（每拍 0.002，三个量同比例缩，0 → 中档推满约 0.33 s，失能时清零）。② 果篮脉宽：开 2500、关 1000（节点 `node_config.h`）。③ RS05 往果框那头差几十度：没改，等用户量（往果框那头再加超过约 0.5 rad 就会碰到 zero_sta = 1 的 +π 跳变点，要先量出实际要到的读数再定怎么改）。
- 构建：C 板 Debug 0 警告（FLASH 44768 B，RAM 4640 B），Release 0 警告（FLASH 25724 B）；节点 Debug / Release 0 警告（大小没变）。
- 验证：节点三组主机测试 PASS；升降模拟里的底盘段改成：限幅 8000 / 积分上限 6000 / 升降积分上限仍 1000、中档推满 ±1097 rpm、目标每拍涨 3~4 rpm 第 323 ms 到 1090 以上、轮子卡住（反馈 0）时电流顶到 ±8000、松杆一拍一拍降到 0、锁车失能后再使能从 0 加速，PASS；节点、RS05 两套模拟和 OMNI / MECANUM PASS；反向检查 2 个（去掉加速度限制、失能不清零）都被抓到。
- 地址：底盘四台电机没变，其余全部往后挪了 20 字节（上面四张表已按新 ELF 用 gdb 重算）。
未烧录。修改前的文件备份在 `C:\Project\Fruit-picking robot\backups\chassis-basket-20261025-012813`。

2026-10-25（之后）：用户说 RS05 往果框那头差的不止十几度，问能不能再大，随后自己说不用了、把果篮往左挪一点，所以 RS05 没改。升降按用户要求把限流从 6000 调到 8000（约 9.8 A，`Arm_Lift.c`），其余没动（积分上限仍是 1000）。C 板 Debug / Release 0 警告，大小和上一版一样，调试地址没变；升降模拟加了"顶住时输出到 ±8000"的断言，三套模拟 PASS。未烧录。备份在 `C:\Project\Fruit-picking robot\backups\lift-current-20261025-015354`。

2026-10-25（节点）：果篮脉宽按用户纠正是开 1000 / 关 2500（上面一条记录里的"开 2500、关 1000"是我理解反了）；夹爪闭合 360 → 380。节点后来加的"首次夹爪动作前同步舵机上限"把动作挡在了回包后面，实物上 SC09 完全不动；已改成同步只试一次、不挡动作、每次先开扭矩再移动（细节和上板怎么看 `g_node_app` 见节点 README 最后一条）。C 板这边只改了 `ArmNode_Command.h` 的注释（360 → 380），帧没变。
