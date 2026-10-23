# 夹爪 SC09 舵机位置区间测试

从 `C:\Clion\SC09ServoTest` 移植 `servo.c`、`servo_read.c`、`servo_control.c` 及对应头文件，保留原驱动接口和数据字节顺序。目标芯片为 STM32F103C8T6。

## 当前行为

上电初始化后等待 100 ms，调用 `Servo_SetID_Broadcast(&ServoManager, SERVO_ID)` 广播设置 ID 为 1，读回 ID 后向 ID 1 发送扭矩失能命令，并读取扭矩使能寄存器确认。主循环约每 100 ms 检查扭矩状态、读取当前位置；失能未确认时会重新发送失能命令。程序不会调用运动、PWM 输出或扭矩使能操作。

只有本轮确认失能且位置读取成功，才更新测量区间。位置读取失败时保留上次值，同时将 `position_ok` 清零并累计错误次数。不要把保留值当成本轮新位置。

## 串口与接线

| 项目 | 配置 |
| --- | --- |
| USART1_TX | PA9，接原方案舵机总线转换板的 MCU 侧 RX |
| USART1_RX | PA10，接转换板的 MCU 侧 TX |
| 通信格式 | 1,000,000 bps，8N1，轮询收发，无中断/DMA |
| 调试下载 | PA13 SWDIO、PA14 SWCLK、GND |
| 时钟 | HSI / 2 × 16 = 64 MHz；APB1 32 MHz，APB2 64 MHz |

沿用原工程的双线 UART 接口方式，经原来的串口转舵机单总线电路接舵机，MCU 与转换板共地。当前程序没有配置单引脚半双工或方向控制 GPIO；请按实际转换板接线。舵机使用适配其型号的独立电源。

目标工程原来使用 HSI 8 MHz，无法满足 UART 16 倍过采样下的 1 Mbps。本次使用内部 HSI 的 PLL 升至 64 MHz，无需假定板上有外部晶振。已同步修改 `.ioc`、HAL UART 开关、MSP 和 CMake。

## ID 与测量

在 `Core/Inc/gripper_test.h` 修改：

```c
#define SERVO_ID 1U
#define SERVO_SET_ID_ON_STARTUP 1U
```

默认每次启动调用一次现有函数 `Servo_SetID_Broadcast`：广播解锁 EEPROM → 广播写入 ID 1 → 向新 ID 1 发送重新锁定命令，再读取 ID 寄存器验证。无需知道原 ID。广播写入会影响总线上的所有舵机，因此设置时总线上只接一只舵机。

在 `g_gripper_test.id_verified == 1` 后，可将 `SERVO_SET_ID_ON_STARTUP` 改为 `0U` 并重新编译下载，后续直接使用 ID 1，避免每次启动重复写 EEPROM。设置步骤只在初始化调用，不会在主循环重试。启动时读回失败则 `id_verified` 为 0；之后仍尝试向 ID 1 发送失能并读取位置。

下载运行后，在调试器 Watch 中展开全局变量 `g_gripper_test`：

| 字段 | 含义 |
| --- | --- |
| `id_write_status` | 广播改 ID 函数的发送结果；HAL_OK 本身不能证明改 ID 成功 |
| `id_read_ok` / `id_verified` | 启动时 ID 寄存器读取成功 / 读回值与 `SERVO_ID` 一致 |
| `position` / `position_ok` | 最近成功的位置原始计数 / 本轮位置读取是否成功 |
| `torque_disabled` | 本轮已读回扭矩寄存器为 0；只有值为 1 才确认失能 |
| `torque_read_ok` / `torque_enabled` | 扭矩寄存器读取结果 / 最近成功的寄存器值 |
| `torque_write_status` | 失能写命令的应答状态；HAL_OK 本身不能证明失能 |
| `range_valid` | 是否至少采到一次已失能的位置 |
| `measured_min` / `measured_max` | 本次运行手动活动时采到的最小、最大位置 |
| `read_count` / `read_error_count` | 成功、失败的位置读取次数 |
| `limits_attempted` / `limits_verified` | 是否尝试写限位 / 写入及读回核对是否成功 |
| `limits_write_status` / `limits_read_ok` | 限位写入应答状态 / 限位读回结果 |
| `stored_min` / `stored_max` | 限位配置后回读的 EEPROM 数值 |

确认 `torque_disabled == 1` 且 `position_ok == 1` 后，缓慢将夹爪活动到可用行程两端，记录计数。重新启动 MCU 会清空已测区间。调试器暂停或断开通信后，变量只是此前快照，需恢复运行并重新确认状态。

## 设置电子角度限位

默认不写角度限位 EEPROM（启动广播改 ID 会写 ID 配置）：

```c
#define SERVO_APPLY_ANGLE_LIMITS 0U
#define SERVO_MIN_POSITION 0U
#define SERVO_MAX_POSITION 0U
```

测量完成后，在机械端点内留出余量，将最小、最大计数填入宏，再把 `SERVO_APPLY_ANGLE_LIMITS` 设为 `1U`，重新编译下载。代码检查 `min < max <= 1023`（沿用原 SC09 驱动的计数范围），不能直接使用尚未测量的 0/0。旧工程中的 190/300 没有作为本夹爪的默认限位。

每次 MCU 启动后，只在失能已确认时尝试一次“解锁 EEPROM → 写上下限 → 重新锁定 → 回读核对”，失败不在主循环反复写入。`limits_verified == 1` 才表示写入应答和回读数值均通过。写入失败也会尝试重新锁定 EEPROM。配置确认后将写入开关恢复为 0，避免后续启动重复写 EEPROM。

本测试仍保持舵机失能。电子角度限位用于舵机后续的位置控制，不会在失能时阻止手动活动，也不能替代机械限位。实际机械安全区间及控制时限位效果需要接板确认。

## 编译与验证

在本工程目录执行（环境中需有 CMake、Ninja、arm-none-eabi 工具链）：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出：`build/Debug/Node_debugging.elf`。

2026-10-22 验证记录：

- ARM Debug 固件编译、链接通过。
- 广播设置 ID 已接入初始化：模拟原 ID 10 改为 ID 1、重新锁定及 ID 寄存器回读通过，确认主循环不重复广播改 ID。
- 使用真实移植的驱动源文件和应用源文件，配合主机模拟 UART/舵机寄存器，默认测量配置、开启限位配置均通过 `-Wall -Wextra -Werror` 编译和断言测试。
- 覆盖失能寄存器确认、写应答缺失、失能未生效、采样区间、位置校验和/ID/状态/长度错误、超时、断线恢复、计时器回绕、噪声下有界超时、限位回读不匹配、写失败后重新锁定，以及不发送运动或使能命令。
- 未连接可识别的调试器，尚未烧录；没有真实舵机位置、扭矩状态或机械限位测试结果。

驱动局部修正：收包使用总超时，清 RX 有时间上限，避免噪声使轮询无限等待；调整校验和比较以通过主机编译检查；限位写失败也尝试锁回 EEPROM。原 `SC09ServoTest` 工程未修改。
