# Fruit-picking robot · 摘果机器人

基于 STM32 的摘果机器人固件工程，使用 STM32CubeMX、CMake、Ninja 和 GNU Arm Embedded 工具链。

## 工程目录

| 目录 | 硬件 | 用途 |
| --- | --- | --- |
| [`Fruit_Picking`](Fruit_Picking/README.md) | RoboMaster C 型开发板，STM32F407 | 机器人主控：ELRS 遥控、全向轮底盘、升降 M3508、旋转 RS05，以及机械臂节点指令 |
| [`Node_debugging`](Node_debugging/README.md) | STM32F103C8T6 | SC09 夹爪舵机 ID 配置、失能位置测量、电子角度限位配置 |

两个目录是独立固件工程，各自包含 CubeMX `.ioc`、HAL/CMSIS、启动文件、链接脚本和 CMake 配置。

`Node_debugging` 是夹爪测量与配置程序。主控 README 中提到的完整机械臂节点固件 `SC09ServoTest` 位于另一个工程中，本仓库未包含该固件；连接主控 USART6 的伸缩/夹爪控制需要配套节点固件及协议。

## 主控功能

- CAN1：4 台 M3508/C620 全向轮底盘，电调 ID 1～4 从右前开始顺时针排列。
- CAN2：M3508 升降速度控制、RS05 旋转位置控制。
- USART1：ELRS CRSF 遥控输入，420000 bps，DMA 接收。
- USART6：向机械臂 STM32 节点发送伸缩与夹爪二进制命令。
- 裸机主循环处理遥控与通信，TIM8 以 1 kHz 调度机构控制。

接线、遥控通道、参数、调试变量和上板验证步骤见各子工程 README。

## 编译

需要 CMake 3.22 或更新版本、Ninja，以及提供 `arm-none-eabi-gcc`、`arm-none-eabi-g++` 的 GNU Arm 工具链；先将这些工具加入 `PATH`。

在仓库根目录打开 PowerShell，编译主控：

```powershell
Set-Location Fruit_Picking
cmake --preset Debug
cmake --build --preset Debug
```

输出为 `Fruit_Picking/build/Debug/Fruit_Picking.elf`。在 `Node_debugging` 目录执行相同命令，输出为 `Node_debugging/build/Debug/Node_debugging.elf`。将两条命令中的 `Debug` 改为 `Release` 可构建发布配置。

仓库根目录用于组织工程，运行 CMake 时应进入对应子工程目录。

## 主机测试

安装主机版 GCC 后，在 `Fruit_Picking` 目录运行：

```powershell
New-Item -ItemType Directory -Force build/host | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -I UserModules/APP/Chassis Tests/OMNI_REMOTE_TEST.c -lm -o build/host/omni_test.exe
if ($LASTEXITCODE -ne 0) { throw 'omni test compilation failed' }
./build/host/omni_test.exe
if ($LASTEXITCODE -ne 0) { throw 'omni test failed' }
gcc -std=c11 -Wall -Wextra -Werror -I UserModules/APP/Chassis Tests/MECANUM_REMOTE_TEST.c -lm -o build/host/mecanum_test.exe
if ($LASTEXITCODE -ne 0) { throw 'mecanum test compilation failed' }
./build/host/mecanum_test.exe
if ($LASTEXITCODE -ne 0) { throw 'mecanum test failed' }
```

这些测试验证底盘速度解算。固件编译、主机测试、烧录及真实机构运行属于不同验证阶段；实物行为应按子工程中的上板步骤确认。调试变量地址随构建变化，以当前 ELF 为准。

## 本次发布验证（2026-10-23）

从待提交文件导出干净源码目录，独立验证结果：

- `Fruit_Picking`：Debug、Release 全量编译和链接通过，均为 0 警告。
- `Node_debugging`：Debug、Release 全量编译和链接通过，均为 0 警告。
- 主机 GCC 测试：`PASS: omni`、`PASS: mecanum`。

本次发布未烧录固件、未连接硬件测试；历史实测记录见各子工程 README。

## 仓库内容

保留源码、工程配置、测试与必要技术文档。`.gitignore` 排除 `note*.md` 笔记、本地备份、根目录参考手册 PDF、IDE 缓存和编译产物；这些本地文件无需删除。

第三方 HAL/CMSIS 等代码保留原有版权和许可说明；其适用条款以各目录中的许可文件及源文件声明为准。
