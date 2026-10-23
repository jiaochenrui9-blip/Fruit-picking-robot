cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Fruit_Picking)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # BSP：CAN 收发（大疆电机和 RS05 共用）
    UserModules/BSP/can_bus.c
    # Algorithm
    UserModules/Algorithm/PID.c
    # Devices/DJI_Motor：M3508 / M2006 / GM6020 + 电流/速度/位置控制
    UserModules/Devices/DJI_Motor/dji_motor.c
    UserModules/Devices/DJI_Motor/dji_motor_manager.c
    UserModules/Devices/DJI_Motor/dji_m3508.c
    UserModules/Devices/DJI_Motor/dji_m2006.c
    UserModules/Devices/DJI_Motor/dji_gm6020.c
    UserModules/Devices/DJI_Motor/motor_control.c
    # Devices/RS05：私有协议（扩展帧）
    UserModules/Devices/RS05/RS05_app.c
    UserModules/Devices/RS05/RS05_codec.c
    UserModules/Devices/RS05/RS05_parameter.c
    UserModules/Devices/RS05/RS05_receive.c
    # Devices/ELRS：CRSF 解析 + USART1 DMA 空闲接收
    UserModules/Devices/ELRS/elrs_crsf.c
    UserModules/Devices/ELRS/ELRS_DMA.c
    # APP/Chassis：旧 ELRS 工程的底盘遥控 + 全向轮/麦轮速度解算
    UserModules/APP/Chassis/Chassis_Remote.c
    UserModules/APP/Chassis/Omni_Remote.c
    UserModules/APP/Chassis/Mecanum_Remote.c
    # 机构（2026-10-23 从 BareMetal.c 拆出来）：底盘电机 / 升降 / 旋转 RS05 / 机械臂节点，由 BareMetal 调度。
    # Chassis_Control.c 故意排在拆分前 BareMetal.c 的位置：变量的 RAM 地址按这里的顺序排，
    # 这样底盘四台电机的地址和拆分前一样（README 里 VOFA 记的底盘地址不用改）；其余地址以 README 为准。
    UserModules/APP/Chassis/Chassis_Control.c
    UserModules/APP/Arm/Arm_Lift.c
    UserModules/APP/Arm/Arm_Rotate.c
    UserModules/APP/Arm/Arm_Node.c
    # Task：裸机调度（沿用 Chassis_BareMetal：主循环遥控 + TIM8 1kHz 控制）
    UserModules/Task/BareMetal.c
    # APP/Arm：机械臂拨杆（SB 升降、SD 旋转、SC 伸缩，左摇杆上下夹爪）
    UserModules/APP/Arm/Arm_Remote.c
    # Devices/ArmNode：和机械臂 STM32 节点的二进制帧协议（USART6，拷自 C:\Clion\SC09ServoTest，
    # 只改了 rx_calculated_crc 的初值，见文件里的注释：初值 0xFFFF 会进 .data，把后面所有变量地址挤后 4 字节）。
    UserModules/Devices/ArmNode/Binary_Protocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    UserModules/BSP
    UserModules/Algorithm
    UserModules/Devices/DJI_Motor
    UserModules/Devices/RS05
    UserModules/Devices/ELRS
    UserModules/Devices/ArmNode
    UserModules/APP/Chassis
    UserModules/APP/Arm
    UserModules/Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
