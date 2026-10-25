#ifndef ARMNODE_COMMAND_H
#define ARMNODE_COMMAND_H

/*
 * 机械臂 STM32 节点（C:\Clion\SC09ServoTest）的命令表。
 * C 板 USART6（3-pin 口，115200 8N1）↔ 节点 USART2，帧格式见 Binary_Protocol.h：
 *   AA 55 | 地址 | 命令 | 长度（小端）| 数据 | CRC16/MODBUS（小端）
 * 九条命令都不带数据（长度 0），和节点 node_app.h 里的 Node_Command 一一对应，改的话两边一起改。
 * 节点收到一条就执行（步进、夹爪要等驱动器回包，会阻塞；剪线、果篮只改 PWM），不回 ACK，也没有掉线超时。
 */
#define ARM_NODE_ADDRESS                0x01U

#define ARM_NODE_CMD_SERVO_OPEN         0x01U   /* 夹爪打开（舵机目标 270） */
#define ARM_NODE_CMD_SERVO_CLOSE        0x02U   /* 夹爪闭合（舵机目标 380，2026-10-25 起） */
#define ARM_NODE_CMD_STEPPER_FORWARD    0x03U   /* 步进正转 20 rpm（通信速度模式） */
#define ARM_NODE_CMD_STEPPER_STOP       0x04U   /* 步进停止（节点里先 0xFC 立即停止，回包通过后 0xFB 清状态） */
#define ARM_NODE_CMD_STEPPER_REVERSE    0x05U   /* 步进反转 20 rpm */
#define ARM_NODE_CMD_CUT_START          0x06U   /* 剪线开始：LD-2015 在两端之间一直往复 */
#define ARM_NODE_CMD_CUT_STOP           0x07U   /* 剪线停止：回到剪刀张开的位置停住 */
#define ARM_NODE_CMD_BASKET_OPEN        0x08U   /* 果篮打开 */
#define ARM_NODE_CMD_BASKET_CLOSE       0x09U   /* 果篮关闭 */

#endif
