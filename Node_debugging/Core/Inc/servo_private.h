#ifndef SC09SERVOTEST_SERVO_PRIVATE_H
#define SC09SERVOTEST_SERVO_PRIVATE_H

#include "servo.h"

#define SERVO_BROADCAST_ID 0xFEU
#define SERVO_MAX_PARAM_LEN 122U
#define SERVO_FRAME_MAX_LEN 128U
#define SERVO_DEFAULT_TIMEOUT_MS 100U
#define SERVO_VERSION_ADDR 0x03U
#define SERVO_ID_ADDR 0x05U
#define SERVO_MIN_ANGLE_LIMIT_ADDR 0x09U
#define SERVO_TORQUE_ENABLE_ADDR 0x28U
#define SERVO_GOAL_POSITION_ADDR 0x2AU
#define SERVO_GOAL_TIME_ADDR 0x2CU
#define SERVO_LOCK_ADDR 0x30U
#define SERVO_PRESENT_POSITION_ADDR 0x38U
#define SERVO_PRESENT_SPEED_ADDR 0x3AU
#define SERVO_PRESENT_LOAD_ADDR 0x3CU
#define SERVO_PRESENT_VOLTAGE_ADDR 0x3EU
#define SERVO_PRESENT_TEMPERATURE_ADDR 0x3FU
#define SERVO_MOVING_ADDR 0x42U
#define SERVO_PRESENT_CURRENT_ADDR 0x46U
#define SERVO_SYNC_MOVE_DATA_LEN 7U
#define SERVO_SYNC_MOVE_MAX_COUNT ((SERVO_MAX_PARAM_LEN - 2U) / SERVO_SYNC_MOVE_DATA_LEN)

HAL_StatusTypeDef Servo_SendRawPacket(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t instruction,
                                      const uint8_t *params, uint8_t param_len);
HAL_StatusTypeDef Servo_ReceiveRawReply(Servo_ManagerTypeDef *manager, uint8_t *rx,
                                        uint16_t *rx_len, uint16_t rx_max,
                                        uint32_t timeout_ms);
HAL_StatusTypeDef Servo_ValidateFrame(uint8_t expected_id, const uint8_t *reply,
                                      uint16_t reply_len);
HAL_StatusTypeDef Servo_ValidateReply(uint8_t expected_id, const uint8_t *reply,
                                      uint16_t reply_len);
void Servo_ClearRx(Servo_ManagerTypeDef *manager);
uint16_t Servo_MakeWord(uint8_t high, uint8_t low);
int16_t Servo_MakeSignedWord(uint16_t value, uint8_t sign_bit);

#endif /* SC09SERVOTEST_SERVO_PRIVATE_H */
