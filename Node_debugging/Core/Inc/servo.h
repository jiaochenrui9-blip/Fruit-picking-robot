#ifndef SC09SERVOTEST_SERVO_H
#define SC09SERVOTEST_SERVO_H

#include "main.h"

#define SERVO_MAX_MOTORS 16U

typedef struct
{
  uint16_t position;
  int16_t speed;
  int16_t load;
  uint8_t voltage;
  uint8_t temperature;
  uint8_t moving;
  int16_t current;
} Servo_Feedback_t;

typedef struct
{
  uint8_t device_id;
  uint16_t version;
  uint16_t min_position;
  uint16_t max_position;
  uint8_t mode;
  Servo_Feedback_t feedback;
} Servo_MotorTypeDef;

typedef struct
{
  UART_HandleTypeDef *huart;
  Servo_MotorTypeDef motors[SERVO_MAX_MOTORS];
  uint8_t motor_count;
} Servo_ManagerTypeDef;

void Servo_ManagerInit(Servo_ManagerTypeDef *manager, UART_HandleTypeDef *huart);
uint8_t Servo_RegisterMotor(Servo_ManagerTypeDef *manager, uint8_t device_id);
Servo_MotorTypeDef *Servo_FindMotor(Servo_ManagerTypeDef *manager, uint8_t device_id);

HAL_StatusTypeDef Servo_SendPacket(Servo_ManagerTypeDef *manager,
                                   uint8_t id,
                                   uint8_t instruction,
                                   const uint8_t *params,
                                   uint8_t param_len,
                                   uint8_t *rx,
                                   uint16_t *rx_len,
                                   uint16_t rx_max,
                                   uint32_t timeout_ms);
uint8_t Servo_ReadValue(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                        uint8_t *data, uint8_t data_len, uint32_t timeout);
uint8_t Servo_GetByte(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                      uint8_t *value, uint32_t timeout);
uint8_t Servo_GetWord(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                      uint16_t *value, uint32_t timeout);

uint8_t Servo_GetID(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetVersion(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetPosition(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetSpeed(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetLoad(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetVoltage(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetTemperature(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetMoving(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetCurrent(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetAngleLimit(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetMode(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
uint8_t Servo_GetFeedback(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);

HAL_StatusTypeDef Servo_Ping(Servo_ManagerTypeDef *manager, uint8_t id);
uint8_t Servo_PingCheck(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout);
HAL_StatusTypeDef Servo_ReadData(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address, uint8_t read_len);
HAL_StatusTypeDef Servo_WriteData(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                                  const uint8_t *data, uint8_t data_len);
HAL_StatusTypeDef Servo_RegWrite(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                                 const uint8_t *data, uint8_t data_len);
HAL_StatusTypeDef Servo_Action(Servo_ManagerTypeDef *manager);
HAL_StatusTypeDef Servo_SyncRead(Servo_ManagerTypeDef *manager, uint8_t address, uint8_t read_len,
                                 const uint8_t *ids, uint8_t id_count);
HAL_StatusTypeDef Servo_SyncWrite(Servo_ManagerTypeDef *manager, uint8_t address, uint8_t data_len,
                                  const uint8_t *data, uint8_t servo_count);
HAL_StatusTypeDef Servo_Reset(Servo_ManagerTypeDef *manager, uint8_t id);
HAL_StatusTypeDef Servo_SetID(Servo_ManagerTypeDef *manager, uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef Servo_SetID_Broadcast(Servo_ManagerTypeDef *manager, uint8_t new_id);
HAL_StatusTypeDef Servo_EnableTorque(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t enable);
HAL_StatusTypeDef Servo_UnlockEprom(Servo_ManagerTypeDef *manager, uint8_t id);
HAL_StatusTypeDef Servo_LockEprom(Servo_ManagerTypeDef *manager, uint8_t id);
HAL_StatusTypeDef Servo_SetAngleLimit(Servo_ManagerTypeDef *manager, uint8_t id,
                                      uint16_t min_position, uint16_t max_position);
HAL_StatusTypeDef Servo_PositionMode(Servo_ManagerTypeDef *manager, uint8_t id);
HAL_StatusTypeDef Servo_PWMMode(Servo_ManagerTypeDef *manager, uint8_t id);
HAL_StatusTypeDef Servo_WritePWM(Servo_ManagerTypeDef *manager, uint8_t id, int16_t pwm);
HAL_StatusTypeDef Servo_Move(Servo_ManagerTypeDef *manager, uint8_t id,
                             uint16_t position, uint16_t time, uint16_t speed);
HAL_StatusTypeDef Servo_SyncMove(Servo_ManagerTypeDef *manager, const uint8_t *ids, uint8_t servo_count,
                                 uint16_t position, uint16_t time, uint16_t speed);
HAL_StatusTypeDef Servo_SyncMoveList(Servo_ManagerTypeDef *manager, const uint8_t *ids,
                                     const uint16_t *positions, uint8_t servo_count,
                                     uint16_t time, uint16_t speed);
HAL_StatusTypeDef Servo_SyncMove123(Servo_ManagerTypeDef *manager, uint16_t position,
                                    uint16_t time, uint16_t speed);

#endif /* SC09SERVOTEST_SERVO_H */
