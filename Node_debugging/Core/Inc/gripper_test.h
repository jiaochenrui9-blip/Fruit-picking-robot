#ifndef GRIPPER_TEST_H
#define GRIPPER_TEST_H

#include "servo.h"

/* Broadcast ID assignment requires exactly one servo on the bus. */
#ifndef SERVO_ID
#define SERVO_ID 1U
#endif
#ifndef SERVO_SET_ID_ON_STARTUP
#define SERVO_SET_ID_ON_STARTUP 1U
#endif
#define SERVO_READ_INTERVAL_MS 100U
#define SERVO_READ_TIMEOUT_MS 30U

/* First measure the mechanism with torque off, then enter safe endpoints. */
#ifndef SERVO_APPLY_ANGLE_LIMITS
#define SERVO_APPLY_ANGLE_LIMITS 0U
#endif
#ifndef SERVO_MIN_POSITION
#define SERVO_MIN_POSITION 0U
#endif
#ifndef SERVO_MAX_POSITION
#define SERVO_MAX_POSITION 0U
#endif

#if SERVO_ID >= 254U
#error "SERVO_ID must be a unicast ID (0..253)"
#endif
#if SERVO_APPLY_ANGLE_LIMITS && (SERVO_MIN_POSITION >= SERVO_MAX_POSITION || SERVO_MAX_POSITION > 1023U)
#error "Set measured SC09 limits: 0 <= min < max <= 1023"
#endif

typedef struct
{
    HAL_StatusTypeDef id_write_status; /* Broadcast TX status, not proof of ID. */
    uint8_t id_read_ok;
    uint8_t id_verified;               /* Readback matched SERVO_ID. */
    uint16_t position;                 /* Last successful position sample. */
    uint8_t position_ok;               /* Valid in the current polling cycle. */
    uint8_t range_valid;               /* At least one torque-off sample. */
    uint16_t measured_min;
    uint16_t measured_max;
    uint32_t read_count;
    uint32_t read_error_count;
    uint8_t torque_enabled;            /* Last successful register read. */
    uint8_t torque_read_ok;
    uint8_t torque_disabled;           /* Readback confirmed zero this cycle. */
    HAL_StatusTypeDef torque_write_status; /* ACK status, not proof of disable. */
    uint8_t limits_attempted;
    HAL_StatusTypeDef limits_write_status;
    uint8_t limits_read_ok;
    uint8_t limits_verified;           /* EEPROM values read back and matched. */
    uint16_t stored_min;
    uint16_t stored_max;
} Gripper_TestState;

extern volatile Gripper_TestState g_gripper_test;

void Gripper_TestInit(UART_HandleTypeDef *huart);
void Gripper_TestPoll(void);

#endif
