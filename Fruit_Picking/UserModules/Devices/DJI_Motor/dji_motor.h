#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "main.h"
#include "PID.h"

#define DJI_MOTOR_COUNT 8U

typedef enum
{
    DJI_MOTOR_M3508 = 0,
    DJI_MOTOR_M2006,
    DJI_MOTOR_GM6020
} DJI_MotorType_e;

typedef enum
{
    MOTOR_MODE_DISABLE = 0,
    MOTOR_MODE_CURRENT,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION
} MotorControlMode_e;

typedef struct
{
    uint16_t angle;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    int32_t total_encoder;
} DJI_MotorFeedback_t;

typedef struct
{
    float target_rpm;
    PID_t pid;
} MotorSpeedControl_t;

typedef struct
{
    float target_deg;
    PID_t pid;
    int32_t zero_encoder;
    uint32_t last_update_tick;
    uint8_t reference_ready;
} MotorPositionControl_t;

typedef struct
{
    DJI_MotorType_e type;
    MotorControlMode_e mode;
    uint8_t esc_id;
    uint16_t feedback_id;

    uint16_t tx_id;    // 0x200、0x1FF、0x1FE……
    uint8_t tx_slot;   // 0~3
    uint8_t enable;

    float reduction_ratio;
    int16_t current_limit;

    int8_t motor_direction;     // 电机方向：1或-1，统一作用于输出和反馈

    volatile uint16_t encoder;
    volatile int16_t speed_rpm;
    volatile int16_t feedback_current;
    volatile uint8_t temperature;
    volatile int16_t last_encoder;
    volatile int32_t total_encoder;

    MotorSpeedControl_t speed;
    MotorPositionControl_t position;

    int16_t target_current;
    int16_t output_current;

    volatile uint8_t feedback_updated;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_tick;
} DJI_Motor_t;

HAL_StatusTypeDef DJI_Motor_SetCurrent(DJI_Motor_t *motor,
                                       int16_t target_current);

uint8_t DJI_Motor_GetFeedback(DJI_Motor_t *motor,
                              DJI_MotorFeedback_t *feedback);

uint8_t DJI_Motor_IsOnline(const DJI_Motor_t *motor,
                           uint32_t now_tick,
                           uint32_t timeout_ms);

#endif /* DJI_MOTOR_H */
