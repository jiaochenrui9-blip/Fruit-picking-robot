#include "gripper_test.h"
#include "servo_private.h"

volatile Gripper_TestState g_gripper_test;
static Servo_ManagerTypeDef ServoManager;
static uint32_t position_read_tick;

static void Gripper_ReadTorque(void)
{
    uint8_t enabled;
    g_gripper_test.torque_read_ok = Servo_GetByte(&ServoManager, SERVO_ID,
        SERVO_TORQUE_ENABLE_ADDR, &enabled, SERVO_READ_TIMEOUT_MS);
    g_gripper_test.torque_disabled = 0U;
    if (g_gripper_test.torque_read_ok)
    {
        g_gripper_test.torque_enabled = enabled;
        g_gripper_test.torque_disabled = (enabled == 0U);
    }
}

static void Gripper_DisableTorque(void)
{
    g_gripper_test.torque_write_status = Servo_EnableTorque(&ServoManager, SERVO_ID, 0U);
    /* A missing write ACK may still mean the write took effect. Verify it. */
    Gripper_ReadTorque();
}

void Gripper_TestInit(UART_HandleTypeDef *huart)
{
    g_gripper_test = (Gripper_TestState){
        .id_write_status = HAL_ERROR,
        .torque_enabled = 0xFFU,
        .torque_write_status = HAL_ERROR,
        .limits_write_status = HAL_ERROR
    };
    Servo_ManagerInit(&ServoManager, huart);
    if (!Servo_RegisterMotor(&ServoManager, SERVO_ID))
    {
        Error_Handler();
        return;
    }
    HAL_Delay(100U);
#if SERVO_SET_ID_ON_STARTUP
    g_gripper_test.id_write_status = Servo_SetID_Broadcast(&ServoManager, SERVO_ID);
#endif
    uint8_t device_id = 0xFFU;
    g_gripper_test.id_read_ok = Servo_GetByte(&ServoManager, SERVO_ID,
        SERVO_ID_ADDR, &device_id, SERVO_READ_TIMEOUT_MS);
    g_gripper_test.id_verified = (g_gripper_test.id_read_ok && device_id == SERVO_ID);
    Gripper_DisableTorque();
    /* Read position on the first poll, then once per interval. */
    position_read_tick = HAL_GetTick() - SERVO_READ_INTERVAL_MS;
}

void Gripper_TestPoll(void)
{
    if ((uint32_t)(HAL_GetTick() - position_read_tick) < SERVO_READ_INTERVAL_MS)
    {
        return;
    }
    position_read_tick = HAL_GetTick();

    Gripper_ReadTorque();
    if (!g_gripper_test.torque_disabled)
    {
        Gripper_DisableTorque();
    }

    g_gripper_test.position_ok = Servo_GetPosition(&ServoManager, SERVO_ID, SERVO_READ_TIMEOUT_MS);
    if (g_gripper_test.position_ok)
    {
        g_gripper_test.position = ServoManager.motors[0].feedback.position;
        g_gripper_test.read_count++;
        if (g_gripper_test.torque_disabled)
        {
            if (!g_gripper_test.range_valid)
            {
                g_gripper_test.measured_min = g_gripper_test.position;
                g_gripper_test.measured_max = g_gripper_test.position;
                g_gripper_test.range_valid = 1U;
            }
            else
            {
                if (g_gripper_test.position < g_gripper_test.measured_min)
                    g_gripper_test.measured_min = g_gripper_test.position;
                if (g_gripper_test.position > g_gripper_test.measured_max)
                    g_gripper_test.measured_max = g_gripper_test.position;
            }
        }
    }
    else
    {
        /* Keep the previous position; position_ok marks it as stale. */
        g_gripper_test.read_error_count++;
    }

#if SERVO_APPLY_ANGLE_LIMITS
    if (g_gripper_test.torque_disabled && !g_gripper_test.limits_attempted)
    {
        g_gripper_test.limits_attempted = 1U;
        g_gripper_test.limits_write_status = Servo_SetAngleLimit(&ServoManager,
            SERVO_ID, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
        g_gripper_test.limits_read_ok = Servo_GetAngleLimit(&ServoManager,
            SERVO_ID, SERVO_READ_TIMEOUT_MS);
        if (g_gripper_test.limits_read_ok)
        {
            g_gripper_test.stored_min = ServoManager.motors[0].min_position;
            g_gripper_test.stored_max = ServoManager.motors[0].max_position;
            g_gripper_test.limits_verified =
                (g_gripper_test.limits_write_status == HAL_OK &&
                 g_gripper_test.stored_min == SERVO_MIN_POSITION &&
                 g_gripper_test.stored_max == SERVO_MAX_POSITION);
        }
    }
#endif
}
