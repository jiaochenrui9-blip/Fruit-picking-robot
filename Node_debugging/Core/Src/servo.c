#include "servo_private.h"

void Servo_ManagerInit(Servo_ManagerTypeDef *manager, UART_HandleTypeDef *huart)
{
    if (manager == NULL)
    {
        return;
    }

    manager->huart = huart;
    manager->motor_count = 0U;
    for (uint8_t i = 0U; i < SERVO_MAX_MOTORS; i++)
    {
        manager->motors[i] = (Servo_MotorTypeDef){0};
    }
}

uint8_t Servo_RegisterMotor(Servo_ManagerTypeDef *manager, uint8_t device_id)
{
    Servo_MotorTypeDef *motor;

    if (manager == NULL || device_id >= SERVO_BROADCAST_ID ||
        manager->motor_count >= SERVO_MAX_MOTORS || Servo_FindMotor(manager, device_id) != NULL)
    {
        return 0U;
    }

    motor = &manager->motors[manager->motor_count];
    *motor = (Servo_MotorTypeDef){0};
    motor->device_id = device_id;
    manager->motor_count++;
    return 1U;
}

Servo_MotorTypeDef *Servo_FindMotor(Servo_ManagerTypeDef *manager, uint8_t device_id)
{
    if (manager == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < manager->motor_count; i++)
    {
        if (manager->motors[i].device_id == device_id)
        {
            return &manager->motors[i];
        }
    }

    return NULL;
}

HAL_StatusTypeDef Servo_SendPacket(Servo_ManagerTypeDef *manager,
                                   uint8_t id,
                                   uint8_t instruction,
                                   const uint8_t *params,
                                   uint8_t param_len,
                                   uint8_t *rx,
                                   uint16_t *rx_len,
                                   uint16_t rx_max,
                                   uint32_t timeout_ms)
{
    uint8_t ack[SERVO_FRAME_MAX_LEN];
    uint8_t *target_rx;
    uint16_t local_rx_len;
    uint16_t *target_rx_len;
    uint16_t target_rx_max;
    HAL_StatusTypeDef status;

    if (id != SERVO_BROADCAST_ID && Servo_FindMotor(manager, id) == NULL)
    {
        return HAL_ERROR;
    }

    Servo_ClearRx(manager);
    status = Servo_SendRawPacket(manager, id, instruction, params, param_len);
    if (status != HAL_OK || id == SERVO_BROADCAST_ID)
    {
        return status;
    }

    if (rx != NULL && rx_len != NULL && rx_max > 0U)
    {
        target_rx = rx;
        target_rx_len = rx_len;
        target_rx_max = rx_max;
    }
    else
    {
        target_rx = ack;
        target_rx_len = &local_rx_len;
        target_rx_max = sizeof(ack);
    }

    status = Servo_ReceiveRawReply(manager, target_rx, target_rx_len, target_rx_max, timeout_ms);
    if (status != HAL_OK)
    {
        return status;
    }

    return Servo_ValidateReply(id, target_rx, *target_rx_len);
}

HAL_StatusTypeDef Servo_SendRawPacket(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t instruction,
                                      const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[SERVO_FRAME_MAX_LEN];
    uint8_t length;
    uint8_t sum;

    if (manager == NULL || manager->huart == NULL || param_len > SERVO_MAX_PARAM_LEN ||
        (params == NULL && param_len != 0U))
    {
        return HAL_ERROR;
    }

    length = (uint8_t)(param_len + 2U);
    buf[0] = 0xFFU;
    buf[1] = 0xFFU;
    buf[2] = id;
    buf[3] = length;
    buf[4] = instruction;
    sum = (uint8_t)(id + length + instruction);

    for (uint8_t i = 0U; i < param_len; i++)
    {
        buf[5U + i] = params[i];
        sum = (uint8_t)(sum + params[i]);
    }

    buf[5U + param_len] = (uint8_t)~sum;
    return HAL_UART_Transmit(manager->huart, buf, (uint16_t)(param_len + 6U), 1000U);
}

HAL_StatusTypeDef Servo_ReceiveRawReply(Servo_ManagerTypeDef *manager, uint8_t *rx,
                                        uint16_t *rx_len, uint16_t rx_max,
                                        uint32_t timeout_ms)
{
    uint8_t byte;
    uint16_t expected_len = 0U;
    uint32_t start_tick;
    uint32_t timeout;

    if (manager == NULL || manager->huart == NULL || rx == NULL || rx_len == NULL || rx_max < 6U)
    {
        return HAL_ERROR;
    }

    *rx_len = 0U;
    timeout = timeout_ms > 0U ? timeout_ms : SERVO_DEFAULT_TIMEOUT_MS;
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout && *rx_len < rx_max)
    {
        if (HAL_UART_Receive(manager->huart, &byte, 1U, 5U) != HAL_OK)
        {
            continue;
        }

        if (*rx_len == 0U)
        {
            if (byte == 0xFFU)
            {
                rx[(*rx_len)++] = byte;
            }
            continue;
        }
        if (*rx_len == 1U)
        {
            if (byte == 0xFFU)
            {
                rx[(*rx_len)++] = byte;
            }
            else
            {
                *rx_len = 0U;
            }
            continue;
        }

        rx[(*rx_len)++] = byte;
        if (*rx_len == 4U)
        {
            expected_len = (uint16_t)(rx[3] + 4U);
            if (expected_len > rx_max || expected_len < 6U)
            {
                return HAL_ERROR;
            }
        }
        if (expected_len > 0U && *rx_len == expected_len)
        {
            return HAL_OK;
        }
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef Servo_ValidateFrame(uint8_t expected_id, const uint8_t *reply,
                                      uint16_t reply_len)
{
    uint8_t sum;

    if (reply == NULL || reply_len < 6U || reply[0] != 0xFFU || reply[1] != 0xFFU ||
        reply_len != (uint16_t)(reply[3] + 4U) ||
        (expected_id != SERVO_BROADCAST_ID && reply[2] != expected_id))
    {
        return HAL_ERROR;
    }

    sum = (uint8_t)(reply[2] + reply[3]);
    for (uint16_t i = 4U; i < reply_len - 1U; i++)
    {
        sum = (uint8_t)(sum + reply[i]);
    }

    sum = (uint8_t)~sum;
    return sum == reply[reply_len - 1U] ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Servo_ValidateReply(uint8_t expected_id, const uint8_t *reply,
                                      uint16_t reply_len)
{
    if (Servo_ValidateFrame(expected_id, reply, reply_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return reply[4] == 0U ? HAL_OK : HAL_ERROR;
}

void Servo_ClearRx(Servo_ManagerTypeDef *manager)
{
    uint8_t data;
    uint32_t start_tick;

    if (manager == NULL || manager->huart == NULL)
    {
        return;
    }
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < 2U &&
           HAL_UART_Receive(manager->huart, &data, 1U, 1U) == HAL_OK)
    {
    }
}
