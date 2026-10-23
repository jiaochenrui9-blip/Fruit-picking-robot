#include "servo_private.h"

HAL_StatusTypeDef Servo_Ping(Servo_ManagerTypeDef *manager, uint8_t id)
{
    return Servo_SendPacket(manager, id, 0x01U, NULL, 0U, NULL, NULL, 0U, 0U);
}

uint8_t Servo_PingCheck(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    if (Servo_FindMotor(manager, id) == NULL)
    {
        return 0U;
    }

    return Servo_SendPacket(manager, id, 0x01U, NULL, 0U, NULL, NULL, 0U, timeout) == HAL_OK;
}

HAL_StatusTypeDef Servo_WriteData(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                                  const uint8_t *data, uint8_t data_len)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];

    if (data_len + 1U > SERVO_MAX_PARAM_LEN || (data == NULL && data_len != 0U))
    {
        return HAL_ERROR;
    }
    params[0] = address;
    for (uint8_t i = 0U; i < data_len; i++)
    {
        params[i + 1U] = data[i];
    }
    return Servo_SendPacket(manager, id, 0x03U, params, (uint8_t)(data_len + 1U), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef Servo_RegWrite(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                                 const uint8_t *data, uint8_t data_len)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];

    if (data_len + 1U > SERVO_MAX_PARAM_LEN || (data == NULL && data_len != 0U))
    {
        return HAL_ERROR;
    }
    params[0] = address;
    for (uint8_t i = 0U; i < data_len; i++)
    {
        params[i + 1U] = data[i];
    }
    return Servo_SendPacket(manager, id, 0x04U, params, (uint8_t)(data_len + 1U), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef Servo_Action(Servo_ManagerTypeDef *manager)
{
    return Servo_SendRawPacket(manager, SERVO_BROADCAST_ID, 0x05U, NULL, 0U);
}

HAL_StatusTypeDef Servo_SyncRead(Servo_ManagerTypeDef *manager, uint8_t address, uint8_t read_len,
                                 const uint8_t *ids, uint8_t id_count)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];
    uint16_t param_len = (uint16_t)id_count + 2U;

    if (ids == NULL || param_len > SERVO_MAX_PARAM_LEN)
    {
        return HAL_ERROR;
    }
    params[0] = address;
    params[1] = read_len;
    for (uint8_t i = 0U; i < id_count; i++)
    {
        params[i + 2U] = ids[i];
    }
    return Servo_SendRawPacket(manager, SERVO_BROADCAST_ID, 0x82U, params, (uint8_t)param_len);
}

HAL_StatusTypeDef Servo_SyncWrite(Servo_ManagerTypeDef *manager, uint8_t address, uint8_t data_len,
                                  const uint8_t *data, uint8_t servo_count)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];
    uint16_t data_total_len = (uint16_t)servo_count * ((uint16_t)data_len + 1U);
    uint16_t param_len = data_total_len + 2U;

    if (data == NULL || param_len > SERVO_MAX_PARAM_LEN)
    {
        return HAL_ERROR;
    }
    params[0] = address;
    params[1] = data_len;
    for (uint16_t i = 0U; i < data_total_len; i++)
    {
        params[i + 2U] = data[i];
    }
    return Servo_SendRawPacket(manager, SERVO_BROADCAST_ID, 0x83U, params, (uint8_t)param_len);
}

HAL_StatusTypeDef Servo_Reset(Servo_ManagerTypeDef *manager, uint8_t id)
{
    return Servo_SendPacket(manager, id, 0x06U, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef Servo_SetID(Servo_ManagerTypeDef *manager, uint8_t old_id, uint8_t new_id)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, old_id);
    uint8_t data[1] = {new_id};
    HAL_StatusTypeDef status;

    if (motor == NULL || new_id >= SERVO_BROADCAST_ID || Servo_FindMotor(manager, new_id) != NULL)
    {
        return HAL_ERROR;
    }
    status = Servo_UnlockEprom(manager, old_id);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(20U);
    status = Servo_WriteData(manager, old_id, SERVO_ID_ADDR, data, sizeof(data));
    if (status != HAL_OK)
    {
        return status;
    }
    motor->device_id = new_id;
    HAL_Delay(20U);
    status = Servo_LockEprom(manager, new_id);
    HAL_Delay(20U);
    return status;
}

HAL_StatusTypeDef Servo_SetID_Broadcast(Servo_ManagerTypeDef *manager, uint8_t new_id)
{
    uint8_t unlock[2] = {SERVO_LOCK_ADDR, 0U};
    uint8_t set_id[2] = {SERVO_ID_ADDR, new_id};
    uint8_t lock[2] = {SERVO_LOCK_ADDR, 1U};
    HAL_StatusTypeDef status;

    if (new_id >= SERVO_BROADCAST_ID)
    {
        return HAL_ERROR;
    }
    status = Servo_SendRawPacket(manager, SERVO_BROADCAST_ID, 0x03U, unlock, sizeof(unlock));
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(20U);
    status = Servo_SendRawPacket(manager, SERVO_BROADCAST_ID, 0x03U, set_id, sizeof(set_id));
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(20U);
    status = Servo_SendRawPacket(manager, new_id, 0x03U, lock, sizeof(lock));
    HAL_Delay(20U);
    return status;
}

HAL_StatusTypeDef Servo_EnableTorque(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t enable)
{
    return Servo_WriteData(manager, id, SERVO_TORQUE_ENABLE_ADDR, &enable, 1U);
}

HAL_StatusTypeDef Servo_UnlockEprom(Servo_ManagerTypeDef *manager, uint8_t id)
{
    uint8_t value = 0U;
    return Servo_WriteData(manager, id, SERVO_LOCK_ADDR, &value, 1U);
}

HAL_StatusTypeDef Servo_LockEprom(Servo_ManagerTypeDef *manager, uint8_t id)
{
    uint8_t value = 1U;
    return Servo_WriteData(manager, id, SERVO_LOCK_ADDR, &value, 1U);
}

HAL_StatusTypeDef Servo_SetAngleLimit(Servo_ManagerTypeDef *manager, uint8_t id,
                                      uint16_t min_position, uint16_t max_position)
{
    uint8_t data[4] = {(uint8_t)(min_position >> 8), (uint8_t)min_position,
                       (uint8_t)(max_position >> 8), (uint8_t)max_position};
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef lock_status;

    if (min_position > max_position || max_position > 1023U)
    {
        return HAL_ERROR;
    }

    status = Servo_UnlockEprom(manager, id);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(20U);
    status = Servo_WriteData(manager, id, SERVO_MIN_ANGLE_LIMIT_ADDR, data, sizeof(data));
    HAL_Delay(20U);
    /* Attempt to relock even when the limit write did not acknowledge. */
    lock_status = Servo_LockEprom(manager, id);
    HAL_Delay(20U);
    return status == HAL_OK ? lock_status : status;
}

HAL_StatusTypeDef Servo_PositionMode(Servo_ManagerTypeDef *manager, uint8_t id)
{
    return Servo_SetAngleLimit(manager, id, 0U, 1023U);
}

HAL_StatusTypeDef Servo_PWMMode(Servo_ManagerTypeDef *manager, uint8_t id)
{
    return Servo_SetAngleLimit(manager, id, 0U, 0U);
}

HAL_StatusTypeDef Servo_WritePWM(Servo_ManagerTypeDef *manager, uint8_t id, int16_t pwm)
{
    uint16_t value;
    uint8_t data[2];

    if (pwm > 1000)
    {
        pwm = 1000;
    }
    if (pwm < -1000)
    {
        pwm = -1000;
    }
    value = pwm < 0 ? (uint16_t)(-pwm) | (1U << 10) : (uint16_t)pwm;
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return Servo_WriteData(manager, id, SERVO_GOAL_TIME_ADDR, data, sizeof(data));
}

HAL_StatusTypeDef Servo_Move(Servo_ManagerTypeDef *manager, uint8_t id,
                             uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t params[7] = {SERVO_GOAL_POSITION_ADDR, (uint8_t)(position >> 8), (uint8_t)position,
                         (uint8_t)(time >> 8), (uint8_t)time,
                         (uint8_t)(speed >> 8), (uint8_t)speed};
    return Servo_SendPacket(manager, id, 0x03U, params, sizeof(params), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef Servo_SyncMove(Servo_ManagerTypeDef *manager, const uint8_t *ids, uint8_t servo_count,
                                 uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t data[SERVO_SYNC_MOVE_MAX_COUNT * SERVO_SYNC_MOVE_DATA_LEN];

    if (ids == NULL || servo_count == 0U)
    {
        return HAL_ERROR;
    }
    if (servo_count > SERVO_SYNC_MOVE_MAX_COUNT)
    {
        servo_count = SERVO_SYNC_MOVE_MAX_COUNT;
    }
    for (uint8_t i = 0U; i < servo_count; i++)
    {
        uint8_t offset = (uint8_t)(i * SERVO_SYNC_MOVE_DATA_LEN);
        data[offset] = ids[i];
        data[offset + 1U] = (uint8_t)(position >> 8);
        data[offset + 2U] = (uint8_t)position;
        data[offset + 3U] = (uint8_t)(time >> 8);
        data[offset + 4U] = (uint8_t)time;
        data[offset + 5U] = (uint8_t)(speed >> 8);
        data[offset + 6U] = (uint8_t)speed;
    }
    return Servo_SyncWrite(manager, SERVO_GOAL_POSITION_ADDR, 6U, data, servo_count);
}

HAL_StatusTypeDef Servo_SyncMoveList(Servo_ManagerTypeDef *manager, const uint8_t *ids,
                                     const uint16_t *positions, uint8_t servo_count,
                                     uint16_t time, uint16_t speed)
{
    uint8_t data[SERVO_SYNC_MOVE_MAX_COUNT * SERVO_SYNC_MOVE_DATA_LEN];

    if (ids == NULL || positions == NULL || servo_count == 0U)
    {
        return HAL_ERROR;
    }
    if (servo_count > SERVO_SYNC_MOVE_MAX_COUNT)
    {
        servo_count = SERVO_SYNC_MOVE_MAX_COUNT;
    }
    for (uint8_t i = 0U; i < servo_count; i++)
    {
        uint8_t offset = (uint8_t)(i * SERVO_SYNC_MOVE_DATA_LEN);
        data[offset] = ids[i];
        data[offset + 1U] = (uint8_t)(positions[i] >> 8);
        data[offset + 2U] = (uint8_t)positions[i];
        data[offset + 3U] = (uint8_t)(time >> 8);
        data[offset + 4U] = (uint8_t)time;
        data[offset + 5U] = (uint8_t)(speed >> 8);
        data[offset + 6U] = (uint8_t)speed;
    }
    return Servo_SyncWrite(manager, SERVO_GOAL_POSITION_ADDR, 6U, data, servo_count);
}

HAL_StatusTypeDef Servo_SyncMove123(Servo_ManagerTypeDef *manager, uint16_t position,
                                    uint16_t time, uint16_t speed)
{
    static const uint8_t ids[3] = {1U, 2U, 3U};
    return Servo_SyncMove(manager, ids, 3U, position, time, speed);
}
