#include "servo_private.h"

HAL_StatusTypeDef Servo_ReadData(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address, uint8_t read_len)
{
    uint8_t params[2] = {address, read_len};
    return Servo_SendPacket(manager, id, 0x02U, params, sizeof(params), NULL, NULL, 0U, 0U);
}

uint8_t Servo_ReadValue(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                        uint8_t *data, uint8_t data_len, uint32_t timeout)
{
    uint8_t params[2] = {address, data_len};
    uint8_t reply[SERVO_FRAME_MAX_LEN];
    uint16_t reply_len;

    if (Servo_FindMotor(manager, id) == NULL || data == NULL || data_len == 0U ||
        data_len > SERVO_MAX_PARAM_LEN)
    {
        return 0U;
    }

    if (Servo_SendPacket(manager, id, 0x02U, params, sizeof(params),
                         reply, &reply_len, sizeof(reply), timeout) != HAL_OK ||
        reply[3] != (uint8_t)(data_len + 2U))
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < data_len; i++)
    {
        data[i] = reply[5U + i];
    }
    return 1U;
}

uint8_t Servo_GetByte(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                      uint8_t *value, uint32_t timeout)
{
    return value != NULL && Servo_ReadValue(manager, id, address, value, 1U, timeout);
}

uint8_t Servo_GetWord(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t address,
                      uint16_t *value, uint32_t timeout)
{
    uint8_t data[2];

    if (value == NULL || Servo_ReadValue(manager, id, address, data, sizeof(data), timeout) == 0U)
    {
        return 0U;
    }
    *value = Servo_MakeWord(data[0], data[1]);
    return 1U;
}

uint8_t Servo_GetID(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint8_t value;
    if (motor == NULL || Servo_GetByte(manager, id, SERVO_ID_ADDR, &value, timeout) == 0U)
    {
        return 0U;
    }
    motor->device_id = value;
    return 1U;
}

uint8_t Servo_GetVersion(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    return motor != NULL && Servo_GetWord(manager, id, SERVO_VERSION_ADDR, &motor->version, timeout);
}

uint8_t Servo_GetPosition(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    return motor != NULL && Servo_GetWord(manager, id, SERVO_PRESENT_POSITION_ADDR, &motor->feedback.position, timeout);
}

uint8_t Servo_GetSpeed(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint16_t value;
    if (motor == NULL || Servo_GetWord(manager, id, SERVO_PRESENT_SPEED_ADDR, &value, timeout) == 0U)
    {
        return 0U;
    }
    motor->feedback.speed = Servo_MakeSignedWord(value, 15U);
    return 1U;
}

uint8_t Servo_GetLoad(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint16_t value;
    if (motor == NULL || Servo_GetWord(manager, id, SERVO_PRESENT_LOAD_ADDR, &value, timeout) == 0U)
    {
        return 0U;
    }
    motor->feedback.load = Servo_MakeSignedWord(value, 10U);
    return 1U;
}

uint8_t Servo_GetVoltage(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    return motor != NULL && Servo_GetByte(manager, id, SERVO_PRESENT_VOLTAGE_ADDR, &motor->feedback.voltage, timeout);
}

uint8_t Servo_GetTemperature(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    return motor != NULL && Servo_GetByte(manager, id, SERVO_PRESENT_TEMPERATURE_ADDR, &motor->feedback.temperature, timeout);
}

uint8_t Servo_GetMoving(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    return motor != NULL && Servo_GetByte(manager, id, SERVO_MOVING_ADDR, &motor->feedback.moving, timeout);
}

uint8_t Servo_GetCurrent(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint16_t value;
    if (motor == NULL || Servo_GetWord(manager, id, SERVO_PRESENT_CURRENT_ADDR, &value, timeout) == 0U)
    {
        return 0U;
    }
    motor->feedback.current = Servo_MakeSignedWord(value, 15U);
    return 1U;
}

uint8_t Servo_GetAngleLimit(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint8_t data[4];
    if (motor == NULL || Servo_ReadValue(manager, id, SERVO_MIN_ANGLE_LIMIT_ADDR, data, sizeof(data), timeout) == 0U)
    {
        return 0U;
    }
    motor->min_position = Servo_MakeWord(data[0], data[1]);
    motor->max_position = Servo_MakeWord(data[2], data[3]);
    return 1U;
}

uint8_t Servo_GetMode(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    if (motor == NULL || Servo_GetAngleLimit(manager, id, timeout) == 0U)
    {
        return 0U;
    }
    motor->mode = (motor->min_position == 0U && motor->max_position == 0U) ? 1U : 0U;
    return 1U;
}

uint8_t Servo_GetFeedback(Servo_ManagerTypeDef *manager, uint8_t id, uint32_t timeout)
{
    Servo_MotorTypeDef *motor = Servo_FindMotor(manager, id);
    uint8_t data[16];
    if (motor == NULL || Servo_ReadValue(manager, id, SERVO_PRESENT_POSITION_ADDR, data, sizeof(data), timeout) == 0U)
    {
        return 0U;
    }
    motor->feedback.position = Servo_MakeWord(data[0], data[1]);
    motor->feedback.speed = Servo_MakeSignedWord(Servo_MakeWord(data[2], data[3]), 15U);
    motor->feedback.load = Servo_MakeSignedWord(Servo_MakeWord(data[4], data[5]), 10U);
    motor->feedback.voltage = data[6];
    motor->feedback.temperature = data[7];
    motor->feedback.moving = data[10];
    motor->feedback.current = Servo_MakeSignedWord(Servo_MakeWord(data[14], data[15]), 15U);
    return 1U;
}

uint16_t Servo_MakeWord(uint8_t high, uint8_t low)
{
    return ((uint16_t)high << 8) | low;
}

int16_t Servo_MakeSignedWord(uint16_t value, uint8_t sign_bit)
{
    if ((value & (1U << sign_bit)) != 0U)
    {
        value &= (uint16_t)~(1U << sign_bit);
        return -(int16_t)value;
    }
    return (int16_t)value;
}
