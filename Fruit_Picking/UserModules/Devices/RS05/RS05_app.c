//
// Created by game on 2026/7/16.
//
#include "RS05_app.h"
#include "RS05_codec.h"
#include "RS05_receive.h"
#include <string.h>

#include "RS05_Command.h"

static uint8_t RS05_ValueInRange(float value, float minimum, float maximum)
{
    return (uint8_t)((value >= minimum) && (value <= maximum));
}
static RS05_MotorTypedef *RS05_Manager_FindMotor(const RS05_ManagerTypedef *manager,
uint8_t motor_id)
{
    uint8_t i;

    if (manager == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < manager->num_motors; i++)
    {
        if ((manager->motors[i] != NULL) &&
            (manager->motors[i]->motor_id == motor_id))
        {
            return manager->motors[i];
        }
    }

    return NULL;
}

uint32_t RS05_BuildExtId(uint8_t comm_type,
                         uint16_t data2,
                         uint8_t motor_id)
{
    return ((uint32_t)(comm_type & 0x1FU) << 24) |
           ((uint32_t)data2 << 8) |
           motor_id;
}

HAL_StatusTypeDef RS05_SendData(CAN_HandleTypeDef *hcan,
                                uint8_t comm_type,
                                const uint8_t *data,
                                uint16_t data2,
                                uint8_t motor_id)
{
    if ((hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    return CAN_Bus_SendExtData(hcan,
                               RS05_BuildExtId(comm_type, data2, motor_id),
                               data,
                               8U);
}

HAL_StatusTypeDef RS05_Enable(CAN_HandleTypeDef *hcan,
                              uint8_t master_id,
                              uint8_t motor_id)
{
    static const uint8_t data[8] = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    return RS05_SendData(hcan, RS05_COMM_TYPE_ENABLE, data, master_id, motor_id);
}

HAL_StatusTypeDef RS05_MotionControl(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     float position_rad,
                                     float velocity_rad_s,
                                     float kp,
                                     float kd,
                                     float torque_nm)
{
    uint8_t data[8] = {0};
    uint16_t torque_raw;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16BE(&data[0], RS05_Codec_FloatToU16(position_rad,
                                                     RS05_POSITION_MIN_RAD,
                                                     RS05_POSITION_MAX_RAD));
    RS05_Codec_WriteU16BE(&data[2], RS05_Codec_FloatToU16(velocity_rad_s,
                                                     RS05_VELOCITY_MIN_RAD_S,
                                                     RS05_VELOCITY_MAX_RAD_S));
    RS05_Codec_WriteU16BE(&data[4], RS05_Codec_FloatToU16(kp, RS05_KP_MIN, RS05_KP_MAX));
    RS05_Codec_WriteU16BE(&data[6], RS05_Codec_FloatToU16(kd, RS05_KD_MIN, RS05_KD_MAX));
    torque_raw = RS05_Codec_FloatToU16(torque_nm, RS05_TORQUE_MIN_NM,
                                    RS05_TORQUE_MAX_NM);

    return RS05_SendData(hcan, RS05_COMM_TYPE_MOTION_CONTROL,
                         data, torque_raw, motor_id);
}

HAL_StatusTypeDef RS05_Stop(CAN_HandleTypeDef *hcan,
                            uint8_t master_id,
                            uint8_t motor_id)
{
    static const uint8_t data[8] = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    return RS05_SendData(hcan, RS05_COMM_TYPE_STOP, data, master_id, motor_id);
}

HAL_StatusTypeDef RS05_SetMechanicalZero(RS05_ManagerTypedef *manager,
                                          RS05_MotorTypedef *motor)
{
    uint8_t data[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    HAL_StatusTypeDef status;

    if ((manager == NULL) || (manager->hcan == NULL) || (motor == NULL) ||
        (RS05_Manager_FindMotor(manager, motor->motor_id) != motor))
    {
        return HAL_ERROR;
    }

    status = RS05_Stop(manager->hcan,
                       manager->master_id,
                       motor->motor_id);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(RS05_COMMAND_INTERVAL_MS);


    return RS05_SendData(manager->hcan,
                         RS05_COMM_TYPE_MECHANICAL_ZERO,
                         data,
                         manager->master_id,
                         motor->motor_id);
}

void RS05_Manager_Init(RS05_ManagerTypedef *manager,
                       CAN_HandleTypeDef *hcan,
                       uint8_t master_id)
{
    if (manager == NULL)
    {
        return;
    }

    memset(manager, 0, sizeof(*manager));
    manager->hcan = hcan;
    manager->master_id = master_id;
}

HAL_StatusTypeDef RS05_Manager_RegisterMotor(RS05_ManagerTypedef *manager,
                                              RS05_MotorTypedef *motor)
{
    if ((manager == NULL) || (motor == NULL) ||
        (manager->num_motors >= RS05_MOTOR_MAX_COUNT))
    {
        return HAL_ERROR;
    }

    if (RS05_Manager_FindMotor(manager, motor->motor_id) != NULL)
    {
        return HAL_ERROR;
    }

    manager->motors[manager->num_motors] = motor;
    manager->num_motors++;
    return HAL_OK;
}

HAL_StatusTypeDef RS05_Manager_Start(RS05_ManagerTypedef *manager)
{
    HAL_StatusTypeDef status;
    uint32_t filter_bank;

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    /* 过滤器组分配见can_bus.h：RS05用CAN1的组1、CAN2的组15。 */
    if (manager->hcan->Instance == CAN1)
    {
        filter_bank = 1U;
    }
    else if (manager->hcan->Instance == CAN2)
    {
        filter_bank = 15U;
    }
    else
    {
        return HAL_ERROR;
    }

    /* 私有协议全是扩展帧：掩码为0，接收所有扩展数据帧，是哪个电机的由RS05_ProcessFrame判断。 */
    status = CAN_Bus_ConfigExtIdMask(manager->hcan, filter_bank, 14U, 0U, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    return CAN_Bus_StartRxFifo0(manager->hcan);
}

static HAL_StatusTypeDef RS05_WriteParameterRaw(RS05_ManagerTypedef *manager,
                                                 uint8_t motor_id,
                                                 uint16_t index,
                                                 const uint8_t value[4])
{
    uint8_t data[8] = {0};

    if ((manager == NULL) || (manager->hcan == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(&data[0], index);
    memcpy(&data[4], value, 4U);

    return RS05_SendData(manager->hcan,
                         RS05_COMM_TYPE_WRITE_PARAMETER,
                         data,
                         manager->master_id,
                         motor_id);
}

HAL_StatusTypeDef RS05_Command_ParaRead(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index)
{

    uint8_t data[8] = {0};

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(&data[0], index);
    return RS05_SendData(manager->hcan,
                         RS05_COMM_TYPE_READ_PARAMETER,
                         data,
                         manager->master_id,
                         motor_id);

}

HAL_StatusTypeDef RS05_WriteParameterU8(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index,
                                        uint8_t value)
{
    uint8_t raw[4] = {value, 0U, 0U, 0U};

    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterU16(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint16_t value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteU16LE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterU32(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint32_t value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteU32LE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterFloat(RS05_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint16_t index,
                                           float value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteFloatLE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

static uint8_t RS05_MotorIsReady(const RS05_ManagerTypedef *manager,
                                 const RS05_MotorTypedef *motor)
{
    return (uint8_t)((manager != NULL) &&
                     (manager->hcan != NULL) &&
                     (motor != NULL) &&
                     (RS05_Manager_FindMotor(manager, motor->motor_id) == motor));
}

static HAL_StatusTypeDef RS05_WriteMotorFloat(RS05_ManagerTypedef *manager,
                                               RS05_MotorTypedef *motor,
                                               uint16_t index,
                                               float value)
{
    if (RS05_MotorIsReady(manager, motor) == 0U)
    {
        return HAL_ERROR;
    }

    return RS05_WriteParameterFloat(manager,
                                    motor->motor_id,
                                    index,
                                    value);
}

static HAL_StatusTypeDef RS05_WriteMotorFloatAndWait(RS05_ManagerTypedef *manager,
                                                      RS05_MotorTypedef *motor,
                                                      uint16_t index,
                                                      float value)
{
    HAL_StatusTypeDef status = RS05_WriteMotorFloat(manager, motor, index, value);

    if (status == HAL_OK)
    {
        HAL_Delay(RS05_COMMAND_INTERVAL_MS);
    }

    return status;
}

HAL_StatusTypeDef RS05_SetMode(RS05_ManagerTypedef *manager,
                               RS05_MotorTypedef *motor,
                               uint8_t mode)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        ((mode != RS05_MODE_MOTION) &&
         (mode != RS05_MODE_POSITION_PP) &&
         (mode != RS05_MODE_SPEED) &&
         (mode != RS05_MODE_CURRENT) &&
         (mode != RS05_MODE_POSITION_CSP)))
    {
        return HAL_ERROR;
    }

    return RS05_WriteParameterU8(manager,
                                 motor->motor_id,
                                 RS05_PARAMETER_RUN_MODE,
                                 mode);
}

static HAL_StatusTypeDef RS05_BeginModeConfiguration(
    RS05_ManagerTypedef *manager,
    RS05_MotorTypedef *motor,
    uint8_t mode)
{
    HAL_StatusTypeDef status;

    if (RS05_MotorIsReady(manager, motor) == 0U)
    {
        return HAL_ERROR;
    }

    status = RS05_Stop(manager->hcan,
                       manager->master_id,
                       motor->motor_id);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(RS05_COMMAND_INTERVAL_MS);

    status = RS05_SetMode(manager, motor, mode);
    if (status == HAL_OK)
    {
        HAL_Delay(RS05_COMMAND_INTERVAL_MS);
    }
    return status;
}

static HAL_StatusTypeDef RS05_EnableConfiguredMotor(
    RS05_ManagerTypedef *manager,
    RS05_MotorTypedef *motor)
{
    if (RS05_MotorIsReady(manager, motor) == 0U)
    {
        return HAL_ERROR;
    }

    return RS05_Enable(manager->hcan,
                       manager->master_id,
                       motor->motor_id);
}

HAL_StatusTypeDef RS05_EnterSpeedMode(RS05_ManagerTypedef *manager,
                                      RS05_MotorTypedef *motor,
                                      float acceleration_rad_s2,
                                      float current_limit_a)
{
    HAL_StatusTypeDef status;

    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(acceleration_rad_s2,
                           RS05_SPEED_ACCELERATION_MIN_RAD_S2,
                           RS05_SPEED_ACCELERATION_MAX_RAD_S2) == 0U) ||
        (RS05_ValueInRange(current_limit_a,
                           RS05_CURRENT_LIMIT_MIN_A,
                           RS05_CURRENT_LIMIT_MAX_A) == 0U))
    {
        return HAL_ERROR;
    }

    status = RS05_BeginModeConfiguration(manager, motor, RS05_MODE_SPEED);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_EnableConfiguredMotor(manager, motor);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(RS05_COMMAND_INTERVAL_MS);

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_CURRENT_LIMIT,
                                         current_limit_a);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_SPEED_ACCELERATION,
                                         acceleration_rad_s2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_SPEED_REF,
                                         0.0f);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef RS05_SetSpeedTarget(RS05_ManagerTypedef *manager,
                                      RS05_MotorTypedef *motor,
                                      float target_speed_rad_s)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(target_speed_rad_s,
                           RS05_SPEED_REF_MIN_RAD_S,
                           RS05_SPEED_REF_MAX_RAD_S) == 0U))
    {
        return HAL_ERROR;
    }

    return RS05_WriteMotorFloat(manager, motor,
                                RS05_PARAMETER_SPEED_REF,
                                target_speed_rad_s);
}

HAL_StatusTypeDef RS05_EnterCurrentMode(RS05_ManagerTypedef *manager,
                                        RS05_MotorTypedef *motor)
{
    HAL_StatusTypeDef status;

    status = RS05_BeginModeConfiguration(manager, motor,
                                         RS05_MODE_CURRENT);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_CURRENT_REF,
                                         0.0f);
    if (status != HAL_OK)
    {
        return status;
    }

    return RS05_EnableConfiguredMotor(manager, motor);
}

HAL_StatusTypeDef RS05_SetCurrentTarget(RS05_ManagerTypedef *manager,
                                        RS05_MotorTypedef *motor,
                                        float target_current_a)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(target_current_a,
                           RS05_CURRENT_REF_MIN_A,
                           RS05_CURRENT_REF_MAX_A) == 0U))
    {
        return HAL_ERROR;
    }

    return RS05_WriteMotorFloat(manager, motor,
                                RS05_PARAMETER_CURRENT_REF,
                                target_current_a);
}

HAL_StatusTypeDef RS05_EnterPPMode(RS05_ManagerTypedef *manager,
                                   RS05_MotorTypedef *motor,
                                   float max_velocity_rad_s,
                                   float acceleration_rad_s2,
                                   float deceleration_rad_s2,
                                   float current_limit_a)
{
    HAL_StatusTypeDef status;

    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(max_velocity_rad_s,
                           RS05_PP_SPEED_MIN_RAD_S,
                           RS05_PP_SPEED_MAX_RAD_S) == 0U) ||
        (RS05_ValueInRange(acceleration_rad_s2,
                           RS05_PP_ACCELERATION_MIN_RAD_S2,
                           RS05_PP_ACCELERATION_MAX_RAD_S2) == 0U) ||
        (RS05_ValueInRange(deceleration_rad_s2,
                           RS05_PP_ACCELERATION_MIN_RAD_S2,
                           RS05_PP_ACCELERATION_MAX_RAD_S2) == 0U) ||
        (RS05_ValueInRange(current_limit_a,
                           RS05_CURRENT_LIMIT_MIN_A,
                           RS05_CURRENT_LIMIT_MAX_A) == 0U))
    {
        return HAL_ERROR;
    }

    status = RS05_BeginModeConfiguration(manager, motor,
                                         RS05_MODE_POSITION_PP);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_CURRENT_LIMIT,
                                         current_limit_a);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_PP_SPEED,
                                         max_velocity_rad_s);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_PP_ACCELERATION,
                                         acceleration_rad_s2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_PP_DECELERATION,
                                         deceleration_rad_s2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_POSITION_REF,
                                         motor->position_rad);
    if (status != HAL_OK)
    {
        return status;
    }

    return RS05_EnableConfiguredMotor(manager, motor);
}

HAL_StatusTypeDef RS05_SetPPTarget(RS05_ManagerTypedef *manager,
                                   RS05_MotorTypedef *motor,
                                   float target_position_rad)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(target_position_rad,
                           RS05_POSITION_MIN_RAD,
                           RS05_POSITION_MAX_RAD) == 0U))
    {
        return HAL_ERROR;
    }

    return RS05_WriteMotorFloat(manager, motor,
                                RS05_PARAMETER_POSITION_REF,
                                target_position_rad);
}

HAL_StatusTypeDef RS05_EnterCSPMode(RS05_ManagerTypedef *manager,
                                    RS05_MotorTypedef *motor,
                                    float speed_limit_rad_s,
                                    float current_limit_a)
{
    HAL_StatusTypeDef status;

    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(speed_limit_rad_s,
                           RS05_CSP_SPEED_MIN_RAD_S,
                           RS05_CSP_SPEED_MAX_RAD_S) == 0U) ||
        (RS05_ValueInRange(current_limit_a,
                           RS05_CURRENT_LIMIT_MIN_A,
                           RS05_CURRENT_LIMIT_MAX_A) == 0U))
    {
        return HAL_ERROR;
    }

    status = RS05_BeginModeConfiguration(manager, motor,
                                         RS05_MODE_POSITION_CSP);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_CURRENT_LIMIT,
                                         current_limit_a);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_CSP_SPEED_LIMIT,
                                         speed_limit_rad_s);
    if (status != HAL_OK)
    {
        return status;
    }

    status = RS05_WriteMotorFloatAndWait(manager, motor,
                                         RS05_PARAMETER_POSITION_REF,
                                         motor->position_rad);
    if (status != HAL_OK)
    {
        return status;
    }

    return RS05_EnableConfiguredMotor(manager, motor);
}

HAL_StatusTypeDef RS05_SetCSPTarget(RS05_ManagerTypedef *manager,
                                    RS05_MotorTypedef *motor,
                                    float target_position_rad)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(target_position_rad,
                           RS05_POSITION_MIN_RAD,
                           RS05_POSITION_MAX_RAD) == 0U))
    {
        return HAL_ERROR;
    }

    return RS05_WriteMotorFloat(manager, motor,
                                RS05_PARAMETER_POSITION_REF,
                                target_position_rad);
}

HAL_StatusTypeDef RS05_EnterMotionMode(RS05_ManagerTypedef *manager,
                                       RS05_MotorTypedef *motor)
{
    HAL_StatusTypeDef status;

    status = RS05_BeginModeConfiguration(manager, motor,
                                         RS05_MODE_MOTION);
    if (status != HAL_OK)
    {
        return status;
    }

    return RS05_EnableConfiguredMotor(manager, motor);
}

HAL_StatusTypeDef RS05_SetMotionTarget(RS05_ManagerTypedef *manager,
                                       RS05_MotorTypedef *motor,
                                       float position_rad,
                                       float velocity_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_ff_nm)
{
    if ((RS05_MotorIsReady(manager, motor) == 0U) ||
        (RS05_ValueInRange(position_rad,
                           RS05_POSITION_MIN_RAD,
                           RS05_POSITION_MAX_RAD) == 0U) ||
        (RS05_ValueInRange(velocity_rad_s,
                           RS05_VELOCITY_MIN_RAD_S,
                           RS05_VELOCITY_MAX_RAD_S) == 0U) ||
        (RS05_ValueInRange(kp, RS05_KP_MIN, RS05_KP_MAX) == 0U) ||
        (RS05_ValueInRange(kd, RS05_KD_MIN, RS05_KD_MAX) == 0U) ||
        (RS05_ValueInRange(torque_ff_nm,
                           RS05_TORQUE_MIN_NM,
                           RS05_TORQUE_MAX_NM) == 0U))
    {
        return HAL_ERROR;
    }

    return RS05_MotionControl(manager->hcan,
                              motor->motor_id,
                              position_rad,
                              velocity_rad_s,
                              kp,
                              kd,
                              torque_ff_nm);
}
uint8_t RS05_CheckIfFault(const RS05_MotorTypedef *motor)
{
    if (motor == NULL)
    {
        return 0U;
    }

    return (uint8_t)(motor->fault_flags != RS05_FAULT_NONE);
}
HAL_StatusTypeDef RS05_SaveParameters(RS05_ManagerTypedef *manager,
                                      uint8_t motor_id)
{
    /* 手册类型 22：数据区固定是 01 02 03 04 05 06 07 08，电机回一帧类型 2。 */
    static const uint8_t data[8] =
    {
        0x01U, 0x02U, 0x03U, 0x04U,
        0x05U, 0x06U, 0x07U, 0x08U
    };

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    return RS05_SendData(manager->hcan, RS05_COMM_TYPE_SAVE_PARAMETER, data,
                         manager->master_id, motor_id);
}

HAL_StatusTypeDef RS05_SetActiveReport(RS05_ManagerTypedef *manager,
                                       uint8_t motor_id,
                                       uint8_t enable)
{
    uint8_t data[8] =
    {
        0x01U, 0x02U, 0x03U, 0x04U,
        0x05U, 0x06U, 0x00U, 0x00U
    };

    if ((manager == NULL) ||
        (manager->hcan == NULL) ||
        (enable > 1U))
    {
        return HAL_ERROR;
    }

    data[6] = enable;

    return RS05_SendData(manager->hcan, RS05_COMM_TYPE_ACTIVE_REPORT, data,
                         manager->master_id, motor_id);
}



uint8_t RS05_IsOnline(const RS05_MotorTypedef *motor)
{
    if ((motor == NULL) || (motor->last_feedback_tick == 0U))
    {
        return 0U;
    }

    return (uint8_t)((HAL_GetTick() - motor->last_feedback_tick) < RS05_TIMEOUT_MS);
}
