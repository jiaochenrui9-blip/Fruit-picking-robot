#include "RS05_receive.h"
#include "RS05_parameter.h"
#include "RS05_codec.h"

#define RS05_COMM_TYPE_FEEDBACK          0x02U
#define RS05_COMM_TYPE_READ_PARAMETER    0x11U

#define RS05_POSITION_MIN_RAD       (-12.57f)
#define RS05_POSITION_MAX_RAD       (12.57f)
#define RS05_VELOCITY_MIN_RAD_S     (-50.0f)
#define RS05_VELOCITY_MAX_RAD_S     (50.0f)
#define RS05_TORQUE_MIN_NM          (-5.5f)
#define RS05_TORQUE_MAX_NM          (5.5f)

static RS05_MotorTypedef *RS05_FindMotor(const RS05_ManagerTypedef *manager,
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

void RS05_ParseFeedback(RS05_MotorTypedef *motor,
                        uint32_t ext_id,
                        const uint8_t data[8])
{
    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    motor->position_rad = RS05_Codec_U16ToFloat(
        RS05_Codec_ReadU16BE(&data[0]),
        RS05_POSITION_MIN_RAD,
        RS05_POSITION_MAX_RAD);
    motor->velocity_rad_s = RS05_Codec_U16ToFloat(
        RS05_Codec_ReadU16BE(&data[2]),
        RS05_VELOCITY_MIN_RAD_S,
        RS05_VELOCITY_MAX_RAD_S);
    motor->torque_nm = RS05_Codec_U16ToFloat(
        RS05_Codec_ReadU16BE(&data[4]),
        RS05_TORQUE_MIN_NM,
        RS05_TORQUE_MAX_NM);
    motor->temperature_c = (float)RS05_Codec_ReadU16BE(&data[6]) * 0.1f;
    motor->state = (uint8_t)((ext_id >> 22) & 0x03U);
    motor->fault_flags = (uint8_t)((ext_id >> 16) & 0x3FU);
    motor->last_feedback_tick = HAL_GetTick();
}

void RS05_ParseParameterReply(RS05_MotorTypedef *motor,
                               uint32_t ext_id,
                               const uint8_t data[8])
{
    uint16_t index;
    RS05_ParameterType type;
    RS05_ParameterCache *parameter;
    RS05_ParameterCache *oldest;
    uint32_t now;
    uint32_t oldest_age;
    uint8_t i;

    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    /* 参数响应 ExtId 的 bit16~23 非零表示电机返回了错误。 */
    if (((ext_id >> 16) & 0xFFU) != 0U)
    {
        return;
    }
    /*提取index和数据类型*/
    index = RS05_Codec_ReadU16LE(&data[0]);
    type = RS05_Match_Type(index);
    if (type == RS05_PARAMETER_UNKNOWN)
    {
        return;
    }

    parameter = RS05_FindParameterCache(motor, index);
    if (parameter == NULL)
    {
        for (i = 0U; i < RS05_PARAMETER_CACHE_COUNT; ++i)
        {
            if (motor->parameters[i].valid == 0U)
            {
                parameter = &motor->parameters[i];
                break;
            }
        }
    }

    now = HAL_GetTick();
    if (parameter == NULL)
    {
        oldest = &motor->parameters[0];
        oldest_age = now - oldest->last_update_tick;
        for (i = 1U; i < RS05_PARAMETER_CACHE_COUNT; ++i)
        {
            const uint32_t age = now - motor->parameters[i].last_update_tick;

            if (age > oldest_age)
            {
                oldest = &motor->parameters[i];
                oldest_age = age;
            }
        }
        parameter = oldest;
    }

    parameter->valid = 0U;
    parameter->index = index;
    parameter->type = type;
    RS05_Process_Parameter(parameter, &data[4]);
    parameter->last_update_tick = now;
    parameter->valid = 1U;
}

RS05_ParameterCache *RS05_FindParameterCache(RS05_MotorTypedef *motor,
                                              uint16_t index)
{
    if (motor == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < RS05_PARAMETER_CACHE_COUNT; ++i)
    {
        if ((motor->parameters[i].valid != 0U) &&
            (index == motor->parameters[i].index))
        {
            return &motor->parameters[i];
        }
    }
    return NULL;
}
void RS05_ProcessFrame(RS05_ManagerTypedef *manager,
                       uint32_t ext_id,
                       const uint8_t data[8])
{
    uint8_t comm_type;
    uint8_t motor_id;
    RS05_MotorTypedef *motor;

    if ((manager == NULL) || (data == NULL))
    {
        return;
    }

    comm_type = (uint8_t)((ext_id >> 24) & 0x1FU);
    motor_id = (uint8_t)((ext_id >> 8) & 0xFFU);
    motor = RS05_FindMotor(manager, motor_id);
    if (motor == NULL)
    {
        return;
    }

    switch (comm_type)
    {
        case RS05_COMM_TYPE_FEEDBACK:
            RS05_ParseFeedback(motor, ext_id, data);
            break;

        case RS05_COMM_TYPE_ACTIVE_REPORT:
            RS05_ParseFeedback(motor, ext_id, data);
            break;

        case RS05_COMM_TYPE_READ_PARAMETER:
            RS05_ParseParameterReply(motor,ext_id, data);
            break;

        default:
            break;
    }
}

void RS05_ProcessRxFrame(RS05_ManagerTypedef *manager,
                         const CAN_RxHeaderTypeDef *header,
                         const uint8_t data[8])
{
    if ((manager == NULL) || (header == NULL) || (data == NULL))
    {
        return;
    }

    /* 私有协议只用扩展数据帧；标准帧（如CAN2上3508的反馈）不归RS05管。 */
    if ((header->IDE != CAN_ID_EXT) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U))
    {
        return;
    }

    RS05_ProcessFrame(manager, header->ExtId, data);
}
