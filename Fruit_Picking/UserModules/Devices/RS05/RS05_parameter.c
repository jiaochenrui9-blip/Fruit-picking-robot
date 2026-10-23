#include "RS05_parameter.h"

#include "RS05_codec.h"
#include <stddef.h>

RS05_ParameterType RS05_Match_Type(uint16_t index)
{
    switch (index)
    {
        /* uint8_t */
    case 0x7005U:  /* run_mode */
    case 0x7029U:  /* zero_sta */
    case 0x702CU:  /* alveolous_open */
    case 0x702DU:  /* iq_test */
        return RS05_PARAMETER_U8;

        /* uint16_t */
    case 0x7026U:  /* EPScan_time */
        return RS05_PARAMETER_U16;

        /* uint32_t */
    case 0x7028U:  /* canTimeout */
        return RS05_PARAMETER_U32;

        /* float */
    case 0x7006U:  /* iq_ref */
    case 0x700AU:  /* spd_ref */
    case 0x700BU:  /* limit_torque */
    case 0x7010U:  /* cur_kp */
    case 0x7011U:  /* cur_ki */
    case 0x7014U:  /* cur_filt_gain */
    case 0x7016U:  /* loc_ref */
    case 0x7017U:  /* limit_spd */
    case 0x7018U:  /* limit_cur */
    case 0x7019U:  /* mechPos */
    case 0x701AU:  /* iqf */
    case 0x701BU:  /* mechVel */
    case 0x701CU:  /* VBUS */
    case 0x701EU:  /* loc_kp */
    case 0x701FU:  /* spd_kp */
    case 0x7020U:  /* spd_ki */
    case 0x7021U:  /* spd_filt_gain */
    case 0x7022U:  /* acc_rad */
    case 0x7024U:  /* vel_max */
    case 0x7025U:  /* acc_set */
    case 0x702BU:  /* add_offset */
    case 0x702EU:  /* dcc_set */
        return RS05_PARAMETER_FLOAT;

    default:
        return RS05_PARAMETER_UNKNOWN;
    }
}

void RS05_Process_Parameter(RS05_ParameterCache *parameter,
                            const uint8_t data[4])
{
    if ((parameter == NULL) || (data == NULL))
    {
        return;
    }

    switch (parameter->type)
    {
        case RS05_PARAMETER_U8:
        parameter->value.u8 = data[0];
        break;
        case RS05_PARAMETER_U16:
        parameter->value.u16 = RS05_Codec_ReadU16LE(&data[0]);
        break;
        case RS05_PARAMETER_U32:
        parameter->value.u32 = RS05_Codec_ReadU32LE(&data[0]);
        break;
        case RS05_PARAMETER_FLOAT:
        parameter->value.f32 = RS05_Codec_ReadFloatLE(&data[0]);
        break;
        case RS05_PARAMETER_UNKNOWN:
        break;
    }
}
