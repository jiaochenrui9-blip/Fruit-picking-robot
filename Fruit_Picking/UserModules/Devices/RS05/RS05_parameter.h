#ifndef RS05_PARAMETER_H
#define RS05_PARAMETER_H

#include <stdint.h>

typedef enum
{
    RS05_PARAMETER_UNKNOWN = 0,
    RS05_PARAMETER_U8,
    RS05_PARAMETER_U16,
    RS05_PARAMETER_U32,
    RS05_PARAMETER_FLOAT
} RS05_ParameterType;

typedef union
{
    uint8_t u8;
    uint16_t u16;
    uint32_t u32;
    float f32;
} RS05_ParameterValue;

typedef struct
{
    uint16_t index;
    RS05_ParameterType type;
    RS05_ParameterValue value;
    uint32_t last_update_tick;
    uint8_t valid;
} RS05_ParameterCache;


RS05_ParameterType RS05_Match_Type(uint16_t index);
void RS05_Process_Parameter(RS05_ParameterCache *parameter,
                            const uint8_t data[4]);

#endif /* RS05_PARAMETER_H */
