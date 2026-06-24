#ifndef DVC_ENCODER_BASE_H
#define DVC_ENCODER_BASE_H

#include <stdint.h>

class Class_Encoder_Base
{
public:
    enum class Enum_Encoder_Status
    {
        Encoder_Status_DISABLE = 0,
        Encoder_Status_ENABLE = 1
    };

    virtual ~Class_Encoder_Base() {}

    virtual float Get_Total_Angle() const = 0;

    virtual float Get_Normalized_Angle() const = 0;

    virtual int32_t Get_Total_Round() const = 0;

    virtual float Get_Omega() const = 0;

    virtual Enum_Encoder_Status Get_Status() const = 0;
};

#endif