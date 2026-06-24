#ifndef DVC_SERVO_BASE_H
#define DVC_SERVO_BASE_H

#include "stdint.h"

class Class_Servo_Base
{
public:
    enum class Enum_Servo_State
    {
        Servo_State_Disable = 0,
        Servo_State_Enable,
    };

    virtual ~Class_Servo_Base() {}

    // 状态控制接口
    virtual void Set_State(Enum_Servo_State state) = 0; // 设置舵机状态
    virtual Enum_Servo_State Get_State() const = 0;     // 获取舵机状态

    // 运动控制接口
    virtual void Set_Normalized_Position(float position) = 0; // 设置位置 [0，1]
};

#endif // DVC_SERVO_BASE_H