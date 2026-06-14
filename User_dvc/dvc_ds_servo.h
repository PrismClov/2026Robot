#ifndef DVC_DS_SERVO_H
#define DVC_DS_SERVO_H

#include "dvc_servo_base.h"
#include "drv_tim.h"

class Class_DS_Servo : public Class_Servo_Base
{
public:
    void Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t min_pulse, uint16_t max_pulse);

    // 实现基类接口
    inline void Set_State(Enum_Servo_State state) override;
    inline Enum_Servo_State Get_State() const override;

    void Set_Normalized_Position(float position) override;

private:
    uint16_t Min_Pulse = 0;
    uint16_t Max_Pulse = 0;

    TIM_HandleTypeDef *htim;
    uint32_t Channel;

    Enum_Servo_State State = Enum_Servo_State::Servo_State_Disable;
};

inline void Class_DS_Servo::Set_State(Class_Servo_Base::Enum_Servo_State state) 
{
    State = state;
}

inline Class_Servo_Base::Enum_Servo_State Class_DS_Servo::Get_State() const 
{
    return State;
}


#endif