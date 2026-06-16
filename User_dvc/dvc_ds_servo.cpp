#include "dvc_ds_servo.h"

void Class_DS_Servo::Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t min_pulse, uint16_t max_pulse)
{
    this->htim = htim;
    this->Channel = Channel;

    Min_Pulse = min_pulse;
    Max_Pulse = max_pulse;

    State = Enum_Servo_State::Servo_State_Enable;
}

void Class_DS_Servo::Set_Normalized_Position(float position)
{
    if(State != Enum_Servo_State::Servo_State_Enable)
    {
        return;
    }
    
    uint16_t pulse = Min_Pulse + static_cast<uint16_t>(position * (Max_Pulse - Min_Pulse));
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
