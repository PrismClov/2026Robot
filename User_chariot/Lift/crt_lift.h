#ifndef CRT_LIFT_H
#define CRT_LIFT_H

#include "alg_fsm.h"
#include "crt_multi_motor_sync.h"
#include "dvc_motor_dji.h"

class Class_Lift;

/**
 * @brief R2抬升状态机: Wait(等待/下降到位) → Lift(抬升) → Down(下降) 循环
 *
 * 每次状态切换由 Yaw_Flag 触发，到位后自动停止并等待下次触发。
 */
enum Enum_Lift_Status
{
    Lift_Status_Init = 0,
    Lift_Status_Wait_R2,
    Lift_Status_Lift_R2,
    Lift_Status_Down_R2,
};

class Class_FSM_Lift : public Class_FSM
{
public:
    Class_Lift *Lift;

    void Lift_TIM_Status_PeriodElapsedCallback();

    Enum_Lift_Status Lift_Status = Lift_Status_Wait_R2;
};

class Class_Lift : public Class_MultiMotorSync_Base<2>
{
public:
    Class_FSM_Lift FSM_Lift;
    friend class Class_FSM_Lift;

    Motor::Class_Motor_DJI_C620 Motor_Lift_L;
    Motor::Class_Motor_DJI_C620 Motor_Lift_R;

    void TIM_Calculate_PeriodElapsedCallback(); // 1ms控制回调
    void Init();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    inline float Get_Now_Distance_L();
    inline float Get_Now_Distance_R();
    inline void Yaw_Flag_True();

private:
    bool Yaw_Flag = false; // Yaw到位触发，FSM检测到后切换状态

    float Target_Distance_Init = 0.0f;     // 初始化位置
    float Target_Distance_Wait_R2 = 0.25f; // 底部(等待/下降到位)
    float Target_Distance_Lift_R2 = 0.0f;  // 顶部(抬升到位)
    float Target_Distance_Down_R2 = 0.25f; // 底部(下降到位)

    float Empty_Gravity_Compensation[2] = {-0.2f, -0.2f}; // 空载重力补偿
    float Load_Gravity_Compensation[2] = {-2.2f, -2.2f};  // 负载重力补偿
};

inline float Class_Lift::Get_Now_Distance_L()
{
    return Now_Distance[0];
}

inline float Class_Lift::Get_Now_Distance_R()
{
    return Now_Distance[1];
}

inline void Class_Lift::Yaw_Flag_True()
{
    Yaw_Flag = true;
}

#endif
