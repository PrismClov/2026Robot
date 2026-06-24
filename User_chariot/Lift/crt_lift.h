#ifndef CRT_LIFT_H
#define CRT_LIFT_H

#include "crt_multi_motor_sync.h"
#include "alg_fsm.h"
#include "dvc_motor_dji.h"

class Class_Lift;

/**
 * @brief R2抬升状态机: Wait(等待/下降到位) → Lift(抬升) → Down(下降) 循环
 *
 * 每次状态切换由 Yaw_Flag 触发，到位后自动停止并等待下次触发。
 */
enum Enum_Lift_Status
{
    Lift_Status_Wait_R2 = 0,
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

    void TIM_Calculate_PeriodElapsedCallback();       // 1ms控制回调
    void Init();

    void Move_To(float target_l, float target_r);     // 设定期望目标位置(米)

    bool Is_Wait_R2_Finished_step();
    bool Is_Lift_R2_Finished_step();
    bool Is_Down_R2_Finished_step();

    void UP_Cancel();                                  // 急停
    void TIM_100ms_Alive_PeriodElapsedCallback();

    inline float Get_Now_Distance_L();
    inline float Get_Now_Distance_R();
    inline void Set_Offset(float __offset_l, float __offset_r);  // 设定机械零点偏移(米)
    inline void Yaw_Flag_True();
    inline float Get_Velocity_Max();

private:
    bool Yaw_Flag = false;                             // Yaw到位触发，FSM检测到后切换状态

    float Max_Velocity = 200.0f;                       // 冗余，实际使用Param.Max_Velocity

    float Target_Distance_Wait_R2[2] = {-0.40f, -0.40f};
    float Target_Distance_Lift_R2[2] = {-0.15f, -0.15f};
    float Target_Distance_Down_R2[2] = {-0.40f, -0.40f};

    float Distance_Error = 0.008f;                     // 到位判定误差(米)
};

inline void Class_Lift::Move_To(float target_l, float target_r)
{
    Target_Distance[0] = target_l;
    Target_Distance[1] = target_r;
}

inline float Class_Lift::Get_Now_Distance_L()
{
    return Now_Distance[0];
}

inline float Class_Lift::Get_Now_Distance_R()
{
    return Now_Distance[1];
}

inline void Class_Lift::Set_Offset(float __offset_l, float __offset_r)
{
    Offset[0] = __offset_l / Param.Angle_To_Distance;
    Offset[1] = __offset_r / Param.Angle_To_Distance;
}

inline void Class_Lift::Yaw_Flag_True()
{
    Yaw_Flag = true;
}

inline float Class_Lift::Get_Velocity_Max()
{
    return Max_Velocity;
}

#endif
