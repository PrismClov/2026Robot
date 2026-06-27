#ifndef DVC_MOTOR_DM_PID_H
#define DVC_MOTOR_DM_PID_H

#include "dvc_motor_dm.h"
#include "alg_pid.h"

enum Enum_Motor_DM_PID_Mode
{
    Motor_DM_PID_Mode_NONE = 0,
    Motor_DM_PID_Mode_OMEGA,
    Motor_DM_PID_Mode_ANGLE,
};

class Class_Motor_DM_PID : public Class_Motor_DM_Normal
{
public:
    Class_PID PID_Angle;
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
              Enum_Motor_DM_PID_Mode __Mode = Motor_DM_PID_Mode_OMEGA,
              float __Angle_Max = 12.5f, float __Omega_Max = 25.0f,
              float __Torque_Max = 10.0f, float __Current_Max = 10.261194f);

    inline Enum_Motor_DM_PID_Mode Get_PID_Mode();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Feedforward_Omega();
    inline float Get_Feedforward_Torque();

    inline void Set_PID_Mode(Enum_Motor_DM_PID_Mode __Mode);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Feedforward_Omega(float __Feedforward_Omega);
    inline void Set_Feedforward_Torque(float __Feedforward_Torque);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    Enum_Motor_DM_PID_Mode Motor_DM_PID_Mode = Motor_DM_PID_Mode_NONE;

    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;
    float Feedforward_Omega = 0.0f;
    float Feedforward_Torque = 0.0f;

    void PID_Calculate();
};

inline Enum_Motor_DM_PID_Mode Class_Motor_DM_PID::Get_PID_Mode()
{
    return Motor_DM_PID_Mode;
}

inline float Class_Motor_DM_PID::Get_Target_Angle()
{
    return Target_Angle;
}

inline float Class_Motor_DM_PID::Get_Target_Omega()
{
    return Target_Omega;
}

inline float Class_Motor_DM_PID::Get_Feedforward_Omega()
{
    return Feedforward_Omega;
}

inline float Class_Motor_DM_PID::Get_Feedforward_Torque()
{
    return Feedforward_Torque;
}

inline void Class_Motor_DM_PID::Set_PID_Mode(Enum_Motor_DM_PID_Mode __Mode)
{
    Motor_DM_PID_Mode = __Mode;
}

inline void Class_Motor_DM_PID::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DM_PID::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Motor_DM_PID::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

inline void Class_Motor_DM_PID::Set_Feedforward_Torque(float __Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

#endif
