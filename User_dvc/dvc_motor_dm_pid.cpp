#include "dvc_motor_dm_pid.h"

void Class_Motor_DM_PID::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
                              Enum_Motor_DM_PID_Mode __Mode, float __Angle_Max, float __Omega_Max,
                              float __Torque_Max, float __Current_Max)
{
    Class_Motor_DM_Normal::Init(hfdcan, __CAN_Rx_ID, __CAN_Tx_ID,
                                Motor_DM_Control_Method_NORMAL_MIT,
                                __Angle_Max, __Omega_Max, __Torque_Max, __Current_Max);
    Set_PID_Mode(__Mode);
}

void Class_Motor_DM_PID::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    float tmp_value = Target_Torque + Feedforward_Torque;
    Math_Constrain(&tmp_value, -Torque_Max, Torque_Max);
    Control_Torque = tmp_value;
}

void Class_Motor_DM_PID::PID_Calculate()
{
    switch (Motor_DM_PID_Mode)
    {
    case Motor_DM_PID_Mode_NONE:
    {
        break;
    }
    case Motor_DM_PID_Mode_OMEGA:
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    case Motor_DM_PID_Mode_ANGLE:
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    default:
    {
        Target_Torque = 0.0f;
        break;
    }
    }
}
