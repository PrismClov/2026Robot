/**
 * @file dvc_motor_rs_pid.cpp
 * @author lyh
 * @brief 
 * @date 2026-06-16
 *
 */
#include "dvc_motor_rs_pid.h"

/**
 * @brief 灵足电机PID模式初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_RS_PID_Mode 电机PID模式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_RS_PID::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_RS_PID_Mode __Motor_RS_PID_Mode, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max)
{
    Class_Motor_RS_MIT::Init(hfdcan, __CAN_Rx_ID, __CAN_Tx_ID, Motor_RS_Control_Method_NORMAL, __Angle_Max, __Omega_Max, __Torque_Max, __Current_Max);
    Set_PID_Mode(__Motor_RS_PID_Mode);
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_RS_PID::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    float tmp_value = Target_Torque + Feedforward_Torque;
    Math_Constrain(&tmp_value, -Torque_Max, Torque_Max);
    Control_Torque = tmp_value;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_RS_PID::PID_Calculate()
{
    switch (Motor_RS_PID_Mode)
    {
    case (Motor_RS_PID_Mode_NONE):
    {
        break;
    }
    case (Motor_RS_PID_Mode_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    case (Motor_RS_PID_Mode_ANGLE):
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
