/**
 * @file dvc_motor_rs_pid.h
 * @author lyh
 * @version 1.0
 * @date 2026-06-16
 *
 */
#ifndef DVC_MOTOR_RS_PID_H
#define DVC_MOTOR_RS_PID_H

#include "dvc_motor_rs.h"
#include "alg_pid.h"
#include "drv_math.h"

/**
 * @brief 灵足电机PID模式
 * 
 */
enum Enum_Motor_RS_PID_Mode
{
    Motor_RS_PID_Mode_NONE = 0,
    Motor_RS_PID_Mode_OMEGA,
    Motor_RS_PID_Mode_ANGLE,
};

class Class_Motor_RS_PID: public Class_Motor_RS_MIT
{
public:

    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_RS_PID_Mode __Motor_RS_PID_Mode = Motor_RS_PID_Mode_OMEGA, float __Angle_Max = 12.57f, float __Omega_Max = 33.0f, float __Torque_Max = 14.0f, float __Current_Max = 10.261194f);
    
    inline Enum_Motor_RS_PID_Mode Get_PID_Mode();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Torque();

    inline void Set_PID_Mode(Enum_Motor_RS_PID_Mode __Motor_RS_PID_Mode);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Torque(float __Feedforward_Current);

    void TIM_Calculate_PeriodElapsedCallback();


protected:

    // 读写变量

    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的扭矩, N·m
    float Target_Torque = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的扭矩, N·m
    float Feedforward_Torque = 0.0f;
    
    // PID模式
    Enum_Motor_RS_PID_Mode Motor_RS_PID_Mode = Motor_RS_PID_Mode_NONE;

    void PID_Calculate();


};

inline float Class_Motor_RS_PID::Get_Target_Angle()
{
    return Target_Angle;
}

inline float Class_Motor_RS_PID::Get_Target_Omega()
{
    return Target_Omega;
}

inline float Class_Motor_RS_PID::Get_Feedforward_Omega()
{
    return Feedforward_Omega;
}

inline float Class_Motor_RS_PID::Get_Feedforward_Torque()
{
    return Feedforward_Torque;
}

/**
 * @brief 设定电机控制PID模式
 *
 * @param __Motor_RS_PID_Mode 电机控制PID模式
 */
inline void Class_Motor_RS_PID::Set_PID_Mode(Enum_Motor_RS_PID_Mode __Motor_RS_PID_Mode)
{
    Motor_RS_PID_Mode = __Motor_RS_PID_Mode;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_RS_PID::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_RS_PID::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_RS_PID::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, N·m
 *
 * @param __Feedforward_Torque 前馈的扭矩, N·m
 */
inline void Class_Motor_RS_PID::Set_Feedforward_Torque(float __Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}


#endif /* DVC_MOTOR_RS_PID_H */
