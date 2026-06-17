#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "alg_pid.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_mksesc.h"
#include "dvc_steer_encoder.h"
#include "dvc_swerve_module.h"

/**
 * @brief 力控舵轮底盘
 */
class Class_Chassis
{
public:
    /**
     * @brief 底盘控制类型
     */
    enum class Enum_Chassis_Control_Type
    {
        Chassis_Control_Type_DISABLE = 0,
        Chassis_Control_Type_NORMAL,
    };

    // 底盘速度PID
    Class_PID PID_Velocity_X;
    Class_PID PID_Velocity_Y;
    Class_PID PID_Omega;

    // 舵向电机
    Motor::Class_Motor_DJI_C610 Motor_Steer[4];

    // 轮向电机
    Class_Motor_MKSESC Motor_Wheel[4];

    // 四个舵轮编码器
    Class_Swerve_Steer_Encoder Steer_Encoder[4];

    // 四个舵轮模块
    Class_Swerve_Module Swerve_Modules[4];

    float Velocity_X_Max = 1.5f;
    float Velocity_Y_Max = 1.5f;
    float Omega_Max = 3.0f;

    void Init();

    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_1ms_Control_PeriodElapsedCallback();

protected:
    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE;

    // 轮组安装方位角 rad，前x右y上z
    static constexpr float Steer_Azimuth[4] = {-0.785f, -2.356f, 2.356f, 0.785f};

    // 当前底盘速度
    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega = 0.0f;

    // 目标速度
    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega = 0.0f;

    // 底盘合力 (Calculate 输出)
    float Chassis_Force_X = 0.0f;
    float Chassis_Force_Y = 0.0f;
    float Chassis_Torque = 0.0f;

    // 每个轮向的牵引力 (Calculate 输出)
    float Wheel_Force[4];

    // 轮向参数
    static constexpr float Wheel_To_Core_Distance[4] = {0.18f, 0.18f, 0.18f, 0.18f}; // m, 四轮到几何中心距离

    // 内部函数
    void Calculate();
    void Self_Resolution();
    void Kinematics_Inverse_Resolution();
};

/**
 * @brief 获取当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return Now_Velocity_X;
}

/**
 * @brief 获取当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return Now_Velocity_Y;
}

/**
 * @brief 获取当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return Now_Omega;
}

/**
 * @brief 获取底盘控制类型
 */
inline Class_Chassis::Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return Chassis_Control_Type;
}

/**
 * @brief 获取目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return Target_Velocity_X;
}

/**
 * @brief 获取目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return Target_Velocity_Y;
}

/**
 * @brief 获取目标角速度
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return Target_Omega;
}

/**
 * @brief 获取速度X上限
 */
inline float Class_Chassis::Get_Velocity_X_Max()
{
    return Velocity_X_Max;
}

/**
 * @brief 获取速度Y上限
 */
inline float Class_Chassis::Get_Velocity_Y_Max()
{
    return Velocity_Y_Max;
}

/**
 * @brief 获取角速度上限
 */
inline float Class_Chassis::Get_Omega_Max()
{
    return Omega_Max;
}

/**
 * @brief 设置底盘控制类型
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设置目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设置目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设置目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

#endif // CRT_CHASSIS_H
