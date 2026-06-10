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

    // 底盘速度值PID
    Class_PID PID_Velocity_X;

    // 底盘速度方向PID
    Class_PID PID_Velocity_Y;

    // 底盘角速度PID
    Class_PID PID_Omega;

    // 舵向电机
    Motor::Class_Motor_DJI_C610 Motor_Steer[4];

    // 轮向电机
    Class_Motor_MKSESC Motor_Wheel[4];

    // 四个舵轮编码器
    Class_Swerve_Steer_Encoder Steer_Encoder[4];

    // 四个舵轮模块
    Class_Swerve_Module Swerve_Modules[4];

    void Init();

    inline float Get_Now_Velocity_X();

    inline float Get_Now_Velocity_Y();

    inline float Get_Now_Omega();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);

    inline void Set_Target_Omega(float __Target_Omega);

    inline float Get_Angle_Yaw();

    inline float Get_Target_Steer_Angle(uint8_t x);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_1ms_Control_PeriodElapsedCallback();

protected:
    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE;

    // 舵向电机角度目标值
    float Target_Steer_Angle[4];
    // 当前舵向角度
    float Now_Steer_Angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 轮组方位角
    float Steer_Azimuth[4] = {-0.785, -2.356, 2.356, 0.785}; // 轮组方位角，单位为弧度，顺时针为正，前x右y上z

    // 初始化相关常量
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    float Angle_Yaw = 0.0f;

    // 内部变量

    // 舵向电机PID 参数
    Motor::PID_Parameters PID_Position_Parameters = {
        .K_P = 0.0f,
        .K_I = 0.0f,
        .K_D = 0.0f,

    };
    Motor::PID_Parameters PID_Omega_Parameters = {
        .K_P = 0.0f,
        .K_I = 0.0f,
        .K_D = 0.0f,
    };

    // 轮向参数
    static constexpr float Wheel_Radius = 0.0265f;         // m
    static constexpr float Wheel_Motor_Reduction = 7.5f;    // 减速比
    static constexpr float Wheel_To_Core_Distance[4] = {0.18f, 0.18f, 0.18f, 0.18f}; // m, 四轮到几何中心距离
    static constexpr float Swerve_Module_Force_To_Current = 0.2f; // N -> A

    // 轮向电机静摩擦阻力电流值
    float Static_Resistance_Wheel_Current[4] = {3.0f,
                                                3.0f,
                                                3.0f,
                                                3.0f};
    // 轮向电机动摩擦阻力电流值
    float Dynamic_Resistance_Wheel_Current[4] = {0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f};
    // 轮向电机输出电流值，小于这个电流则不进行输出，防止底盘静止时，高频注入发出噪声
    float Wheel_Current_Limit = 0.0f;

    // 轮向电机摩擦阻力连续化的角速度阈值
    float Wheel_Resistance_Omega_Threshold = 1.0f;
    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.1f;

    // 轮向电机电流目标值
    float Target_Wheel_Current[4];
    // 轮向电机角速度目标值
    float Target_Wheel_Omega[4];

    // 底盘合力 (Calculate 输出)
    float Chassis_Force_X = 0.0f;
    float Chassis_Force_Y = 0.0f;
    float Chassis_Torque = 0.0f;

    // 每个轮向的牵引力 (Calculate 输出)
    float Wheel_Force[4];

    // 低电流前馈控制相关参数

    // 低电流死区设置
    // float Low_Current_Deadzone = 0.3f;
    float Low_Current_Deadzone = 0.1f;
    // float Low_Current_Threshold = 1.9f;  // 低电流阈
    float Low_Current_Threshold = 1.3f;                          // 低电流阈
                                                                 // float Low_Current_Feedforward[4] = {1.6f, 1.6f, 1.6f, 1.6f};  // 低电流前馈值
    float Low_Current_Feedforward[4] = {1.3f, 1.3f, 1.3f, 1.3f}; // 低电流前馈值
    // 读变量

    // 写变量

    // 读写变量

    // 舵向电机标定状态
    Enum_Chassis_Control_Type Steer_Calibration_Status[4] = {Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE,
                                                             Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE,
                                                             Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE,
                                                             Enum_Chassis_Control_Type::Chassis_Control_Type_DISABLE};
    // 舵向标定电机误差
    float Steer_Calibration_Error[4] = {0};
    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;

    // 内部函数

    void Calculate();
    
    void Self_Resolution();

    void Steer_Angle_Self_Resolution();

    void Kinematics_Inverse_Resolution();
};

inline float Class_Chassis::Get_Now_Velocity_X()
{
    return Now_Velocity_X;
}

inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return Now_Velocity_Y;
}

inline float Class_Chassis::Get_Now_Omega()
{
    return Now_Omega;
}

inline Class_Chassis::Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return Chassis_Control_Type;
}

inline float Class_Chassis::Get_Target_Velocity_X()
{
    return Target_Velocity_X;
}

inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return Target_Velocity_Y;
}

inline float Class_Chassis::Get_Target_Omega()
{
    return Target_Omega;
}

inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline float Class_Chassis::Get_Angle_Yaw()
{
    return Angle_Yaw;
}

inline float Class_Chassis::Get_Target_Steer_Angle(uint8_t x)
{
    if (x < 4)
    {
        return Target_Steer_Angle[x];
    }
    else
    {
        return 0.0f;
    }
}

#endif // CRT_CHASSIS_H
