/**
 * @file crt_chassis.cpp
 * @author hzy by Lucy (2478427315@qq.com)
 * @brief 舵轮底盘电控
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Robopioneer (c) 2025-2026
 *
 */

/**
 * @brief 轮组编号
 * 1[0] 4[3]
 * 2[1] 3[2]
 * 前x右y上z
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_dwt.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_mksesc.h"
#include "dvc_steer_encoder.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_UNCALIBRATED = 0,
    Chassis_Control_Type_DISABLE,
    Chassis_Control_Type_NORMAL,
    Chassis_Control_Type_LOW_CURRENT_FEEDFORWARD,
};

/**
 * @brief 底盘舵向角度标定枚举
 *
 */
enum Enum_Chassis_Steer_Calibration_Type
{
    Chassis_Steer_Calibration_Type_UNCALIBRATED = 0, // 未标定
    Chassis_Steer_Calibration_Type_CALIBRATED,       // 标定完成
};

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
class Class_Chassis
{
public:
    Class_Slope Slope_Velocity_X;

    Class_Slope Slope_Velocity_Y;

    Class_Slope Slope_Omega;

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

    // 编码器
    Class_Swerve_Steer_Encoder Steer_Encoder[4];

    void Init(float __Velocity_X_Max = 8.0f, float __Velocity_Y_Max = 8.0f, float __Omega_Max = 16.0f);

    inline float Get_Velocity_X_Max();

    inline float Get_Velocity_Y_Max();

    inline float Get_Omega_Max();

    inline float Get_Now_Velocity_X();

    inline float Get_Now_Velocity_Y();

    inline float Get_Now_Omega();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline float Get_Now_Steer_Angle(uint8_t num);

    inline float Get_Now_Target_Wheel_Current(uint8_t num);

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);

    inline void Set_Target_Omega(float __Target_Omega);

    inline float Get_Angle_Yaw();

    void Steer_Angle_Set(uint8_t __Steer_Motor_ID);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_2ms_Control_PeriodElapsedCallback();

protected:
    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_UNCALIBRATED;

    // 舵向电机角度目标值
    float Target_Steer_Angle[4];
    // 当前舵向角度
    float Now_Steer_Angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 轮组方位角
    float Steer_Azimuth[4] = {0.785, 2.356, 3.925, 5.495}; // 轮组方位角，单位为弧度，顺时针为正，前x右y上z

    // 初始化相关常量
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 速度X限制
    float Velocity_X_Max = 0.0f;
    // 速度Y限制
    float Velocity_Y_Max = 0.0f;
    // 角速度限制
    float Omega_Max = 0.0f;

    float Angle_Yaw = 0.0f;

    // 内部变量

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
    Enum_Chassis_Steer_Calibration_Type Steer_Calibration_Status[4] = {Chassis_Steer_Calibration_Type_UNCALIBRATED,
                                                                       Chassis_Steer_Calibration_Type_UNCALIBRATED,
                                                                       Chassis_Steer_Calibration_Type_UNCALIBRATED,
                                                                       Chassis_Steer_Calibration_Type_UNCALIBRATED};
    // 舵向标定电机误差

    float steer_offset_deg[4] = {
        // 0.0f, 0.0f, 0.0f, 0.0f}; // 舵向电机标定误差，单位为度，顺时针为正，前x右y上z
        -53.105652f,   // [0] 编码器朝前 307.33°
        90.252700806f, // [1] 编码器朝前 90.55°
        -138.162598f,  // [2] 编码器朝前 45.75°
        -28.231094f,   // [3] 编码器朝前 324.14°
    };

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;

    // 内部函数

    void Self_Resolution();

    void Steer_Angle_Self_Resolution();

    void Kinematics_Inverse_Resolution();

    void Output_To_Dynamics();

    void Dynamics_Inverse_Resolution();

    void _Steer_Motor_Kinematics_Nearest_Transposition();

    void Output_To_Motor();
};

// 常量

// const float Steer_Azimuth[4] = {0.0f, 0.0f, 0.0f, 0.0f,};
//  轮组半径(米)
const float Wheel_Radius = 0.0535f;
// 轮距中心长度(米)
const float Wheel_To_Core_Distance[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // m, 四轮到几何中心距离
// 舵向电机减速比
const float Steer_Motor_Reduction = 1.0f;
// 轮向电机行星减速比
const float Wheel_Motor_Reduction = 1.0f;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标速度方向
 *
 * @return float 目标速度方向
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 获取当前偏航角
 *
 * @return float 当前偏航角（弧度）
 */
inline float Class_Chassis::Get_Angle_Yaw()
{
    return (Angle_Yaw);
}

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Steer_Angle(uint8_t num)
{
    return (Now_Steer_Angle[num]);
}

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Target_Wheel_Current(uint8_t num)
{
    return (Target_Wheel_Current[num]);
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
