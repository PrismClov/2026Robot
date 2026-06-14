#include "crt_chassis.h"
#include <math.h>

/**
 * @brief 初始化底盘 PID、电机、编码器和舵轮模块
 */
void Class_Chassis::Init()
{
    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.001f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.001f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.001f);

    // 舵向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].Init(&hfdcan1, static_cast<Motor::Enum_Motor_DJI_ID>(0x201 + i),
                            Motor::Class_Motor_DJI_C610::Parameters{PID_Position_Parameters, PID_Omega_Parameters, false}, 36.0f, 10.0f);
    }

    // 舵轮编码器
    for (uint8_t i = 0; i < 4; i++)
    {
        Steer_Encoder[i].Init(&hfdcan3, 0x201 + i); // CAN ID: 0x201, 0x202, 0x203, 0x204
    }

    // 模组绑定电机和编码器
    for (uint8_t i = 0; i < 4; i++)
    {
        Swerve_Modules[i].Init(Motor_Steer[i], Motor_Wheel[i], Steer_Encoder[i], Class_Swerve_Module::Parameters{}, Class_Swerve_Module::Mode::Force);
    }
}

void Class_Chassis::Self_Resolution()
{
    float tmp_velocity_x = 0.0f;
    float tmp_velocity_y = 0.0f;
    float tmp_omega = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        Now_Steer_Angle[i] = Swerve_Modules[i].Get_Current_Angle();
    }

    for (int i = 0; i < 4; i++)
    {
        float wheel_radps = Motor_Wheel[i].Get_Now_Omega() / Wheel_Motor_Reduction;
        float v_x = wheel_radps * Wheel_Radius * arm_cos_f32(Now_Steer_Angle[i]);
        float v_y = wheel_radps * Wheel_Radius * arm_sin_f32(Now_Steer_Angle[i]);
        float omega_contrib = wheel_radps * Wheel_Radius * arm_sin_f32(Now_Steer_Angle[i] - Steer_Azimuth[i]) / Wheel_To_Core_Distance[i];

        tmp_velocity_x += v_x / 4.0f;
        tmp_velocity_y += v_y / 4.0f;
        tmp_omega += omega_contrib / 4.0f;
    }

    Now_Velocity_X = tmp_velocity_x;
    Now_Velocity_Y = tmp_velocity_y;
    Now_Omega = tmp_omega;
}

void Class_Chassis::Steer_Angle_Self_Resolution()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        float angle = Swerve_Modules[i].Get_Target_Angle();
        if (isfinite(angle))
        {
            Target_Steer_Angle[i] = angle;
        }
    }
}

void Class_Chassis::Calculate()
{
    if (Chassis_Control_Type != Enum_Chassis_Control_Type::Chassis_Control_Type_NORMAL)
    {
        for (int i = 0; i < 4; i++)
        {
            PID_Velocity_X.Set_Integral_Error(0.0f);
            PID_Velocity_Y.Set_Integral_Error(0.0f);
            PID_Omega.Set_Integral_Error(0.0f);
            Wheel_Force[i] = 0.0f;
        }
        Target_Velocity_X = 0.0f;
        Target_Velocity_Y = 0.0f;
        Target_Omega = 0.0f;
        Chassis_Force_X = 0.0f;
        Chassis_Force_Y = 0.0f;
        Chassis_Torque = 0.0f;
        return;
    }

    // 底盘速度PID
    PID_Velocity_X.Set_Target(Target_Velocity_X);
    PID_Velocity_X.Set_Now(Now_Velocity_X);
    PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    PID_Velocity_Y.Set_Target(Target_Velocity_Y);
    PID_Velocity_Y.Set_Now(Now_Velocity_Y);
    PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    PID_Omega.Set_Target(Target_Omega);
    PID_Omega.Set_Now(Now_Omega);
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();

    Chassis_Force_X = PID_Velocity_X.Get_Out();
    Chassis_Force_Y = PID_Velocity_Y.Get_Out();
    Chassis_Torque = PID_Omega.Get_Out();

    // 合力分解至每个轮向
    for (int i = 0; i < 4; i++)
    {
        float steer_angle = Now_Steer_Angle[i];

        Wheel_Force[i] = Chassis_Force_X * arm_cos_f32(steer_angle) + Chassis_Force_Y * arm_sin_f32(steer_angle) - Chassis_Torque / Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i] - steer_angle);
    }
}

void Class_Chassis::Kinematics_Inverse_Resolution()
{
    // 逆运动学解算：根据目标底盘速度计算每个舵轮的目标角度和轮速
    for (uint8_t i = 0; i < 4; i++)
    {
        // 轮子 i 的位置坐标 (相对于底盘中心)
        float x_i = Wheel_To_Core_Distance[i] * arm_cos_f32(Steer_Azimuth[i]);
        float y_i = Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i]);

        // 轮子线速度 = 底盘平移速度 + 旋转速度
        // v_wheel = v_chassis + omega × r
        // v_wheel_x = vx - omega * y_i
        // v_wheel_y = vy + omega * x_i
        float v_wheel_x = Target_Velocity_X - Target_Omega * y_i;
        float v_wheel_y = Target_Velocity_Y + Target_Omega * x_i;

        // 目标舵角 = atan2(vy, vx)
        Target_Steer_Angle[i] = atan2f(v_wheel_y, v_wheel_x);

        // 设置模块目标
        Swerve_Modules[i].Set_Target_Angle(Target_Steer_Angle[i]);
        Swerve_Modules[i].Set_Target_Force(Wheel_Force[i]);
    }
}

void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}

void Class_Chassis::TIM_1ms_Control_PeriodElapsedCallback()
{
    Calculate();

    Kinematics_Inverse_Resolution();

    for (int i = 0; i < 4; i++)
    {
        Swerve_Modules[i].TIM_1ms_PeriodElapsedCallback();
    }

    Self_Resolution();

    Steer_Angle_Self_Resolution();
}
