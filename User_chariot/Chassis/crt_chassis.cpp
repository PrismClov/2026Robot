#include "crt_chassis.h"
#include <math.h>

/**
 * @brief 初始化底盘 PID、电机、编码器和舵轮模块
 */
void Class_Chassis::Init()
{
    // 底盘速度PID
    PID_Velocity_X.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.001f);
    PID_Velocity_Y.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.001f);
    PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.001f);

    // 舵向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].Init(&hfdcan1, static_cast<Motor::Enum_Motor_DJI_ID>(0x201 + i),
                            Motor::Class_Motor_DJI_C610::Parameters{
                                .PID_Position = Motor::PID_Parameters{
                                    .K_P = 23.0f,
                                    .K_I = 0.0f,
                                    .K_D = 0.0f,
                                    .K_F = 0.0f,
                                    .I_Out_Max = 0.0f,
                                    .Out_Max = 10.0f,
                                },
                                .PID_Omega = Motor::PID_Parameters{
                                    .K_P = 1.3f,
                                    .K_I = 0.0f,
                                    .K_D = 0.0f,
                                    .K_F = 0.0f,
                                    .I_Out_Max = 3.0f,
                                    .Out_Max = 6.0f,
                                },
                                .Use_External_Position_Feedback = true,
                            },
                            108.0f, 10.0f);
    }

    // 舵轮编码器
    for (uint8_t i = 0; i < 4; i++)
    {
        Steer_Encoder[i].Init(&hfdcan3, 0x201 + i);
    }

    // 轮向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Init(&hfdcan2, 0x31 + i, 14, 6.2831f, 50.0f, 50.0f, 30.0f, MOTOR_CONTROL_METHOD_CURRENT);
    }

    // 模组绑定电机和编码器
    for (uint8_t i = 0; i < 4; i++)
    {
        Swerve_Modules[i].Init(Motor_Steer[i], Motor_Wheel[i], Steer_Encoder[i],
                               Class_Swerve_Module::Parameters{
                                   .Motor_Kt = 1.0f,
                                   .Wheel_Radius = 0.0535f,
                                   .Wheel_Motor_Reduction = 1.0f,
                               },
                               Class_Swerve_Module::Mode::Force);
    }
}

void Class_Chassis::Self_Resolution()
{
    float tmp_velocity_x = 0.0f;
    float tmp_velocity_y = 0.0f;
    float tmp_omega = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        float steer_angle = Swerve_Modules[i].Get_Current_Angle();
        float wheel_radps = Motor_Wheel[i].Get_Now_Omega() / Swerve_Modules[i].Get_Parameters().Wheel_Motor_Reduction;
        float v_x = wheel_radps * Swerve_Modules[i].Get_Parameters().Wheel_Radius * arm_cos_f32(steer_angle);
        float v_y = wheel_radps * Swerve_Modules[i].Get_Parameters().Wheel_Radius * arm_sin_f32(steer_angle);
        float omega_contrib = wheel_radps * Swerve_Modules[i].Get_Parameters().Wheel_Radius * arm_sin_f32(steer_angle - Steer_Azimuth[i]) / Wheel_To_Core_Distance[i];

        tmp_velocity_x += v_x / 4.0f;
        tmp_velocity_y += v_y / 4.0f;
        tmp_omega += omega_contrib / 4.0f;
    }

    Now_Velocity_X = tmp_velocity_x;
    Now_Velocity_Y = tmp_velocity_y;
    Now_Omega = tmp_omega;
}

void Class_Chassis::Calculate()
{
    if (Chassis_Control_Type != Enum_Chassis_Control_Type::Chassis_Control_Type_NORMAL)
    {
        PID_Velocity_X.Set_Integral_Error(0.0f);
        PID_Velocity_Y.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        for (int i = 0; i < 4; i++)
        {
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

    // 合力分解至每个轮向的牵引力
    for (int i = 0; i < 4; i++)
    {
        float steer_angle = Swerve_Modules[i].Get_Current_Angle();

        Wheel_Force[i] = Chassis_Force_X * arm_cos_f32(steer_angle) + Chassis_Force_Y * arm_sin_f32(steer_angle) - Chassis_Torque / Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i] - steer_angle);
    }
}

void Class_Chassis::Kinematics_Inverse_Resolution()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        float x_i = Wheel_To_Core_Distance[i] * arm_cos_f32(Steer_Azimuth[i]);
        float y_i = Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i]);

        float v_wheel_x = Target_Velocity_X - Target_Omega * y_i;
        float v_wheel_y = Target_Velocity_Y + Target_Omega * x_i;

        float target_angle = atan2(v_wheel_y, v_wheel_x);

        Swerve_Modules[i].Set_Target_Angle(target_angle);
        Swerve_Modules[i].Set_Target_Force(Wheel_Force[i]);
    }
}

void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
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
}
