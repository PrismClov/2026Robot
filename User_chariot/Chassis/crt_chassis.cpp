#include "crt_chassis.h"

void Class_Chassis::Init()
{
    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.002f);

    // 舵向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].Init(&hfdcan1, static_cast<Enum_Motor_DJI_C610_ID>(0x201 + i),
                            Motor::Class_Motor_DJI_C610::Parameters{PID_Position_Parameters, PID_Omega_Parameters, false}, 36.0f, 10.0f);
    }

    // 模组绑定电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Swerve_Modules[i].Init(Motor_Steer[i], Motor_Wheel[i], Class_Swerve_Module::Parameters{}, Class_Swerve_Module::Mode::Force);
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

void Class_Chassis::Kinematics_Inverse_Resolution()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        /*
         * 全向运动学逆解：
         *   v_i = v_chassis + omega x r_i
         *
         * r_i 从中心指向轮子位置:
         *   r_x = distance * cos(azimuth)
         *   r_y = distance * sin(azimuth)
         *
         * omega x r 在底盘坐标系下:
         *   (omega x r)_x = -omega * r_y
         *   (omega x r)_y =  omega * r_x
         */
        float r_x = Wheel_To_Core_Distance[i] * arm_cos_f32(Steer_Azimuth[i]);
        float r_y = Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i]);

        float v_module_x = Target_Velocity_X - Target_Omega * r_y;
        float v_module_y = Target_Velocity_Y + Target_Omega * r_x;

        float angle = atan2f(v_module_y, v_module_x);
        float speed = sqrtf(v_module_x * v_module_x + v_module_y * v_module_y);

        Swerve_Modules[i].Set_Target_Angle(angle);
        Swerve_Modules[i].Set_Target_Speed(speed);
        Swerve_Modules[i].Set_Target_Force((v_module_x * arm_cos_f32(Now_Steer_Angle[i]) + v_module_y * arm_sin_f32(Now_Steer_Angle[i])) * 1.0f);
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
    Kinematics_Inverse_Resolution();

    for (int i = 0; i < 4; i++)
    {
        Swerve_Modules[i].TIM_1ms_PeriodElapsedCallback();
    }

    Self_Resolution();

    Steer_Angle_Self_Resolution();
}
