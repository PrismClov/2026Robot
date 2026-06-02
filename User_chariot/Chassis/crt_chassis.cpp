#include "crt_chassis.h"

void Class_Chassis::Init()
{
    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.002f);

    // 舵向电机PID
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].Init(&hfdcan1, static_cast<Enum_Motor_DJI_C610_ID>(0x201 + i), 36.0f, 10.0f);
        Motor_Steer[i].PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.002f);
        Motor_Steer[i].PID_Position.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.002f);
    }

    // 模组绑定电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Swerve_Modules[i].Init(Motor_Steer[i], Motor_Wheel[i], Class_Swerve_Module::Parameters{}, Class_Swerve_Module::Mode::Force);
    }
}

void Class_Chassis::Self_Resolution()
{
    
}


