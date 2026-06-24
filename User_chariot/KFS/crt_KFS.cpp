#include "crt_KFS.h"

void Class_KFS::Init()
{
    // 抬升电机初始化 (CAN1, ID 0x202-0x203)
    Motor_Lift[0].Init(&hfdcan1, Motor::Motor_DJI_ID_0x202,
                       Motor::Class_Motor_DJI_C620::Parameters{
                           .PID_Omega = PID_Parameters{
                               .K_P = 0.0f,
                               .K_I = 0.0f,
                               .K_D = 0.0f,
                               .Out_Max = 20.0f,
                           },
                       },3591.0f / 187.0f * 30.0f / 20.0f);
    Motor_Lift[1].Init(&hfdcan1, Motor::Motor_DJI_ID_0x203,
                       Motor::Class_Motor_DJI_C620::Parameters{
                           .PID_Omega = PID_Parameters{
                               .K_P = 0.0f,
                               .K_I = 0.0f,
                               .K_D = 0.0f,
                               .Out_Max = 20.0f,
                           },
                       },3591.0f / 187.0f * 30.0f / 20.0f);

    // 抬升路程环
    Lift.Init({&Motor_Lift[0], &Motor_Lift[1]},
              Class_MultiMotorSync_Base<2>::Parameters{
                  .PID_Distance = {
                      PID_Parameters{
                          .K_P = 0.0f,
                          .K_I = 0.0f,
                          .K_D = 0.0f,
                          .Out_Max = 5.0f,
                      },
                      PID_Parameters{
                          .K_P = 0.0f,
                          .K_I = 0.0f,
                          .K_D = 0.0f,
                          .Out_Max = 5.0f,
                      },
                  },
                  .Angle_To_Distance = 0.16f / (2.0f * PI), // 32齿×5mm齿距=160mm周长
                  .Calibrate = {.motion_mode = CALIBRATE_MOTION_NONE},
              });

    // 移动电机初始化 (CAN1, ID 0x201)
    Motor_Move.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x201,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 8.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 7.0f, // 位置环输出速度目标限制
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f,  // 速度环输出电流目标限制
                .Dead_Zone = 0.5f, // 速度环死区，避免小幅抖动
            },
        });

    // // 机械臂电机初始化
    // Motor_Arm.Init(&hfdcan2, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT);

    // // 手腕电机初始化
    // Motor_Wrist.Init(&hfdcan2, 0x03, Motor_DM_Control_Method_NORMAL_MIT);

    // // 气泵初始化
    // Air_Pump.Init(GPIOC, GPIO_PIN_13);

    // // 状态机初始化
    // FSM_KFS.KFS = this;
    // FSM_KFS.Init(1, 0);
}

void Class_KFS::Move_To_Position(float x)
{
    if (!Is_Move_Calibrated)
    {
        Is_Move_Calibrated = Motor_Move.Calibrate(calibarate_param, Calibrate_Offset);
    }
    else
    {
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(x + Calibrate_Offset);
    }
}

void Class_KFS::TIM_Control_PeriodElapsedCallback()
{
    // Motor_Move.Update_Feedback();
    // Motor_Move.Calculate();

    // 抬升控制
    Lift.Set_Target_Position(Target_Distance);
    Lift.Distance_Update();
    Lift.Move_To_Position();
    for (int i = 0; i < 2; i++)
    {
        Motor_Lift[i].Update_Feedback();
        Motor_Lift[i].Calculate();
    }
}

void Class_KFS::TIM_Alive_PeriodElapsedCallback()
{
    // Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Lift[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Lift[1].TIM_100ms_Alive_PeriodElapsedCallback();
}