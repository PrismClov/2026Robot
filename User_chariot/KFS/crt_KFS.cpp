#include "crt_KFS.h"

void Class_KFS::Init()
{
    // 抬升电机初始化 (CAN1, ID 0x202-0x203)
    Motor_Lift[0].Init(&hfdcan1, Motor::Motor_DJI_ID_0x202,
                       Motor::Class_Motor_DJI_C620::Parameters{
                           .PID_Omega = PID_Parameters{
                               .K_P = 1.0f,
                               .K_I = 0.0f,
                               .K_D = 0.0f,
                               .Out_Max = 20.0f,
                           },
                       },3591.0f / 187.0f * 30.0f / 20.0f);
    Motor_Lift[1].Init(&hfdcan1, Motor::Motor_DJI_ID_0x203,
                       Motor::Class_Motor_DJI_C620::Parameters{
                           .PID_Omega = PID_Parameters{
                               .K_P = 1.0f,
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
                          .K_P = 1200.0f,
                          .K_I = 0.0f,
                          .K_D = 0.0f,
                          .Out_Max = 5.0f,
                      },
                      PID_Parameters{
                          .K_P = 1200.0f,
                          .K_I = 0.0f,
                          .K_D = 0.0f,
                          .Out_Max = 5.0f,
                      },
                  },
                  .Distance_Approach_Threshold = 0.01f, // 速度环位置环切换阈值
                  .Max_Velocity = 12.0f, // 速度
                  .Angle_To_Distance = 0.16f / (2.0f * PI),
                  .Direction_Sign = {1, -1},  // 右电机镜像安装，方向反向
                  .Calibrate = {
                      .motion_mode = CALIBRATE_MOTION_SPEED,
                      .motion_value = -5.0f,
                      .detect_mode = CALIBRATE_DETECT_SPEED,
                      .detect_threshold = 0.5f,
                      .debounce_us = 200000,
                  },
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
                .Out_Max = 7.0f, 
                .Dead_Zone = 0.1f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f,  
                .Dead_Zone = 0.5f, 
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
        Is_Move_Calibrated = Motor_Move.Calibrate(move_calibarate_param, Calibrate_Offset);
    }
    else
    {
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(x + Calibrate_Offset);
    }
}

void Class_KFS::Move_To_Height(float height)
{
    Target_Distance = height;
}

void Class_KFS::TIM_Control_PeriodElapsedCallback()
{
    // 抬升校准（未完成则执行校准并跳过控制）
    if (!Lift.Get_Is_Calibrated())
    {
        Lift.Calibrate_Update();
        return;
    }

    // Motor_Move.Update_Feedback();
    // Move_To_Position(Target_Distance);
    // Motor_Move.Calculate();

    // 抬升控制
    Lift.Set_Target_Position(Target_Distance);
    Lift.Distance_Update();
    Lift.Move_To_Position();
    for (int i = 0; i < 2; i++)
    {
        Motor_Lift[i].Set_Feedforward_Current(Force_Compensation[i]);
        Motor_Lift[i].Calculate();
    }
}

void Class_KFS::TIM_Alive_PeriodElapsedCallback()
{
    // Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Lift[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Lift[1].TIM_100ms_Alive_PeriodElapsedCallback();
}