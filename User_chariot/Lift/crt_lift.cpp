#include "crt_lift.h"

void Class_Lift::Init()
{
    Motor_Lift_L.Init(&hfdcan2, Motor::Motor_DJI_ID_0x201,
                      Motor::Class_Motor_DJI_C620::Parameters{
                          .PID_Omega = PID_Parameters{
                              .K_P = 6.0f,
                              .K_I = 0.1f,
                              .K_D = 0.0f,
                              .Out_Max = 20.0f,
                          },
                      });
    Motor_Lift_R.Init(&hfdcan2, Motor::Motor_DJI_ID_0x202,
                      Motor::Class_Motor_DJI_C620::Parameters{
                          .PID_Omega = PID_Parameters{
                              .K_P = 6.0f,
                              .K_I = 0.1f,
                              .K_D = 0.0f,
                              .Out_Max = 20.0f,
                          },
                      });

    Motor_Lift_L.Set_Feedforward_Omega(0.6f);
    Motor_Lift_R.Set_Feedforward_Omega(0.6f);

    // 电机 rad → 抬升行程(米) 转换:
    // 减速比 2.5:1, 34齿同步轮, 5mm齿距, 电机转一圈行程 = 34*0.005 / (2*PI*gear) m/rad
    float gear = 2.50f / 1.0f;
    float angle_to_dist = -1.0f * 34.0f * 0.005f / (2.0f * PI * gear);

    Class_MultiMotorSync_Base::Init({&Motor_Lift_L, &Motor_Lift_R},
                          Class_MultiMotorSync_Base::Parameters{
                              .PID_Distance = {
                                  PID_Parameters{
                                      // 左同步带
                                      .K_P = -60.0f,
                                      .K_I = 0.07f,
                                      .K_D = 0.02f,
                                      .Out_Max = 3.0f,
                                  },
                                  PID_Parameters{
                                      // 右同步带
                                      .K_P = -60.0f,
                                      .K_I = 0.07f,
                                      .K_D = 0.02f,
                                      .Out_Max = 3.0f,
                                  },
                              },
                              .Distance_Approach_Threshold = 0.01f,
                              .Max_Velocity = 5.0f,
                              .Angle_To_Distance = angle_to_dist,
                              .Calibrate = {.motion_mode = CALIBRATE_MOTION_NONE},
                          });

    FSM_Lift.Lift = this;
    FSM_Lift.Init(3, Lift_Status_Wait_R2); // 3个状态, 初始为Wait
}

bool Class_Lift::Is_Wait_R2_Finished_step()
{
    return (Math_Abs(Now_Distance[0] - Target_Distance_Wait_R2[0]) <= Distance_Error) &&
           (Math_Abs(Now_Distance[1] - Target_Distance_Wait_R2[1]) <= Distance_Error) &&
           (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
           (Motor_Lift_R.Get_Now_Omega() <= 0.05f);
}

bool Class_Lift::Is_Lift_R2_Finished_step()
{
    return (Math_Abs(Now_Distance[0] - Target_Distance_Lift_R2[0]) <= Distance_Error) &&
           (Math_Abs(Now_Distance[1] - Target_Distance_Lift_R2[1]) <= Distance_Error) &&
           (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
           (Motor_Lift_R.Get_Now_Omega() <= 0.05f);
}

bool Class_Lift::Is_Down_R2_Finished_step()
{
    return (Math_Abs(Now_Distance[0] - Target_Distance_Down_R2[0]) <= Distance_Error) &&
           (Math_Abs(Now_Distance[1] - Target_Distance_Down_R2[1]) <= Distance_Error) &&
           (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
           (Motor_Lift_R.Get_Now_Omega() <= 0.05f);
}

void Class_Lift::UP_Cancel()
{
    Motor_Lift_L.Set_Target_Speed(0.0f);
    Motor_Lift_R.Set_Target_Speed(0.0f);
}

void Class_Lift::TIM_Calculate_PeriodElapsedCallback()
{
    // 更新行程
    Distance_Update();

    // 状态机更新
    FSM_Lift.Lift_TIM_Status_PeriodElapsedCallback();

    // 3. 执行运动控制(开环/闭环切换 + 单步限幅)
    Move_To_Position();

    // 4. 电机计算(速度PID → 电流输出)
    Motor_Lift_L.Calculate();
    Motor_Lift_R.Calculate();
}

void Class_Lift::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Lift_L.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Lift_R.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Class_FSM_Lift::Lift_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
        case Lift_Status_Wait_R2:
        {
            if (!Lift->Is_Wait_R2_Finished_step())
            {
                // 未到位 → 持续向目标运动
                Lift->Move_To(Lift->Target_Distance_Wait_R2[0], Lift->Target_Distance_Wait_R2[1]);
            }
            else if (Lift->Is_Wait_R2_Finished_step())
            {
                Lift->UP_Cancel();

                // 等待Yaw到位信号到来后才切换
                if (Lift->Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Lift_Status_Lift_R2);
                }
            }
            break;
        }

        case Lift_Status_Lift_R2:
        {
            if (!Lift->Is_Lift_R2_Finished_step())
            {
                Lift->Move_To(Lift->Target_Distance_Lift_R2[0], Lift->Target_Distance_Lift_R2[1]);
            }
            else if (Lift->Is_Lift_R2_Finished_step())
            {
                Lift->UP_Cancel();

                if (Lift->Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Lift_Status_Down_R2);
                }
            }
            break;
        }

        case Lift_Status_Down_R2:
        {
            if (!Lift->Is_Down_R2_Finished_step())
            {
                Lift->Move_To(Lift->Target_Distance_Down_R2[0], Lift->Target_Distance_Down_R2[1]);
            }
            else if (Lift->Is_Down_R2_Finished_step())
            {
                Lift->UP_Cancel();

                if (Lift->Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Lift_Status_Wait_R2);
                }
            }
            break;
        }
    }
    // Yaw_Flag 仅生效一次，每个状态只响应一次触发
    Lift->Yaw_Flag = false;
}
