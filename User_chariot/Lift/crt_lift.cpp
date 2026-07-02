#include "crt_lift.h"

void Class_Lift::Init()
{
    // Lift抬升电机 (CAN3, ID 0x207-0x208)
    Motor_Lift_L.Init(&hfdcan3, Motor::Motor_DJI_ID_0x207,
                      Motor::Class_Motor_DJI_C620::Parameters{
                          .PID_Omega = PID_Parameters{
                              .K_P = 1.8f,
                              .K_I = 0.0f,
                              .K_D = 0.0f,
                              .Out_Max = 20.0f,
                          },
                      },
                      3591.0f / 187.0f * 50.0f / 20.0f);
    Motor_Lift_R.Init(&hfdcan3, Motor::Motor_DJI_ID_0x208,
                      Motor::Class_Motor_DJI_C620::Parameters{
                          .PID_Omega = PID_Parameters{
                              .K_P = 1.8f,
                              .K_I = 0.0f,
                              .K_D = 0.0f,
                              .Out_Max = 20.0f,
                          },
                      },
                      3591.0f / 187.0f * 50.0f / 20.0f);

    Class_MultiMotorSync_Base<2>::Init({&Motor_Lift_L, &Motor_Lift_R},
                                       Class_MultiMotorSync_Base::Parameters{
                                           .PID_Distance = {
                                               PID_Parameters{
                                                   // 左同步带
                                                   .K_P = 1500.0f,
                                                   .K_I = 0.0f,
                                                   .K_D = 0.0f,
                                                   .Out_Max = 3.0f,
                                               },
                                               PID_Parameters{
                                                   // 右同步带
                                                   .K_P = 1500.0f,
                                                   .K_I = 0.0f,
                                                   .K_D = 0.0f,
                                                   .Out_Max = 3.0f,
                                               },
                                           },
                                           .Max_Velocity = 12.0f,
                                           .Distance_Approach_Threshold = 0.01f,
                                           .Speed_Approach_Threshold = 0.005f,
                                           .Angle_To_Distance = 1.0f * 34.0f * 0.005f / (2.0f * PI),
                                           .Direction_Sign = {1, 1},
                                           .Calibrate = {
                                               .motion_mode = CALIBRATE_MOTION_SPEED,
                                               .motion_value = -5.0f,
                                               .detect_mode = CALIBRATE_DETECT_SPEED,
                                               .detect_threshold = 0.005f,
                                               .debounce_us = 200000,
                                           }});

    FSM_Lift.Lift = this;
    FSM_Lift.Init(3, Lift_Status_Init);
}

void Class_Lift::TIM_Calculate_PeriodElapsedCallback()
{
    // 1. 校准(未完成则执行校准并跳过控制)
    if (!Get_Is_Calibrated())
    {
        Calibrate_Update();
        return;
    }

    // 2. 更新行程
    Distance_Update();

    // 3. 状态机更新
    FSM_Lift.Lift_TIM_Status_PeriodElapsedCallback();

    // 4. 执行运动控制
    Move_To_Position();

    // 5. 电机计算(速度PID → 电流输出)
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
        case Lift_Status_Init:
        {
            Lift->Motor_Lift_L.Set_Feedforward_Current(Lift->Empty_Gravity_Compensation[0]);
            Lift->Motor_Lift_R.Set_Feedforward_Current(Lift->Empty_Gravity_Compensation[1]);

            Lift->Set_Target_Position(Lift->Target_Distance_Init);

            if (Lift->Get_Is_Motion_Finished() && Lift->Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Lift_Status_Wait_R2);
            }
            break;
        }
        case Lift_Status_Wait_R2:
        {
            Lift->Motor_Lift_L.Set_Feedforward_Current(Lift->Empty_Gravity_Compensation[0]);
            Lift->Motor_Lift_R.Set_Feedforward_Current(Lift->Empty_Gravity_Compensation[1]);

            Lift->Set_Target_Position(Lift->Target_Distance_Wait_R2);

            if (Lift->Get_Is_Motion_Finished() && Lift->Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Lift_Status_Lift_R2);
            }
            break;
        }

        case Lift_Status_Lift_R2:
        {
            Lift->Motor_Lift_L.Set_Feedforward_Current(Lift->Load_Gravity_Compensation[0]);
            Lift->Motor_Lift_R.Set_Feedforward_Current(Lift->Load_Gravity_Compensation[1]);

            Lift->Set_Target_Position(Lift->Target_Distance_Lift_R2);

            if (Lift->Get_Is_Motion_Finished() && Lift->Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Lift_Status_Down_R2);
            }
            break;
        }

        case Lift_Status_Down_R2:
        {
            Lift->Motor_Lift_L.Set_Feedforward_Current(Lift->Load_Gravity_Compensation[0]);
            Lift->Motor_Lift_R.Set_Feedforward_Current(Lift->Load_Gravity_Compensation[1]);

            Lift->Set_Target_Position(Lift->Target_Distance_Down_R2);

            if (Lift->Get_Is_Motion_Finished() && Lift->Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Lift_Status_Wait_R2);
            }
            break;
        }
    }
    // Yaw_Flag 仅生效一次，每个状态只响应一次触发
    Lift->Yaw_Flag = false;
}
