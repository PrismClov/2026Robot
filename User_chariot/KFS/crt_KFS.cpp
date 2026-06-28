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
                       },
                       3591.0f / 187.0f * 30.0f / 20.0f);
    Motor_Lift[1].Init(&hfdcan1, Motor::Motor_DJI_ID_0x203,
                       Motor::Class_Motor_DJI_C620::Parameters{
                           .PID_Omega = PID_Parameters{
                               .K_P = 1.0f,
                               .K_I = 0.0f,
                               .K_D = 0.0f,
                               .Out_Max = 20.0f,
                           },
                       },
                       3591.0f / 187.0f * 30.0f / 20.0f);

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
                  .Max_Velocity = 12.0f,                // 速度
                  .Distance_Approach_Threshold = 0.01f, // 速度环位置环切换阈值
                  .Angle_To_Distance = 0.16f / (2.0f * PI),
                  .Direction_Sign = {1, -1}, // 右电机镜像安装，方向反向
                  .Calibrate = {CALIBRATE_MOTION_NONE},
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

    // 机械臂电机初始化
    Motor_Arm.Init(&hfdcan1, 0x00, 0x01, Motor_DM_PID_Mode_ANGLE);
    Motor_Arm.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.002f, 0.0f);
    Motor_Arm.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 0.002f, 0.0f);

    // 主动使能电机
    Motor_Arm.CAN_Send_Enter();

    // 手腕电机初始化
    Motor_Wrist.Init(&hfdcan1, 0xF0, 0x70, Motor_RS_PID_Mode_ANGLE);
    Motor_Wrist.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.002f, 0.0f);
    Motor_Wrist.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 0.002f, 0.0f);

    // // 气泵初始化
    // Air_Pump.Init(GPIOC, GPIO_PIN_13);

    // 状态机初始化
    FSM_KFS.KFS = this;
    FSM_KFS.Init(MAX_KFS_STATUS, KFS_Status_Init);
}

void Class_KFS::Move_To_Position(float x)
{
    if (!Is_Move_Calibrated)
    {
        Is_Move_Calibrated = Motor_Move.Calibrate(move_calibarate_param, Move_Calibrate_Offset);
    }
    else
    {
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(x + Move_Calibrate_Offset);
    }
}

void Class_KFS::Lift_To_Height(float height)
{
    Lift.Set_Target_Position(height);
}

void Class_KFS::Wrist_To_Angle(float angle)
{
    float wrist_error = angle - Processed_Wrist_Angle_Rad;
    if (fabsf(wrist_error) > Wrist_Distance_Approach_Threshold)
    {
        Motor_Wrist.Set_PID_Mode(Motor_RS_PID_Mode_OMEGA);
        Motor_Wrist.Set_Target_Omega((wrist_error > 0.0f ? 1.0f : -1.0f) * Wrist_Max_Velocity);
    }
    else
    {
        Motor_Wrist.Set_PID_Mode(Motor_RS_PID_Mode_ANGLE);
        Motor_Wrist.Set_Target_Angle(angle + Wrist_Parallel_With_Arm_Angle_Offset);
    }
}

void Class_KFS::Arm_To_Angle(float angle)
{
    float arm_error = angle - Processed_Arm_Angle_Rad;
    if (fabsf(arm_error) > Arm_Distance_Approach_Threshold)
    {
        Motor_Arm.Set_PID_Mode(Motor_DM_PID_Mode_OMEGA);
        Motor_Arm.Set_Target_Omega((arm_error > 0.0f ? 1.0f : -1.0f) * Arm_Max_Velocity);
    }
    else
    {
        Motor_Arm.Set_PID_Mode(Motor_DM_PID_Mode_ANGLE);
        Motor_Arm.Set_Target_Angle(angle + Arm_Horizontal_Offset);
    }
}

void Class_KFS::TIM_Control_PeriodElapsedCallback()
{
    // 抬升校准（未完成则执行校准并跳过控制）
    if (!Lift.Get_Is_Calibrated())
    {
        Lift.Calibrate_Update();
        return;
    }

    // 状态机驱动（设置各子系统目标）
    FSM_KFS.KFS_TIM_Status_PeriodElapsedCallback();

    // 更新处理后的角度
    Update_Processed_Arm_Angle_Rad();
    Update_Processed_Wrist_Angle_Rad();

    Motor_Move.Update_Feedback();
    Motor_Move.Calculate();

    // 抬升控制
    Lift.Distance_Update();
    Lift.Move_To_Position();
    for (int i = 0; i < 2; i++)
    {
        Motor_Lift[i].Set_Feedforward_Current(Lift_Force_Compensation[i]);
        Motor_Lift[i].Calculate();
    }

    // 手腕控制
    Motor_Wrist.Set_Feedforward_Torque(((float)Is_KFS_Picked * KFS_Gravity_Compensation_Ratio_Wrist + Wrist_Gravity_Compensation_Ratio) * arm_cos_f32(Processed_Arm_Angle_Rad + Processed_Wrist_Angle_Rad));
    Motor_Wrist.TIM_Calculate_PeriodElapsedCallback();
    Motor_Wrist.TIM_Send_PeriodElapsedCallback();

    // 机械臂控制
    Motor_Arm.Set_Feedforward_Torque(
        Arm_Gravity_Compensation_Ratio * arm_cos_f32(Processed_Arm_Angle_Rad) +                                                            // m1g×d1 × cos(θ₁)
        Wrist_Gravity_Compensation_Ratio * arm_cos_f32(Processed_Arm_Angle_Rad + Processed_Wrist_Angle_Rad) +                              // m2g×d2 × cos(θ₁+θ₂)
        (float)Is_KFS_Picked * (KFS_Gravity_Compensation_Ratio_Arm * arm_cos_f32(Processed_Arm_Angle_Rad) +                                // m3g×d1 × cos(θ₁)
                                KFS_Gravity_Compensation_Ratio_Wrist * arm_cos_f32(Processed_Arm_Angle_Rad + Processed_Wrist_Angle_Rad))); // m3g×d3 × cos(θ₁+θ₂)
    Motor_Arm.TIM_Calculate_PeriodElapsedCallback();
    Motor_Arm.TIM_Send_PeriodElapsedCallback();
}

void Class_KFS::TIM_Alive_PeriodElapsedCallback()
{
    // 移动电机存活函数
    Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    // 手腕电机存活函数
    Motor_Wrist.TIM_Alive_PeriodElapsedCallback();

    // 机械臂电机存活函数
    Motor_Arm.TIM_Alive_PeriodElapsedCallback();

    // 抬升电机存活函数
    Motor_Lift[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Lift[1].TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 获取手腕相对于机械臂的角度，顺时针为正
 */
void Class_KFS::Update_Processed_Wrist_Angle_Rad()
{
    Processed_Wrist_Angle_Rad = Motor_Wrist.Get_Now_Angle() - Wrist_Parallel_With_Arm_Angle_Offset;
}

/**
 * @brief 获取机械臂相对于水平的角度，顺时针为正
 */
void Class_KFS::Update_Processed_Arm_Angle_Rad()
{
    Processed_Arm_Angle_Rad = Motor_Arm.Get_Now_Angle() - Arm_Horizontal_Offset;
}

/**
 * @brief KFS状态机任务函数
 */
void Class_KFS::KFS_Status_Task()
{
    // 四自由度控制
    // 移动
    Move_To_Position(Target.Move_Position[FSM_KFS.KFS_Status]);

    // 抬升
    Lift_To_Height(Target.Lift_Height[Lift_Height_Index][FSM_KFS.KFS_Status]);

    // 手腕角度
    Wrist_To_Angle(Target.Wrist_Angle[FSM_KFS.KFS_Status]);

    // 机械臂角度
    Arm_To_Angle(Target.Arm_Angle[FSM_KFS.KFS_Status]);

    // 气泵
    Target.Pump_Status[FSM_KFS.KFS_Status] ? Air_Pump.AIRPUMP_Open() : Air_Pump.AIRPUMP_Close();
}

/**
 * @brief 判断当前动作是否完成
 */
bool Class_KFS::Is_Action_Finished()
{
    // 避免重复判断耗时
    if (!Move_Task_Finished)
    {
        Check_Move_Task_Completion();
    }

    if (!Lift_Task_Finished)
    {
        Check_Lift_Task_Completion();
    }

    if (!Wrist_Task_Finished)
    {
        Check_Wrist_Task_Completion();
    }

    if (!Arm_Task_Finished)
    {
        Check_Arm_Task_Completion();
    }

    // 在其他动作到位后再开始气泵消抖
    if (Move_Task_Finished && Lift_Task_Finished && Wrist_Task_Finished && Arm_Task_Finished && !Pump_Task_Finished)
    {
        Check_Pump_Task_Completion();
    }

    return Move_Task_Finished && Lift_Task_Finished && Wrist_Task_Finished && Arm_Task_Finished && Pump_Task_Finished;
}

/**
 * @brief 移动电机到位判定
 */
void Class_KFS::Check_Move_Task_Completion()
{
    if (Math_Abs(Motor_Move.Get_Position() - Motor_Move.Get_Target_Position()) < Move_Position_Approach_Threshold &&
        Math_Abs(Motor_Move.Get_Speed()) < Move_Speed_Approach_Threshold)
    {
        Move_Task_Finished = true;
    }
}

/**
 * @brief 抬升到位判定
 */
void Class_KFS::Check_Lift_Task_Completion()
{
    if (Lift.Get_Is_Motion_Finished())
    {
        Lift_Task_Finished = true;
    }
}

/**
 * @brief 手腕到位判定
 */
void Class_KFS::Check_Wrist_Task_Completion()
{
    if (Math_Abs(Processed_Wrist_Angle_Rad - Target.Wrist_Angle[FSM_KFS.KFS_Status]) < Wrist_Distance_Approach_Threshold &&
        Math_Abs(Motor_Wrist.Get_Now_Omega()) < Wrist_Speed_Approach_Threshold)
    {
        Wrist_Task_Finished = true;
    }
}

/**
 * @brief 机械臂到位判定
 */
void Class_KFS::Check_Arm_Task_Completion()
{
    if (Math_Abs(Processed_Arm_Angle_Rad - Target.Arm_Angle[FSM_KFS.KFS_Status]) < Arm_Distance_Approach_Threshold &&
        Math_Abs(Motor_Arm.Get_Now_Omega()) < Arm_Speed_Approach_Threshold)
    {
        Arm_Task_Finished = true;
    }
}

/**
 * @brief 判断气泵动作是否完成
 */
void Class_KFS::Check_Pump_Task_Completion()
{
    // 防止数组越界
    if (FSM_KFS.KFS_Status == KFS_Status_Init)
    {
        Pump_Task_Finished = true;
        return;
    }

    // 判断是否夹取KFS
    if (Target.Pump_Status[FSM_KFS.KFS_Status] && !Target.Pump_Status[FSM_KFS.KFS_Status - 1])
    {
        Is_KFS_Picked = 1; // KFS已夹取
        if (Pump_Delay_Timer.start_time == 0)
        {
            Pump_Delay_Timer.start_time = DWT_GetCurrentTimeUs();
            Pump_Delay_Timer.expire_time = Pump_Delay_Us;
            Pump_Task_Finished = false;
        }
        else if (Is_Timer_ExpiredUs(&Pump_Delay_Timer, Expire_Once))
        {
            Pump_Task_Finished = true;
            Pump_Delay_Timer.start_time = 0;
        }
        else
        {
            Pump_Task_Finished = false;
        }
    }
    else if (!Target.Pump_Status[FSM_KFS.KFS_Status] && Target.Pump_Status[FSM_KFS.KFS_Status - 1])
    {
        Is_KFS_Picked = 0; // KFS已释放，释放是瞬间完成的，无需延时
        Pump_Task_Finished = true;
    }
    else
    {
        Pump_Task_Finished = true;
    }
}

/**
 * @brief 进入新状态时清除所有完成标志
 */
void Class_KFS::Enter_New_Status_Clear_Completion_Flag()
{
    Move_Task_Finished = false;
    Lift_Task_Finished = false;
    Wrist_Task_Finished = false;
    Arm_Task_Finished = false;
    Pump_Task_Finished = false;
}

/**
 * @brief KFS状态机定时函数
 */
void Class_FSM_KFS::KFS_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
        case KFS_Status_Init:
        {
            // 初次进入清完成标志
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            // 执行任务
            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_First_Pick_Prepare);
                }
            }
            break;
        }

        case KFS_Status_First_Pick_Prepare:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Init);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_First_Pick);
                }
            }
            break;
        }

        case KFS_Status_First_Pick:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_First_Pick_Up);
            }
            break;
        }

        case KFS_Status_First_Pick_Up:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                KFS->Is_KFS_Picked = 0;
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_First_Pick_Prepare);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Storage);
                }
            }
            break;
        }

        case KFS_Status_Storage:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Protect_Storage);
            }
            break;
        }

        case KFS_Status_Protect_Storage:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Second_Pick_Prepare);
                }
            }
            break;
        }

        case KFS_Status_Second_Pick_Prepare:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Protect_Storage);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Second_Pick);
                }
            }
            break;
        }

        case KFS_Status_Second_Pick:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Second_Pick_Up);
            }
            break;
        }

        case KFS_Status_Second_Pick_Up:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Protect_Storage_Again);
            }
            break;
        }

        case KFS_Status_Protect_Storage_Again:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                KFS->Is_KFS_Picked = 0;
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Second_Pick_Prepare);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_First_Release_Prepare);
                }
            }
            break;
        }
        case KFS_Status_First_Release_Prepare:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Protect_Storage_Again);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_First_Release);
                }
            }
            break;
        }

        case KFS_Status_First_Release:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Get_Storage);
                }
            }
            break;
        }

        case KFS_Status_Get_Storage:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Second_Release_Prepare);
                }
            }
            break;
        }

        case KFS_Status_Second_Release_Prepare:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(KFS_Status_Get_Storage);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Second_Release);
                }
            }
            break;
        }

        case KFS_Status_Second_Release:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished())
            {
                if (KFS->Forward_Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(KFS_Status_Recover_Init);
                }
            }
            break;
        }

        case KFS_Status_Recover_Init:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                KFS->Enter_New_Status_Clear_Completion_Flag();
            }

            KFS->KFS_Status_Task();

            // 末状态保持即可

            break;
        }
    }

    // 清空标志位
    KFS->Forward_Yaw_Flag = false;
    KFS->Backward_Yaw_Flag = false;
}