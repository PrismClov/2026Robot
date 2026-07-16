#include "crt_KFS.h"
#include "ita_robot.h"

/**
 * @brief 初始化
 */
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
                .Dead_Zone = 0.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 1.5f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f,
                .Dead_Zone = 0.5f,
            },
        });

    // 机械臂电机初始化
    Motor_Arm.Init(&hfdcan1, 0x00, 0x01, Motor_DM_PID_Mode_ANGLE);
    Motor_Arm.PID_Angle.Init(125.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f);
    Motor_Arm.PID_Omega.Init(0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.002f, 0.0f);

    // 手腕电机初始化
    Motor_Wrist.Init(&hfdcan1, 0xF0, 0x70, Motor_RS_PID_Mode_ANGLE);
    Motor_Wrist.PID_Omega.Init(0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f);
    Motor_Wrist.PID_Angle.Init(30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 15.0f, 0.002f, 0.0f);

    Arm_Speed_Slope.Init(Arm_Max_Velocity * 0.02f, Arm_Max_Velocity * 0.1f, Slope_First_REAL);
    Wrist_Speed_Slope.Init(Wrist_Max_Velocity * 0.02f, Wrist_Max_Velocity * 0.1f, Slope_First_REAL);

    Motor_Arm.CAN_Send_Enter();
    Motor_Wrist.CAN_Send_Enter();

    // 气泵初始化
    Air_Pump.Init(GPIOC, GPIO_PIN_13);

    // 状态机初始化
    FSM_KFS.KFS = this;
    FSM_KFS.Init(MAX_KFS_STATUS, KFS_Status_Init);
}

/**
 * @brief 移动控制
 * @param x 目标位置
 */
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

/**
 * @brief 抬升控制
 * @param height 目标高度
 */
void Class_KFS::Lift_To_Height(float height)
{
    Lift.Set_Target_Position(height);
}

/**
 * @brief 手腕控制
 * @param angle 目标角度
 */
void Class_KFS::Wrist_To_Angle(float angle)
{
    float wrist_error = angle - Processed_Wrist_Angle_Rad;
    float motor_target = -angle + Wrist_Parallel_With_Arm_Angle_Offset;

    if (Math_Abs(wrist_error) > Wrist_Distance_Lock_Threshold &&
        (Math_Abs(Motor_Wrist.Get_Target_Angle() - motor_target) > 0.001f || Motor_Wrist.Get_PID_Mode() != Motor_RS_PID_Mode_ANGLE))
    {
        Wrist_Speed_Slope.Set_Now_Real(Motor_Wrist.Get_Now_Omega());
        Wrist_Speed_Slope.Set_Target(wrist_error > 0.0f ? -Wrist_Max_Velocity : Wrist_Max_Velocity);
        Wrist_Speed_Slope.TIM_Calculate_PeriodElapsedCallback();

        Motor_Wrist.Set_PID_Mode(Motor_RS_PID_Mode_OMEGA);
        Motor_Wrist.Set_Target_Omega(Wrist_Speed_Slope.Get_Out());
    }
    else
    {
        Motor_Wrist.Set_PID_Mode(Motor_RS_PID_Mode_ANGLE);
        Motor_Wrist.Set_Target_Angle(motor_target);
    }
}

/**
 * @brief 机械臂控制
 * @param angle 目标角度
 */
void Class_KFS::Arm_To_Angle(float angle)
{
    float arm_error = angle - Processed_Arm_Angle_Rad;
    float motor_target = Motor_Arm.Get_Now_Angle() + arm_error;

    if (Math_Abs(arm_error) > Arm_Distance_Lock_Threshold &&
        (Math_Abs(Motor_Arm.Get_Target_Angle() - motor_target) > 0.001f || Motor_Arm.Get_PID_Mode() != Motor_DM_PID_Mode_ANGLE))
    {
        Arm_Speed_Slope.Set_Now_Real(Motor_Arm.Get_Now_Omega());
        Arm_Speed_Slope.Set_Target(arm_error > 0.0f ? Arm_Max_Velocity : -Arm_Max_Velocity);
        Arm_Speed_Slope.TIM_Calculate_PeriodElapsedCallback();

        Motor_Arm.Set_PID_Mode(Motor_DM_PID_Mode_OMEGA);
        Motor_Arm.Set_Target_Omega(Arm_Speed_Slope.Get_Out());
    }
    else
    {
        Motor_Arm.Set_PID_Mode(Motor_DM_PID_Mode_ANGLE);
        Motor_Arm.Set_Target_Angle(motor_target);
    }
}

/**
 * @brief 控制回调
 */
void Class_KFS::TIM_Control_PeriodElapsedCallback()
{
    // 更新反馈
    Motor_Move.Update_Feedback();
    Lift.Distance_Update();
    Update_Processed_Arm_Angle_Rad();
    Update_Processed_Wrist_Angle_Rad();

    // 状态机驱动
    FSM_KFS.KFS_TIM_Status_PeriodElapsedCallback();

    // 移动电机
    Motor_Move.Calculate();

    // 抬升
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
    Motor_Arm.Set_Feedforward_Torque(Arm_Gravity_Compensation_Ratio * arm_cos_f32(Processed_Arm_Angle_Rad) -
                                     (float)Is_KFS_Picked * KFS_Gravity_Compensation_Ratio_Wrist * arm_cos_f32(Processed_Arm_Angle_Rad + Processed_Wrist_Angle_Rad) +
                                     (float)Is_KFS_Picked * KFS_Gravity_Compensation_Ratio_Arm * arm_cos_f32(Processed_Arm_Angle_Rad));

    Motor_Arm.TIM_Calculate_PeriodElapsedCallback();
    Motor_Arm.TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 存活检测
 */
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
    Processed_Wrist_Angle_Rad = -(Motor_Wrist.Get_Now_Angle() - Wrist_Parallel_With_Arm_Angle_Offset);
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
    // 更新气泵消抖（每周期必须执行，不受gate影响）
    Check_Pump_Task_Completion();

    // 气泵（不受消抖影响，优先执行）
    Target.Pump_Status[FSM_KFS.Get_Now_Status_Serial()] ? Air_Pump.AIRPUMP_Open() : Air_Pump.AIRPUMP_Close();

    if (Pump_Task_Finished || FSM_KFS.Status[FSM_KFS.Get_Now_Status_Serial()].Count_Time == 1)
    {
        // 移动
        Move_To_Position(Target.Move_Position[FSM_KFS.Get_Now_Status_Serial()]);

        // 抬升
        Lift_To_Height(Target.Lift_Height[Lift_Height_Index][FSM_KFS.Get_Now_Status_Serial()]);

        // 手腕角度
        Wrist_To_Angle(Target.Wrist_Angle[FSM_KFS.Get_Now_Status_Serial()]);

        // 机械臂角度

        Arm_To_Angle(Target.Arm_Angle[FSM_KFS.Get_Now_Status_Serial()] + (Add_Bias ? Arm_Bias_Rad : 0.0f));
    }
}

/**
 * @brief 判断当前动作是否完成
 */
bool Class_KFS::Is_Action_Finished()
{
    Check_Move_Task_Completion();

    Check_Lift_Task_Completion();

    Check_Wrist_Task_Completion();

    Check_Arm_Task_Completion();

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
    if (Math_Abs(Processed_Wrist_Angle_Rad - Target.Wrist_Angle[FSM_KFS.Get_Now_Status_Serial()]) < Wrist_Distance_Approach_Threshold &&
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
    if (Math_Abs(Processed_Arm_Angle_Rad - Target.Arm_Angle[FSM_KFS.Get_Now_Status_Serial()]) < Arm_Distance_Approach_Threshold &&
        Math_Abs(Motor_Arm.Get_Now_Omega()) < Arm_Speed_Approach_Threshold)
    {
        Arm_Task_Finished = true;
    }
}

/**
 * @brief 气泵吸→放切换消抖判定
 *
 * 上一状态为吸(1)、当前状态为放(0)时，强制1s消抖。
 */
void Class_KFS::Check_Pump_Task_Completion()
{
    if (FSM_KFS.Get_Now_Status_Serial() == 0)
    {
        Pump_Task_Finished = true;
        return;
    }

    uint8_t prev_pump = Target.Pump_Status[(Enum_KFS_Status)(FSM_KFS.Get_Now_Status_Serial() - 1)];
    uint8_t now_pump = Target.Pump_Status[FSM_KFS.Get_Now_Status_Serial()];

    if (prev_pump == 1 && now_pump == 0)
    {
        if (Pump_Debounce_Timer.start_time == 0)
        {
            Pump_Debounce_Timer.start_time = DWT_GetCurrentTimeUs();
            Pump_Debounce_Timer.expire_time = 2000000;
        }
        if (Is_Timer_ExpiredUs(&Pump_Debounce_Timer, Expire_Once))
        {
            Pump_Task_Finished = true;
        }
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
    Pump_Debounce_Timer.start_time = 0;
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
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

#if defined(SKILL_COMPETITION_1) || defined(MAIN_COMPETITION)
        case KFS_Status_First_Pick_Prepare:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_First_Pick:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_First_Pick_Up:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_First_Pick_Prepare);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Prepare_Storage_Lift_Up:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Storage:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_First_Pick_Up);
            }

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Storage_Lift_Down:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_First_Pick_Up);
            }

            if (KFS->Forward_Yaw_Flag)
            {
                KFS->Is_KFS_Picked = 0;
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Storage_Arm_Up:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Protect_Storage:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Second_Pick_Arm_Up:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Second_Pick_Prepare:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Second_Pick:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

#endif

        case KFS_Status_Second_Pick_Up:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
#if defined(SKILL_COMPETITION_1) || defined(MAIN_COMPETITION)
            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Second_Pick_Prepare);
            }
#endif
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

#if defined(SKILL_COMPETITION_1) || defined(MAIN_COMPETITION)
        case KFS_Status_Prepare_Protect_Arm_Wrist:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Protect_Storage_Again:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                if (Now_Status_Serial + 1 < MAX_KFS_STATUS)
                {
                    Set_Status(Now_Status_Serial + 1);
                }
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }
#endif

#if defined(MAIN_COMPETITION)
        case KFS_Status_First_Release_Prepare:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 1;
            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_First_Release:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Get_Storage_Prepare:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Is_Action_Finished() || KFS->Forward_Yaw_Flag)
            {

                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Get_Storage:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }
#endif

#if !defined(SKILL_COMPETITION_1)
        case KFS_Status_Second_Release_Prepare:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 1;
            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Second_Release:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Recover_Init);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }
#endif

        case KFS_Status_Pick_From_Ground_Prepare:
        {
            KFS->Add_Bias = false;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Pick_From_Ground:
        {
            KFS->Add_Bias = false;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Pick_From_Ground_Prepare);
            }

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Pick_Up_From_Ground:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Pick_From_Ground_Prepare);
            }

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Third_Release_Prepare:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 1;
            KFS->KFS_Status_Task();

            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Pick_Up_From_Ground);
            }

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case KFS_Status_Third_Release:
        {
            KFS->Add_Bias = true;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();

            if (KFS->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

#if !defined(SKILL_COMPETITION_1)
        case KFS_Status_Recover_Init:
        {
            KFS->Add_Bias = false;
            KFS->Is_KFS_Picked = 0;
            KFS->KFS_Status_Task();
            if (KFS->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                KFS->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(KFS_Status_Pick_From_Ground_Prepare);
            }
            // 末状态保持即可

            if (KFS->Chariot->Get_Robot_Mode() == Robot_Mode_KFS)
            {
                KFS->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }
#endif
    }

    // 清空标志位
    KFS->Forward_Yaw_Flag = false;
    KFS->Backward_Yaw_Flag = false;
}