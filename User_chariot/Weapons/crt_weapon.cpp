#include "crt_weapon.h"
#include "dvc_serialscreen.h"
#include "ita_robot.h"

/**
 * @brief 初始化
 */
void Class_Weapon::Init()
{
    // 夹取舵机
#if defined(SKILL_COMPETITION_1)
    Pick_Servo[0].Init(&htim1, TIM_CHANNEL_3, 500, 2500);
#endif
#if !defined(SKILL_COMPETITION_2)
    Pick_Servo[1].Init(&htim1, TIM_CHANNEL_1, 500, 2500);
#endif
#if defined(SKILL_COMPETITION_1)
    Pick_Servo[2].Init(&htim2, TIM_CHANNEL_3, 500, 2500);
#endif

    // 抓取电机 (CAN1, ID 0x206)
    Grab_Servo.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x206,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 30.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 15.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 1.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 10.0f,
                // .Dead_Zone = 0.5f,
            },
        });

    // 机械臂电机 (CAN1, ID 0x205)
    Motor_Arm.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x205,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 50.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 6.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
                .Dead_Zone = 0.5f,
            },
        },
        3591.0f / 187.0f / 18.0f * 28.0f);

    // 移动电机 (CAN1, ID 0x204)
    Motor_Move.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x204,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 30.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 10.0f,
                .Dead_Zone = 0.015f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
                .Dead_Zone = 0.5f,
            },
        });

    // 旋转电机
    Motor_Rotate.Init(&hfdcan1, 0xFD, 0x7F, Motor_RS_PID_Mode_ANGLE);
    Motor_Rotate.PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f);
    Motor_Rotate.PID_Angle.Init(30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.002f, 0.0f);

    // 俯仰电机
    Motor_Pitch.Init(
        &hfdcan3,
        Motor::Motor_DJI_ID_0x205,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 40.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 8.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f,
                .Dead_Zone = 0.5f,
            },
        });

    Motor_Rotate.CAN_Send_Enter();

    FSM_Weapon.Weapon = this;

    FSM_Weapon.Init(MAX_WEAPON_STATUS, Weapon_Status_Init);
}

/**
 * @brief 移动控制
 * @param x 目标位置
 */
void Class_Weapon::Move_To_Position(float x)
{
    if (!Move_Calibrated)
    {
        // 低速运行到堵转位置，完成机械零点校准
        float offset;
        if (Motor_Move.Calibrate(Move_Calibration_Param, offset))
        {
            Move_Calibration_Offset = offset;
            Motor_Move.Set_Target_Position(Move_Calibration_Offset);
            Move_Calibrated = true;
        }
    }
    else
    {
        // 已校准，角度环运行到目标绝对角度
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(x + Move_Calibration_Offset);
    }
}

/**
 * @brief 机械臂控制
 * @param x 目标位置
 */
#if !defined(SKILL_COMPETITION_2)
void Class_Weapon::Arm_To_Position(float x)
{
    if (!Arm_Calibrated)
    {
        // 低速运行到堵转位置，完成机械零点校准
        float offset;
        if (Motor_Arm.Calibrate(Arm_Calibrate_Params, offset))
        {
            Arm_Calibration_Offset = offset;
            Motor_Arm.Set_Target_Position(Arm_Calibration_Offset);
            Arm_Calibrated = true;
        }
    }
    else
    {
        // 已校准，角度环运行到目标绝对角度
        Motor_Arm.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Arm.Set_Target_Position(x + Arm_Calibration_Offset);
    }
}
#endif
/**
 * @brief 抓取控制
 * @param x 目标位置
 */
void Class_Weapon::Grab_To_Position(float x)
{
    if (!Grab_Calibrated)
    {
        float offset;
        if (Grab_Servo.Calibrate(Grab_Calibrate_Params, offset))
        {
            Grab_Calibration_Offset = offset;
            Grab_Servo.Set_Target_Position(Grab_Calibration_Offset);
            Grab_Calibrated = true;
        }
    }
    else
    {
        Grab_Servo.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Grab_Servo.Set_Target_Position(x + Grab_Calibration_Offset);
    }
}

/**
 * @brief 俯仰控制
 * @param x 目标位置
 */
void Class_Weapon::Pitch_To_Position(float x)
{
    if (!Pitch_Calibrated)
    {
        // 低速运行到堵转位置，完成机械零点校准
        float offset;
        if (Motor_Pitch.Calibrate(Pitch_Calibrate_Params, offset))
        {
            Pitch_Calibration_Offset = offset;
            Motor_Pitch.Set_Target_Position(Pitch_Calibration_Offset);
            Pitch_Calibrated = true;
        }
    }
    else
    {
        // 已校准，角度环运行到目标绝对角度
        Motor_Pitch.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Pitch.Set_Target_Position(x + Pitch_Calibration_Offset);
    }
}
float position = -0.3f;
float angle = 0.0f;
/**
 * @brief 周期中断函数
 */
void Class_Weapon::TIM_Weapon_PeriodElapsedCallback()
{
    FSM_Weapon.Weapon_TIM_Status_PeriodElapsedCallback();

    // Motor_Rotate.Set_Target_Angle(angle + Rotate_Bias_Rad);
    Motor_Rotate.TIM_Calculate_PeriodElapsedCallback();
    Motor_Rotate.TIM_Send_PeriodElapsedCallback();

#if !defined(SKILL_COMPETITION_2)
    Motor_Arm.Calculate();
#endif
    // Grab_To_Position(position);
    Grab_Servo.Calculate();

    // Pitch_To_Position(0.00f);
    Motor_Pitch.Calculate();

    Motor_Move.Calculate();
}

/**
 * @brief 存活检测
 */
void Class_Weapon::TIM_Alive_PeriodElapsedCallback()
{
    Motor_Rotate.TIM_Alive_PeriodElapsedCallback();

#if !defined(SKILL_COMPETITION_2)
    Motor_Arm.TIM_100ms_Alive_PeriodElapsedCallback();
#endif

    Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    Grab_Servo.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Pitch.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 判断当前动作是否完成
 */
bool Class_Weapon::Is_Action_Finished()
{
#if !defined(SKILL_COMPETITION_2)
    Check_Arm_Task_Completion();
#endif

    Check_Move_Task_Completion();

    Check_Pitch_Task_Completion();

    Check_Rotate_Task_Completion();

    return
#if !defined(SKILL_COMPETITION_2)
        Arm_Task_Finished &&
#endif
        Move_Task_Finished && Pitch_Task_Finished;
}

/**
 * @brief 机械臂到位判定
 */
void Class_Weapon::Check_Arm_Task_Completion()
{
    if (Arm_Calibrated &&
        Math_Abs(Motor_Arm.Get_Target_Position() - Motor_Arm.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Arm.Get_Target_Omega() - Motor_Arm.Get_Now_Omega()) < Omega_Threshold)
    {
        Arm_Task_Finished = true;
    }
}

/**
 * @brief 移动到位判定
 */
void Class_Weapon::Check_Move_Task_Completion()
{
    if (Move_Calibrated &&
        Math_Abs(Motor_Move.Get_Target_Position() - Motor_Move.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Move.Get_Target_Omega() - Motor_Move.Get_Now_Omega()) < Omega_Threshold)
    {
        Move_Task_Finished = true;
    }
}

/**
 * @brief 俯仰到位判定
 */
void Class_Weapon::Check_Pitch_Task_Completion()
{
    if (Pitch_Calibrated &&
        Math_Abs(Motor_Pitch.Get_Target_Position() - Motor_Pitch.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Pitch.Get_Target_Omega() - Motor_Pitch.Get_Now_Omega()) < Omega_Threshold)
    {
        Pitch_Task_Finished = true;
    }
}

/**
 * @brief 旋转到位判定
 */
void Class_Weapon::Check_Rotate_Task_Completion()
{
    if (Math_Abs(Motor_Rotate.Get_Target_Angle() - Motor_Rotate.Get_Now_Angle() < Position_Threshold) &&
        Math_Abs(Motor_Rotate.Get_Target_Omega() - Motor_Rotate.Get_Now_Omega()) < Omega_Threshold)
    {
        Rotate_Task_Finished = true;
    }
}

/**
void Class_Weapon::Check_Servo_Task_Completion()
{
    // 防止数组越界
    if (FSM_Weapon.Get_Now_Status_Serial() == Weapon_Status_Init)
    {
        Servo_Task_Finished = true;
        return;
    }

    // 前一状态舵机位置为0且当前状态不为0时（从关到开），启动延时
    if (Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]<= 0.001f && Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial() - 1] >= 0.999f)
    {
        if (Servo_Delay_Timer.start_time == 0)
        {
            Servo_Delay_Timer.start_time = DWT_GetCurrentTimeUs();
            Servo_Delay_Timer.expire_time = Servo_Delay_Us;
            Servo_Task_Finished = false;
        }
        else if (Is_Timer_ExpiredUs(&Servo_Delay_Timer, Expire_Once))
        {
            Servo_Task_Finished = true;
            Servo_Delay_Timer.start_time = 0;
        }
        else
        {
            Servo_Task_Finished = false;
        }
    }
    else
    {
        // 从开到关或保持不变，瞬间完成
        Servo_Task_Finished = true;
    }
}
*/

/**
 * @brief 清除完成标志
 */
void Class_Weapon::Enter_New_Status_Clear_Completion_Flag()
{
#if !defined(SKILL_COMPETITION_2)
    Arm_Task_Finished = false;
#endif
    Move_Task_Finished = false;
    Pitch_Task_Finished = false;
    Rotate_Task_Finished = false;
    // Servo_Task_Finished = false;
}

/**
 * @brief 夹取状态任务
 */
void Class_Weapon::Weapon_Grab_Status_Task()
{
    // 旋转电机
    Motor_Rotate.Set_Target_Angle(Target.Rotate_Target_Position[FSM_Weapon.Get_Now_Status_Serial()] + Rotate_Bias_Rad);

    // 机械臂电机
#if !defined(SKILL_COMPETITION_2)
    Arm_To_Position(Target.Arm_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
#endif

    // 位移电机
    Move_To_Position(Move_Target_Position[Move_Index]);

    // 俯仰电机
    Pitch_To_Position(Target.Pitch_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 夹取舵机
#if defined(SKILL_COMPETITION_1)
    if (Need_All_Servo_Action)
    {
        Pick_Servo[0].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
        Pick_Servo[1].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
        Pick_Servo[2].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    }
    else
    {
        Pick_Servo[2 - Move_Index].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    }
#elif defined(MAIN_COMPETITION)
    Pick_Servo[1].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
#endif

    // 抓取电机
    Grab_To_Position(Target.Grab_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
}

/**
 * @brief 状态周期中断函数
 *
 */
void Class_FSM_Weapon::Weapon_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
        case Weapon_Status_Init:
        {
            Weapon->Need_All_Servo_Action = true;

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            // }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
        case Weapon_Status_Grab_Prepare:
        {
            Weapon->Need_All_Servo_Action = true;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            // }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }

            break;
        }

        case Weapon_Status_Grab:
        {
            Weapon->Need_All_Servo_Action = true;

            Weapon->Weapon_Grab_Status_Task();
            if (Weapon->Forward_Yaw_Flag)
            {
                // if (Weapon->Is_Action_Finished())
                // {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
                // }
            }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }

            break;
        }

        case Weapon_Status_Lift_1:
        {
            Weapon->Need_All_Servo_Action = true;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            // }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Pick:
        {
            Weapon->Need_All_Servo_Action = true;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {

            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            // }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }

            break;
        }

        case Weapon_Status_Lift_2_Prepare:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Lift_2:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            // }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Rotate_To_Connection:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Lift_2);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            // }
            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Show_Completion_Graph:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial - 1);
            }
            
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }

            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_1);
            }
            break;
        }
#endif

        case Weapon_Status_Rotate_To_Storage_Prepare:
        {
            Weapon->Need_All_Servo_Action = false;

#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Lift_2_Prepare);
            }
#endif

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Now_Status_Serial + 1);
            }
            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Attack_Postition_1:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Rotate_To_Storage_Prepare);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Attack_Postition_2);
            }
            // }
            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0);
            }
            break;
        }

        case Weapon_Status_Attack_Postition_2:
        {
            Weapon->Need_All_Servo_Action = false;

            if (Weapon->Backward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Rotate_To_Storage_Prepare);
            }

            Weapon->Weapon_Grab_Status_Task();

            // if (Weapon->Is_Action_Finished())
            // {
            if (Weapon->Forward_Yaw_Flag)
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Weapon->Enter_New_Status_Clear_Completion_Flag();
                Set_Status(Weapon_Status_Attack_Postition_1);
            }
            // }
            if (Weapon->Chariot->Get_Robot_Mode() == Robot_Mode_Weapon  && Weapon->Chariot->CRSF.Get_SA() == CRSF_SWITCH_HIGH)
            {
                Weapon->Chariot->Serial_Screen.Jump_To_Page(SCREEN_PAGE_0 );
            }
            break;
        }
    }

    // 清空标志位
    Weapon->Forward_Yaw_Flag = false;
    Weapon->Backward_Yaw_Flag = false;
}