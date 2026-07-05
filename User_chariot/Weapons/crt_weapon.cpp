#include "crt_weapon.h"

void Class_Weapon::Init()
{
    // 夹取舵机
    Pick_Servo[0].Init(&htim1, TIM_CHANNEL_1, 500, 2500);
    Pick_Servo[1].Init(&htim1, TIM_CHANNEL_3, 500, 2500);
    Pick_Servo[2].Init(&htim2, TIM_CHANNEL_1, 500, 2500);

    // 抓取舵机
    Grab_Servo.Init(&htim2, TIM_CHANNEL_3, 500, 2500);

    // 机械臂电机 (CAN1, ID 0x205)
    Motor_Arm.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x205,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 6.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 7.5f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
            },
        },
        3591.0f / 187.0f / 18.0f * 28.0f);

    // 移动电机 (CAN1, ID 0x204)
    Motor_Move.Init(
        &hfdcan1,
        Motor::Motor_DJI_ID_0x204,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 0.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
                .Dead_Zone = 0.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
                .Dead_Zone = 0.0f,
            },
        });

    // 旋转电机
    Motor_Rotate.Init(&hfdcan2, 0x01, 0x01, Motor_RS_PID_Mode_ANGLE);
    Motor_Rotate.PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f);
    Motor_Rotate.PID_Angle.Init(30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 15.0f, 0.002f, 0.0f);

    // 俯仰电机
    Motor_Pitch.Init(
        &hfdcan3,
        Motor::Motor_DJI_ID_0x205,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 0.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f,
            },
            .PID_Omega = PID_Parameters{
                .K_P = 0.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f,
            },
        });

    FSM_Weapon.Weapon = this;

    FSM_Weapon.Init(MAX_WEAPON_STATUS, Weapon_Status_Init);
}

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

/**
 * @brief 周期中断函数
 */
float position_arm = 0.0f;
void Class_Weapon::TIM_Weapon_PeriodElapsedCallback()
{
    // FSM_Weapon.Weapon_TIM_Status_PeriodElapsedCallback();

    // Motor_Rotate.TIM_Calculate_PeriodElapsedCallback();
    // Motor_Rotate.TIM_Send_PeriodElapsedCallback();
    Arm_To_Position(position_arm);
    Motor_Arm.Calculate();

    // Motor_Move.Calculate();

    // Motor_Pitch.Calculate();

    // // Move位置切换
    // if (Move_Yaw_Flag)
    // {
    //     Move_Yaw_Flag = false;

    //     Move_Index = (Move_Index + 1) % 3;

    //     Move_To_Position(Move_Target_Position[Move_Index]);
    // }
}

/**
 * @brief 存活检测
 */
void Class_Weapon::TIM_Alive_PeriodElapsedCallback()
{
    Motor_Rotate.TIM_Alive_PeriodElapsedCallback();

    Motor_Arm.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Pitch.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 判断当前动作是否完成
 */
bool Class_Weapon::Is_Action_Finished()
{
    if (!Arm_Task_Finished)
    {
        Check_Arm_Task_Completion();
    }

    if (!Move_Task_Finished)
    {
        Check_Move_Task_Completion();
    }

    if (!Pitch_Task_Finished)
    {
        Check_Pitch_Task_Completion();
    }

    // 在其他动作到位后再开始舵机延时
    if (Arm_Task_Finished && Move_Task_Finished && Pitch_Task_Finished && !Servo_Task_Finished)
    {
        Check_Servo_Task_Completion();
    }

    return Arm_Task_Finished && Move_Task_Finished && Pitch_Task_Finished && Servo_Task_Finished;
}

void Class_Weapon::Check_Arm_Task_Completion()
{
    if (Arm_Calibrated &&
        Math_Abs(Motor_Arm.Get_Target_Position() - Motor_Arm.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Arm.Get_Target_Omega() - Motor_Arm.Get_Now_Omega()) < Omega_Threshold)
    {
        Arm_Task_Finished = true;
    }
}

void Class_Weapon::Check_Move_Task_Completion()
{
    if (Move_Calibrated &&
        Math_Abs(Motor_Move.Get_Target_Position() - Motor_Move.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Move.Get_Target_Omega() - Motor_Move.Get_Now_Omega()) < Omega_Threshold)
    {
        Move_Task_Finished = true;
    }
}

void Class_Weapon::Check_Pitch_Task_Completion()
{
    if (Pitch_Calibrated &&
        Math_Abs(Motor_Pitch.Get_Target_Position() - Motor_Pitch.Get_Now_Angle()) < Position_Threshold &&
        Math_Abs(Motor_Pitch.Get_Target_Omega() - Motor_Pitch.Get_Now_Omega()) < Omega_Threshold)
    {
        Pitch_Task_Finished = true;
    }
}

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

void Class_Weapon::Enter_New_Status_Clear_Completion_Flag()
{
    Arm_Task_Finished = false;
    Move_Task_Finished = false;
    Pitch_Task_Finished = false;
    Servo_Task_Finished = false;
}

/**
 * @brief 夹取状态任务
 */
void Class_Weapon::Weapon_Grab_Status_Task()
{
    // 旋转电机
    Motor_Rotate.Set_Target_Angle(Target.Rotate_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 机械臂电机
    Arm_To_Position(Target.Arm_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 位移电机
    Move_To_Position(Move_Target_Position[Move_Index]);

    // 俯仰电机
    Pitch_To_Position(Target.Pitch_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 夹取舵机
    Pick_Servo[0].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    Pick_Servo[1].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    Pick_Servo[2].Set_Normalized_Position(Target.Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 抓取舵机
    Grab_Servo.Set_Normalized_Position(Target.Grab_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
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
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Grab_Prepare);
                }
            }
            break;
        }

        case Weapon_Status_Grab_Prepare:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Grab);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Init);
                }
            }
            break;
        }

        case Weapon_Status_Grab:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                Weapon->Forward_Yaw_Flag = false;
                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Weapon_Status_Lift_1);
            }
            break;
        }

        case Weapon_Status_Lift_1:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Pick);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Grab);
                }
            }
            break;
        }

        case Weapon_Status_Pick:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Lift_2);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Lift_1);
                }
            }
            break;
        }

        case Weapon_Status_Lift_2:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Rotate_To_Connection);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Pick);
                }
            }
            break;
        }

        case Weapon_Status_Rotate_To_Connection:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Rotate_To_Storage);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Lift_2);
                }
            }
            break;
        }

        case Weapon_Status_Rotate_To_Storage:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Attack_Postition_1);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Servo_Action);
                }
            }
            break;
        }

        case Weapon_Status_Attack_Postition_1:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Attack_Postition_2);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Rotate_To_Storage);
                }
            }
            break;
        }

        case Weapon_Status_Attack_Postition_2:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Attack_Postition_1);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Rotate_To_Storage);
                }
            }
            break;
        }

        case Weapon_Status_Servo_Action:
        {
            if (Status[Now_Status_Serial].Count_Time == 0)
            {
                Weapon->Enter_New_Status_Clear_Completion_Flag();
            }

            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Rotate_To_Storage);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Lift_1);
                }
            }
            break;
        }
    }
}