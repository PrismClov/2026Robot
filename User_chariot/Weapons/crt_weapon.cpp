#include "crt_weapon.h"

void Class_Weapon::Init()
{
    // 夹取舵机
    Pick_Servo[0].Init(&htim1, TIM_CHANNEL_1, 500, 2500);
    Pick_Servo[1].Init(&htim1, TIM_CHANNEL_3, 500, 2500);
    Pick_Servo[2].Init(&htim2, TIM_CHANNEL_1, 500, 2500);

    // 抓取舵机
    Grab_Servo.Init(&htim2, TIM_CHANNEL_3, 500, 2500);

    // 机械臂电机 (CAN2, ID 0x206)
    Motor_Arm.Init(
        &hfdcan2,
        Motor::Motor_DJI_ID_0x206,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 6.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 30.0f},
            .PID_Omega = PID_Parameters{.K_P = 8.0f, .K_I = 0.0f, .K_D = 0.0f, .Out_Max = 30.0f},
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
    Motor_Rotate.Init(&hfdcan2, 0x01, 0x01, Motor_DM_Control_Method_NORMAL_MIT);

    // 俯仰电机
    Motor_Pitch[0].Init(&hfdcan3, Motor::Motor_DJI_ID_0x205);
    Motor_Pitch[1].Init(&hfdcan3, Motor::Motor_DJI_ID_0x206);

    // 俯仰双电机同步控制
    Pitch.Init({&Motor_Pitch[0], &Motor_Pitch[1]},
               Class_MultiMotorSync_Base<2>::Parameters{
                   .PID_Distance = {
                       PID_Parameters{}, // TODO
                       PID_Parameters{}, // TODO
                   },
                   .Distance_Approach_Threshold = 0.01f,
                   .Calibrate = Pitch_Calibrate_Params,
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

/**
 * @brief 周期中断函数
 */
void Class_Weapon::TIM_Weapon_PeriodElapsedCallback()
{
    FSM_Weapon.Weapon_TIM_Status_PeriodElapsedCallback();

    Motor_Rotate.TIM_Send_PeriodElapsedCallback();

    Motor_Arm.Calculate();

    Motor_Move.Calculate();

    // 俯仰控制 (校准完成后才运行Move_To_Position和Calculate)
    if (Pitch.Get_Is_Calibrated())
    {
        Pitch.Distance_Update();
        Pitch.Move_To_Position();
        Motor_Pitch[0].Calculate();
        Motor_Pitch[1].Calculate();
    }

    // Move位置切换
    if (Move_Yaw_Flag)
    {
        Move_Yaw_Flag = false;

        Move_Index = (Move_Index + 1) % 3;

        Move_To_Position(Move_Target_Position[Move_Index]);
    }
}

/**
 * @brief 存活检测
 */
void Class_Weapon::TIM_Alive_PeriodElapsedCallback()
{
    Motor_Rotate.TIM_Alive_PeriodElapsedCallback();

    Motor_Arm.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Pitch[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Pitch[1].TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 判断动作是否完成
 *
 * @return true 完成
 * @return false 未完成
 */
bool Class_Weapon::Is_Action_Finished()
{
    // 只看了电机，舵机动作较快且不易测量，暂不考虑
    bool Motor_Is_Finished = Math_Abs(Motor_Arm.Get_Target_Position() - Motor_Arm.Get_Now_Angle()) < Position_Threshold &&
                             Math_Abs(Motor_Arm.Get_Target_Omega() - Motor_Arm.Get_Now_Omega()) < Omega_Threshold &&
                             Math_Abs(Motor_Move.Get_Target_Position() - Motor_Move.Get_Now_Angle()) < Position_Threshold &&
                             Math_Abs(Motor_Move.Get_Target_Omega() - Motor_Move.Get_Now_Omega()) < Omega_Threshold &&
                             Move_Calibrated &&         // 位移电机需要完成校准
                             Pitch.Get_Is_Calibrated(); // 俯仰电机需要完成校准

    return Motor_Is_Finished;
}

/**
 * @brief 夹取状态任务
 */
void Class_Weapon::Weapon_Grab_Status_Task()
{
    // 旋转电机
    Motor_Rotate.Set_Control_Angle(Target.Rotate_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 机械臂电机
    Motor_Arm.Set_Target_Position(Target.Arm_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 位移电机
    Move_To_Position(Move_Target_Position[Move_Index]);

    // 俯仰电机同步
    if (!Pitch.Get_Is_Calibrated())
    {
        Pitch.Calibrate_Update();
    }
    else
    {
        Pitch.Set_Target_Position(Target.Pitch_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    }

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
                    Set_Status(Weapon_Status_Rotate_To_Connection);
                }
            }
            break;
        }

        case Weapon_Status_Attack_Postition_1:
        {
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
            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished())
            {
                if (Weapon->Forward_Yaw_Flag)
                {
                    Weapon->Forward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Init);
                }
                if (Weapon->Backward_Yaw_Flag)
                {
                    Weapon->Backward_Yaw_Flag = false;
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Weapon_Status_Attack_Postition_1);
                }
            }
            break;
        }
    }
}