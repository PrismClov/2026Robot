#include "crt_weapon.h"

void Class_Weapon::Init()
{
    // 夹取舵机
    Pick_Servo[0].Init(&htim1, TIM_CHANNEL_1, 500, 2500);
    Pick_Servo[1].Init(&htim1, TIM_CHANNEL_3, 500, 2500);
    Pick_Servo[2].Init(&htim2, TIM_CHANNEL_1, 500, 2500);

    // 抓取舵机
    Grab_Servo.Init(&htim2, TIM_CHANNEL_3, 500, 2500);

    // 机械臂电机
    Motor_Arm.Init(
        &hfdcan2,
        Motor::Motor_DJI_ID_0x205,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = Motor::PID_Parameters{}, // TODO
            .PID_Omega = Motor::PID_Parameters{},    // TODO
        });

    // 移动电机
    Motor_Move.Init(
        &hfdcan2,
        Motor::Motor_DJI_ID_0x206,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = Motor::PID_Parameters{}, // TODO
            .PID_Omega = Motor::PID_Parameters{},    // TODO
        });

    // 旋转电机
    Motor_Rotate.Init(
        &hfdcan2,
        0x01, // CAN Rx ID
        0x01, // CAN Tx ID
        Motor_DM_Control_Method_NORMAL_MIT);

    FSM_Weapon.Weapon = this;

    FSM_Weapon.Init(5, 0);
}

void Class_Weapon::Move_To_Position(float x)
{
    // 目标从米转换为相对于校准零点的弧度
    float target_rad = x / Stroke * 2.0f * PI;
    // 转换为绝对角度
    float absolute_target_rad = Move_Calibration_Offset + target_rad;

    // 长度误差
    float error_linear = Math_Abs(absolute_target_rad - Motor_Move.Get_Now_Angle()) / (2.0f * PI) * Stroke;

    if (error_linear > Position_Threshold)
    {
        // 需要移动，先校准再走
        if (!Move_Calibrated)
        {
            // 低速运行到堵转位置，完成机械零点校准
            float offset;
            if (Motor_Move.Calibrate(Calibrate_Speed, Locked_Rotor_Current_Threshold, offset))
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
            Motor_Move.Set_Target_Position(absolute_target_rad);
        }
    }
    else
    {
        // 已到达目标附近，保持位置
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(absolute_target_rad);
        Move_Calibrated = false;
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
                             Math_Abs(Motor_Move.Get_Target_Position() - Motor_Move.Get_Now_Angle()) / (2.0f * PI) * Stroke < Position_Threshold &&
                             Math_Abs(Motor_Move.Get_Target_Omega() - Motor_Move.Get_Now_Omega()) < Omega_Threshold &&
                             Move_Calibrated &&  // 位移电机需要完成校准
                             Pitch_Calibrated && // 俯仰电机需要完成校准
                             Arm_Calibrated;     // 机械臂电机需要完成校准

    return Motor_Is_Finished;
}

/**
 * @brief 夹取状态任务
 */
void Class_Weapon::Weapon_Grab_Status_Task()
{
    // 旋转电机
    Motor_Rotate.Set_Control_Angle(Rotate_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 机械臂电机
    if (!Arm_Calibrated)
    {
        // 低速运行到堵转位置，完成机械零点校准
        float offset;
        if (Motor_Arm.Calibrate(Calibrate_Speed, Locked_Rotor_Current_Threshold, offset))
        {
            Arm_Calibration_Offset = offset;
            Arm_Calibrated = true;
        }
    }
    Motor_Arm.Set_Target_Position(Arm_Target_Position[FSM_Weapon.Get_Now_Status_Serial()] + Arm_Calibration_Offset);

    // 位移电机
    Move_To_Position(Move_Target_Position[Move_Index]);

    // 俯仰电机
    if (!Pitch_Calibrated)
    {
        // 低速运行到堵转位置，完成机械零点校准
        float offset;
        if (Motor_Pitch[0].Calibrate(Calibrate_Speed, Locked_Rotor_Current_Threshold, offset))
        {
            Pitch_Calibration_Offset = offset;
            Pitch_Calibrated = true;
        }
    }
    Motor_Pitch[0].Set_Target_Position(Pitch_Target_Position[FSM_Weapon.Get_Now_Status_Serial()] + Pitch_Calibration_Offset);
    Motor_Pitch[1].Set_Target_Position(Pitch_Target_Position[FSM_Weapon.Get_Now_Status_Serial()] + Pitch_Calibration_Offset);

    // 夹取舵机
    Pick_Servo[0].Set_Normalized_Position(Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    Pick_Servo[1].Set_Normalized_Position(Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
    Pick_Servo[2].Set_Normalized_Position(Pick_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);

    // 抓取舵机
    Grab_Servo.Set_Normalized_Position(Grab_Servo_Target_Position[FSM_Weapon.Get_Now_Status_Serial()]);
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

            if (Weapon->Is_Action_Finished() && Weapon->Pick_Yaw_Flag)
            {
                Weapon->Pick_Yaw_Flag = false;

                Status[Now_Status_Serial].Count_Time = 0;

                Set_Status(Weapon_Status_Grab);
            }
            break;
        }

        case Weapon_Status_Grab:
        {
            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished() && Weapon->Pick_Yaw_Flag)
            {
                Weapon->Pick_Yaw_Flag = false;

                Status[Now_Status_Serial].Count_Time = 0;

                Set_Status(Weapon_Status_Lift);
            }
            break;
        }

        case Weapon_Status_Lift:
        {
            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished() && Weapon->Pick_Yaw_Flag)
            {
                Weapon->Pick_Yaw_Flag = false;

                Status[Now_Status_Serial].Count_Time = 0;

                Set_Status(Weapon_Status_Rotate);
            }
            break;
        }

        case Weapon_Status_Rotate:
        {
            Weapon->Weapon_Grab_Status_Task();

            if (Weapon->Is_Action_Finished() && Weapon->Pick_Yaw_Flag)
            {
                Weapon->Pick_Yaw_Flag = false;

                Status[Now_Status_Serial].Count_Time = 0;

                Set_Status(Weapon_Status_Init);
            }
            break;
        }
    }

    Weapon->Pick_Yaw_Flag = false;
}