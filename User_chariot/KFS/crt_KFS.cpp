#include "crt_KFS.h"

void Class_KFS::Init()
{
    // 抬升电机初始化
    Motor_Lift[0].Init(&hfdcan2, Motor::Motor_DJI_ID_0x201);
    Motor_Lift[1].Init(&hfdcan2, Motor::Motor_DJI_ID_0x202);

    // 抬升速度环
    Motor_Lift[0].PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);
    Motor_Lift[1].PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);
    Motor_Lift[0].Set_Feedforward_Omega(0.6f);
    Motor_Lift[1].Set_Feedforward_Omega(0.6f);

    // 抬升行程环
    PID_Lift_Distance[0].Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);
    PID_Lift_Distance[1].Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);

    // 移动电机初始化
    Motor_Move.Init(
        &hfdcan2,
        Motor::Motor_DJI_ID_0x203,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = Motor::PID_Parameters{},
            .PID_Omega = Motor::PID_Parameters{},
        });

    // 机械臂电机初始化
    Motor_Arm.Init(&hfdcan2, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT);

    // 手腕电机初始化
    Motor_Wrist.Init(&hfdcan2, 0x03, Motor_DM_Control_Method_NORMAL_MIT);

    // 气泵初始化
    Air_Pump.Init(GPIOC, GPIO_PIN_13);

    // 状态机初始化
    FSM_KFS.KFS = this;
    FSM_KFS.Init(1, 0);
}

void Class_KFS::Up(float x)
{
    for (int i = 0; i < 2; i++)
    {
        Now_Distance[i] = Motor_Lift[i].Get_Now_Angle() * Angle_to_Distance - Offset[i];
        PID_Lift_Distance[i].Set_Now(Now_Distance[i]);
    }

    for (int i = 0; i < 2; i++)
    {
        float error = x - Now_Distance[i];

        if (Math_Abs(error) > Position_Threshold)
        {
            Lift_Control_Mode[i] = KFS_Lift_MODE_SPEED_OPEN;
            float speed = (error < 0) ? Target_Speed_Open[i] : -Target_Speed_Open[i];
            Motor_Lift[i].Set_Target_Speed(speed);
        }
        else
        {
            Lift_Control_Mode[i] = KFS_Lift_MODE_POSITION_CLOSE;
            PID_Lift_Distance[i].Set_Target(x);
            PID_Lift_Distance[i].TIM_Calculate_PeriodElapsedCallback();
            Motor_Lift[i].Set_Target_Speed(PID_Lift_Distance[i].Get_Out());
        }
    }
}

void Class_KFS::Move_To_Position(float x)
{
    float target_rad = x / Stroke * 2.0f * PI;
    float absolute_target_rad = Move_Calibration_Offset + target_rad;

    float error_linear = Math_Abs(absolute_target_rad - Motor_Move.Get_Now_Angle()) / (2.0f * PI) * Stroke;

    if (error_linear > Position_Threshold)
    {
        if (!Move_Calibrated)
        {
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
            Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
            Motor_Move.Set_Target_Position(absolute_target_rad);
        }
    }
    else
    {
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(absolute_target_rad);
        Move_Calibrated = false;
    }
}