#include "crt_KFS.h"
#include "dvc_dwt.h"

void Class_KFS::Init()
{
    // // 抬升电机初始化
    // Motor_Lift[0].Init(&hfdcan2, Motor::Motor_DJI_ID_0x201);
    // Motor_Lift[1].Init(&hfdcan2, Motor::Motor_DJI_ID_0x202);

    // // 抬升速度环
    // Motor_Lift[0].PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);
    // Motor_Lift[1].PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);
    // Motor_Lift[0].Set_Feedforward_Omega(0.6f);
    // Motor_Lift[1].Set_Feedforward_Omega(0.6f);

    // 抬升行程环
    // PID_Lift_Distance[0].Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);
    // PID_Lift_Distance[1].Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);

    // 移动电机初始化
    Motor_Move.Init(
        &hfdcan3,
        Motor::Motor_DJI_ID_0x201,
        Motor::Class_Motor_DJI_C620::Parameters{
            .PID_Position = PID_Parameters{
                .K_P = 8.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 7.0f, // 位置环输出速度目标限制
            },
            .PID_Omega = PID_Parameters{
                .K_P = 2.0f,
                .K_I = 0.0f,
                .K_D = 0.0f,
                .Out_Max = 20.0f, // 速度环输出电流目标限制
                .Dead_Zone = 0.5f, // 速度环死区，避免小幅抖动
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

uint32_t cnt = 0;
void Class_KFS::Move_To_Position(float x)
{
    if (!Is_Move_Calibrated)
    {
        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
        Motor_Move.Set_Target_Speed(Move_Calibrate_Speed);
        if (Math_Abs(Motor_Move.Get_Speed()) < 0.05f) // 速度接近零且已持续一段时间，认为已堵转
        {
            cnt++;
            if (cnt > 200) // 连续200次（约200ms）速度接近零，认为堵转完成
            {
                Is_Move_Calibrated = true;
                Calibrate_Offset = Motor_Move.Get_Now_Angle(); // 记录堵转时的角度作为校准零点
                // Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
                // Motor_Move.Set_Target_Position(Calibrate_Offset - 0.1f); // 先移动到校准零点位置
            }
        }
        else
        {
            cnt = 0;
        }
    }
    else
    {

        Motor_Move.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
        Motor_Move.Set_Target_Position(x + Calibrate_Offset); // 使用校准零点调整目标位置
    }
}

void Class_KFS::TIM_Control_PeriodElapsedCallback()
{
    Motor_Move.Update_Feedback();
    Motor_Move.Calculate();
}

void Class_KFS::TIM_Alive_PeriodElapsedCallback()
{
    Motor_Move.TIM_100ms_Alive_PeriodElapsedCallback();
}