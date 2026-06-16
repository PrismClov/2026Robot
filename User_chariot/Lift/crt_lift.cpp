#include "crt_lift.h"

/**
 * @brief 初始化函数
 */
void Class_Lift::Init()
{

    Motor_Lift_L.Init(&hfdcan2, Motor::Motor_DJI_ID_0x201);

    Motor_Lift_R.Init(&hfdcan2, Motor::Motor_DJI_ID_0x202);

    // PID初始化
    Motor_Lift_L.PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);

    Motor_Lift_R.PID_Omega.Init(6.0f, 0.1f, 0.00f, 0.0f);

    Motor_Lift_L.Set_Feedforward_Omega(0.6f);
    Motor_Lift_R.Set_Feedforward_Omega(0.6f);

    PID_Distance_L.Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);

    PID_Distance_R.Init(-60.0f, 0.07f, 0.02f, 0.0f, 0.0f, 3.0f);

    // 状态机初始化
    FSM_Lift.Lift = this;

    FSM_Lift.Init(3, 0);
}

/*
 * @brief 获取同步带行程
 */

void Class_Lift::Distance_Caculate()
{
    // 计算同步带行程 齿数*节距
    Now_Distance[0] = Motor_Lift_L.Get_Now_Angle() * Angle_to_Distance - Offset[0];

    Now_Distance[1] = Motor_Lift_R.Get_Now_Angle() * Angle_to_Distance - Offset[1];

    // 传入行程环
    PID_Distance_L.Set_Now(Now_Distance[0]);

    PID_Distance_R.Set_Now(Now_Distance[1]);
}

/*-----------------------校准状态---------------------------*/

/**
 * @brief 校准
 */
float Caliberate_Offset[2] = {300.0f, 0.0f}; // 右边阻力较大  目前：左-1800 右-2000
bool test_in_close = false;
uint32_t Caliberate_Close_cnt = 0;
void Class_Lift::Calibrate()
{

    /*修改为某个行程内使用速度闭环，之后使用开环达到校准位置*/
    if (Math_Abs(Now_Distance[0]) <= Caliberate_Distance_Close && Math_Abs(Now_Distance[1]) <= Caliberate_Distance_Close)
    {
        Caliberate_Close_cnt = 0;
        Motor_Lift_L.Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
        Motor_Lift_R.Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);

        Motor_Lift_L.Set_Target_Speed(Calibrate_Output[0]);
        Motor_Lift_R.Set_Target_Speed(Calibrate_Output[1]);
    }
    else if (Math_Abs(Now_Distance[0]) >= Caliberate_Distance_Close || Math_Abs(Now_Distance[1]) >= Caliberate_Distance_Close)
    {
        Caliberate_Close_cnt++;
        Motor_Lift_L.Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);
        Motor_Lift_R.Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);
        Motor_Lift_L.Set_Out(Caliberate_Output + Caliberate_Offset[0]);
        Motor_Lift_R.Set_Out(Caliberate_Output + Caliberate_Offset[1]);

        test_in_close = true;
    }
}

/*
 * @brief 获取校准完成标志
 */
SoftTimer_t Caliberate_Time_t_L = {
    .start_time = 0,
    .expire_time = 500000};

SoftTimer_t Caliberate_Time_t_R = {
    .start_time = 0,
    .expire_time = 500000};

bool Class_Lift::Is_Calibrate_Finished_Open()
{

    // 更新校准时间
    if (Caliberate_Close_cnt >= 400 && test_in_close)
    {
        if (Math_Abs(Now_Distance[0]) >= Distance_Caliberate && Math_Abs(Motor_Lift_L.Get_Now_Omega()) <= Calibrate_Out[0])
        {

            if (Is_Timer_ExpiredUs(&Caliberate_Time_t_L, Expire_Loop))
            {
                Caliberate_Flag[0] = true;
            }
        }
        if (Math_Abs(Now_Distance[1]) >= Distance_Caliberate && Math_Abs(Motor_Lift_R.Get_Now_Omega()) <= Calibrate_Out[1])
        {

            if (Is_Timer_ExpiredUs(&Caliberate_Time_t_R, Expire_Loop))
            {
                Caliberate_Flag[1] = true;
            }
        }
    }

    return Caliberate_Flag[0] && Caliberate_Flag[1];
    // return 0;
}

/*
 *  @brief 校准注销
 */
void Class_Lift::Calibrate_Cancel()
{
    Motor_Lift_L.Set_Out(0.0f);
    Motor_Lift_R.Set_Out(0.0f);

    // 速度环锁死
    Motor_Lift_L.Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
    Motor_Lift_R.Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);

    Motor_Lift_L.Set_Target_Speed(0.0f);
    Motor_Lift_R.Set_Target_Speed(0.0f);
}

/*-----------------------上升状态---------------------------*/

/*
 * @brief 通用抬升函数 ：可控制平台在0-70cm范围内抬升
 */
void Class_Lift::Up(float Target_Distance_L, float Target_Distance_R)
{
    // 如果已经堵转，直接返回，不执行任何控制
    if (Block_Flag)
        return;

    // 防止超出上限
    float temp_distance_l = (Math_Abs(Target_Distance_L - Now_Distance[0]) < Target_Distance_Limit) ? Target_Distance_L : Target_Distance_Limit;

    float temp_distance_r = (Math_Abs(Target_Distance_R - Now_Distance[1]) < Target_Distance_Limit) ? Target_Distance_R : Target_Distance_Limit;

    // 计算位置误差
    float error_l = temp_distance_l - Now_Distance[0];
    float error_r = temp_distance_r - Now_Distance[1];

    if (Math_Abs(error_l) > Position_Threshold)
    {
        // 速度开环模式
        Lift_Control_Mode_L = MODE_SPEED_OPEN;
        // 根据误差方向设置速度（正负映射：目标<当前 → 正速度）
        float speed = (error_l < 0) ? Target_Speed_Open[0] : -Target_Speed_Open[0];
        Motor_Lift_L.Set_Target_Speed(speed);
    }
    else
    {
        // 位置闭环模式
        Lift_Control_Mode_L = MODE_POSITION_CLOSE;
        PID_Distance_L.Set_Target(temp_distance_l);
        PID_Distance_L.TIM_Calculate_PeriodElapsedCallback();
        Motor_Lift_L.Set_Target_Speed(PID_Distance_L.Get_Out());
    }

    if (Math_Abs(error_r) > Position_Threshold)
    {
        // 速度开环模式
        Lift_Control_Mode_R = MODE_SPEED_OPEN;
        float speed = (error_r < 0) ? Target_Speed_Open[1] : -Target_Speed_Open[1];
        Motor_Lift_R.Set_Target_Speed(speed);
    }
    else
    {
        // 位置闭环模式
        Lift_Control_Mode_R = MODE_POSITION_CLOSE;
        PID_Distance_R.Set_Target(temp_distance_r);
        PID_Distance_R.TIM_Calculate_PeriodElapsedCallback();
        Motor_Lift_R.Set_Target_Speed(PID_Distance_R.Get_Out());
    }
}

/**
 * @brief 查看到达等待R2位置是否完成
 */
bool Class_Lift::Is_Wait_R2_Finished_step()
{
    bool res = false;

    // 位置误差小于阈值
    bool pos_ok = (Math_Abs(Now_Distance[0] - Target_Distance_Wait_R2[0]) <= Distance_Error) &&
                  (Math_Abs(Now_Distance[1] - Target_Distance_Wait_R2[1]) <= Distance_Error);

    // 处于位置闭环模式（确保不是速度开环）
    bool mode_ok = (Lift_Control_Mode_L == MODE_POSITION_CLOSE) &&
                   (Lift_Control_Mode_R == MODE_POSITION_CLOSE);

    // 速度足够小
    bool speed_ok = (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
                    (Motor_Lift_R.Get_Now_Omega() <= 0.05f);

    if (pos_ok && mode_ok && speed_ok)
    {
        res = true;
    }

    return res;
}

/**
 * @brief 查看到达抬起R2是否完成
 */
bool Class_Lift::Is_Lift_R2_Finished_step()
{
    bool res = false;

    // 位置误差小于阈值
    bool pos_ok = (Math_Abs(Now_Distance[0] - Target_Distance_Lift_R2[0]) <= Distance_Error) &&
                  (Math_Abs(Now_Distance[1] - Target_Distance_Lift_R2[1]) <= Distance_Error);

    // 处于位置闭环模式（确保不是速度开环）
    bool mode_ok = (Lift_Control_Mode_L == MODE_POSITION_CLOSE) &&
                   (Lift_Control_Mode_R == MODE_POSITION_CLOSE);

    // 速度足够小
    bool speed_ok = (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
                    (Motor_Lift_R.Get_Now_Omega() <= 0.05f);

    if (pos_ok && mode_ok && speed_ok)
    {
        res = true;
    }

    return res;
}

/**
 * @brief 查看到达放下R2是否完成
 */
bool Class_Lift::Is_Down_R2_Finished_step()
{

    bool res = false;

    // 位置误差小于阈值
    bool pos_ok = (Math_Abs(Now_Distance[0] - Target_Distance_Down_R2[0]) <= Distance_Error) &&
                  (Math_Abs(Now_Distance[1] - Target_Distance_Down_R2[1]) <= Distance_Error);

    // 处于位置闭环模式（确保不是速度开环）
    bool mode_ok = (Lift_Control_Mode_L == MODE_POSITION_CLOSE) &&
                   (Lift_Control_Mode_R == MODE_POSITION_CLOSE);

    // 速度足够小
    bool speed_ok = (Motor_Lift_L.Get_Now_Omega() <= 0.05f) &&
                    (Motor_Lift_R.Get_Now_Omega() <= 0.05f);

    if (pos_ok && mode_ok && speed_ok)
    {
        res = true;
    }

    return res;
}

/**
 * @brief 通用抬升注销函数
 */
void Class_Lift::UP_Cancel()
{

    // 速度环锁死
    Motor_Lift_L.Set_Target_Speed(0.0f);

    Motor_Lift_R.Set_Target_Speed(0.0f);
}

/*-----------------------回调函数---------------------------*/
/**
 * @brief 定时器回调函数
 */
void Class_Lift::TIM_Calculate_PeriodElapsedCallback()
{
    // 实时计算同步带行程
    Distance_Caculate();

    // 状态机回调
    FSM_Lift.Lift_TIM_Status_PeriodElapsedCallback();

    // 电机一直跑速度环
    Motor_Lift_L.Calculate();

    Motor_Lift_R.Calculate();

    // Block_Flag = Check_Motor_Block();

    if (Block_Flag)
    {
        // 强制停止所有输出
        Motor_Lift_L.Set_Target_Speed(0.0f);
        Motor_Lift_R.Set_Target_Speed(0.0f);

        // 将位置环目标设为当前位置，消除误差
        PID_Distance_L.Set_Target(Now_Distance[0]);
        PID_Distance_R.Set_Target(Now_Distance[1]);

        // 强制切换到位置环，但将目标位置设为当前位置
        Lift_Control_Mode_L = MODE_POSITION_CLOSE;
        Lift_Control_Mode_R = MODE_POSITION_CLOSE;
    }
}

/**
 * @brief 状态机回调函数
 */
void Class_FSM_Lift::Lift_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    if (Lift->Block_Flag)
    {
        // 停止所有电机
        Lift->UP_Cancel();
        // Lift->Down_Cancel();

        // 保持在当前状态，等待人工干预
        return;
    }

    switch (Now_Status_Serial)
    {
        case Lift_Status_INIT:
        {

            if (!Lift->Is_Calibrate_Finished_Open())
            {
                Lift->Calibrate();
            }

            // 查看校准是否完成
            else if (Lift->Is_Calibrate_Finished_Open())
            {

                Lift->Calibrate_Cancel();

                // 跳转条件
                if (Lift->Yaw_Flag)
                {
                    // 设置同步带偏移
                    Lift->Set_Offset(Lift->Get_Now_Distance_L(), Lift->Get_Now_Distance_R());

                    Status[Now_Status_Serial].Count_Time = 0;

                    Set_Status(Lift_Status_Wait_R2); // 校准完自动切换到等待R2状态or手动切换
                }
            }
        }
        break;

        case Lift_Status_Wait_R2: // 目标设置：-50
        {

            if (!Lift->Is_Wait_R2_Finished_step())
            {
                Lift->Up(Lift->Target_Distance_Wait_R2[0], Lift->Target_Distance_Wait_R2[1]);
            }
            else if (Lift->Is_Wait_R2_Finished_step())
            {
                Lift->UP_Cancel();

                if (Lift->Yaw_Flag)
                {
                    Status[Now_Status_Serial].Count_Time = 0;
                    Set_Status(Lift_Status_Lift_R2);
                }
            }
        }
        break;

        case Lift_Status_Lift_R2: // 目标设置 ：-20
        {

            if (!Lift->Is_Lift_R2_Finished_step())
            {
                Lift->Up(Lift->Target_Distance_Lift_R2[0], Lift->Target_Distance_Lift_R2[1]);
            }
            else if (Lift->Is_Lift_R2_Finished_step())
            {

                Lift->UP_Cancel();

                if (Lift->Yaw_Flag)
                {
                    Set_Status(Lift_Status_Down_R2);
                }
            }
        }
        break;

        case Lift_Status_Down_R2: // 目标设置 ：-50
        {
            if (!Lift->Is_Down_R2_Finished_step())
            {
                Lift->Up(Lift->Target_Distance_Down_R2[0], Lift->Target_Distance_Down_R2[1]);
            }
            else if (Lift->Is_Down_R2_Finished_step())
            {
                Lift->UP_Cancel();

                if (Lift->Yaw_Flag)
                {
                    Set_Status(Lift_Status_Wait_R2);
                }
            }
        }
        break;
    }
    Lift->Yaw_Flag = false;
}

void Class_Lift::TIM_100ms_Alive_PeriodElapsedCallback()
{

    // 检查电机是否堵转
    Check_Motor_Block();

    Motor_Lift_L.TIM_100ms_Alive_PeriodElapsedCallback();

    Motor_Lift_R.TIM_100ms_Alive_PeriodElapsedCallback();
}
// 9A  0.2RAD/S
bool Class_Lift::Check_Motor_Block()
{
    // 检查电机是否堵转
    if (Math_Abs(Motor_Lift_L.Get_Now_Omega()) <= 0.2f && Math_Abs(Motor_Lift_L.Get_Now_Current()) >= 9.5f)
    {
        if (Now_Distance[0] > 0.05f)
        {
            Lift_Block_Flag[0] = true;
        }
    }
    if (Math_Abs(Motor_Lift_R.Get_Now_Omega()) <= 0.2f && Math_Abs(Motor_Lift_R.Get_Now_Current()) >= 9.5f)
    {
        if (Now_Distance[1] > 0.05f)
        {
            Lift_Block_Flag[1] = true;
        }
    }

    return Lift_Block_Flag[0] || Lift_Block_Flag[1];
}

/*-----------------------数学辅助函数---------------------------*/

/**
 * @brief 符号函数
 *
 */
inline int8_t Sign(float __value)
{
    int8_t res = 0;

    if (__value > 0)
    {

        res = 1;
    }

    else if (__value < 0)
    {

        res = -1;
    }

    return res;
}
