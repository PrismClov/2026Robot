#ifndef CRT_KFS_H
#define CRT_KFS_H

#include "alg_fsm.h"
#include "alg_pid.h"
#include "dvc_airtool.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"

class Class_KFS;

enum Enum_KFS_Status
{
    KFS_Status_Init = 0, // 初始化
};

enum Enum_KFS_Lift_Control_Mode
{
    KFS_Lift_MODE_SPEED_OPEN,
    KFS_Lift_MODE_POSITION_CLOSE,
};

class Class_FSM_KFS : public Class_FSM
{
public:
    Class_KFS *KFS;

    // 夹取状态
    void KFS_TIM_Status_PeriodElapsedCallback();

    Enum_KFS_Status KFS_Status = KFS_Status_Init;
};

class Class_KFS
{
public:
    // 抬升PID
    Class_PID PID_Lift_Distance[2];

    // 抬升电机
    Motor::Class_Motor_DJI_C620 Motor_Lift[2];

    // 移动电机
    Motor::Class_Motor_DJI_C620 Motor_Move;

    // 机械臂电机
    Class_Motor_DM_Normal Motor_Arm;

    // 手腕电机
    Class_Motor_DM_Normal Motor_Wrist;

    // 气泵
    Class_AIRPUMP Air_Pump;

    // 状态机
    Class_FSM_KFS FSM_KFS;
    friend class Class_FSM_KFS;

    void Init();

    void TIM_Control_PeriodElapsedCallback();

    void TIM_Alive_PeriodElapsedCallback();

    /**
     * @brief 移动电机位置控制
     *
     * @param x 目标位置 (m)
     */
    void Move_To_Position(float x);

private:
    // 当前行程 m
    float Now_Distance[2] = {0.0f, 0.0f};
    float Offset[2] = {0.0f, 0.0f};

    // 抬升控制模式
    Enum_KFS_Lift_Control_Mode Lift_Control_Mode[2] = {KFS_Lift_MODE_POSITION_CLOSE, KFS_Lift_MODE_POSITION_CLOSE};

    float Target_Speed_Open[2] = {5.0f, 5.0f};
    float Position_Threshold = 0.01f;
    float Omega_Threshold = 0.01f;

    // 移动电机参数
    float Stroke = 0.1f; // 导程 m/rev
    bool Move_Calibrated = false;
    float Move_Calibration_Offset = 0.0f;

    // 校准参数
    float Calibrate_Speed = -0.3f;
    float Locked_Rotor_Current_Threshold = 5.0f;

    // 机械参数 → 角度转行程
    // 齿轮减速比
    constexpr static float Gearbox_Rate = 2.50f / 1.0f;

    // 同步带轮齿数
    constexpr static float Tooth_Number = 34.0f;

    // 同步带正方向
    constexpr static int8_t Belt_Sign = -1;

    // 同步带节距 m
    constexpr static float Step = 0.005f;

    // 角度转行程系数 m/rad
    constexpr static float Angle_to_Distance = Belt_Sign * Tooth_Number * Step / (2 * PI * Gearbox_Rate);

    // 抬升函数
    void Up(float x);
};
#endif