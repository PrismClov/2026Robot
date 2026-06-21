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
     * @param x 目标位置
     */
    void Move_To_Position(float x);

private:
    bool Is_Move_Calibrated = false;

    float Calibrate_Offset = 0.0f;

    Calibrate_Params calibarate_param = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = 10.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000,
    };
};
#endif