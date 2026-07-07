#ifndef CRT_KFS_H
#define CRT_KFS_H

#include "alg_fsm.h"
#include "alg_pid.h"
#include "alg_slope.h"
#include "crt_multi_motor_sync.h"
#include "drv_math.h"
#include "dvc_airtool.h"
#include "dvc_dwt.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dm_pid.h"
#include "dvc_motor_rs_pid.h"

#define MAX_KFS_STATUS 20
#define MAX_LIFT_POSITION_INDEX 3

class Class_KFS;

enum Enum_KFS_Status
{
    KFS_Status_Init = 0,                  // 初始化
    KFS_Status_First_Pick_Prepare,        // 夹取准备
    KFS_Status_First_Pick,                // 夹取第一个KFS
    KFS_Status_First_Pick_Up,             // 夹取第一个KFS后抬起
    KFS_Status_Prepare_Storage_Lift_Up,   // 准备存储抬升
    KFS_Status_Storage,                   // KFS存储
    KFS_Status_Storage_Lift_Down,         // 储存抬升放下来
    KFS_Status_Storage_Arm_Up,            // 储存大臂抬起
    KFS_Status_Protect_Storage,           // 抵住KFS
    KFS_Status_Second_Pick_Prepare,       // 夹取准备
    KFS_Status_Second_Pick,               // 夹取第二个KFS
    KFS_Status_Second_Pick_Up,            // 夹取第二个KFS后抬起
    KFS_Status_Prepare_Protect_Arm_Wrist, // 大臂手腕先动
    KFS_Status_Protect_Storage_Again,     // 再次保护存储的KFS
    KFS_Status_First_Release_Prepare,     // 释放准备
    KFS_Status_First_Release,             // 释放第一个KFS
    KFS_Status_Get_Storage,               // 获取KFS存储
    KFS_Status_Second_Release_Prepare,    // 释放准备
    KFS_Status_Second_Release,            // 释放第二个KFS
    KFS_Status_Recover_Init               // 回到初始化状态
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
    // 抬升电机
    Motor::Class_Motor_DJI_C620 Motor_Lift[2];
    Class_MultiMotorSync_Base<2> Lift;

    // 移动电机
    Motor::Class_Motor_DJI_C620 Motor_Move;

    // 机械臂电机
    Class_Motor_DM_PID Motor_Arm;

    // 手腕电机
    Class_Motor_RS_PID Motor_Wrist;

    // 气泵
    Class_AIRPUMP Air_Pump;

    // 状态机
    Class_FSM_KFS FSM_KFS;
    friend class Class_FSM_KFS;

    void Init();

    void TIM_Control_PeriodElapsedCallback();

    void TIM_Alive_PeriodElapsedCallback();

    inline void Set_Lift_Height_Index(uint8_t index);

    inline void Status_Forward();

    inline void Status_Backward();

private:
    // KFS标志
    bool Forward_Yaw_Flag = false;

    bool Backward_Yaw_Flag = false;

    uint8_t Is_KFS_Picked = 0; // 0: 未抓取, 1: 已抓取 用于是否加入KFS重力补偿

    float KFS_Gravity_Compensation_Ratio_Arm = -0.3f;

    float KFS_Gravity_Compensation_Ratio_Wrist = 2.0f;

    // 移动电机参数
    bool Move_Task_Finished = false;

    bool Is_Move_Calibrated = false;

    float Move_Calibrate_Offset = 0.0f;

    const Calibrate_Params move_calibarate_param = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = 10.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000,
    };

    const float Move_Position_Approach_Threshold = 0.1f;

    const float Move_Speed_Approach_Threshold = 0.1f;

    // 抬升模块参数
    bool Lift_Task_Finished = false;

    uint8_t Lift_Height_Index = 0;

    const float Lift_Force_Compensation[2] = {0.5f, -0.4f};

    const float Lift_Height_Approach_Threshold = 0.01f; // 高度判断阈值

    const float Lift_Speed_Approach_Threshold = 0.1f; // 速度判断阈值

    // 手腕电机参数
    bool Wrist_Task_Finished = false;

    float Processed_Wrist_Angle_Rad = 0.0f;    // 手腕和大臂的夹角，顺时针为正

    const float Wrist_Distance_Lock_Threshold = 0.02f;

    const float Wrist_Distance_Approach_Threshold = 0.02f;

    const float Wrist_Speed_Approach_Threshold = 0.1f;

    const float Wrist_Max_Velocity = 5.0f;

    const float Wrist_Gravity_Compensation_Ratio = 0.85f;

    const float Wrist_Parallel_With_Arm_Angle_Offset = 3.01663522f + PI;

    Class_Slope Wrist_Speed_Slope;

    // 机械臂电机参数
    bool Arm_Task_Finished = false;

    float Processed_Arm_Angle_Rad = 0.0f; // 大臂和水平夹角，顺时针为正

    const float Arm_Distance_Lock_Threshold = 0.02f;

    const float Arm_Distance_Approach_Threshold = 0.02f;

    const float Arm_Speed_Approach_Threshold = 0.1f;

    const float Arm_Max_Velocity = 6.0f;

    const float Arm_Gravity_Compensation_Ratio = -2.2f;

    const float Arm_Horizontal_Offset = 4.00f;

    Class_Slope Arm_Speed_Slope;

    // 气泵参数
    // （去掉了吸盘延时）

    // 状态机配置表
    struct Target
    {
        // 移动电机目标位置
        float Move_Position[MAX_KFS_STATUS] = {
            -0.2f, // Init
            -0.2f, // First_Pick_Prepare
            -0.2f, // First_Pick
            -0.2f, // First_Pick_Up
            -5.8f, // Prepare_Storage_Lift_Up
            -5.8f, // Storage
            -5.8f, // Storage_Lift_Down
            -5.8f, // Storage_Arm_Up
            -5.8f, // Protect_Storage
            -0.2f, // Second_Pick_Prepare
            -0.2f, // Second_Pick
            -0.2f, // Second_Pick_Up
            -0.2f, // Prepare_Protect_Arm_Wrist
            -0.2f, // Protect_Storage_Again
            -0.2f, // First_Release_Prepare
            -0.2f, // First_Release
            0.0f,  // Get_Storage
            0.0f,  // Second_Release_Prepare
            0.0f,  // Second_Release
            0.0f,  // Recover_Init
        };

        // 抬升高度
        float Lift_Height[MAX_LIFT_POSITION_INDEX][MAX_KFS_STATUS] = {
            { // Lift_Height_Index = 0
                0.00f, // Init
                0.00f, // First_Pick_Prepare
                0.00f, // First_Pick
                0.00f, // First_Pick_Up
                0.35f, // Prepare_Storage_Lift_Up
                0.35f, // Storage
                0.20f, // Storage_Lift_Down
                0.20f, // Storage_Arm_Up
                0.20f, // Protect_Storage
                0.00f, // Second_Pick_Prepare
                0.00f, // Second_Pick
                0.20f, // Second_Pick_Up
                0.35f, // Prepare_Protect_Arm_Wrist
                0.20f, // Protect_Storage_Again
                0.35f, // First_Release_Prepare
                0.35f, // First_Release
                0.00f, // Get_Storage
                0.00f, // Second_Release_Prepare
                0.00f, // Second_Release
                0.00f, // Recover_Init
            },
            { // Lift_Height_Index = 1
                0.20f, // Init
                0.20f, // First_Pick_Prepare
                0.20f, // First_Pick
                0.20f, // First_Pick_Up
                0.35f, // Prepare_Storage_Lift_Up
                0.35f, // Storage
                0.20f, // Storage_Lift_Down
                0.20f, // Storage_Arm_Up
                0.20f, // Protect_Storage
                0.20f, // Second_Pick_Prepare
                0.20f, // Second_Pick
                0.20f, // Second_Pick_Up
                0.35f, // Prepare_Protect_Arm_Wrist
                0.20f, // Protect_Storage_Again
                0.35f, // First_Release_Prepare
                0.35f, // First_Release
                0.00f, // Get_Storage
                0.00f, // Second_Release_Prepare
                0.00f, // Second_Release
                0.00f, // Recover_Init
            },
            { // Lift_Height_Index = 2
                0.35f, // Init
                0.35f, // First_Pick_Prepare
                0.35f, // First_Pick
                0.35f, // First_Pick_Up
                0.35f, // Prepare_Storage_Lift_Up
                0.35f, // Storage
                0.20f, // Storage_Lift_Down
                0.20f, // Storage_Arm_Up
                0.20f, // Protect_Storage
                0.35f, // Second_Pick_Prepare
                0.35f, // Second_Pick
                0.20f, // Second_Pick_Up
                0.35f, // Prepare_Protect_Arm_Wrist
                0.20f, // Protect_Storage_Again
                0.35f, // First_Release_Prepare
                0.35f, // First_Release
                0.00f, // Get_Storage
                0.00f, // Second_Release_Prepare
                0.00f, // Second_Release
                0.00f, // Recover_Init
            },
        };

        // 手腕目标角度
        float Wrist_Angle[MAX_KFS_STATUS] = {
            0.0f,   // Init
            2.2f,  // First_Pick_Prepare
            2.04f,  // First_Pick
            2.27f,  // First_Pick_Up
            2.27f,  // Prepare_Storage_Lift_Up
            2.0f,   // Storage
            2.0f,   // Storage_Lift_Down
            -1.0f,  // Storage_Arm_Up
            -1.0f,  // Protect_Storage
            2.07f,  // Second_Pick_Prepare
            1.81f,  // Second_Pick
            2.27f,  // Second_Pick_Up
            -1.6f,  // Prepare_Protect_Arm_Wrist
            -1.6f,  // Protect_Storage_Again
            -1.3f,  // First_Release_Prepare
            -1.3f,  // First_Release
            0.0f,   // Get_Storage
            0.0f,   // Second_Release_Prepare
            0.0f,   // Second_Release
            0.0f,   // Recover_Init
        };

        // 机械臂目标角度
        float Arm_Angle[MAX_KFS_STATUS] = {
            -1.35f, // Init
            -0.5f,  // First_Pick_Prepare
            -0.18f, // First_Pick
            -0.7f,  // First_Pick_Up
            -0.7f,  // Prepare_Storage_Lift_Up
            1.14f,  // Storage
            1.14f,  // Storage_Lift_Down
            0.8f,   // Storage_Arm_Up
            1.57f,  // Protect_Storage
            -0.5f,  // Second_Pick_Prepare
            -0.22f, // Second_Pick
            -0.7f,  // Second_Pick_Up
            1.57f,  // Prepare_Protect_Arm_Wrist
            1.57f,  // Protect_Storage_Again
            1.0f,   // First_Release_Prepare
            1.0f,   // First_Release
            0.0f,   // Get_Storage
            0.0f,   // Second_Release_Prepare
            0.0f,   // Second_Release
            0.0f,   // Recover_Init
        };

        // 气泵状态（1=吸, 0=放）
        uint8_t Pump_Status[MAX_KFS_STATUS] = {
            0, // Init
            0, // First_Pick_Prepare
            1, // First_Pick
            1, // First_Pick_Up
            1, // Prepare_Storage_Lift_Up
            1, // Storage
            1, // Storage_Lift_Down
            0, // Storage_Arm_Up
            0, // Protect_Storage
            0, // Second_Pick_Prepare
            1, // Second_Pick
            1, // Second_Pick_Up
            1, // Prepare_Protect_Arm_Wrist
            1, // Protect_Storage_Again
            1, // First_Release_Prepare
            1, // First_Release
            0, // Get_Storage
            1, // Second_Release_Prepare
            1, // Second_Release
            0, // Recover_Init
        };
    } Target;

    // 控制内部接口
    void Move_To_Position(float x);

    void Lift_To_Height(float height);

    void Wrist_To_Angle(float angle);

    void Arm_To_Angle(float angle);

    // 获取处理后的角度
    void Update_Processed_Wrist_Angle_Rad();

    void Update_Processed_Arm_Angle_Rad();

    // 状态机任务以及动作完成判定
    void KFS_Status_Task();

    bool Is_Action_Finished();

    // 任务完成子函数
    void Check_Move_Task_Completion();

    void Check_Lift_Task_Completion();

    void Check_Wrist_Task_Completion();

    void Check_Arm_Task_Completion();

    void Enter_New_Status_Clear_Completion_Flag();
};

/**
 * @brief 遥控器控制高度接口
 */
inline void Class_KFS::Set_Lift_Height_Index(uint8_t index)
{
    if (index < MAX_LIFT_POSITION_INDEX)
    {
        Lift_Height_Index = index;
    }
}

/**
 * @brief 状态前进
 */
inline void Class_KFS::Status_Forward()
{
    Forward_Yaw_Flag = true;
}

/**
 * @brief 状态后退
 */
inline void Class_KFS::Status_Backward()
{
    Backward_Yaw_Flag = true;
}

#endif