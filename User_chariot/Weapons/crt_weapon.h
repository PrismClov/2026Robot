#ifndef CRT_WEAPON_H
#define CRT_WEAPON_H

#include "alg_fsm.h"
#include "config.h"
#include "dvc_ds_servo.h"
#include "dvc_dwt.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_rs_pid.h"

class Class_Chariot;
class Class_Weapon;

enum Enum_Weapon_Status
{
    Weapon_Status_Init = 0, // 初始化
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
    Weapon_Status_Grab_Prepare, // 抓取准备
    Weapon_Status_Grab,         // 抓取
    Weapon_Status_Lift_1,       // 抬起
    Weapon_Status_Pick, // 夹取
    Weapon_Status_Lift_2_Prepare, // 抬前准备(打开pick舵机)
    Weapon_Status_Lift_2,                // 抬起
    Weapon_Status_Rotate_To_Connection,  // 旋转到对接位置
    Weapon_Status_Show_Completion_Graph, // 显示对接成功图像
#endif
    Weapon_Status_Rotate_To_Storage_Prepare, // 旋转到存储位置-先旋转，机械臂保持缩回
    Weapon_Status_Attack_Postition_1, // 第一个攻击位置
    Weapon_Status_Attack_Postition_2, // 第二个攻击位置

    MAX_WEAPON_STATUS
};

/**
 * @brief 武器夹取FSM类
 *
 */
class Class_FSM_Weapon : public Class_FSM
{
public:
    Class_Weapon *Weapon;

    // 夹取状态
    void Weapon_TIM_Status_PeriodElapsedCallback();

    Enum_Weapon_Status Weapon_Status = Weapon_Status_Init;
};

class Class_Weapon
{
public:
    // 战车指针
    Class_Chariot *Chariot;

    // 夹取机械臂
    Class_DS_Servo Pick_Servo[3]; // 夹取舵机

    // 旋转机械臂
    Motor::Class_Motor_DJI_C620 Grab_Servo;

    // 机械臂电机
    Motor::Class_Motor_DJI_C620 Motor_Arm;

    Motor::Class_Motor_DJI_C620 Motor_Move; // 移动电机

    Motor::Class_Motor_DJI_C620 Motor_Pitch; // 俯仰电机

    Class_Motor_RS_PID Motor_Rotate; // 旋转电机

    Class_FSM_Weapon FSM_Weapon;

    friend class Class_FSM_Weapon;

    const float Bias_Max = PI / 24; // 5°微调范围

    void Init();

    inline void Set_Move_Index(uint8_t index);

    inline void Set_Rotate_Bias(float bias);

    inline void Status_Forward();

    inline void Status_Backward();

    void TIM_Weapon_PeriodElapsedCallback();

    void TIM_Alive_PeriodElapsedCallback();

private:
    bool Forward_Yaw_Flag = false;
    bool Backward_Yaw_Flag = false;

    // 机械臂电机(C620)参数
    bool Arm_Calibrated = false;         // 机械臂电机是否已完成堵转校准
    float Arm_Calibration_Offset = 0.0f; // 校准时记录的电机绝对角度
    Calibrate_Params Arm_Calibrate_Params = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = -5.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000};

    // 抓取电机(C610)参数
    bool Grab_Calibrated = false;
    float Grab_Calibration_Offset = 0.0f;
    Calibrate_Params Grab_Calibrate_Params = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = 5.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000};

    // 俯仰电机(C620)参数
    bool Pitch_Calibrated = false;         // 俯仰电机是否已完成堵转校准
    float Pitch_Calibration_Offset = 0.0f; // 校准时记录的电机绝对角度
    Calibrate_Params Pitch_Calibrate_Params = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = -5.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000};

    // 位移电机(M3508)参数
    float Move_Target_Position[3] = {-1.5f, -11.6f, -21.5f}; // 移动电机目标位置
    uint8_t Move_Index = 0;                                  // 移动位置索引
    bool Move_Calibrated = false;                            // 移动电机是否已完成堵转校准
    float Move_Calibration_Offset = 0.0f;                    // 校准时记录的电机绝对角度，用于将目标转为绝对角度
    Calibrate_Params Move_Calibration_Param = {
        .motion_mode = CALIBRATE_MOTION_SPEED,
        .motion_value = 5.0f,
        .detect_mode = CALIBRATE_DETECT_SPEED,
        .detect_threshold = 0.05f,
        .debounce_us = 200000};

    // 旋转电机参数
    float Rotate_Bias_Rad = 0.0f; // 遥控器微调值

    // 动作完成判定阈值
    const float Position_Threshold = 0.01f; // 位置误差阈值 m
    const float Omega_Threshold = 0.01f;    // 速度误差阈值 rad/s

    const float Locked_Rotor_Current_Threshold = 5.0f; // 堵转电流阈值 A

    // 任务完成标志
    bool Arm_Task_Finished = false;

    bool Move_Task_Finished = false;

    bool Pitch_Task_Finished = false;

    bool Servo_Task_Finished = false;

    bool Rotate_Task_Finished = false;

    bool Need_All_Servo_Action = false; // true: 三个夹取舵机同时动, false: 只动第2 - Move_Index个

    SoftTimer_t Servo_Delay_Timer = {0, 0};
    const uint32_t Servo_Delay_Us = 1000000;

    // 状态机配置表
    struct Target
    {
        // 机械臂目标位置
        float Arm_Target_Position[MAX_WEAPON_STATUS] = {
            0.0f, // Init
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            1.5f,  // Grab_Prepare
            1.5f,  // Grab
            0.65f, // Lift_1
            0.65f, // Pick
            0.65f, // Lift_2_Prepare
            0.65f, // Lift_2
            0.0f,  // Rotate_To_Connection
            0.0f,  // Show_Completion_Graph
#endif
            0.0f, // Rotate_To_Storage_Prepare
            0.0f, // Attack_Postition_1
            0.0f, // Attack_Postition_2
        };

        // 夹取舵机目标位置
        float Pick_Servo_Target_Position[MAX_WEAPON_STATUS] = {
            1.0f, // Init
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            1.0f, // Grab_Prepare
            0.0f, // Grab
            0.0f, // Lift_1
            0.0f, // Pick
            1.0f, // Lift_2_Prepare
            1.0f, // Lift_2
            1.0f, // Rotate_To_Connection
            1.0f, // Show_Completion_Graph
#endif
            1.0f, // Rotate_To_Storage_Prepare
            1.0f, // Attack_Postition_1
            1.0f, // Attack_Postition_2
        };

        // 抓取舵机目标位置
        float Grab_Servo_Target_Position[MAX_WEAPON_STATUS] = {
            -1.5f, // Init
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            -1.5f, // Grab_Prepare
            -1.5f, // Grab
            -1.5f, // Lift_1
            0.0f, // Pick
            0.0f, // Lift_2_Prepare
            0.0f, // Lift_2
            0.0f, // Rotate_To_Connection
            0.0f, // Show_Completion_Graph
#endif
            0.0f, // Rotate_To_Storage_Prepare
            0.0f, // Attack_Postition_1
            0.0f, // Attack_Postition_2
        };

        // 旋转电机目标位置
        float Rotate_Target_Position[MAX_WEAPON_STATUS] = {
            3.65f, // Init
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            3.65f, // Grab_Prepare
            3.65f, // Grab
            3.65f, // Lift_1
            3.65f, // Pick
            3.65f, // Lift_2_Prepare
            3.65f, // Lift_2
            5.2f,    // Rotate_To_Connection
            5.2f,    // Show_Completion_Graph
#endif
            3.65f, // Rotate_To_Storage_Prepare
            3.65f + PI / 6.0f, // Attack_Postition_1
            3.65f + PI / 3.0f, // Attack_Postition_2
        };

        // 俯仰电机目标位置
        float Pitch_Target_Position[MAX_WEAPON_STATUS] = {
            2.5f, // Init
#if defined(MAIN_COMPETITION) || defined(SKILL_COMPETITION_1)
            3.5f, // Grab_Prepare
            2.5f, // Grab
            2.5f, // Lift_1
            2.5f, // Pick
            2.5f, // Lift_2_Prepare
            1.0f, // Lift_2
            2.5f, // Rotate_To_Connection
            2.5f, // Show_Completion_Graph
#endif
            2.5f, // Rotate_To_Storage_Prepare
            2.5f, // Attack_Postition_1
            2.5f, // Attack_Postition_2
        };
    } Target;

    void Arm_To_Position(float x);

    void Grab_To_Position(float x);

    void Move_To_Position(float x);

    void Pitch_To_Position(float x);

    void Weapon_Grab_Status_Task();

    // 动作完成判定
    bool Is_Action_Finished();

    void Check_Arm_Task_Completion();

    void Check_Move_Task_Completion();

    void Check_Pitch_Task_Completion();

    void Check_Rotate_Task_Completion();

    void Check_Servo_Task_Completion();

    void Enter_New_Status_Clear_Completion_Flag();
};

inline void Class_Weapon::Set_Move_Index(uint8_t index)
{
    Move_Index = index < 3 ? index : Move_Index;
}

inline void Class_Weapon::Set_Rotate_Bias(float bias)
{
    Rotate_Bias_Rad = Math_Abs(bias) <= Bias_Max ? bias : Rotate_Bias_Rad;
}

inline void Class_Weapon::Status_Forward()
{
    Forward_Yaw_Flag = true;
}

inline void Class_Weapon::Status_Backward()
{
    Backward_Yaw_Flag = true;
}

#endif