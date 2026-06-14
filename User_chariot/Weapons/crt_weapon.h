#ifndef CRT_WEAPON_H
#define CRT_WEAPON_H

#include "alg_fsm.h"
#include "dvc_ds_servo.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"

class Class_Weapon;

enum Enum_Weapon_Status
{
    Weapon_Status_Init = 0, // 初始化
    Weapon_Status_Grab,     // 抓取
    Weapon_Status_Lift,     // 抬起
    Weapon_Status_Rotate,   // 旋转
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
    Class_DS_Servo Pick_Servo[3]; // 夹取舵机

    Class_DS_Servo Grab_Servo; // 抓取舵机

    Class_Motor_DM_Normal Motor_Rotate; // 旋转电机

    Motor::Class_Motor_DJI_C620 Motor_Arm; // 机械臂电机

    Motor::Class_Motor_DJI_C620 Motor_Move; // 移动电机

    Class_FSM_Weapon FSM_Weapon;

    friend class Class_FSM_Weapon;

    void Init();

    void Move_To_Position(float x);

    void Weapon_Grab_Status_Task();

    bool Is_Action_Finished();

    void TIM_Weapon_PeriodElapsedCallback();

    void TIM_Alive_PeriodElapsedCallback();

private:
    // 夹取机构的几何参数
    const float Boom_Length = 0.5f;    // 大臂长度 m
    const float Forearm_Length = 0.3f; // 前臂长度 m

    // 机械臂参数
    float Arm_Target_Position[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 机械臂目标位置
    float Pick_Servo_Target_Position[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 夹取舵机目标位置
    float Grab_Servo_Target_Position[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 抓取舵机目标位置
    float Rotate_Target_Position[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 旋转电机目标位置

    // 位移电机参数
    float Move_Target_Position[3] = {0.0f, 0.0f, 0.0f}; // 移动电机目标位置

    const float Stroke = 0.1f;             // 移动机构导程，输出轴每圈前进距离 m/rev
    const float Position_Threshold = 0.01f; // 位置误差阈值 m
    const float Omega_Threshold = 0.01f;    // 速度误差阈值 rad/s

    const float Locked_Rotor_Current_Threshold = 5.0f; // 堵转电流阈值 A

    bool Is_Calibrated = false;      // 移动电机是否已完成堵转校准
    float Calibration_Offset = 0.0f; // 校准时记录的电机绝对角度，用于将目标转为绝对角度
    float Calibrate_Speed = -0.3f;   // 校准时的目标速度 rad/s
};

#endif