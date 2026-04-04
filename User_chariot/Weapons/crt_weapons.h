/**
 * @file crt_weapons.cpp
 * @author hzy
 * @brief 武器夹取
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright NEUQ (c) 2025-2026
 *
 */
// 前x左y上z

#ifndef CRT_WEAPONS_H
#define CRT_WEAPONS_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dm.h"
#include "dvc_motor_rs.h"
#include "alg_fsm.h"
#include "dvc_airtool.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Class_Weapon_Grab;
/**
 * @brief 武器夹取状态枚举
 *
 */
enum Enum_Weapon_Grab_Status
{
    Weapon_Grab_Status_Init = 0, // 初始化
    Weapon_Grab_Status_Grab,     // 抓取
    Weapon_Grab_Status_Lift,     // 抬起
    Weapon_Grab_Status_Rotate,   // 旋转
    Weapon_Grab_Status_Fold,     // 翻折
    Weapon_Grab_Status_Release,  // 释放
};

/**
 * @brief 武器夹取FSM类
 *
 */
class Class_FSM_Weapon_Grab : public Class_FSM
{
public:
    Class_Weapon_Grab *Weapon_Grab;

    // 夹取状态
    void Weapon_Grab_TIM_Status_PeriodElapsedCallback();

    Enum_Weapon_Grab_Status Weapon_Grab_Status = Weapon_Grab_Status_Init;
};

/**
 * @brief 武器夹取类
 *
 */
class Class_Weapon_Grab
{
public:
    // 大臂
    Class_Motor_DM_Normal Motor_Boom;
    // 前臂
    Class_Motor_RS_MIT Motor_Forearm;
    // 旋转
    Class_Motor_RS_MIT Motor_Rotate;

    Class_FSM_Weapon_Grab FSM_Weapon_Grab;

    Class_AIRPUMP AIRPUMP_Weapon_Grab;

    friend class Class_FSM_Weapon_Grab;

    void Init();

    // 获取正解x坐标 m
    inline float Get_Coordinate_X();
    // 获取正解z坐标 m
    inline float Get_Coordinate_Z();

    inline float Get_Boom_Angle();

    inline float Get_Forearm_Angle();

    inline float Get_Rotate_Angle();

    /*状态机初始动作*/
    void Weapon_Grab_Status_Task();

    bool Is_Action_Finished();

    void TIM_Weapon_Grab_PeriodElapsedCallback();

    void TIM_Alive_PeriodElapsedCallback();

private:
    float Coordinate_X = 0.0f;

    float Coordinate_Z = 0.0f;

    // 关节电机角度
    float Boom_Angle = 0.0f;

    float Forearm_Angle = 0.0f;

    float Rotate_Angle = 0.0f;

    // 夹取机构的几何参数
    const float Boom_Length = 0.5f;    // 大臂长度 m
    const float Forearm_Length = 0.3f; // 前臂长度 m

    float Position_Target_Angle[6][3] = {
        {0.0f, 0.0f, 0.0f},  // Init
        {0.8f, 0.0f, 0.0f},  // Grab
        {1.2f, 0.0f, 0.0f},  // Lift
        {1.2f, 0.0f, -1.4f}, // Rotate
        {0.0f, 0.0f, 0.0f},  // Fold
        {0.0f, 0.0f, 0.0f}   // Release
    };

    const float Position_Threshold = 0.05f; // 位置误差阈值 m
    const float Omega_Threshold = 0.05f;    // 速度误差阈值 rad/s

    float boom_horizontal_angle = 0.0f;
    float forearm_horizontal_angle = 0.0f;
    float boom_compensation = 0.0f;
    float forearm_compensation = 0.0f;
    float k1 = -2.0f;
    float k2 = 1.0f;
};

/**
 * @brief 获取正解x坐标
 */

float Class_Weapon_Grab::Get_Coordinate_X()
{
    return Coordinate_X;
}

/**
 * @brief 获取正解z坐标
 *
 */
float Class_Weapon_Grab::Get_Coordinate_Z()
{
    return Coordinate_Z;
}

/**
 * @brief 获取大臂角度
 *
 */
float Class_Weapon_Grab::Get_Boom_Angle()
{
    return Boom_Angle;
}

/**
 * @brief 获取前臂角度
 *
 */
float Class_Weapon_Grab::Get_Forearm_Angle()
{
    return Forearm_Angle;
}

/**
 * @brief 获取旋转角度
 *
 */
float Class_Weapon_Grab::Get_Rotate_Angle()
{
    return Rotate_Angle;
}

#endif // CRT_WEAPONS_H
