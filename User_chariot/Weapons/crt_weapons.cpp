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

/* Includes ------------------------------------------------------------------*/

#include "crt_weapons.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 初始化函数
 */
void Class_Weapon_Grab::Init()
{

    Motor_Boom.Init(&hfdcan2, 0x00, 0x01);
    Motor_Boom.Set_K_P(150.0f);
    Motor_Boom.Set_K_D(0.0f);

    Motor_Forearm.Init(&hfdcan2, 0xF0, 0x70);
    Motor_Forearm.Set_K_P(100.0f);
    Motor_Forearm.Set_K_D(5.0f);

    Motor_Rotate.Init(&hfdcan2, 0xFD, 0x7F);
    Motor_Rotate.Set_K_P(150.0f);
    Motor_Rotate.Set_K_D(5.0f);

    Motor_Boom.CAN_Send_Save_Zero();
    Motor_Forearm.CAN_Send_Save_Zero();
    Motor_Rotate.CAN_Send_Save_Zero();

    FSM_Weapon_Grab.Weapon_Grab = this;

    FSM_Weapon_Grab.Init(5, 0);
}

/**
 * @brief 周期中断函数
 */
void Class_Weapon_Grab::TIM_Weapon_Grab_PeriodElapsedCallback()
{
    FSM_Weapon_Grab.Weapon_Grab_TIM_Status_PeriodElapsedCallback();

    //    Motor_Boom.CAN_Send_Save_Zero();
    //    Motor_Forearm.CAN_Send_Save_Zero();
    //    Motor_Rotate.CAN_Send_Save_Zero();
    // 电机发送数据
    Motor_Boom.TIM_Send_PeriodElapsedCallback();
    Motor_Forearm.TIM_Send_PeriodElapsedCallback();
    Motor_Rotate.TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 周期存活中断函数
 */
void Class_Weapon_Grab::TIM_Alive_PeriodElapsedCallback()
{
    Motor_Boom.TIM_Alive_PeriodElapsedCallback();
    Motor_Forearm.TIM_Alive_PeriodElapsedCallback();
    Motor_Rotate.TIM_Alive_PeriodElapsedCallback();
}

/**
 * @brief 夹取状态任务
 */
void Class_Weapon_Grab::Weapon_Grab_Status_Task()
{
    // 电机动作
    Motor_Boom.Set_Control_Angle(Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][0]);
    Motor_Forearm.Set_Control_Angle(Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][1]);
    Motor_Rotate.Set_Control_Angle(Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][2]);

    boom_horizontal_angle = 1.2f - Motor_Boom.Get_Now_Angle();                               // 大臂与水平的夹角
    forearm_horizontal_angle = boom_horizontal_angle - Motor_Forearm.Get_Now_Angle() - 0.1f; // 前臂与水平的夹角

    forearm_compensation = k1 * cos(forearm_horizontal_angle);
    boom_compensation = k2 * cos(boom_horizontal_angle) - forearm_compensation;

    Motor_Boom.Set_Control_Torque(boom_compensation);
    Motor_Forearm.Set_Control_Torque(forearm_compensation);
    // 气缸动作可以添加
    // Pump_Set_State(Pump_State_Grab[FSM_Weapon_Grab.Get_Now_Status_Serial()]]);
}

/**
 * @brief 判断动作是否完成
 *
 * @return true 完成
 * @return false 未完成
 */
bool Class_Weapon_Grab::Is_Action_Finished()
{
    bool Motor_Is_Finished = Math_Abs(Motor_Boom.Get_Now_Angle() - Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][0]) < Position_Threshold &&
                             Math_Abs(Motor_Forearm.Get_Now_Angle() - Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][1]) < Position_Threshold &&
                             Math_Abs(Motor_Rotate.Get_Now_Angle() - Position_Target_Angle[FSM_Weapon_Grab.Get_Now_Status_Serial()][2]) < Position_Threshold &&
                             Math_Abs(Motor_Boom.Get_Now_Omega()) < Omega_Threshold &&
                             Math_Abs(Motor_Forearm.Get_Now_Omega()) < Omega_Threshold &&
                             Math_Abs(Motor_Rotate.Get_Now_Omega()) < Omega_Threshold;

    bool Pump_Is_Finished = true; // 待完成

    return Motor_Is_Finished && Pump_Is_Finished;
}

int temp = 0;

/**
 * @brief 状态周期中断函数
 *
 */
void Class_FSM_Weapon_Grab::Weapon_Grab_TIM_Status_PeriodElapsedCallback()
{

    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
    case Weapon_Grab_Status_Init:
    {
        Weapon_Grab->Weapon_Grab_Status_Task();

        if (Weapon_Grab->Is_Action_Finished())
        {

            Status[Now_Status_Serial].Count_Time = 0;
            if (temp == 1)
            {
                Set_Status(Weapon_Grab_Status_Grab);
            }
        }
    }
    case Weapon_Grab_Status_Grab:
    {
        Weapon_Grab->Weapon_Grab_Status_Task();

        if (Weapon_Grab->Is_Action_Finished())
        {

            Status[Now_Status_Serial].Count_Time = 0;
            if (temp == 2)
            {
                Set_Status(Weapon_Grab_Status_Lift);
            }
        }
    }
    case Weapon_Grab_Status_Lift:
    {
        Weapon_Grab->Weapon_Grab_Status_Task();

        if (Weapon_Grab->Is_Action_Finished())
        {

            Status[Now_Status_Serial].Count_Time = 0;
            if (temp == 3)
            {
                Set_Status(Weapon_Grab_Status_Rotate);
            }
        }
    }
    case Weapon_Grab_Status_Rotate:
    {
        Weapon_Grab->Weapon_Grab_Status_Task();

        if (Weapon_Grab->Is_Action_Finished())
        {

            Status[Now_Status_Serial].Count_Time = 0;
            if (temp == 4)
            {
                Set_Status(Weapon_Grab_Status_Fold);
            }
        }
    }
    case Weapon_Grab_Status_Fold:
    {
        Weapon_Grab->Weapon_Grab_Status_Task();

        if (Weapon_Grab->Is_Action_Finished())
        {

            Status[Now_Status_Serial].Count_Time = 0;
            // Set_Status(Weapon_Grab_Status_Release);
        }
    }
    }
}