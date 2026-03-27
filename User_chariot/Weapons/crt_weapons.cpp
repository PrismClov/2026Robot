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

    Motor_Boom.Init(&hfdcan1,0x00, 0x01);
    Motor_Boom.Set_K_P(0.0f);
    Motor_Boom.Set_K_D(0.0f);

    Motor_Forearm.Init(&hfdcan1,0xF0, 0x70);
    Motor_Forearm.Set_K_P(0.0f);
    Motor_Forearm.Set_K_D(0.0f);

    Motor_Rotate.Init(&hfdcan1,0xF1, 0x71);
    Motor_Rotate.Set_K_P(0.0f);
    Motor_Rotate.Set_K_D(0.0f);

    FSM_Weapon_Grab.Weapon_Grab = this;

    FSM_Weapon_Grab.Init(5,0);


}

/**
 * @brief 周期中断函数
 */
void Class_Weapon_Grab::TIM_Weapon_Grab_PeriodElapsedCallback()
{
    FSM_Weapon_Grab.Weapon_Grab_TIM_Status_PeriodElapsedCallback();

    // 电机发送数据
    // Motor_Boom.TIM_Send_PeriodElapsedCallback();
    // Motor_Forearm.TIM_Send_PeriodElapsedCallback();
    // Motor_Rotate.TIM_Send_PeriodElapsedCallback();
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
 * @brief 夹取初始化
 */
void Class_Weapon_Grab::Weapon_Grab_Init()
{
    Motor_Boom.Set_Control_Angle(Init_Angle[0]);
    Motor_Forearm.Set_Control_Angle(Init_Angle[1]);
    Motor_Rotate.Set_Control_Angle(Init_Angle[2]);

}

/**
 * @brief 判断初始化是否完成
 * 
 * @return true 完成
 * @return false 未完成
 */
bool Class_Weapon_Grab::Is_Init_Finished()
{
    bool res = false;

    return res;
}

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

            Weapon_Grab->Weapon_Grab_Init();
            
            if(Weapon_Grab->Is_Init_Finished())
            {

                Status[Now_Status_Serial].Count_Time = 0;
                Set_Status(Weapon_Grab_Status_GRAB);
                
            }
        }


    }
}
