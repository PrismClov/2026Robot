#include "ita_robot.h"
/**
 * @file ita_robot.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 人机交互控制逻辑
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-17 1.1 更名为ita_robot.h, 引入新功能
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "dvc_crsf.h"
#include "ita_robot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 底盘，云台，发射机构初始化
 *
 */
void Class_Chariot::Init()
{
    // CRSF.Init(&huart7);

    // Chassis.Init();
    // Lift.Init();
    // Weapon.Init();
    KFS.Init();
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // CRSF.TIM1msMod50_Alive_PeriodElapsedCallback();
    // Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    // Lift.TIM_100ms_Alive_PeriodElapsedCallback();
    KFS.TIM_Alive_PeriodElapsedCallback();
    // Weapon.TIM_Alive_PeriodElapsedCallback();
}

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    // // Lift
    // Lift.TIM_Calculate_PeriodElapsedCallback();

    // // 底盘控制
    // Chassis.TIM_2ms_Control_PeriodElapsedCallback();

    // KFS
    KFS.TIM_Control_PeriodElapsedCallback();

    // // Weapon
    // Weapon.TIM_Weapon_PeriodElapsedCallback();
}
/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    if (CRSF.Get_Status() == CRSF_Status_DISABLE)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }
}

/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_CRSF_Control_Type()
{
    if (CRSF.Get_Left_X() != 0 ||
        CRSF.Get_Left_Y() != 0 ||
        CRSF.Get_Right_X() != 0 ||
        CRSF.Get_Right_Y() != 0)
    {
        CRSF_Control_Type = CRSF_Control_Type_REMOTE;
    }
    else
    {
        if (CRSF.Get_Status() == CRSF_Status_DISABLE)
        {
            CRSF_Control_Type = CRSF_Control_Type_NONE;
        }
    }
}
/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
    // 检查CRSF是否有输入
    Judge_CRSF_Control_Type();

    // 判断当前活动的控制器
    if (CRSF_Control_Type != CRSF_Control_Type_NONE)
    {
        Active_Controller = Controller_CRSF;
    }
    else
    {
        Active_Controller = Controller_NONE;
    }
}

/**
 * @brief 底盘，云台，发射机构控制逻辑
 *
 */
void Class_Chariot::Control_Chassis()
{
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;

    /************************************遥控器控制逻辑*********************************************/

    if (Active_Controller == Controller_CRSF && CRSF_Control_Type == CRSF_Control_Type_REMOTE)
    {
        float crsf_l_x, crsf_l_y, crsf_r_x, crsf_r_y;
        // 排除遥控器死区
        crsf_l_x = (Math_Abs(CRSF.Get_Left_X()) > Dead_Zone) ? CRSF.Get_Left_X() : 0;
        crsf_l_y = (Math_Abs(CRSF.Get_Left_Y()) > Dead_Zone) ? CRSF.Get_Left_Y() : 0;
        // 右摇杆X作为旋转
        crsf_r_x = (Math_Abs(CRSF.Get_Right_X()) > Dead_Zone) ? CRSF.Get_Right_X() : 0;
        crsf_r_y = (Math_Abs(CRSF.Get_Right_Y()) > Dead_Zone) ? CRSF.Get_Right_Y() : 0;
        // 遥控器前Y右X
        chassis_velocity_x = crsf_r_y * sqrt(1.0f - crsf_r_x * crsf_r_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_velocity_y = crsf_r_x * sqrt(1.0f - crsf_r_y * crsf_r_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_omega = crsf_l_x * Chassis.Get_Omega_Max();

        // 遥控器开关操作逻辑
        // SA开关控制使能情况
        if (CRSF.Get_SA() == CRSF_SWITCH_HIGH) // SA高档，底盘随动
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
            Chassis.Set_Target_Omega(chassis_omega);
        }
        else if (CRSF.Get_SA() == CRSF_SWITCH_LOW) // SA低档 禁用模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        // SB开关控制控制区域
        if (CRSF.Get_SB() == CRSF_SWITCH_LOW)
        {
            Robot_Mode = Robot_Mode_Weapon;
        }
        else if (CRSF.Get_SB() == CRSF_SWITCH_MIDDLE)
        {
            Robot_Mode = Robot_Mode_KFS;
        }
        else if (CRSF.Get_SB() == CRSF_SWITCH_HIGH)
        {
            Robot_Mode = Robot_Mode_Lift;
        }

        switch (Robot_Mode)
        {
            case Robot_Mode_KFS:
            {
                if (CRSF.Get_SE() == CRSF_SWITCH_LOW && Previous_SE_Pos == CRSF_SWITCH_HIGH)
                {
                    if (CRSF.Get_SD() == CRSF_SWITCH_LOW)
                    {
                        KFS.Status_Forward();
                    }
                    else if (CRSF.Get_SD() == CRSF_SWITCH_HIGH)
                    {
                        KFS.Status_Backward();
                    }
                }
                KFS.Set_Lift_Height_Index(CRSF.Get_SC() % 2);
                break;
            }
            case Robot_Mode_Weapon:
            {
                if (CRSF.Get_SE() == CRSF_SWITCH_LOW && Previous_SE_Pos == CRSF_SWITCH_HIGH)
                {
                    if (CRSF.Get_SD() == CRSF_SWITCH_LOW)
                    {
                        Weapon.Status_Forward();
                    }
                    else if (CRSF.Get_SD() == CRSF_SWITCH_HIGH)
                    {
                        Weapon.Status_Backward();
                    }
                }
                Weapon.Set_Move_Index(CRSF.Get_SC());
                break;
            }
            case Robot_Mode_Lift:
            {
                if (CRSF.Get_SE() == CRSF_SWITCH_LOW && Previous_SE_Pos == CRSF_SWITCH_HIGH)
                {
                    Lift.Yaw_Flag_True();
                }
                break;
            }
        }
        Previous_SE_Pos = CRSF.Get_SE();
    }
}
void Class_Chariot::TIM_Control_Callback()
{
    Judge_Active_Controller();

    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
}