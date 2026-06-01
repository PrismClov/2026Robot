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

#include "ita_robot.h"
#include "dvc_sbus.h"
#include "drv_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
static bool SBUS_Has_Remote_Input(Class_SBUS &sbus)
{
    return (sbus.Get_Left_X() != 0.0f ||
            sbus.Get_Left_Y() != 0.0f ||
            sbus.Get_Right_X() != 0.0f ||
            sbus.Get_Right_Y() != 0.0f );
}
/**
 * @brief 底盘，云台，发射机构初始化
 *
 */
void Class_Chariot::Init(float __Dead_Zone)
{
    SBUS.Init(&huart5);

    Chassis.Init();

    Lift.Init();

    Weapon_Head.Init();
    
    Dead_Zone = __Dead_Zone;


}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    SBUS.TIM1msMod50_Alive_PeriodElapsedCallback();
    Lift.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    if(SBUS.Get_SBUS_Status() == SBUS_Status_DISABLE)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{

    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_ENABLE);
    
    Lift.TIM_Calculate_PeriodElapsedCallback();

    Weapon_Head.TIM_Weapon_Head_PeriodElapsedCallback();
}
/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_SBUS_Control_Type()
{
   if (SBUS_Has_Remote_Input(SBUS) == true)
    {
        SBUS_Control_Type = SBUS_Control_Type_REMOTE;
    }
    else
    {
        SBUS_Control_Type = SBUS_Control_Type_NONE;
    }
    
}
/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
     // 检查SBUS是否有输入
    Judge_SBUS_Control_Type();

    // 判断当前活动的控制器
    if (SBUS.Get_Switch_B() == SBUS_Switch_Status_DOWN && 
        SBUS.Get_SBUS_Status() == SBUS_Status_ENABLE)
    {
        Active_Controller = Controller_SBUS;
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
    //遥控器摇杆值
    Judge_Active_Controller();
    //云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    static float chassis_omega = 0;   


	/************************************遥控器控制逻辑*********************************************/
    
    if (Active_Controller == Controller_SBUS && SBUS_Control_Type == SBUS_Control_Type_REMOTE)
    {
       float sbus_l_x, sbus_l_y, sbus_yaw;
        float chassis_velocity_x = 0, chassis_velocity_y = 0;
        static float chassis_omega = 0;
        static float chassis_angle = 0;
        // 排除遥控器死区
        sbus_l_x = (Math_Abs(SBUS.Get_Left_Y()) > Dead_Zone) ? SBUS.Get_Left_Y() : 0;
        sbus_l_y = (Math_Abs(SBUS.Get_Left_X()) > Dead_Zone) ? SBUS.Get_Left_X()*(-1.0f) : 0;
        // yaw和xy的死区是否相同存疑
        sbus_yaw = (Math_Abs(SBUS.Get_Right_X()) > Dead_Zone) ? SBUS.Get_Right_X()*(-1.0f) : 0;
        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = sbus_l_x * sqrt(1.0f - sbus_l_y * sbus_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = sbus_l_y * sqrt(1.0f - sbus_l_x * sbus_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_omega = sbus_yaw * Chassis.Get_Omega_Max();
        chassis_angle += chassis_omega;
        //键盘遥控器操作逻辑
        if (SBUS.Get_Switch_A() == SBUS_Switch_Status_DOWN) // 底盘失能
        {
			Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else if (SBUS.Get_Switch_A() == SBUS_Switch_Status_UP) // 底盘随动，遥控器专用
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
            Chassis.Set_Target_Omega(chassis_omega);
        }

        //抬升导轮速度
        float lift_move_speed = sbus_l_x * sqrt(1.0f - sbus_l_y * sbus_l_y 
            / 2.0f) * Lift.Get_Velocity_Max();

        Lift.Set_Move_Speed(lift_move_speed);
        
        if (SBUS.Get_Switch_D() == SBUS_Switch_Status_UP)  //Switch_D实际下拨，抬升控制
        {
            if(SBUS.Get_Right_X() > 0.95)
            {
                Lift.Yaw_Flag_True();
            }
        }

        if(SBUS.Get_Switch_C() == SBUS_Switch_Status_UP)  //Switch_C实际下拨，R2->R1
        {
            Lift.R2_To_R1_Flag_True();
        }
        else{
            Lift.R2_To_R1_Flag_False();
        }

        if(SBUS.Get_Switch_D() == SBUS_Switch_Status_DOWN)  //Switch_D实际上拨，夹爪控制
        {
            if(SBUS.Get_Right_X() > 0.95)
            {
               Weapon_Head.yaw_flag = true;
            }
        }


    }
    
    
}
void Class_Chariot::TIM_Control_Callback()
{
    Judge_Active_Controller();

    //底盘，云台，发射机构控制逻辑
    Control_Chassis();

}

/*遥控器操作*/
/*初始状态下，拨杆全部为上拨*/
//A杆：上拨底盘失能 下拨底盘使能
//D杆 ：上拨控制夹爪  下拨控制抬升
//C杆 ：下拨控制抬升距离为R2->R1