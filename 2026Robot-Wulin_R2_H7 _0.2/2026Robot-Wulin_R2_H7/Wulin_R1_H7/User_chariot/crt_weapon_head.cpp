/**
 * @file crt_weapon_head.cpp
 * @author hzy 
 * @brief 武器头电控
 * @version 0.1
 * @date 2024-11-25 0.1 26赛季定稿
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "crt_weapon_head.h"



/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 武器头电控初始化
 */

void Class_Weapon_Head::Init()
{
    Motor_Lead_Screw.Init(&hfdcan1, Motor_DJI_ID_0x205, Motor_DJI_Control_Method_OMEGA);
    PID_Distance_Lead_Screw.Init(12.0f, 0.01f, 0.0f, 0.0f, 1.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    Motor_Lead_Screw.PID_Omega.Init(10.0f, 0.01f, 0.001f, 0.0f, 0.01f, 6.0f, 0.001f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    
    AIRPUMP_Grab.Init(GPIOC, GPIO_PIN_13);

    FSM_Weapon_Head.Weapon_Head = this;
    FSM_Weapon_Head.Init(4, 0);
}

/**
 * @brief 距离计算
 *
 */
void Class_Weapon_Head::Distance_Calculate()
{
    Distance_Now = (Motor_Lead_Screw.Get_Now_Angle() - Angle_Calibrate) * Lead / (2 * PI);

    PID_Distance_Lead_Screw.Set_Now(Distance_Now);
}
/**
 * @brief 校准
 */
void Class_Weapon_Head::Calibrate()
{
    Motor_Lead_Screw.Set_Target_Omega(Calibrate_Omega);
}
/*
* @brief 获取校准完成标志
*/
bool Class_Weapon_Head::Is_Calibrate_Finished()
{
    bool res = !bool(Calibrate_Switch);
    return res;
}

/*
*  @brief 校准注销
*/
void Class_Weapon_Head::Calibrate_Cancel()
 {
    //速度环锁死
    Motor_Lead_Screw.Set_Target_Omega(0.0f);
    Angle_Calibrate = Motor_Lead_Screw.Get_Now_Angle();
    
 }


void Class_Weapon_Head::Up()
{
    //复位到校准位置
    PID_Distance_Lead_Screw.Set_Target(Distance_Up);

    PID_Distance_Lead_Screw.TIM_Calculate_PeriodElapsedCallback();

    Motor_Lead_Screw.Set_Target_Omega(PID_Distance_Lead_Screw.Get_Out());
}
bool Class_Weapon_Head::Is_Up_Finished()
{
    bool res = false;
    if(Math_Abs(Distance_Now - Distance_Up) < 0.4f && Math_Abs(Motor_Lead_Screw.Get_Now_Omega()) < 0.05f)//距离误差修改
    {
        res = true;
    }
    return res;
}
void Class_Weapon_Head::Up_Cancel()
{
    //速度环锁死
    Motor_Lead_Screw.Set_Target_Omega(0.0f);
    
}
void Class_Weapon_Head::Down()
{
    //复位到校准位置
    PID_Distance_Lead_Screw.Set_Target(Distance_Down);

    PID_Distance_Lead_Screw.TIM_Calculate_PeriodElapsedCallback();

    Motor_Lead_Screw.Set_Target_Omega(PID_Distance_Lead_Screw.Get_Out());
}
void Class_Weapon_Head::Weapon_Head_Fixed()
{
    /*Motor_Lead_Screw.Set_Control_Method(Motor_MKSESC_Control_Method_Current);
    //设置目标电流
    Motor_Lead_Screw.Set_Target_Current(0.5f);*/
    
}
bool Class_Weapon_Head::Is_Down_Finished()
{
    bool res = false;
    if(Math_Abs(Distance_Now - Distance_Down) < 0.2f && Math_Abs(Motor_Lead_Screw.Get_Now_Omega()) < 0.05f)//距离误差修改
    {
        res = true;
    }
    return res;
}
void Class_Weapon_Head::Down_Cancel()
{
    //速度环锁死
    Motor_Lead_Screw.Set_Target_Omega(0.0f);
}
bool temp_cali = false;
bool temp_up = false;
bool temp_air_open = false;
bool temp_air_close = false;
bool temp_down = false;

void Class_FSM_Weapon_Head::Weapon_Head_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;
    switch (Now_Status_Serial)
    {
        case Weapon_Head_Status_Calibrate:
        {
            temp_cali = Weapon_Head->Is_Calibrate_Finished();
            if(!Weapon_Head->Is_Calibrate_Finished())
            {
                Weapon_Head->Calibrate();
            } 
            else if(Weapon_Head->Is_Calibrate_Finished())
            {
                Weapon_Head->Calibrate_Cancel();
                Status[Now_Status_Serial].Count_Time = 0;
                if(Weapon_Head->Get_Yaw_Flag())
                {
                    Set_Status(Weapon_Head_Status_UP);
                }
                
            }
        }
        break;
        case Weapon_Head_Status_UP:
        {
            temp_up = Weapon_Head->Is_Up_Finished();
            if( ! Weapon_Head->Is_Up_Finished())
            {
                Weapon_Head->Up();
            }    
            else if(Weapon_Head->Is_Up_Finished())
            {
               Weapon_Head->Up_Cancel();
               Status[Now_Status_Serial].Count_Time = 0;
               if(Weapon_Head->Get_Yaw_Flag())
               {
                 Set_Status(Weapon_AirPumb_Open);
               }
               
        
            }
        }
        break;
        case Weapon_AirPumb_Open:
        {
            temp_air_open = true;
            Weapon_Head->AIRPUMP_Grab.AIRPUMP_Open();
            Status[Now_Status_Serial].Count_Time = 0;
            if(Weapon_Head->Get_Yaw_Flag())
            {
                Set_Status(Weapon_Head_Status_DOWN);
            }

            
        }
        break;
        case Weapon_Head_Status_DOWN:
        {
            if(!Weapon_Head->Is_Down_Finished())
            {
                Weapon_Head->Down();
            }    
            else if(Weapon_Head->Is_Down_Finished())
            {
                Weapon_Head->Down_Cancel();
                Status[Now_Status_Serial].Count_Time = 0;
                temp_down = true;
                 if(Weapon_Head->Get_Yaw_Flag())
              {
                //Set_Status(Weapon_Head_Status_Fixed);
              }
               
            }

        }
        break;
       /* case Weapon_Head_Status_Fixed:
        {
            if(Status[Now_Status_Serial].Count_Time<=100&&Status[Now_Status_Serial].Count_Time>=0)
            {
               Weapon_Head->Weapon_Head_Fixed();
            }
            else 
            {
                Status[Now_Status_Serial].Count_Time = 0;
                Motor_Lead_Screw.Set_Control_Method(Motor_MKSESC_Control_Method_Omega);
                Motor_Lead_Screw.Set_Target_Omega(0.0f);

            }
        }
        break;*/
       /* case Weapon_AirPumb_Close:
        {
            temp_air_close = true;
            Weapon_Head->AIRPUMP_Grab.AIRPUMP_Close();
           
        }
        break;*/
        default:
            break;
    }
    Weapon_Head->yaw_flag = false;
 }

/**
 * @brief  武器头回调函数
 */
void Class_Weapon_Head::TIM_Weapon_Head_PeriodElapsedCallback()
{
    //微动开关电平
    Calibrate_Switch = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    
    Distance_Calculate(); 

    FSM_Weapon_Head.Weapon_Head_TIM_Status_PeriodElapsedCallback();

    //PID回调
    Motor_Lead_Screw.TIM_Calculate_PeriodElapsedCallback();
}

 bool Class_Weapon_Head::Get_Yaw_Flag()
{
    return yaw_flag;
}