/**
 * @file crt_weapon_head.h
 * @author hzy 
 * @brief 武器头电控
 * @version 0.1
 * @date 2026-04-4
 *
 */

#ifndef __CRT_WEAPON_HEAD_H
#define __CRT_WEAPON_HEAD_H



/* Includes ------------------------------------------------------------------*/
#include "dvc_motor_dji.h"
#include "dvc_airtool.h"
#include "alg_fsm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
class Class_Weapon_Head;
enum Enum_Weapon_Head_Status
{
    Weapon_Head_Status_Calibrate = 0,
    Weapon_Head_Status_UP,
    Weapon_AirPumb_Open,
    Weapon_Head_Status_DOWN,
    Weapon_Head_Status_Fixed,
    Weapon_AirPumb_Close,
};

/**
 * @brief 抬升状态机
 * 
 */

class Class_FSM_Weapon_Head: public Class_FSM
{
    public:

    Class_Weapon_Head *Weapon_Head;

    void Weapon_Head_TIM_Status_PeriodElapsedCallback();

    Enum_Weapon_Head_Status Weapon_Head_Status =  Weapon_Head_Status_Calibrate;

};
/**
 * @brief 武器头电控类
 */

class Class_Weapon_Head
{
public:
    
    Class_PID PID_Distance_Lead_Screw;

    Class_Motor_DJI_C610 Motor_Lead_Screw;
    Class_AIRPUMP AIRPUMP_Grab;
    Class_FSM_Weapon_Head FSM_Weapon_Head;

    friend class Class_FSM_Weapon_Head;
    bool yaw_flag;

    void Init();
    //校准
    void Calibrate();
    bool Is_Calibrate_Finished();
    void Calibrate_Cancel();
    //抬升
    void Up();
    bool Is_Up_Finished();
    void Up_Cancel();
    void Down();
    void Weapon_Head_Fixed();
    bool Is_Down_Finished();
    void Down_Cancel();
    void TIM_Weapon_Head_PeriodElapsedCallback();
    bool Get_Yaw_Flag();

private:
    void Distance_Calculate();

    uint8_t Calibrate_Switch = 0;
    //丝杆导程 单位cm
    float Lead = 0.4;
    //校准行程
    float Angle_Calibrate = 0.0;
    //丝杆行程
    float Distance_Now = 0.0f;
    float Distance_Up = -2.0f;//test
    float Distance_Down = -14.0f;
    //校准速度
    float Calibrate_Omega = 20.0f;
};


#endif
