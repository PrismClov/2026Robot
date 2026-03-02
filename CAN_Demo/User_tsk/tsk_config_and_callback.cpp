/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "ita_robot.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_mi.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Chariot chariot;

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):  
        {
<<<<<<< HEAD
                chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
=======
            chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
>>>>>>> d6d6a17 (R2_Code)
        }
        break;
		case (0x202):
		{
			chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case (0x203):
		{	
			chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case (0x204):
		{
			chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
    }
}

/**
 * @brief Chassis_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):  
    {
<<<<<<< HEAD
        chariot.Lift.Motor_Lift_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
=======
        chariot.Lift.Motor_Lift_L.CAN_RxCpltCallback(CAN_RxMessage->Data);
>>>>>>> d6d6a17 (R2_Code)
    }
    break;
    case (0x202):
    {
<<<<<<< HEAD
        chariot.Lift.Motor_Lift_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
=======
        chariot.Lift.Motor_Lift_R.CAN_RxCpltCallback(CAN_RxMessage->Data);
>>>>>>> d6d6a17 (R2_Code)
    }
    break;
    case (0x203):
    {
<<<<<<< HEAD
        chariot.Lift.Motor_Move_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
=======
        chariot.Lift.Motor_Move_L.CAN_RxCpltCallback(CAN_RxMessage->Data);
>>>>>>> d6d6a17 (R2_Code)
    }
    break;
    case (0x204):
    {
<<<<<<< HEAD
        chariot.Lift.Motor_Move_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
=======
        chariot.Lift.Motor_Move_R.CAN_RxCpltCallback(CAN_RxMessage->Data);
>>>>>>> d6d6a17 (R2_Code)
    }
    break;

    }
}
/**
 * @brief DR16_UART3回调函数
 * 
 * @param DR16_RxMessage DR16收到消息
 */
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}
<<<<<<< HEAD
uint16_t mod50 = 0;
=======

uint16_t mod50 = 0;

>>>>>>> d6d6a17 (R2_Code)
void Task_1ms_Callback()
{
		mod50++;
		if(mod50 >= 50)
		{
			mod50 = 0;
			chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
		}
   
    chariot.TIM_Unline_Protect_PeriodElapsedCallback();
    chariot.TIM_Calculate_PeriodElapsedCallback();
    TIM_CAN_PeriodElapsedCallback();
}

void Task_2ms_Callback()
{
}
extern "C" void Task_Init()
{

    /********************************** 驱动层初始化 **********************************/
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    CAN_Init(&hcan2, Chassis_Device_CAN2_Callback);

    UART_Init(&huart3, DR16_UART3_Callback, 18);

    //定时器循环任务
    TIM_Init(&htim1, Task_2ms_Callback);
    TIM_Init(&htim4, Task_1ms_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init(0.0484848469);

    /********************************* 使能调度时钟 *********************************/
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
}
