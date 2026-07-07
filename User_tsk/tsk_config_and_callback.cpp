/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如底盘陀螺仪对象, 底盘类要调用它做小陀螺, 云台要调用它做方位感知, 因此底盘陀螺仪是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "crt_KFS.h"
#include "crt_chassis.h"
#include "crt_weapon.h"
#include "drv_bsp.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "dvc_ds_servo.h"
#include "dvc_dwt.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_mksesc.h"
#include "dvc_motor_rs.h"
#include "dvc_swerve_module.h"
#include "ita_robot.h"
Class_Chariot chariot;

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;
uint32_t flag = 0;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        // KFS 移动电机数据反馈
        case 0x201:
        {
            chariot.KFS.Motor_Move.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            // chariot.Lift.Motor_Lift_L.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
            // KFS 抬升电机数据反馈
        case 0x202:
        {
            chariot.KFS.Motor_Lift[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            // chariot.Lift.Motor_Lift_R.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);

            break;
        }
        case 0x203:
        {
            chariot.KFS.Motor_Lift[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // Weapon 移动电机数据反馈
        case 0x204:
        {
            chariot.Weapon.Motor_Move.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // Weapon 机械臂电机数据反馈
        case 0x205:
        {
            chariot.Weapon.Motor_Arm.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        //  KFS 手腕电机数据反馈
        case 0xF0:
        {
            chariot.KFS.Motor_Wrist.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // KFS 机械臂电机数据反馈 (达妙, Master_ID=0x00)
        case 0x00:
        {
            chariot.KFS.Motor_Arm.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // 武器夹取旋转电机
        case 0xFD:
        {
            chariot.Weapon.Motor_Rotate.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief FDCAN2回调函数
 *
 * @param FDCAN_RxMessage FDCAN2收到的消息
 */
void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
            // VESC 电调数据反馈
        case 0x901:
        case 0x1001:
        {
            chariot.Chassis.Motor_Wheel[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x902:
        case 0x1002:
        {
            chariot.Chassis.Motor_Wheel[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x903:
        case 0x1003:
        {
            chariot.Chassis.Motor_Wheel[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x904:
        case 0x1004:
        {
            chariot.Chassis.Motor_Wheel[3].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
    }
}

/**
 * @brief FDCAN3回调函数,此函数为本杰明电调通信用，FDCAN配置为扩展ID
 *
 * @param FDCAN_RxMessage FDCAN3收到的消息
 */
void Device_FDCAN3_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        // 舵向编码器返回数据处理 (0x101-0x104)
        case 0x101:
        {
            chariot.Chassis.Steer_Encoder[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x102:
        {
            chariot.Chassis.Steer_Encoder[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x103:
        {
            chariot.Chassis.Steer_Encoder[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x104:
        {
            chariot.Chassis.Steer_Encoder[3].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // C610 舵向电机数据反馈
        case 0x201:
        {
            chariot.Chassis.Motor_Steer[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x202:
        {
            chariot.Chassis.Motor_Steer[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x203:
        {
            chariot.Chassis.Motor_Steer[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x204:
        {
            chariot.Chassis.Motor_Steer[3].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // Weapon 俯仰电机数据反馈
        case 0x205:
        {
            chariot.Weapon.Motor_Pitch.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        // Lift 抬升电机数据反馈
        case 0x207:
        {
            chariot.Lift.Motor_Lift_L.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x208:
        {
            chariot.Lift.Motor_Lift_R.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief UART7 CRSF遥控器回调函数
 *
 * @param Buffer UART7收到的消息
 * @param Length 长度
 */
void CRSF_UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.CRSF.CRSF_UART_RxCpltCallback(Buffer, Length);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}

float Target_Position = 0.0f;
/**
 * @brief TIM5任务回调函数
 *
 */
uint8_t mod100 = 0;
uint8_t mod2 = 0;
void Task1ms_TIM5_Callback()
{
    DWT_Update();
    flag++;

    // 10ms检测存活状态
    mod100++;
    if (mod100 >= 100)
    {
        chariot.TIM_100ms_Alive_PeriodElapsedCallback();
        mod100 = 0;
    }

    mod2++;
    if (mod2 >= 10)
    {
        chariot.TIM_Calculate_PeriodElapsedCallback();
        Motor::DJI_TIM_Send_Group(&hfdcan1, Motor::CAN_Tx_ID_Both);
        Motor::DJI_TIM_Send_Group(&hfdcan3, Motor::CAN_Tx_ID_Both);
        mod2 = 0;
    }

    chariot.TIM_Unline_Protect_PeriodElapsedCallback();
}

void Task_Init()
{
    // 驱动层初始化
    DWT_Init();
    // 点俩灯, 开24V
    BSP_Init(BSP_DC24_L_OFF | BSP_DC24_R_OFF | BSP_DC5_ON, 0.0, 0.0);
    // CAN总线初始化
    FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);
    FDCAN_Init(&hfdcan2, Device_FDCAN2_Callback);
    FDCAN_Init(&hfdcan3, Device_FDCAN3_Callback);
    // UART初始化
    UART_Init(&huart7, CRSF_UART7_Callback, 64);
    UART_Init(&huart1, nullptr, 128);
    // 定时器初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    HAL_Delay(2000);
    // 战车层初始化
    chariot.Init();
    // 交互层初始化

    // 机器人战车初始化

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim5);
    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
// uint32_t cmp = 500;
void Task_Loop()
{
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cmp);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, cmp);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
