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
#include "crt_chassis.h"
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
#include "crt_KFS.h"

Class_Chariot chariot;

/* Private macros ------------------------------------------------------------*/
// static float vbat = 0;
/* Private types -------------------------------------------------------------*/
Class_DS_Servo Pick_Servo[3];
// Motor::Class_Motor_DJI_C620 Motor_Arm; // 机械臂电机
Class_KFS KFS; // 夹取机构
/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;
uint32_t flag = 0;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief FDCAN1回调函数
 *
 * @param FDCAN_RxMessage FDCAN1收到的消息
 */
void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        case 0x201:
            chariot.Chassis.Motor_Steer[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        case 0x202:
            chariot.Chassis.Motor_Steer[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        case 0x203:
            chariot.Chassis.Motor_Steer[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        case 0x204:
            chariot.Chassis.Motor_Steer[3].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
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

        // 舵向编码器返回数据处理
        case (0x001):
        {

            break;
        }
        case (0x003):
        {

            break;
        }
        case (0x004):
        {

            break;
        }
        case (0x200):
        {

            break;
        }
        // 舵向编码器返回数据处理 (0x201-0x204)
        case 0x201:
        {
            //chariot.Chassis.Steer_Encoder[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            // Motor_Arm.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            KFS.Motor_Move.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x202:
        {
            chariot.Chassis.Steer_Encoder[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x203:
        {
            chariot.Chassis.Steer_Encoder[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x204:
        {
            chariot.Chassis.Steer_Encoder[3].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
    }
}

/**
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART5收到的消息
 * @param Length 长度
 */
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}

/**
 * @brief TIM4任务回调函数
 *
 */

void Task100us_TIM4_Callback()
{
}
float Target_Position = -0.2f;
/**
 * @brief TIM5任务回调函数
 *
 */
uint8_t mod100 = 0;
void Task1ms_TIM5_Callback()
{
    DWT_Update();
    flag++;

    // 10ms检测存活状态
    mod100++;
    if (mod100 >= 100)
    {
        //chariot.Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
        KFS.TIM_Alive_PeriodElapsedCallback();
        mod100 = 0;
    }
    // Motor_Arm.Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
    // Motor_Arm.Set_Target_Position(Target_Position);
    // Motor_Arm.Calculate();

    KFS.Move_To_Position(Target_Position);
    KFS.TIM_Control_PeriodElapsedCallback();
    Motor::DJI_TIM_Send_Group(&hfdcan3, Motor::CAN_Tx_ID_0x200_Only);
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
    UART_Init(&huart5, DR16_UART5_Callback, 36);
    // 定时器初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    // {
    //     Motor::Class_Motor_DJI_C620::Parameters params;
    //     params.PID_Position.Out_Max = 10.0f;
    //     M3508.Init(&hfdcan1, Motor::Motor_DJI_ID_0x201, params);
    // }
    // chariot.Init();
    Pick_Servo[0].Init(&htim1, TIM_CHANNEL_1, 500, 2500);
    Pick_Servo[1].Init(&htim1, TIM_CHANNEL_3, 500, 2500);
    Pick_Servo[2].Init(&htim2, TIM_CHANNEL_3, 500, 2500);
    //     Motor_Arm.Init(
    //     &hfdcan3,
    //     Motor::Motor_DJI_ID_0x201,
    //     Motor::Class_Motor_DJI_C620::Parameters{
    //         .PID_Position = PID_Parameters{
    //             .K_P = 0.0f,
    //             .K_I = 0.0f,
    //             .K_D = 0.0f,
    //             .Out_Max = 30.0f
    //         }, 
    //         .PID_Omega = PID_Parameters{
    //             .K_P = 0.0f,
    //             .K_I = 0.0f,
    //             .K_D = 0.0f,
    //             .Out_Max = 30.0f
    //         },   
    //     },3591.0f / 187.0f / 18.0f * 28.0f);
    // 战车层初始化
    // KFS.Init();
    // 交互层初始化

    // 机器人战车初始化

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    // 标记初始化完成
    init_finished = true;
}
float test_position = 0.0f; 
/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Pick_Servo[0].Set_Normalized_Position(test_position);
    Pick_Servo[1].Set_Normalized_Position(test_position);
    Pick_Servo[2].Set_Normalized_Position(test_position);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
