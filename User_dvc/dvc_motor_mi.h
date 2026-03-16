/**
 * @file dvc_motor_mi.h
 * @author lytg (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-11-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef DVC_MOTOR_XIAOMI_H
#define DVC_MOTOR_XIAOMI_H


/*include*/
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "drv_math.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
#define MAX_P 720
#define MIN_P -720
// 主机CANID设置
#define Master_CAN_ID 0x00 // 主机ID
// 控制命令宏定义
#define Communication_Type_GetID 0x00         // 获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 // 用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02  // 用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03   // 电机使能运行
#define Communication_Type_MotorStop 0x04     // 电机停止运行
#define Communication_Type_SetPosZero 0x06    // 设置电机机械零位
#define Communication_Type_CanID 0x07         // 更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11 // 读取单个参数
#define Communication_Type_SetSingleParameter 0x12 // 设定单个参数
#define Communication_Type_ErrorFeedback 0x15      // 故障反馈帧
// 参数读取宏定义
#define Run_mode 0x7005
#define Iq_Ref 0x7006
#define Spd_Ref 0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720 / 32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30 / 32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12 / 32767.0
#define Bias_Torque 0x8000
#define Temp_Gain 0.1

#define Motor_Error 0x00
#define Motor_OK 0X01


enum ERROR_TAG // 错误回传对照
{
    OK = 0,                // 无故障
    BAT_LOW_ERR = 1,       // 欠压故障
    OVER_CURRENT_ERR = 2,  // 过流
    OVER_TEMP_ERR = 3,     // 过温
    MAGNETIC_ERR = 4,      // 磁编码故障
    HALL_ERR_ERR = 5,      // HALL编码故障
    NO_CALIBRATION_ERR = 6 // 未标定
};

/**
 * @brief 小米电机源数据
 *
 */
struct Struct_Motor_MI_CAN_Rx_Data
{
    uint16_t Now_Angle;
    uint16_t Now_Omega;
    uint16_t Now_Torque;
    uint16_t Now_Temperature;
} __attribute__((packed));

/**
 *@brief 小米数据包
 */
typedef struct
{                   // 小米电机结构体
    uint8_t CAN_ID; // CAN ID
    uint8_t MCU_ID; // MCU唯一标识符[后8位，共64位]
    float Angle;    // 回传角度
    float Omega;    // 回传速度
    float Torque;   // 回传力矩
    float Temp;     // 回传温度

    uint16_t set_current;
    uint16_t set_speed;
    uint16_t set_position;

    uint8_t error_code;

    float Angle_Bias;

} Struct_Motor_MI_Rx_Data;

/**
 * @brief 小米状态
 *
 */
enum Enum_Motor_MI_Status
{
    Motor_MI_Status_DISABLE = 0,
    Motor_MI_Status_ENABLE,
};

/**
 * @brief 小米电机控制模式
 * 
 */
enum Enum_Motor_MI_Control_Method// 控制模式定义
{
    Motion_Mode = 0, // 运控模式
    Position_Mode,   // 位置模式
    Speed_Mode,      // 速度模式
    Current_Mode     // 电流模式
};
/**
 * @brief 小米电机ID
 *
 */
 enum Enum_Motor_MI_ID
 {
    Motor_MI_ID_0 = 0x72,
 };

class Class_Motor_MI
{
public:

    void Init(CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, 
        Enum_Motor_MI_Control_Method control_method = Position_Mode);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_Calculate_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void Motor_ControlMode(float torque, float MechPosition, float speed, float kp, float kd);
    
    uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, uint8_t __CAN_ID);

  private:
    CAN_HandleTypeDef *Hcan;

    uint8_t CAN_ID = 0;

    uint32_t Ext_ID = 0;

    uint32_t Flag = 0;

    uint32_t Pre_Flag = 0;

    Struct_CAN_Manage_Object *CAN_Manage_Object;

    uint8_t *Tx_Data;

    Enum_Motor_MI_Status Motor_MI_Status = Motor_MI_Status_DISABLE;

    Enum_Motor_MI_Control_Method Motor_MI_Control_Method = Position_Mode;

    Struct_Motor_MI_Rx_Data Rx_Data;

    void Chack_ID(uint8_t id);

    void Start_Motor();

    void Stop_Motor(uint8_t clear_error);

    void Set_Motor_Parameter(uint16_t Index,float Value,char Value_type);

    void Set_Mode(uint8_t Mode);

    void Set_Current(float Current);

    void Set_Zeropos();

    void Data_Process();
    
    

    void Output();

};
#endif