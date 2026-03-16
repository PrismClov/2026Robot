/**
 * @file dvc_motor_mi.cpp
 * @author lytg (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-11-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "dvc_motor_mi.h"

uint8_t *Class_Motor_MI::allocate_tx_data(CAN_HandleTypeDef *hcan, uint8_t __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;

    if (hcan == &hcan1)
    {
        tmp_tx_data_ptr = &(CAN1_0x72_Tx_Data[0]);
    }

    return tmp_tx_data_ptr;
}

void Class_Motor_MI::Init(CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID,
                          Enum_Motor_MI_Control_Method control_method)
{
    Hcan = hcan;
	
		if (hcan == &hcan1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan == &hcan2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }

    CAN_ID = __CAN_Rx_ID;

    Motor_MI_Control_Method = control_method;

    Tx_Data = allocate_tx_data(hcan, CAN_ID);

    Set_Mode(Motor_MI_Control_Method);

    Start_Motor();

}

void Class_Motor_MI::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_MI::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_MI_Status = Motor_MI_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_MI_Status = Motor_MI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief          浮点数转4字节函数
 * @param[in]      f:浮点数
 * @retval         4字节数组
 * @description  : IEEE 754 协议
 */
static uint8_t *Float_to_Byte(float f, uint8_t *byte)
{
    unsigned long longdata = 0;
    longdata = *(unsigned long *)&f;
    byte[3] = (longdata & 0xFF000000) >> 24;
    byte[2] = (longdata & 0x00FF0000) >> 16;
    byte[1] = (longdata & 0x0000FF00) >> 8;
    byte[0] = (longdata & 0x000000FF);
    return byte;
}

/**
 * @brief          小米电机回文16位数据转浮点
 * @param[in]      x:16位回文
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      bits:参数位数
 * @retval         返回浮点值
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
 * @brief          小米电机发送浮点转16位数据
 * @param[in]      x:浮点
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      bits:参数位数
 * @retval         返回浮点值
 */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief          写入电机参数
 * @param[in]      Motor:对应控制电机结构体
 * @param[in]      Index:写入参数对应地址
 * @param[in]      Value:写入参数值
 * @param[in]      Value_type:写入参数数据类型
 * @retval         none
 */
void Class_Motor_MI::Set_Motor_Parameter(uint16_t Index, float Value, char Value_type)
{
    Ext_ID = Communication_Type_SetSingleParameter << 24 | Master_CAN_ID << 8 | CAN_ID;
    Tx_Data[0] = Index;
    Tx_Data[1] = Index >> 8;
    Tx_Data[2] = 0x00;
    Tx_Data[3] = 0x00;
    if (Value_type == 'f')
    {
        Float_to_Byte(Value, &(Tx_Data[4]));
    }
    else if (Value_type == 's')
    {
        Tx_Data[4] = (uint8_t)Value;
        Tx_Data[5] = 0x00;
        Tx_Data[6] = 0x00;
        Tx_Data[7] = 0x00;
    }
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}
/**
 * @brief          设置电机模式(必须停止时调整！)
 * @param[in]      Motor:  电机结构体
 * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
 * @retval         none
 */
void Class_Motor_MI::Set_Mode(uint8_t Mode)
{
    Set_Motor_Parameter(Run_mode, Mode, 's');
}

/**
 * @brief          电流控制模式下设置电流
 * @param[in]      Motor:  电机结构体
 * @param[in]      Current:电流设置
 * @retval         none
 */
void Class_Motor_MI::Set_Current(float Current)
{
    Set_Motor_Parameter(Iq_Ref, Current, 'f');
}

/**
 * @brief          设置电机零点
 * @param[in]      Motor:  电机结构体
 * @retval         none
 */
void Class_Motor_MI::Set_Zeropos()
{
    for (int i = 0; i < 8; i++)
    {
        Tx_Data[i] = 0x00;
    }
    Tx_Data[0] = 1;
    Ext_ID = Communication_Type_SetPosZero << 24 | Master_CAN_ID << 8 | CAN_ID;
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}

/**
 * @brief          小米电机ID检查
 * @param[in]      id:  控制电机CAN_ID【出厂默认0x7F】
 * @retval         none
 */
void Class_Motor_MI::Chack_ID(uint8_t id)
{
    for (int i = 0; i < 8; i++)
    {
        Tx_Data[i] = 0x00;
    }
    Ext_ID = Communication_Type_GetID << 24 | Master_CAN_ID << 8 | id;
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}

/**
 * @brief          使能小米电机
 * @param[in]      Motor:对应控制电机结构体
 * @retval         none
 */
void Class_Motor_MI::Start_Motor()
{
    for (int i = 0; i < 8; i++)
    {
        Tx_Data[i] = 0x00;
    }
    Ext_ID = Communication_Type_MotorEnable << 24 | Master_CAN_ID << 8 | CAN_ID;
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}

/**
 * @brief          停止电机
 * @param[in]      Motor:对应控制电机结构体
 * @param[in]      clear_error:清除错误位（0 不清除 1清除）
 * @retval         None
 */
void Class_Motor_MI::Stop_Motor(uint8_t clear_error)
{
    for (int i = 0; i < 8; i++)
    {
        Tx_Data[i] = 0x00;
    }
    Tx_Data[0] = clear_error; // 清除错误位设置
    Ext_ID = Communication_Type_MotorStop << 24 | Master_CAN_ID << 8 | CAN_ID;
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}

/**
 * @brief          小米运控模式指令
 * @param[in]      Motor:  目标电机结构体
 * @param[in]      torque: 力矩设置[-12,12] N*M
 * @param[in]      MechPosition: 位置设置[-12.5,12.5] rad
 * @param[in]      speed: 速度设置[-30,30] rad/s
 * @param[in]      kp: 比例参数设置
 * @param[in]      kd: 微分参数设置
 * @retval         none
 */
void Class_Motor_MI::Motor_ControlMode(float torque, float MechPosition, float speed, float kp, float kd)
{
    // 装填发送数据
    Tx_Data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    Tx_Data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16);
    Tx_Data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    Tx_Data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
    Tx_Data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    Tx_Data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    Tx_Data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    Tx_Data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    Ext_ID = Communication_Type_MotionControl << 24 | float_to_uint(torque, T_MIN, T_MAX, 16) << 8 | CAN_ID; // 装填扩展帧数据
    CAN_Send_EXT_Data(Hcan, Ext_ID, Tx_Data, 8);
}
void Class_Motor_MI::Data_Process()
{
    uint16_t temp_angle = 0;
    uint16_t temp_omega = 0;
    uint16_t temp_torque = 0;
    uint16_t temp_temperature = 0;

    Struct_Motor_MI_CAN_Rx_Data *temp_buff = (Struct_Motor_MI_CAN_Rx_Data *)CAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16((void *)&temp_buff->Now_Angle, (void *)&temp_angle);
    Math_Endian_Reverse_16((void *)&temp_buff->Now_Omega, (void *)&temp_omega);
    Math_Endian_Reverse_16((void *)&temp_buff->Now_Torque, (void *)&temp_torque);
    Math_Endian_Reverse_16((void *)&temp_buff->Now_Temperature, (void *)&temp_temperature);

    Rx_Data.Angle = uint16_to_float(temp_angle, MIN_P, MAX_P, 16);
    Rx_Data.Omega = uint16_to_float(temp_omega, V_MIN, V_MAX, 16);
    Rx_Data.Torque = uint16_to_float(temp_torque, T_MIN, T_MAX, 16);
    Rx_Data.Temp = temp_temperature / 10.0;

    Rx_Data.CAN_ID = (CAN_Manage_Object->Rx_Buffer.Header.ExtId & 0xFFFF) >> 8;
}