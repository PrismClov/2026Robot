/**
 * @file dvc_motor_robstride.cpp
 * @author 
 * @brief RobStride (小米CyberGear协议) 电机配置与操作
 * @version 0.1
 * @date 2024-03-18
 *
 * @copyright USTC-RoboWalker (c) 2024
 */

/* Includes ------------------------------------------------------------------*/

#include "Robstride.h"

/* Private macros ------------------------------------------------------------*/

// 协议限制常量
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

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 电机初始化
 * @param hfdcan 绑定的硬件CAN句柄
 * @param __CAN_ID 电机目标ID (0-127)
 * @param __Master_ID 主机ID (通常为0x00或0xFD)
 */
void Class_RobStride_Motor::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_ID, uint8_t __Master_ID)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if(hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }

    CAN_ID = __CAN_ID;
    Master_ID = __Master_ID;
}

/**
 * @brief 构造小米协议专用的扩展帧ID
 * @param Comm_Type 通信类型 (Bit[28:24])
 * @param Data 辅助数据 (Bit[23:8], 视具体指令而定)
 */
uint32_t Class_RobStride_Motor::Build_ExtID(uint8_t Comm_Type, uint16_t Data)
{
    uint32_t extid = 0;
    extid |= (uint32_t)(Comm_Type & 0x1F) << 24;
    extid |= (uint32_t)Data << 8;
    extid |= (uint32_t)CAN_ID;
    return extid;
}

/**
 * @brief 获取设备ID和MCU唯一标识符
 */
void Class_RobStride_Motor::CAN_Send_Get_ID()
{
    uint8_t tx_data[8] = {0};
    uint32_t send_id = Build_ExtID(RS_Comm_GetID, Master_ID);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}
/**
 * @brief 运控模式 (MIT) 发送函数
 */
void Class_RobStride_Motor::CAN_Send_Motion_Control(float Angle, float Omega, float Torque, float Kp, float Kd)
{
    // 限制范围
    Math_Constrain(&Angle, P_MIN, P_MAX);
    Math_Constrain(&Omega, V_MIN, V_MAX);
    Math_Constrain(&Torque, T_MIN, T_MAX);
    Math_Constrain(&Kp, KP_MIN, KP_MAX);
    Math_Constrain(&Kd, KD_MIN, KD_MAX);

    // 构造ID (运控模式下ID包含扭矩信息)
    uint16_t torque_int = Math_Float_To_Int(Torque, T_MIN, T_MAX, 0, 0xFFFF);
    uint32_t send_id = Build_ExtID(RS_Comm_MotionControl, torque_int);

    // 填充数据段
    uint8_t tx_data[8];
    uint16_t p_int = Math_Float_To_Int(Angle, P_MIN, P_MAX, 0, 0xFFFF);
    uint16_t v_int = Math_Float_To_Int(Omega, V_MIN, V_MAX, 0, 0xFFFF);
    uint16_t kp_int = Math_Float_To_Int(Kp, KP_MIN, KP_MAX, 0, 0xFFFF);
    uint16_t kd_int = Math_Float_To_Int(Kd, KD_MIN, KD_MAX, 0, 0xFFFF);

    tx_data[0] = p_int >> 8;
    tx_data[1] = p_int & 0xFF;
    tx_data[2] = v_int >> 8;
    tx_data[3] = v_int & 0xFF;
    tx_data[4] = kp_int >> 8;
    tx_data[5] = kp_int & 0xFF;
    tx_data[6] = kd_int >> 8;
    tx_data[7] = kd_int & 0xFF;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}

/**
 * @brief 使能电机
 */
void Class_RobStride_Motor::CAN_Send_Enable()
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorEnable, Master_ID);
    uint8_t tx_data[8] = {0};
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}

/**
 * @brief MIT模式使能
 */
void Class_RobStride_Motor::CAN_Send_MIT_Enable()
{
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_ID, tx_data, FDCAN_ID_Standard, 8);
}

/**
 * @brief 停止电机
 */
void Class_RobStride_Motor::CAN_Send_Stop(uint8_t clear_error)
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorStop, Master_ID);
    uint8_t tx_data[8] = {0};
    tx_data[0] = clear_error; 
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}


/**
 * @brief 接收回调函数 (仿达妙风格)
 */
void Class_RobStride_Motor::CAN_RxCpltCallback_Private(uint32_t ExtId, uint8_t *Data)
{
    // 判断是否为本电机的反馈报文 (通信类型 0x02)
    if ((ExtId & 0xFF) == CAN_ID && ((ExtId >> 24) & 0x1F) == RS_Comm_MotorRequest)
    {
        Flag++; // 滑动窗口计数
        Data_Process(ExtId, Data);
    }
}
/**
 * @brief  (小米协议专用)
 */
void Class_RobStride_Motor::CAN_RxCpltCallback_MIT(uint8_t *Data)
{
    Flag++; // 滑动窗口计数
    Data_Process(0, Data);
}
/**
 * @brief 数据处理
 */
void Class_RobStride_Motor::Data_Process(uint32_t ExtId, uint8_t *Data)
{
    // 解析物理量 (小米协议为高位在前)
    uint16_t p_int = (Data[1] << 8) | Data[2];
    uint16_t v_int = (Data[3] << 4) | (Data[4] >> 4);
    uint16_t t_int = ((Data[4] & 0xF) << 8) | Data[5];
    uint16_t temp_int = (Data[6] << 8) | Data[7];

    Rx_Data.Now_Angle = Math_Int_To_Float(p_int, 0, 0xFFFF, P_MIN, P_MAX);
    Rx_Data.Now_Omega = Math_Int_To_Float(v_int, 0, 0xFFF, V_MIN, V_MAX);
    Rx_Data.Now_Torque = Math_Int_To_Float(t_int, 0, 0xFFF, T_MIN, T_MAX);
    Rx_Data.Now_Temperature = (float)temp_int * 0.1f;

    // 错误状态解析
    Rx_Data.Error_Code = (uint16_t)((ExtId >> 16) & 0x3F);
}

/**
 * @brief 存活监测回调 (100ms周期)
 */
void Class_RobStride_Motor::TIM_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_Status = RobStride_Status_DISABLE;
    }
    else
    {
        Motor_Status = RobStride_Status_ENABLE;
    }
    Pre_Flag = Flag;

    // 如果掉线且需要运行，可在此尝试自动重连
    if (Motor_Status == RobStride_Status_DISABLE)
    {
        CAN_Send_Enable();
        CAN_Send_MIT_Enable();
    }
}

/**
 * @brief 私有协议下修改电机协议模式 (重新上电生效) 
 * @param Mode 0: 私有协议, 1: CANOpen模式 ,2: MIT模式
 */
void Class_RobStride_Motor::Set_Motor_Mode_Private(Enum_CAN_Mode __CAN_Mode)
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorModeSet, Master_ID);
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, __CAN_Mode, 0x00};
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}