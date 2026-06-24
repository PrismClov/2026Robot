/**
 * @file dvc_motor_mksesc.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief mksesc电调配置与操作,由于MKSESC电调有自己的PID算法，支持多个环控制，无需在本机计算PID，但需返回角度速度等数据，目前关于数据大小端问题存在疑问，待到实验室验证一下
 * @version 0.1
 * @date 2024-10-19 0.1 24-25赛季定稿
 *
 * @copyright NEUQ-RoboPioneer (c) 2024-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_mksesc.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hfdcan 绑定的CAN总线
 * @param __FDCAN_Rx_ID 收数据绑定的CAN ID
 * @param __FDCAN_Tx_ID 发数据绑定的CAN ID
 * @param __Angle_Max 最大位置
 * @param __Omega_Max 最大速度
 * @param __Torque_Max 最大扭矩
 * @param __Motor_MKSESC_Control_Method 电机控制方法，默认为角速度控制方式。
 */
void Class_Motor_MKSESC::Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Motor_ID,uint8_t __Motor_Pole_Pairs, float __Angle_Max, float __Omega_Max, float __Duty_Max, float __Current_Max, Enum_Motor_Control_Method __Control_Method)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }

    FDCAN_Motor_ID = __FDCAN_Motor_ID;

    Pole_Pairs = __Motor_Pole_Pairs;

    Control_Method = __Control_Method;

    Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Duty_Max = __Duty_Max;
    Current_Max = __Current_Max;

    Base_Initialized = true;
}

void Class_Motor_MKSESC::Set_Base_Parameters(const Base_Parameters& parameters)
{
    if (!Is_Finite(parameters.Wheel_Radius_M) ||
        !Is_Finite(parameters.Reduction_Ratio) ||
        parameters.Wheel_Radius_M <= 0.0f ||
        parameters.Reduction_Ratio <= 0.0f)
    {
        return;
    }

    Base_Param = parameters;
}

void Class_Motor_MKSESC::Output_CAN_Data()
{
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    //设置临时变量
    uint32_t tmp_id;
    uint8_t FDCAN_Tx_Buffer[4];
    int32_t tmp_data;

    //根据不同的控制方法选择发送不同的消息，仍然存在一些疑问！
    switch(Control_Method)
    {   
			//这里的比例关系很匪夷所思，不知道该具体怎么换算，但角速度发送和返回对应上了
        //占空比控制方式
        case MOTOR_CONTROL_METHOD_DUTY:
            tmp_id = ((uint32_t) ( (FDCAN_PACKET_SET_DUTY) << 8) | FDCAN_Motor_ID);
				//单位是%
            tmp_data = (int32_t)(Control_Duty * 1e3f);
            FDCAN_Tx_Buffer[0] = tmp_data >> 24 ;
            FDCAN_Tx_Buffer[1] = tmp_data >> 16 ;
            FDCAN_Tx_Buffer[2] = tmp_data >> 8 ;
            FDCAN_Tx_Buffer[3] = tmp_data ;
            break;
        //电流控制方式
        case MOTOR_CONTROL_METHOD_CURRENT:
            tmp_id = ((uint32_t) ( (FDCAN_PACKET_SET_CURRENT) << 8) | FDCAN_Motor_ID);
				//发送给电调的电流是mA，乘上1000单位由A转化为mA
            tmp_data = (int32_t)(Control_Current * 1000.0f);	
            FDCAN_Tx_Buffer[0] = tmp_data >> 24 ;
            FDCAN_Tx_Buffer[1] = tmp_data >> 16 ;
            FDCAN_Tx_Buffer[2] = tmp_data >> 8 ;
            FDCAN_Tx_Buffer[3] = tmp_data ;
            break;
        //转速控制方式
        case MOTOR_CONTROL_METHOD_SPEED:
            tmp_id = ((uint32_t) ( (FDCAN_PACKET_SET_RPM) << 8) | FDCAN_Motor_ID);
            //VESC电调的RPM模式单位为ERPM(ERPM = 物理RPM * 极对数)
            tmp_data = (int32_t)(Control_Omega / RPM_TO_RADPS * Pole_Pairs);
            FDCAN_Tx_Buffer[0] = tmp_data >> 24 ;
            FDCAN_Tx_Buffer[1] = tmp_data >> 16 ;
            FDCAN_Tx_Buffer[2] = tmp_data >> 8 ;
            FDCAN_Tx_Buffer[3] = tmp_data ;
            break;
        //角度控制方式，输入单位是弧度，这个函数中转化为度发送给电调
        case MOTOR_CONTROL_METHOD_POSITION:
            tmp_id = ((uint32_t) ( (FDCAN_PACKET_SET_POS) << 8) | FDCAN_Motor_ID);
            tmp_data = (int32_t)(Control_Angle / DEG_TO_RAD * 1e6f);
            FDCAN_Tx_Buffer[0] = tmp_data >> 24 ;
            FDCAN_Tx_Buffer[1] = tmp_data >> 16 ;
            FDCAN_Tx_Buffer[2] = tmp_data >> 8 ;
            FDCAN_Tx_Buffer[3] = tmp_data ;
            break;
        case MOTOR_CONTROL_METHOD_BRAKE:
            tmp_id = ((uint32_t) ( (FDCAN_PACKET_SET_CURRENT_BRAKE) << 8) | FDCAN_Motor_ID);
            tmp_data = (int32_t)(Brake_Current * 1e3f);
            FDCAN_Tx_Buffer[0] = tmp_data >> 24 ;
            FDCAN_Tx_Buffer[1] = tmp_data >> 16 ;
            FDCAN_Tx_Buffer[2] = tmp_data >> 8 ;
            FDCAN_Tx_Buffer[3] = tmp_data ;
            break;
        default:
		    return;
    }
    //发送数据
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, tmp_id, FDCAN_Tx_Buffer, FDCAN_ID_Extended, 4);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_MKSESC::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_MKSESC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_Status = Motor_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_Status = Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_MKSESC::TIM_Send_PeriodElapsedCallback()
{

    if (Motor_Status == Motor_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Math_Constrain(&Control_Angle, -Angle_Max, Angle_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Duty, -Duty_Max, Duty_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);

        Output_CAN_Data();
    }
    else if (Motor_Status == Motor_Status_DISABLE)
    {
        Motor_Status = Motor_Status_ENABLE;
        return;
    }
}

void Class_Motor_MKSESC::Update_Feedback()
{
    if (!Base_Initialized)
    {
        return;
    }

    Base_Feedback_Current = Rx_Data.Now_Current;
    Base_Feedback_Speed = Motor_Radps_To_Speed_Mps(Rx_Data.Now_Omega);
    Base_Feedback_Position = Motor_Rad_To_Wheel_Rad(Rx_Data.Now_Angle);
}

void Class_Motor_MKSESC::Calculate()
{
    if (!Base_Initialized)
    {
        return;
    }

    Update_Feedback();
    Output();
}

void Class_Motor_MKSESC::Output()
{
    if (!Base_Initialized || !Base_Param.Output_Use_TIM_Send_Callback)
    {
        return;
    }

    TIM_Send_PeriodElapsedCallback();
}

float Class_Motor_MKSESC::Speed_Mps_To_Motor_Radps(float speed_mps) const
{
    return speed_mps / Base_Param.Wheel_Radius_M * Base_Param.Reduction_Ratio;
}

float Class_Motor_MKSESC::Motor_Radps_To_Speed_Mps(float motor_radps) const
{
    return motor_radps / Base_Param.Reduction_Ratio * Base_Param.Wheel_Radius_M;
}

float Class_Motor_MKSESC::Wheel_Rad_To_Motor_Rad(float wheel_rad) const
{
    return wheel_rad * Base_Param.Reduction_Ratio;
}

float Class_Motor_MKSESC::Motor_Rad_To_Wheel_Rad(float motor_rad) const
{
    return motor_rad / Base_Param.Reduction_Ratio;
}

bool Class_Motor_MKSESC::Is_Finite(float value)
{
    return isfinite(value);
}

/**
 * @brief MKSESC电调CAN回调处理函数，解析对应数据存入类
 *
 */
void Class_Motor_MKSESC::Data_Process()
{
    uint32_t tmp_id;
    
    //判断接收拓展ID低八位是否为该电机的地址，不是则返回
    if((FDCAN_Manage_Object->Rx_Buffer.Header.Identifier & 0x000000FF) != FDCAN_Motor_ID)
    {
        return ;
    }

    //储存指令内容
    tmp_id = (FDCAN_Manage_Object->Rx_Buffer.Header.Identifier >> 8);
    //根据指令执行不同动作
    switch(tmp_id)
    {
        case FDCAN_PACKET_STATUS:{		
            int32_t Now_Omega;
            int16_t Now_Current;
            int16_t Now_Duty;
        
            // 数据指针传递
            Struct_Motor_MKSESC_Rx_Data_Origin_Packet_1 *tmp_buffer = (Struct_Motor_MKSESC_Rx_Data_Origin_Packet_1 *)FDCAN_Manage_Object->Rx_Buffer.Data;
        
            //数据处理
            Math_Endian_Reverse_32(&tmp_buffer->Erpm_Reverse_Origin, &Now_Omega);
            Math_Endian_Reverse_16(&tmp_buffer->Current_Reverse_Origin, &Now_Current);
            Math_Endian_Reverse_16(&tmp_buffer->Duty_Reverse_Origin, &Now_Duty);
            Rx_Data.Now_Omega   = ((float)Now_Omega / Pole_Pairs * RPM_TO_RADPS);
            Rx_Data.Now_Current = ((float)Now_Current / 10.0f);
            Rx_Data.Now_Duty    = ((float)Now_Duty / 1000.0f);
            break;
        }
				
        case FDCAN_PACKET_STATUS_4:{
            uint16_t Now_Angle;
            // 数据指针传递
            Struct_Motor_MKSESC_Rx_Data_Origin_Packet_4 *tmp_buffer = (Struct_Motor_MKSESC_Rx_Data_Origin_Packet_4 *)FDCAN_Manage_Object->Rx_Buffer.Data;
            
            //数据处理
            Math_Endian_Reverse_16(&tmp_buffer->PID_Pos_Reverse_Origin, &Now_Angle);
            
            //Rx_Data.Now_Angle = ((float)Now_Angle) / 18000.0f * 2.0f * PI;

            Rx_Data.Now_Angle = ((float)Now_Angle) / 50.0f *DEG_TO_RAD;
            break;
        }

        default:{
                    break;
				}

    }
}

/* function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
