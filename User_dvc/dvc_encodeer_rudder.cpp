/**
 * @file dvc_encoder_rudder.cpp
 * @author hzy(2370905113@qq.com)
 * @brief 舵轮编码器(MT6515，单圈360°，can通信)
 * @version 0.1
 * @date 2026-05-27 0.1 26-27赛季
 *
 * @copyright NEUQ-RoboPioneer (c) 2026-2027
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_encoder_rudder.h"
/* Private macros ------------------------------------------------------------*/


/* Private types -------------------------------------------------------------*/
/**
 * @brief 底层CAN数据接收回调函数
 *
 * @param Rx_Data CAN数据
 * @return void
 */
void Class_Encoder_Rudder::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Encoder_Flag += 1;
    Data_Process();
}
/**
 * @brief TIM定时器中断定期检测编码器是否存活, 检测周期取决于编码器掉线时长
 *
 */
void Class_Encoder_Rudder::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过数据
    if (Encoder_Flag == Pre_Encoder_Flag)
    {
        // 编码器断开连接
        Encoder_Status = Encoder_Rudder_Status_DISABLE;
    }
    else
    {
        // 编码器保持连接
        Encoder_Status = Encoder_Rudder_Status_ENABLE;
    }

    Pre_Encoder_Flag = Encoder_Flag;
}

/**
 * @brief 编码器数据处理
 *
 * @return void
 */
void Class_Encoder_Rudder::Data_Process()
{
    Struct_Encoder_Rudder_CAN_RX_Data *tmp_buffer = (Struct_Encoder_Rudder_CAN_RX_Data *)FDCAN_Manage_Object->Rx_Buffer.Data;
    
    // crc校验待完善，暂不处理校验和字段
    
    
    // 计算角度
    uint16_t raw_angle = (tmp_buffer->raw_angle_high << 8) | tmp_buffer->raw_angle_low;
    Rx_Data.angle = Math_Int_To_Float(raw_angle, 0, 65535, 0.0f, 360.0f) / Gear_Ratio;
    
    // 复制剩余字段 (从status开始到reserved)
    memcpy(&Rx_Data.status, &tmp_buffer->status, 
           sizeof(Struct_Encoder_Rudder_Rx_Data) - offsetof(Struct_Encoder_Rudder_Rx_Data, status));

    // // 复制状态信息
    // Rx_Data.status = tmp_buffer->status;
    // Rx_Data.no_mag_warning = tmp_buffer->no_mag_warning;
    // Rx_Data.over_speed = tmp_buffer->over_speed;
    // Rx_Data.parity_error = tmp_buffer->parity_error;
    // Rx_Data.checksum = tmp_buffer->checksum;  // 保存校验和结果
    // Rx_Data.reserved = 0;
   
    // 防止角度值溢出
    if (Rx_Data.angle >= 360.0f)
    {
        Rx_Data.angle -= 360.0f;
    }
}