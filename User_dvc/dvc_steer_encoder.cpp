#include "dvc_steer_encoder.h"

/**
 * @brief 初始化舵轮 CAN 编码器
 * @param hfdcan 绑定的 CAN 句柄
 * @param __FDCAN_Encoder_ID 编码器绑定的 CAN ID
 */
void Class_Swerve_Steer_Encoder::Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Encoder_ID)
{
    if (hfdcan == nullptr)
    {
        return;
    }

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
    else
    {
        FDCAN_Manage_Object = nullptr;
        return;
    }

    FDCAN_Encoder_ID = __FDCAN_Encoder_ID;

    Encoder_Flag = 0;
    Pre_Encoder_Flag = 0;
    Encoder_Status = Enum_Encoder_Status::Encoder_Status_DISABLE;

    memset(&Rx_Data, 0, sizeof(Rx_Data));
}

/**
 * @brief 底层CAN数据接收回调函数
 * @param Rx_Data CAN数据
 */
void Class_Swerve_Steer_Encoder::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Encoder_Flag += 1;
    Data_Process();
}

void Class_Swerve_Steer_Encoder::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Encoder_Flag == Pre_Encoder_Flag)
    {
        // 编码器断开连接
        Encoder_Status = Enum_Encoder_Status::Encoder_Status_DISABLE;
    }
    else
    {
        // 编码器保持连接
        Encoder_Status = Enum_Encoder_Status::Encoder_Status_ENABLE;
    }

    Pre_Encoder_Flag = Encoder_Flag;
}

void Class_Swerve_Steer_Encoder::Data_Process()
{
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    Struct_Encoder_Steer_CAN_RX_Data *tmp_buffer = (Struct_Encoder_Steer_CAN_RX_Data *)FDCAN_Manage_Object->Rx_Buffer.Data;

    // crc校验
    uint8_t calculated_crc = Algorithm::CRC_Lib::CRC8::Calculate(tmp_buffer, sizeof(Struct_Encoder_Steer_CAN_RX_Data) - sizeof(tmp_buffer->checksum) - sizeof(tmp_buffer->reserved)); // 不包括checksum和reserved

    if (calculated_crc != tmp_buffer->checksum)
    {
        return;
    }

    // 计算角度
    uint16_t raw_angle = (tmp_buffer->raw_angle_high << 8) | tmp_buffer->raw_angle_low;
    // TODO: 确认编码器实际分辨率，替换 16383 为正确值（14位=16383, 12位=4095, 16位=65535）
    raw_angle &= 0x3FFF; // 掩码到 14 位
    Rx_Data.angle = Math_Int_To_Float(raw_angle, 0, 16383, 0.0f, 360.0f);
    Rx_Data.status = tmp_buffer->status;
    Rx_Data.no_mag_warning = tmp_buffer->no_mag_warning;
    Rx_Data.over_speed = tmp_buffer->over_speed_warning;
    Rx_Data.parity_error = tmp_buffer->parity_error;
}