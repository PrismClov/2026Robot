/**
 * @file dvc_swerve_steer_encoder.cpp
 * @author hzy(2370905113@qq.com)
 * @brief 舵轮编码器(MT6515，单圈360°，can通信)
 * @version 0.1
 * @date 2026-05-27 0.1 26-27赛季
 *
 * @copyright NEUQ-RoboPioneer (c) 2026-2027
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_swerve_steer_encoder.h"
#include <stddef.h>
#include <string.h>
/* Private macros ------------------------------------------------------------*/

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kEncoderRawMax = 65535.0f;
} // namespace

/* Private types -------------------------------------------------------------*/
/**
 * @brief 初始化舵轮 CAN 编码器
 *
 * @param hfdcan 绑定的 CAN 句柄
 * @param __FDCAN_Encoder_ID 编码器绑定的 CAN ID
 * @param __Gear_Ratio 编码器到输出端的齿轮比
 */
void Class_Encoder_Rudder::Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Encoder_ID, float __Gear_Ratio)
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
    Gear_Ratio = (__Gear_Ratio > 0.0f) ? __Gear_Ratio : 1.0f;

    Encoder_Flag = 0;
    Pre_Encoder_Flag = 0;
    Encoder_Status = Encoder_Rudder_Status_DISABLE;

    memset(&Rx_Data, 0, sizeof(Rx_Data));
}

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
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    Struct_Encoder_Rudder_CAN_RX_Data *tmp_buffer = (Struct_Encoder_Rudder_CAN_RX_Data *)FDCAN_Manage_Object->Rx_Buffer.Data;

    // crc校验
    uint8_t calculated_crc = Algorithm::CRC_Lib::CRC8::compute(tmp_buffer, sizeof(Struct_Encoder_Rudder_CAN_RX_Data) - sizeof(tmp_buffer->checksum) - sizeof(tmp_buffer->reserved)); // 不包括checksum和reserved

    if (calculated_crc != tmp_buffer->checksum)
    {
        return;
    }

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

void Class_Swerve_Steer_Encoder::Init(float zero_offset_rad, bool reverse)
{
    Zero_Offset_Rad = Normalize_Angle(zero_offset_rad);
    Reverse = reverse;

    Raw = 0;
    Angle_Rad = Normalize_Angle(-Zero_Offset_Rad);
}

void Class_Swerve_Steer_Encoder::Update(uint16_t encoder_raw)
{
    Raw = encoder_raw;

    float angle = static_cast<float>(encoder_raw) / kEncoderRawMax * kTwoPi;

    if (Reverse)
    {
        angle = -angle;
    }

    Angle_Rad = Normalize_Angle(angle - Zero_Offset_Rad);
}

float Class_Swerve_Steer_Encoder::Normalize_Angle(float angle)
{
    angle = fmodf(angle, kTwoPi);

    if (angle > kPi)
    {
        angle -= kTwoPi;
    }
    else if (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}
