#ifndef DVC_SWERVE_STEER_ENCODER_H
#define DVC_SWERVE_STEER_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "alg_crc.h"
#include "drv_can.h"
#include "drv_math.h"
#include "dvc_encoder_base.h"
#include <math.h>
#include <stdint.h>
/* Exported macros -----------------------------------------------------------*/

/**
 * @brief 舵轮编码器 CAN 接收数据结构体
 *
 * @note 1. raw_angle_high 和 raw_angle_low 组合成一个 16 位的原始角度值，范围为 0-3600，表示 0-360 度。
 *       2. status 表示编码器的状态，0 表示正常，1 表示异常
 *       3. no_mag_warning 表示无磁场警告，0 表示无警告，1 表示有警告
 *       4. over_speed_warning 表示过速警告，0 表示无警告，1 表示有警告
 *       5. parity_error 表示奇偶校验错误，0 表示无错误，1 表示有错误
 *       6. checksum 表示数据校验和，计算方法为前 7 个字节的异或值
 */
struct Struct_Encoder_Steer_CAN_RX_Data
{
    uint8_t raw_angle_high;     // 原始角度高字节
    uint8_t raw_angle_low;      // 原始角度低字节
    uint8_t status;             // 编码器状态 (0:正常，1:异常)
    uint8_t no_mag_warning;     // 无磁场警告 (0:无，1:有)
    uint8_t over_speed_warning; // 过速警告 (0:无，1:有)
    uint8_t parity_error;       // 奇偶校验错误 (0:无，1:有)
    uint8_t checksum;           // 数据校验和 (前 7 字节异或值)
    uint8_t reserved;           // 保留字节
} __attribute__((packed));

/**
 * @brief 舵轮编码器数据处理结构体
 * @note 角度范围为 0-360 度，精度 0.1 度
 */
struct Struct_Encoder_Steer_Rx_Data
{
    float angle;            // 解码后的角度值 (0-360 度)
    uint8_t status;         // 编码器状态 (0:正常，1:异常)
    uint8_t no_mag_warning; // 无磁场警告 (0:无，1:有)
    uint8_t over_speed;     // 过速警告 (0:无，1:有)
    uint8_t parity_error;   // 奇偶校验错误 (0:无，1:有)
};

class Class_Swerve_Steer_Encoder : public Class_Encoder_Base
{
public:
    void Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Encoder_ID);

    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    inline float Get_Total_Angle() const override;
    inline float Get_Normalized_Angle() const override;
    inline int32_t Get_Total_Round() const override;
    inline float Get_Omega() const override;
    inline Enum_Encoder_Status Get_Status() const override;
    inline uint8_t Get_NoMagWarning() const;
    inline uint8_t Get_OverSpeed() const;
    inline uint8_t Get_ParityError() const;

private:
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;

    Enum_Encoder_Status Encoder_Status = Enum_Encoder_Status::Encoder_Status_DISABLE;

    uint32_t FDCAN_Encoder_ID;

    uint32_t Encoder_Flag = 0;

    uint32_t Pre_Encoder_Flag = 0;

    Struct_Encoder_Steer_Rx_Data Rx_Data;

    // 数据处理函数，负责将原始 CAN 数据转换为角度值和状态信息
    void Data_Process();
};

inline float Class_Swerve_Steer_Encoder::Get_Total_Angle() const
{
    return Rx_Data.angle;
}

inline int32_t Class_Swerve_Steer_Encoder::Get_Total_Round() const
{
    return 0; // 必然只有一圈，没有多圈接口
}

inline float Class_Swerve_Steer_Encoder::Get_Normalized_Angle() const
{
    return Math_Modulus_Normalization(Rx_Data.angle, 360.0f);
}

inline float Class_Swerve_Steer_Encoder::Get_Omega() const
{
    return 0.0f;
}

inline Class_Encoder_Base::Enum_Encoder_Status Class_Swerve_Steer_Encoder::Get_Status() const
{
    return Encoder_Status;
}

inline uint8_t Class_Swerve_Steer_Encoder::Get_NoMagWarning() const
{
    return Rx_Data.no_mag_warning;
}

inline uint8_t Class_Swerve_Steer_Encoder::Get_OverSpeed() const
{
    return Rx_Data.over_speed;
}

inline uint8_t Class_Swerve_Steer_Encoder::Get_ParityError() const
{
    return Rx_Data.parity_error;
}

#endif
