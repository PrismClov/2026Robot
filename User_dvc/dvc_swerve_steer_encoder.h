/**
 * @file dvc_swerve_steer_encoder.h
 * @author hzy(2370905113@qq.com)
 * @brief 舵轮编码器(MT6515，单圈360°，can通信)
 * @version 0.1
 * @date 2026-05-27 0.1 26-27赛季
 *
 * @copyright NEUQ-RoboPioneer (c) 2026-2027
 *
 */
#ifndef DVC_SWERVE_STEER_ENCODER_H
#define DVC_SWERVE_STEER_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "drv_can.h"
#include "drv_math.h"
#include "alg_crc.h"
/* Exported macros -----------------------------------------------------------*/

/**
 * @brief 舵轮编码器CAN接收数据结构体
 * 
 * @note 1. raw_angle_high和raw_angle_low组合成一个16位的原始角度值，范围为0-3600，表示0-360度。
 *       2. status表示编码器的状态，0表示正常，1表示异常
 *       3. no_mag_warning表示无磁场警告，0表示无警告，1表示有警告
 *       4. over_speed_warning表示过速警告，0表示无警告，1表示有警告
 *       5. parity_error表示奇偶校验错误，0表示无错误，1表示有错误
 *       6. checksum表示数据校验和，计算方法为前7个字节的异或值
 */
struct Struct_Encoder_Rudder_CAN_RX_Data
{
    uint8_t raw_angle_high;      // 原始角度高字节
    uint8_t raw_angle_low;       // 原始角度低字节
    uint8_t status;              // 编码器状态 (0:正常, 1:异常)
    uint8_t no_mag_warning;      // 无磁场警告 (0:无, 1:有)
    uint8_t over_speed_warning;  // 过速警告 (0:无, 1:有)
    uint8_t parity_error;        // 奇偶校验错误 (0:无, 1:有)
    uint8_t checksum;            // 数据校验和 (前7字节异或值)
    uint8_t reserved;            // 保留字节
};


/**
 * @brief 舵轮编码器数据处理结构体
 * @note 角度范围为0-360度，精度0.1度
 */
struct Struct_Encoder_Rudder_Rx_Data
{
    float angle;                // 解码后的角度值 (0-360度)
    uint8_t status;             // 编码器状态 (0:正常, 1:异常)
    uint8_t no_mag_warning;     // 无磁场警告 (0:无, 1:有)
    uint8_t over_speed;         // 过速警告 (0:无, 1:有)
    uint8_t parity_error;       // 奇偶校验错误 (0:无, 1:有)
    uint8_t checksum;           // 数据校验和验证结果
    uint8_t reserved;           // 保留字段，用于字节对齐
};

/**
* @brief  舵轮编码器存活状态枚举
*/
enum Enum_Encoder_Rudder_Status
{
    Encoder_Rudder_Status_DISABLE = 0,
    Encoder_Rudder_Status_ENABLE = 1
};

/**
 * @brief  rudder 编码器类
 */
class Class_Encoder_Rudder
{ 
    public:
    void Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Encoder_ID, float __Gear_Ratio = 1.0f);

    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    // 获取当前角度值，单位为度
    inline float Get_Angle();

    inline uint8_t Get_Status();

    inline uint8_t Get_NoMagWarning(); 

    inline uint8_t Get_OverSpeed();

    inline uint8_t Get_ParityError();

    private:
    
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;

    Enum_Encoder_Rudder_Status Encoder_Status = Encoder_Rudder_Status_DISABLE;

    uint32_t FDCAN_Encoder_ID;

    float Gear_Ratio = 1.0f; // 齿轮比，默认为1.0，即无齿轮

    uint32_t Encoder_Flag;

    uint32_t Pre_Encoder_Flag;

    Struct_Encoder_Rudder_Rx_Data Rx_Data;
    
    // 数据处理函数，负责将原始CAN数据转换为角度值和状态信息
    void Data_Process();


};

/**
 * @brief 舵轮舵向绝对编码器角度转换类
 *
 * 将上层传入的 16 bit 原始编码器值转换为舵向角度(rad)，并处理零位偏置和方向反转。
 */
class Class_Swerve_Steer_Encoder
{
public:
    void Init(float zero_offset_rad = 0.0f, bool reverse = false);

    void Update(uint16_t encoder_raw);

    inline uint16_t Get_Raw() const;

    inline float Get_Angle_Rad() const;

private:
    uint16_t Raw = 0;
    float Angle_Rad = 0.0f;
    float Zero_Offset_Rad = 0.0f;
    bool Reverse = false;

private:
    static float Normalize_Angle(float angle);
};

/**
 * @brief 获取当前角度值，单位为度
 */
inline float Class_Encoder_Rudder::Get_Angle()
{
    return Rx_Data.angle;
}

/**
 * @brief 获取编码器状态
 * @return 0:正常, 1:异常
 */
inline uint8_t Class_Encoder_Rudder::Get_Status()
{
    return Rx_Data.status;
}

/**
 * @brief 获取无磁场警告
 * @return 0:无警告, 1:有警告
 */
inline uint8_t Class_Encoder_Rudder::Get_NoMagWarning()
{
    return Rx_Data.no_mag_warning;
}

/**
 * @brief 获取过速警告
 * @return 0:无警告, 1:有警告
 */
inline uint8_t Class_Encoder_Rudder::Get_OverSpeed()
{
    return Rx_Data.over_speed;
}

/**
 * @brief 获取奇偶校验错误
 * @return 0:无错误, 1:有错误
 */
inline uint8_t Class_Encoder_Rudder::Get_ParityError()
{
    return Rx_Data.parity_error;
}

/**
 * @brief 获取编码器原始值
 */
inline uint16_t Class_Swerve_Steer_Encoder::Get_Raw() const
{
    return Raw;
}

/**
 * @brief 获取当前舵向角度，单位为 rad
 */
inline float Class_Swerve_Steer_Encoder::Get_Angle_Rad() const
{
    return Angle_Rad;
}

#endif
