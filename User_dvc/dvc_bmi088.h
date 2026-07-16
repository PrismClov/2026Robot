/**
 * @file dvc_bmi088.h
 * @author hzy
 * @brief BMI088 驱动（SPI）
 * @version 0.1
 * @date 2026-07-08
 *
 * @copyright NEUQ (c) 2026
 */

#ifndef DVC_BMI088_H
#define DVC_BMI088_H

#include "QuaternionEKF.h"
#include "dvc_imu_base.h"
#include "spi.h"
#include <stdint.h>

class Class_BMI088 : public Class_IMU_Base
{
public:
    enum Enum_Error
    {
        BMI088_NO_ERROR = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR = 0x01,
        BMI088_ACC_PWR_CONF_ERROR = 0x02,
        BMI088_ACC_CONF_ERROR = 0x03,
        BMI088_ACC_SELF_TEST_ERROR = 0x04,
        BMI088_ACC_RANGE_ERROR = 0x05,
        BMI088_INT1_IO_CTRL_ERROR = 0x06,
        BMI088_INT_MAP_DATA_ERROR = 0x07,
        BMI088_GYRO_RANGE_ERROR = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
        BMI088_GYRO_LPM1_ERROR = 0x0A,
        BMI088_GYRO_CTRL_ERROR = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
        BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR = 0x40,
        BMI088_NO_SENSOR = 0xFF,
    };

    void Init(SPI_HandleTypeDef *__hspi,
              GPIO_TypeDef *__Accel_CS_Port, uint16_t __Accel_CS_Pin,
              GPIO_TypeDef *__Gyro_CS_Port, uint16_t __Gyro_CS_Pin);

    void Update() override;

    void Get_Accel(float &__Accel_X, float &__Accel_Y, float &__Accel_Z) const override;

    void Get_Gyro(float &__Gyro_X, float &__Gyro_Y, float &__Gyro_Z) const override;

    float Get_Temperature() const override;

    float Get_Roll() const override { return QEKF_INS.Roll; }
    float Get_Pitch() const override { return QEKF_INS.Pitch; }
    float Get_Yaw() const override { return QEKF_INS.Yaw; }

    uint8_t Get_Error() const { return Error; }

private:
    SPI_HandleTypeDef *SpiHandle = nullptr;
    GPIO_TypeDef *Accel_CS_Port = nullptr;
    uint16_t Accel_CS_Pin = 0;
    GPIO_TypeDef *Gyro_CS_Port = nullptr;
    uint16_t Gyro_CS_Pin = 0;

    uint8_t Error = BMI088_NO_ERROR;
    float Accel[3] = {0.0f};
    float Gyro[3] = {0.0f};
    float Temperature = 0.0f;

    uint32_t dwt_cnt = 0; // DWT计数器，用于计算时间增量

    void SPI_Transfer(const uint8_t *__TxData, uint8_t *__RxData, uint16_t __Len, uint8_t __Is_Gyro);

    uint8_t Accel_Init();
    uint8_t Gyro_Init();

    void Write_Reg(uint8_t __Reg, uint8_t __Value, uint8_t __Is_Gyro);
    uint8_t Read_Reg(uint8_t __Reg, uint8_t __Is_Gyro);
    void Read_Regs(uint8_t __Reg, uint8_t *__Buf, uint8_t __Len, uint8_t __Is_Gyro);
};

#endif
