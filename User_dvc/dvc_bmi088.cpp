/**
 * @file dvc_bmi088.cpp
 * @author hzy
 * @brief BMI088 驱动实现（SPI）
 * @version 0.1
 * @date 2026-07-08
 *
 * @copyright NEUQ (c) 2026
 */

#include "dvc_bmi088.h"
#include "dvc_dwt.h"

/* register map */
namespace
{
constexpr uint8_t ACC_CHIP_ID = 0x00;
constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1E;

constexpr uint8_t ACCEL_XOUT_L = 0x12;
constexpr uint8_t ACC_TEMP_M = 0x22;

constexpr uint8_t ACC_CONF = 0x40;
constexpr uint8_t ACC_CONF_MUST_SET = 0x80;
constexpr uint8_t ACC_NORMAL = 0x02 << 4;
constexpr uint8_t ACC_400_HZ = 0x0A;

constexpr uint8_t ACC_RANGE = 0x41;
constexpr uint8_t ACC_RANGE_3G = 0x00;

constexpr uint8_t ACC_PWR_CONF = 0x7C;
constexpr uint8_t ACC_PWR_ACTIVE = 0x00;

constexpr uint8_t ACC_PWR_CTRL = 0x7D;
constexpr uint8_t ACC_ENABLE = 0x04;

constexpr uint8_t ACC_SOFTRESET = 0x7E;
constexpr uint8_t ACC_SOFTRESET_VALUE = 0xB6;

constexpr uint8_t INT1_IO_CTRL = 0x53;
constexpr uint8_t INT1_IO_ENABLE = 0x01 << 3;
constexpr uint8_t INT1_IO_PP = 0x00 << 2;
constexpr uint8_t INT1_IO_HIGH = 0x01 << 1;

constexpr uint8_t INT_MAP_DATA = 0x58;
constexpr uint8_t INT1_DRDY = 0x01 << 2;

/* gyro */
constexpr uint8_t GYRO_CHIP_ID = 0x00;
constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0F;

constexpr uint8_t GYRO_X_L = 0x02;

constexpr uint8_t GYRO_RANGE = 0x0F;
constexpr uint8_t GYRO_2000 = 0x00;

constexpr uint8_t GYRO_BANDWIDTH = 0x10;
constexpr uint8_t GYRO_BW_MUST_SET = 0x80;
constexpr uint8_t GYRO_2000_532_HZ = 0x00;

constexpr uint8_t GYRO_LPM1 = 0x11;
constexpr uint8_t GYRO_NORMAL = 0x00;

constexpr uint8_t GYRO_SOFTRESET = 0x14;
constexpr uint8_t GYRO_SOFTRESET_VALUE = 0xB6;

constexpr uint8_t GYRO_CTRL = 0x15;
constexpr uint8_t GYRO_DRDY_ON = 0x80;

constexpr uint8_t GYRO_INT_IO_CONF = 0x16;
constexpr uint8_t GYRO_INT3_PP = 0x00 << 1;
constexpr uint8_t GYRO_INT3_HIGH = 0x01 << 0;

constexpr uint8_t GYRO_INT_IO_MAP = 0x18;
constexpr uint8_t GYRO_DRDY_INT3 = 0x01;

constexpr float SEN_ACCEL_3G = 0.0008974358974f;
constexpr float SEN_GYRO_2000 = 0.0010652644360316953f;
constexpr float TEMP_FACTOR = 0.125f;
constexpr float TEMP_OFFSET = 23.0f;
} // namespace

void Class_BMI088::Init(SPI_HandleTypeDef *__hspi,
                        GPIO_TypeDef *__Accel_CS_Port, uint16_t __Accel_CS_Pin,
                        GPIO_TypeDef *__Gyro_CS_Port, uint16_t __Gyro_CS_Pin)
{
    SpiHandle = __hspi;
    Accel_CS_Port = __Accel_CS_Port;
    Accel_CS_Pin = __Accel_CS_Pin;
    Gyro_CS_Port = __Gyro_CS_Port;
    Gyro_CS_Pin = __Gyro_CS_Pin;

    /* ensure CS starts high (inactive) */
    HAL_GPIO_WritePin(Accel_CS_Port, Accel_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Gyro_CS_Port, Gyro_CS_Pin, GPIO_PIN_SET);

    uint8_t err = 0;
    err |= Accel_Init();
    err |= Gyro_Init();
    Error = err;
}

void Class_BMI088::SPI_Transfer(const uint8_t *__TxData, uint8_t *__RxData, uint16_t __Len, uint8_t __Is_Gyro)
{
    auto port = __Is_Gyro ? Gyro_CS_Port : Accel_CS_Port;
    auto pin = __Is_Gyro ? Gyro_CS_Pin : Accel_CS_Pin;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    if (__RxData)
    {
        HAL_SPI_TransmitReceive(SpiHandle, const_cast<uint8_t *>(__TxData), __RxData, __Len, 100);
    }
    else
    {
        HAL_SPI_Transmit(SpiHandle, const_cast<uint8_t *>(__TxData), __Len, 100);
    }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void Class_BMI088::Write_Reg(uint8_t __Reg, uint8_t __Value, uint8_t __Is_Gyro)
{
    uint8_t tx[2] = {__Reg & 0x7F, __Value};
    SPI_Transfer(tx, nullptr, 2, __Is_Gyro);
}

uint8_t Class_BMI088::Read_Reg(uint8_t __Reg, uint8_t __Is_Gyro)
{
    uint8_t tx[2] = {__Reg | 0x80, 0x00};
    uint8_t rx[2];
    SPI_Transfer(tx, rx, 2, __Is_Gyro);
    return rx[1];
}

void Class_BMI088::Read_Regs(uint8_t __Reg, uint8_t *__Buf, uint8_t __Len, uint8_t __Is_Gyro)
{
    uint8_t tx[8] = {__Reg | 0x80};
    uint8_t rx[8];
    SPI_Transfer(tx, rx, __Len + 1, __Is_Gyro);

    for (uint8_t i = 0; i < __Len; i++)
    {
        __Buf[i] = rx[i + 1];
    }
}

/* ------------------------------------------------------------------ */
/*  accel init                                                        */
/* ------------------------------------------------------------------ */

uint8_t Class_BMI088::Accel_Init()
{
    /* soft reset */
    Write_Reg(ACC_SOFTRESET, ACC_SOFTRESET_VALUE, 0);
    HAL_Delay(50);

    /* check chip ID */
    if (Read_Reg(ACC_CHIP_ID, 0) != ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* power: active mode */
    Write_Reg(ACC_PWR_CONF, ACC_PWR_ACTIVE, 0);

    /* enable accel */
    Write_Reg(ACC_PWR_CTRL, ACC_ENABLE, 0);

    /* config: normal mode, 400 Hz ODR */
    Write_Reg(ACC_CONF, ACC_CONF_MUST_SET | ACC_NORMAL | ACC_400_HZ, 0);

    /* range: ±3g */
    Write_Reg(ACC_RANGE, ACC_RANGE_3G, 0);

    /* INT1: enable, push-pull, active high */
    Write_Reg(INT1_IO_CTRL, INT1_IO_ENABLE | INT1_IO_PP | INT1_IO_HIGH, 0);

    /* map DRDY to INT1 */
    Write_Reg(INT_MAP_DATA, INT1_DRDY, 0);

    return BMI088_NO_ERROR;
}

/* ------------------------------------------------------------------ */
/*  gyro init                                                         */
/* ------------------------------------------------------------------ */

uint8_t Class_BMI088::Gyro_Init()
{
    /* soft reset */
    Write_Reg(GYRO_SOFTRESET, GYRO_SOFTRESET_VALUE, 1);
    HAL_Delay(50);

    /* check chip ID */
    if (Read_Reg(GYRO_CHIP_ID, 1) != GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* normal mode */
    Write_Reg(GYRO_LPM1, GYRO_NORMAL, 1);
    HAL_Delay(10);

    /* range: ±2000 dps */
    Write_Reg(GYRO_RANGE, GYRO_2000, 1);
    HAL_Delay(1);

    /* bandwidth: 2000 Hz ODR / 532 Hz BW */
    Write_Reg(GYRO_BANDWIDTH, GYRO_BW_MUST_SET | GYRO_2000_532_HZ, 1);
    HAL_Delay(1);

    /* enable data ready */
    Write_Reg(GYRO_CTRL, GYRO_DRDY_ON, 1);
    HAL_Delay(1);

    /* INT3: push-pull, active high */
    Write_Reg(GYRO_INT_IO_CONF, GYRO_INT3_PP | GYRO_INT3_HIGH, 1);
    HAL_Delay(1);

    /* map DRDY to INT3 */
    Write_Reg(GYRO_INT_IO_MAP, GYRO_DRDY_INT3, 1);
    HAL_Delay(1);

    return BMI088_NO_ERROR;
}

/* ------------------------------------------------------------------ */
/*  IMU base overrides                                                */
/* ------------------------------------------------------------------ */
float dt = 0.0f;
void Class_BMI088::Update()
{
    dt = DWT_GetDeltaT(&dwt_cnt);

    uint8_t buf[6];

    /* read accel (6 bytes from 0x12) */
    Read_Regs(ACCEL_XOUT_L, buf, 6, 0);
    int16_t raw_accel[3];
    raw_accel[0] = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    raw_accel[1] = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    raw_accel[2] = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));

    /* read temperature (2 bytes from 0x22, MSB first) */
    Read_Regs(ACC_TEMP_M, buf, 2, 0);
    int16_t raw_temp = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    /* read gyro (6 bytes from 0x02) */
    Read_Regs(GYRO_X_L, buf, 6, 1);
    int16_t raw_gyro[3];
    raw_gyro[0] = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    raw_gyro[1] = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    raw_gyro[2] = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));

    /* convert to physical units */
    for (int i = 0; i < 3; i++)
    {
        Accel[i] = raw_accel[i] * SEN_ACCEL_3G;
        Gyro[i] = raw_gyro[i] * SEN_GYRO_2000;
    }
    Temperature = raw_temp * TEMP_FACTOR + TEMP_OFFSET;

    IMU_QuaternionEKF_Update(Gyro[0], Gyro[1], Gyro[2], Accel[0], Accel[1], Accel[2], dt);
}

void Class_BMI088::Get_Accel(float &__Accel_X, float &__Accel_Y, float &__Accel_Z) const
{
    __Accel_X = Accel[0];
    __Accel_Y = Accel[1];
    __Accel_Z = Accel[2];
}

void Class_BMI088::Get_Gyro(float &__Gyro_X, float &__Gyro_Y, float &__Gyro_Z) const
{
    __Gyro_X = Gyro[0];
    __Gyro_Y = Gyro[1];
    __Gyro_Z = Gyro[2];
}

float Class_BMI088::Get_Temperature() const
{
    return Temperature;
}
