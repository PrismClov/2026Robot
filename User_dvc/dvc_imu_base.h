/**
 * @file dvc_imu_base.h
 * @author hzy
 * @brief 九轴IMU传感器抽象基类
 * @version 0.1
 * @date 2026-07-08
 *
 * @copyright NEUQ (c) 2026
 */

#ifndef DVC_IMU_BASE_H
#define DVC_IMU_BASE_H

/**
 * @brief 九轴IMU传感器抽象基类
 *
 * 定义统一IMU接口，具体传感器（BMI088、ISM330、MPU6500等）通过继承实现。
 * 没有磁力计或温度的传感器可将对应方法留空。
 */
class Class_IMU_Base
{
public:
    virtual ~Class_IMU_Base() {}

    /**
     * @brief 更新IMU数据（读取传感器最新值）
     */
    virtual void Update() = 0;

    /**
     * @brief 获取加速度计数据
     *
     * @param __Accel_X X轴加速度（m/s²）
     * @param __Accel_Y Y轴加速度（m/s²）
     * @param __Accel_Z Z轴加速度（m/s²）
     */
    virtual void Get_Accel(float &__Accel_X, float &__Accel_Y, float &__Accel_Z) const = 0;

    /**
     * @brief 获取陀螺仪数据
     *
     * @param __Gyro_X  X轴角速度（rad/s）
     * @param __Gyro_Y  Y轴角速度（rad/s）
     * @param __Gyro_Z  Z轴角速度（rad/s）
     */
    virtual void Get_Gyro(float &__Gyro_X, float &__Gyro_Y, float &__Gyro_Z) const = 0;

    /**
     * @brief 获取偏航角
     *
     * @return float 偏航角（rad）
     */
    virtual float Get_Yaw() const { return 0.0f; }

    /**
     * @brief 获取俯仰角
     *
     * @return float 俯仰角（rad）
     */
    virtual float Get_Pitch() const { return 0.0f; }

    /**
     * @brief 获取横滚角
     *
     * @return float 横滚角（rad）
     */
    virtual float Get_Roll() const { return 0.0f; }

    /**
     * @brief 获取磁力计数据
     *
     * 无磁力计的传感器可将此方法留空。
     *
     * @param __Mag_X  X轴磁场（μT）
     * @param __Mag_Y  Y轴磁场（μT）
     * @param __Mag_Z  Z轴磁场（μT）
     */
    virtual void Get_Mag(float &__Mag_X, float &__Mag_Y, float &__Mag_Z) const {}

    /**
     * @brief 获取温度
     *
     * 无温度测量的传感器可将此方法留空。
     *
     * @return float 温度（°C）
     */
    virtual float Get_Temperature() const { return 0.0f; }
};

#endif
