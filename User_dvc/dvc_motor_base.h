/**
 * @file dvc_motor_base.h
 * @author hzy
 * @brief 通用电机配置与操作
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright NEUQ (cpp) 2026
 */


#ifndef DVC_MOTOR_BASE_H
#define DVC_MOTOR_BASE_H

#include <stdint.h>
#include "drv_math.h"
#include "dvc_dwt.h"
/**
 * @brief 通用电机状态
 */
enum Enum_Motor_Status
{
    Motor_Status_DISABLE = 0,  // 电机离线 / 失能
    Motor_Status_ENABLE,       // 电机在线 / 使能
};

/**
 * @brief 通用电机控制模式
 *
 * 这个枚举只描述上层希望电机处于什么控制模式，
 * 具体如何转换成 CAN 指令、PWM、ERPM、电流输出，由子类/适配器负责。
 */
enum Enum_Motor_Control_Method
{
    MOTOR_CONTROL_METHOD_DISABLE = 0,  // 失能 / 零输出
    MOTOR_CONTROL_METHOD_CURRENT,      // 电流模式，单位 A
    MOTOR_CONTROL_METHOD_SPEED,        // 速度模式，单位由子类约定，舵轮工程中建议统一为 rad/s 或 m/s
    MOTOR_CONTROL_METHOD_POSITION,     // 位置模式，单位 rad
    MOTOR_CONTROL_METHOD_MIT,          // 预留 MIT 模式
    MOTOR_CONTROL_METHOD_DUTY,         // 预留占空比模式，单位 %
    MOTOR_CONTROL_METHOD_BRAKE,     // 预留刹车模式，单位 A
};

/**
 * @brief 校准运动模式
 */
enum Enum_Calibrate_Motion_Mode
{
    CALIBRATE_MOTION_NONE = 0,    // 不运动，直接以当前位置为 offset
    CALIBRATE_MOTION_SPEED,       // 恒速运动
    CALIBRATE_MOTION_CURRENT,     // 恒流运动
};

/**
 * @brief 校准堵转检测模式
 */
enum Enum_Calibrate_Detect_Mode
{
    CALIBRATE_DETECT_CURRENT = 0, // 电流超过阈值判定堵转
    CALIBRATE_DETECT_SPEED,       // 速度低于阈值判定堵转
};

/**
 * @brief 校准参数
 *
 * 用法:
 *   Calibrate_Params p;
 *   p.motion_mode = CALIBRATE_MOTION_SPEED;
 *   p.motion_value = -0.3f;
 *   p.detect_mode = CALIBRATE_DETECT_CURRENT;
 *   p.detect_threshold = 5.0f;
 *   mot.Calibrate(p, offset);
 */
struct Calibrate_Params
{
    Enum_Calibrate_Motion_Mode motion_mode = CALIBRATE_MOTION_SPEED;
    float motion_value = 0.3f;                              // 速度(rad/s)或电流(A), 正负决定方向

    Enum_Calibrate_Detect_Mode detect_mode = CALIBRATE_DETECT_CURRENT;
    float detect_threshold = 5.0f;                          // 电流阈值(A)或速度阈值(rad/s)

    uint32_t debounce_us = 0;                               // 检测消抖时间(us), 条件持续满足此时间后判定堵转; 0=立即判定
};

/**
 * @brief 通用电机抽象基类
 *
 * 设计原则：
 * 1. 基类只定义统一接口，不关心具体电机协议。
 * 2. DJI、MKSESC、VESC、达妙等具体电机通过继承或适配器实现这些接口。
 * 3. 上层模块如 Class_Swerve_Module 只依赖 Class_Motor_Base。
 *
 * 单位建议：
 * - Current: A
 * - Position: rad
 * - Speed:
 *   - 舵向电机建议 rad/s
 *   - 轮向电机在舵轮模块中建议使用 m/s，由轮向适配器内部转换成 ERPM/rad/s
 */
class Class_Motor_Base
{
public:
    virtual ~Class_Motor_Base() {}

    /**
     * @brief 设置电机控制模式
     */
    virtual void Set_Control_Method(Enum_Motor_Control_Method __Method) = 0;

    /**
     * @brief 设置目标电流，单位 A
     */
    virtual void Set_Target_Current(float __Target_Current) = 0;

    /**
     * @brief 设置目标速度
     *
     * 对于舵向电机，建议单位 rad/s。
     * 对于轮向电机，在舵轮系统中建议单位 m/s，
     * 由具体轮向电机适配器换算为电机自身单位。
     */
    virtual void Set_Target_Speed(float __Target_Speed) = 0;

    /**
     * @brief 设置目标位置，单位 rad
     */
    virtual void Set_Target_Position(float __Target_Position) = 0;

    /**
     * @brief 写入外部反馈电流，单位 A
     *
     * 某些电机反馈来自自身 CAN；某些闭环反馈来自外部传感器。
     * 例如舵向电机位置反馈常来自绝对编码器。
     */
    virtual void Set_Feedback_Current(float __Feedback_Current) = 0;

    /**
     * @brief 写入外部反馈速度（单位由子类约定）
     */
    virtual void Set_Feedback_Speed(float __Feedback_Speed) = 0;

    /**
     * @brief 写入外部反馈位置，单位 rad
     */
    virtual void Set_Feedback_Position(float __Feedback_Position) = 0;

    /**
     * @brief 获取当前反馈电流，单位 A
     */
    virtual float Get_Current() const = 0;

    /**
     * @brief 获取当前反馈速度（单位由子类约定）
     */
    virtual float Get_Speed() const = 0;

    /**
     * @brief 获取当前反馈位置，单位 rad
     */
    virtual float Get_Position() const = 0;

    /**
     * @brief 更新反馈 — 从底层读取最新数据到统一接口
     *
     * 对于 CAN 电机：从底层 CAN 帧缓存同步到 Feedback_Current/Speed/Position
     * 对于外部传感器反馈：上层已通过 Set_Feedback_* 写入，此函数可作为空操作
     */
    virtual void Update_Feedback() = 0;

    /**
     * @brief 计算控制量 — 根据当前模式和目标值计算输出
     *
     * 典型实现：
     * - CURRENT 模式：直接使用目标电流（可叠加前馈）
     * - SPEED 模式：速度 PID 输出电流目标
     * - POSITION 模式：位置外环 → 速度目标 → 速度内环 → 电流目标
     */
    virtual void Calculate() = 0;

    /**
     * @brief 输出控制量
     *
     * 例如：
     * - 写入 CAN 发送缓冲区
     * - 直接发送 CAN 指令
     * - 输出 PWM
     */
    virtual void Output() = 0;

    /**
     * @brief 电机堵转校准
     *
     * 每控制周期调用一次。支持恒速 / 恒流两种运动模式，
     * 支持电流阈值 / 速度阈值两种堵转判定方式，带 DWT 消抖。
     * 校准完成后将电机置零，通过 offset 导出机械零点绝对角度。
     *
     * @param params  校准参数 (运动/检测模式, 阈值, 消抖时间)
     * @param offset  输出，堵转时的机械零点绝对角度
     * @return true   校准完成 (堵转检测到)
     * @return false  仍在校准中
     */
    bool Calibrate(const Calibrate_Params &params, float &offset);

protected:
    uint32_t Stall_Debounce_Start_Time = 0; // 堵转消抖计时起点
};

#endif
