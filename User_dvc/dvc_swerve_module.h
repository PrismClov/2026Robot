#ifndef DVC_SWERVE_MODULE_H
#define DVC_SWERVE_MODULE_H

#include <math.h>
#include <stdint.h>

#include "dvc_motor_base.h"
#include "dvc_swerve_steer_encoder.h"

/**
 * @brief 单个舵轮模块
 *
 * 职责：
 * 1. 接收轮组下发的目标舵角、目标轮速、目标牵引力
 * 2. 根据当前绝对编码器角度做舵向优化
 * 3. 控制舵向电机进入位置模式
 * 4. 控制轮向电机进入速度模式或电流模式
 *
 * 注意：
 * 这个类不负责底盘运动学解算，也不直接发送 CAN。
 *
 * 初始化方式：
 * 1. 嵌入式推荐：默认构造 + Init()
 * 2. 局部/测试可选：构造函数直接初始化
 */
class Class_Swerve_Module
{
public:
    enum class Mode : uint8_t
    {
        Speed = 0, // 轮向速控
        Force,     // 轮向力控
        Current    // 轮向电流控制
    };

    struct Parameters
    {
        Parameters() = default;

        Parameters(
            float force_to_current,
            float speed_deadband,
            float force_deadband,
            float current_deadband,
            float max_speed_mps,
            float max_force_n,
            float max_current_a,
            float steer_zero_offset_rad,
            bool steer_encoder_reverse)
            : Force_To_Current(force_to_current),
              Speed_Deadband(speed_deadband),
              Force_Deadband(force_deadband),
              Current_Deadband(current_deadband),
              Max_Speed_Mps(max_speed_mps),
              Max_Force_N(max_force_n),
              Max_Current_A(max_current_a),
              Steer_Zero_Offset_Rad(steer_zero_offset_rad),
              Steer_Encoder_Reverse(steer_encoder_reverse)
        {
        }

        /*
         * 牵引力 N -> 电机电流 A 的简化比例。
         *
         * current = force * Force_To_Current
         *
         * 更严谨的公式：
         * current = force * wheel_radius / reduction_ratio / kt
         *
         * 注意：
         * Force_To_Current 必须大于 0。
         * 方向反的问题应通过电机方向、VESC 方向或轮组 Drive_Direction 处理。
         */
        float Force_To_Current = 0.2f;

        /*
         * 小于死区时认为目标为 0，避免停止或低速时舵轮乱优化。
         */
        float Speed_Deadband = 0.03f;  // m/s
        float Force_Deadband = 0.5f;   // N
        float Current_Deadband = 0.0f; // A

        /*
         * 目标限幅。
         * <= 0 表示不启用对应限幅。
         */
        float Max_Speed_Mps = 0.0f; // m/s
        float Max_Force_N = 0.0f;   // N
        float Max_Current_A = 0.0f; // A

        /*
         * 舵向绝对编码器参数。
         */
        float Steer_Zero_Offset_Rad = 0.0f;
        bool Steer_Encoder_Reverse = false;
    };

    struct Target
    {
        float Angle_Rad = 0.0f; // 舵向目标角，rad
        float Speed_Mps = 0.0f; // 轮向目标速度，m/s
        float Force_N = 0.0f;   // 轮向目标牵引力，N
        float Current_A = 0.0f; // 轮向目标电流，A
    };

    /*
     * 默认构造：
     * 适合全局对象、静态对象、数组对象。
     * 使用前必须调用 Init()。
     */
    Class_Swerve_Module() = default;

    /*
     * 直接构造：
     * 适合局部对象、测试对象或依赖初始化顺序明确的场景。
     */
    explicit Class_Swerve_Module(Class_Motor_Base &steer_motor, Class_Motor_Base &drive_motor, const Parameters &parameters, Mode mode = Mode::Speed)
    {
        Init(steer_motor, drive_motor, parameters, mode);
    }

    /*
     * 初始化函数：
     * 嵌入式主路径。建议在外设、电机、编码器相关对象完成初始化后调用。
     *
     * 返回值：
     * true  初始化成功
     * false 参数非法，初始化失败
     */
    bool Init(Class_Motor_Base &steer_motor, Class_Motor_Base &drive_motor, const Parameters &parameters, Mode mode = Mode::Speed);

    inline bool Is_Initialized() const;

    void Update_Encoder(uint16_t encoder_raw);

    // Set
    inline bool Set_Mode(Mode mode);
    inline bool Set_Target_Speed(float speed_mps);
    inline bool Set_Target_Force(float force_n);
    inline bool Set_Target_Angle(float angle_rad);

    // Get
    inline Mode Get_Mode() const;
    inline float Get_Current_Angle() const;
    inline float Get_Target_Angle() const;
    inline float Get_Target_Speed() const;
    inline float Get_Target_Force() const;
    inline float Get_Target_Current() const;

    const inline Target &Get_Target() const;

    void Calculate();

    void Output();

    void TIM_1ms_PeriodElapsedCallback();

    void Stop();

private:
    Class_Motor_Base *Steer_Motor = nullptr;
    Class_Motor_Base *Drive_Motor = nullptr;
    Class_Swerve_Steer_Encoder Steer_Encoder = {};

    Parameters Param = {};
    Target Module_Target = {};

    Mode Current_Mode = Mode::Speed;

    bool Initialized = false;

    bool Check_Parameters(const Parameters &parameters) const;

    void Optimize_Target();

    void Deadband_Process();

    void Clear_Drive_Target();

    void Flip_Direction();

    void Apply_Steer_Target();

    void Apply_Drive_Target();
};

/**
 * @brief 检查是否已初始化
 */
inline bool Class_Swerve_Module::Is_Initialized() const
{
    return Initialized;
}

/**
 * @brief 设置模式
 */
inline bool Class_Swerve_Module::Set_Mode(Mode mode)
{
    if (mode != Mode::Speed && mode != Mode::Force && mode != Mode::Current)
    {
        Clear_Drive_Target();
        return false;
    }

    Current_Mode = mode;

    /*
     * 模式切换时清零目标，避免突变。
     * 例如从速度模式切到力控模式，之前的速度目标不再适用，应该清零。
     * 反之亦然。
     */
    Clear_Drive_Target();

    return true;
}

/**
 * @brief 设置目标速度
 */
inline bool Class_Swerve_Module::Set_Target_Speed(float speed_mps)
{
    if (!isfinite(speed_mps))
    {
        Clear_Drive_Target();
        return false;
    }
    Module_Target.Speed_Mps = Math_Constrain(&speed_mps, Param.Max_Speed_Mps, -Param.Max_Speed_Mps);
    return true;
}

/**
 * @brief 设置目标牵引力
 */
inline bool Class_Swerve_Module::Set_Target_Force(float force_n)
{
    if (!isfinite(force_n))
    {
        Clear_Drive_Target();
        return false;
    }
    Module_Target.Force_N = Math_Constrain(&force_n, Param.Max_Force_N, -Param.Max_Force_N);
    return true;
}

/**
 * @brief 设置目标舵角
 */
inline bool Class_Swerve_Module::Set_Target_Angle(float angle_rad)
{
    if (!isfinite(angle_rad))
    {
        Clear_Drive_Target();
        return false;
    }
    Module_Target.Angle_Rad = Math_Modulus_Normalization(angle_rad, PI);
    return true;
}

/**
 * @brief 获取当前模式
 */
inline Class_Swerve_Module::Mode Class_Swerve_Module::Get_Mode() const
{
    return Current_Mode;
}

/**
 * @brief 获取当前舵角
 */
inline float Class_Swerve_Module::Get_Current_Angle() const
{
    return Steer_Encoder.Get_Angle_Rad();
}

/**
 * @brief 获取目标舵角
 */
inline float Class_Swerve_Module::Get_Target_Angle() const
{
    return Module_Target.Angle_Rad;
}

/**
 * @brief 获取目标速度
 */
inline float Class_Swerve_Module::Get_Target_Speed() const
{
    return Module_Target.Speed_Mps;
}

/**
 * @brief 获取目标牵引力
 */
inline float Class_Swerve_Module::Get_Target_Force() const
{
    return Module_Target.Force_N;
}

/**
 * @brief 获取目标电流
 */
inline float Class_Swerve_Module::Get_Target_Current() const
{
    return Module_Target.Current_A;
}

/**
 * @brief 获取当前目标结构体
 */
const inline Class_Swerve_Module::Target &Class_Swerve_Module::Get_Target() const
{
    return Module_Target;
}

#endif
