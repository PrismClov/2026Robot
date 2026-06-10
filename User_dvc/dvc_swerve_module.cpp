#include "dvc_swerve_module.h"

/**
 * @brief 单个舵轮模块实现
 */
bool Class_Swerve_Module::Init(Class_Motor_Base &steer_motor, Class_Motor_Base &drive_motor, Class_Swerve_Steer_Encoder &steer_encoder, const Parameters &parameters, Mode mode)
{
    if (!Check_Parameters(parameters))
    {
        Initialized = false;
        return false;
    }

    Steer_Motor = &steer_motor;
    Drive_Motor = &drive_motor;
    Steer_Encoder = &steer_encoder;

    Param = parameters;

    Current_Mode = mode;

    Initialized = true;

    return true;
}

/**
 * @brief 校验控制参数合法性
 */
bool Class_Swerve_Module::Check_Parameters(const Parameters &parameters) const
{
    if (!isfinite(parameters.Force_To_Current) ||
        !isfinite(parameters.Speed_Deadband) ||
        !isfinite(parameters.Force_Deadband) ||
        !isfinite(parameters.Current_Deadband) ||
        !isfinite(parameters.Max_Speed_Mps) ||
        !isfinite(parameters.Max_Force_N) ||
        !isfinite(parameters.Max_Current_A) ||
        !isfinite(parameters.Steer_Zero_Offset_Rad))
    {
        return false;
    }

    /*
     * Force_To_Current 必须为正。
     * 方向反的问题不建议通过负比例系数解决。
     */
    if (parameters.Force_To_Current <= 0.0f)
    {
        return false;
    }

    if (parameters.Speed_Deadband < 0.0f ||
        parameters.Force_Deadband < 0.0f)
    {
        return false;
    }

    /*
     * Max_* <= 0 表示不启用限幅，所以这里不判定为非法。
     */

    return true;
}

/**
 * @brief 计算控制量
 */
void Class_Swerve_Module::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    /*
     * 先做舵向角度优化。
     * 可能会改变：
     * 1. Target Angle
     * 2. Target Speed
     * 3. Target Force
     */
    Optimize_Target();

    /*
     * 力控目标转换成电流目标。
     */
    Module_Target.Current_A = Module_Target.Force_N * Param.Force_To_Current;

    Module_Target.Current_A = Math_Constrain(&Module_Target.Current_A, -Param.Max_Current_A, Param.Max_Current_A);

    if (Math_Abs(Module_Target.Current_A) < Param.Current_Deadband)
    {
        Module_Target.Current_A = 0.0f;
    }

    Apply_Steer_Target();
    Apply_Drive_Target();
}

/**
 * @brief 输出舵向控制量
 */
void Class_Swerve_Module::Apply_Steer_Target()
{
    if (Steer_Motor == nullptr)
    {
        return;
    }

    /*
     * 舵向电机始终工作在位置模式。
     */
    Steer_Motor->Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);

    /*
     * 舵向位置反馈使用外置绝对编码器。
     */
    Steer_Motor->Set_Feedback_Position((Steer_Encoder->Get_Normalized_Angle() * DEG_TO_RAD) - Param.Steer_Zero_Offset_Rad);

    Steer_Motor->Set_Target_Position(Module_Target.Angle_Rad);

    Steer_Motor->Calculate();
}

/**
 * @brief 输出轮向控制量
 */
void Class_Swerve_Module::Apply_Drive_Target()
{
    if (Drive_Motor == nullptr)
    {
        return;
    }

    if (Current_Mode == Mode::Speed)
    {
        /*
         * 速度模式：
         * 这里的 Target_Speed 单位是 m/s。
         * 具体电机类需要在内部转换成 RPM / ERPM / 其他驱动器单位。
         */
        Drive_Motor->Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);

        Drive_Motor->Set_Target_Speed(Module_Target.Speed_Mps);
    }
    else
    {
        /*
         * Force 和 Current 模式：
         * 轮向电机跑电流环 / 力控。
         */
        Drive_Motor->Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);

        Drive_Motor->Set_Target_Current(Module_Target.Current_A);
    }

    Drive_Motor->Calculate();
}

/**
 * @brief 优化舵向目标，减少不必要的大幅转动
 */
void Class_Swerve_Module::Optimize_Target()
{
    Deadband_Process();

    const float current_angle = (Steer_Encoder->Get_Normalized_Angle() * DEG_TO_RAD) - Param.Steer_Zero_Offset_Rad;

    float error = Module_Target.Angle_Rad - current_angle;
    error = Math_Modulus_Normalization(error, PI);

    /*
     * 舵向优化：
     *
     * 如果目标角度和当前角度误差超过 90°，
     * 说明直接转过去太远。
     *
     * 此时：
     * 1. 舵向目标角反向 180°
     * 2. 轮向速度取反
     * 3. 轮向力取反
     */
    if (Math_Abs(error) > PI * 0.5f)
    {
        Flip_Direction();
    }
}

/**
 * @brief 死区处理
 */
void Class_Swerve_Module::Deadband_Process()
{
    // 死区处理：小于死区时认为目标为 0，避免停止或低速时舵轮乱优化。
    if (Math_Abs(Module_Target.Speed_Mps) < Param.Speed_Deadband)
    {
        Module_Target.Speed_Mps = 0.0f;
    }

    if (Math_Abs(Module_Target.Force_N) < Param.Force_Deadband)
    {
        Module_Target.Force_N = 0.0f;
    }
}

/**
 * @brief 清除轮向目标（速度、力、当前），但不清除舵向目标
 */
void Class_Swerve_Module::Clear_Drive_Target()
{
    Module_Target.Speed_Mps = 0.0f;
    Module_Target.Force_N = 0.0f;
    Module_Target.Current_A = 0.0f;
}

/**
 * @brief 舵向电机反转优化
 */
void Class_Swerve_Module::Flip_Direction()
{
    Module_Target.Angle_Rad = Math_Modulus_Normalization(Module_Target.Angle_Rad + PI, PI);

    Module_Target.Speed_Mps = -Module_Target.Speed_Mps;
    Module_Target.Force_N = -Module_Target.Force_N;
    Module_Target.Current_A = -Module_Target.Current_A;
}

/**
 * @brief 输出控制量
 */
void Class_Swerve_Module::Output()
{
    if (!Initialized)
    {
        return;
    }

    if (Steer_Motor != nullptr)
    {
        Steer_Motor->Output();
    }

    if (Drive_Motor != nullptr)
    {
        Drive_Motor->Output();
    }
}

/**
 * @brief 1ms 定时器回调
 */
void Class_Swerve_Module::TIM_1ms_PeriodElapsedCallback()
{
    Calculate();
    Output();
}

/**
 * @brief Stop 只修改目标并计算，不强制发送
 */
void Class_Swerve_Module::Stop()
{
    Clear_Drive_Target();

    if (!Initialized)
    {
        return;
    }

    if (Drive_Motor != nullptr)
    {
        Drive_Motor->Set_Control_Method(MOTOR_CONTROL_METHOD_DISABLE);
        Drive_Motor->Set_Target_Speed(0.0f);
        Drive_Motor->Set_Target_Current(0.0f);
        Drive_Motor->Calculate();
    }

    /*
     * Stop 时舵向保持当前目标角，不强制回零。
     */
    if (Steer_Motor != nullptr)
    {
        Steer_Motor->Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);

        Steer_Motor->Set_Feedback_Position((Steer_Encoder->Get_Normalized_Angle() * DEG_TO_RAD) - Param.Steer_Zero_Offset_Rad);

        Steer_Motor->Set_Target_Position(Module_Target.Angle_Rad);

        Steer_Motor->Calculate();
    }
}
