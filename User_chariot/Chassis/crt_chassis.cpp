/**
 * @file crt_chassis.cpp
 * @author hzy by Lucy (2478427315@qq.com)
 * @brief 舵轮底盘电控
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Robopioneer (c) 2025-2026
 *
 */

/**
 * @brief 轮组编号
 * 1[0] 4[3]
 * 2[1] 3[2]
 * 前x右y上z
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

// #define CHASSIS_SLOPE_ENABLE

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 */
void Class_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    // 斜坡初始化
    Slope_Velocity_X.Init(0.1f, 0.1f, Slope_First_REAL);

    Slope_Velocity_Y.Init(0.1f, 0.1f, Slope_First_REAL);

    Slope_Omega.Init(0.1f, 0.1f, Slope_First_REAL);

    // PID初始化

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(0.5f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(0.5f, 0.0f, 0.0f, 0.0f, 0.18f, 30.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(0.5f, 0.0f, 0.0f, 0.0f, 0.01f, 10.0f, 0.002f);

    // 舵向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Steer[i].Init(&hfdcan3, static_cast<Motor::Enum_Motor_DJI_ID>(0x201 + i),
                            Motor::Class_Motor_DJI_C610::Parameters{
                                .PID_Position = PID_Parameters{
                                    .K_P = 10.0f,
                                    .K_I = 0.0f,
                                    .K_D = 0.0f,
                                    .K_F = 0.0f,
                                    .Out_Max = 10.0f,
                                    .Dead_Zone = 0.5f,
                                },
                                .PID_Omega = PID_Parameters{
                                    .K_P = 10.0f,
                                    .K_I = 0.0f,
                                    .K_D = 0.0f,
                                    .K_F = 0.0f,
                                    .Out_Max = 5.0f,
                                    .Dead_Zone = 0.3f,
                                },
                                .Use_External_Position_Feedback = true,
                            },
                            108.0f, 10.0f);
    }
    // 舵轮编码器
    for (uint8_t i = 0; i < 4; i++)
    {
        Steer_Encoder[i].Init(&hfdcan3, 0x101 + i, steer_offset_deg[i]);
    }

    // 轮向电机
    for (uint8_t i = 0; i < 4; i++)
    {
        uint32_t index = 0x01;
        Motor_Wheel[i].Init(&hfdcan2, index + i, 14, 6.2831f, 50.0f, 50.0f, 30.0f, MOTOR_CONTROL_METHOD_CURRENT);
    }
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}
/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    // 斜坡函数
#ifdef CHASSIS_SLOPE_ENABLE
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.Set_Now_Real(Now_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.Set_Now_Real(Now_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.Set_Now_Real(Now_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
#endif

    // 自身解算
    Self_Resolution();

    // 运动学逆解算，解算出转向电机的角速度和舵向电机的角度
    Kinematics_Inverse_Resolution();

    Output_To_Dynamics();

    Dynamics_Inverse_Resolution();

    Output_To_Motor();
}

/**
 * @brief 自身解算
 *
 */
void Class_Chassis::Self_Resolution()
{
    // 使用临时变量计算新速度
    float tmp_velocity_x = 0.0f;
    float tmp_velocity_y = 0.0f;
    float tmp_omega = 0.0f;
    float tmp_motor_wheel_omega[4] = {0.0f};
    for (int i = 0; i < 4; i++)
    {
        tmp_motor_wheel_omega[i] = Motor_Wheel[i].Get_Now_Omega();
    }
    // tmp_motor_wheel_omega[1] = - tmp_motor_wheel_omega[1];
    // tmp_motor_wheel_omega[2] = - tmp_motor_wheel_omega[2];
    for (int i = 0; i < 4; i++)
    {
        // 待验证是否正确
        tmp_velocity_x += (tmp_motor_wheel_omega[i] / Wheel_Motor_Reduction * arm_cos_f32(Now_Steer_Angle[i]) * Wheel_Radius) / 4.0f;
        tmp_velocity_y += (tmp_motor_wheel_omega[i] / Wheel_Motor_Reduction * arm_sin_f32(Now_Steer_Angle[i]) * Wheel_Radius) / 4.0f;
        tmp_omega += (tmp_motor_wheel_omega[i] / Wheel_Motor_Reduction * arm_sin_f32(Now_Steer_Angle[i] - Steer_Azimuth[i]) * Wheel_Radius / Wheel_To_Core_Distance[i]) / 4.0f;
    }

    // 更新类成员变量（可在此处加入滤波）
    Now_Velocity_X = tmp_velocity_x;
    Now_Velocity_Y = tmp_velocity_y;
    Now_Omega = tmp_omega;

    Steer_Angle_Self_Resolution();
}

/**
 * @brief 获取舵向电机角度
 *
 */
void Class_Chassis::Steer_Angle_Self_Resolution()
{
    for (int i = 0; i < 4; i++)
    {
        float tmp_angle;

        // 计算角度
        tmp_angle = Steer_Encoder[i].Get_Total_Angle() * DEG_TO_RAD;

        Now_Steer_Angle[i] = tmp_angle;

        // Now_Steer_Angle[i] = Math_Modulus_Normalization(Now_Steer_Angle[i], 2.0f * PI);
    }
}

/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    for (int i = 0; i < 4; i++)
    {
        float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

        // 解算到每个轮组的具体线速度
#ifdef CHASSIS_SLOPE_ENABLE
        tmp_velocity_x = Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i]);
        tmp_velocity_y = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * Wheel_To_Core_Distance[i] * arm_cos_f32(Steer_Azimuth[i]);
#else
        tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i]);
        tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * arm_cos_f32(Steer_Azimuth[i]);
#endif
        arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

        // 根据线速度决定轮向电机角速度
        Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;

        // 根据速度的xy分量分别决定舵向电机角度
        if (tmp_velocity_modulus == 0.0f)
        {
            // 排除除零问题
            Target_Steer_Angle[i] = Now_Steer_Angle[i];
        }
        else
        {
            // 没有除零问题
            Target_Steer_Angle[i] = atan2f(tmp_velocity_y, tmp_velocity_x);
        }
    }

    _Steer_Motor_Kinematics_Nearest_Transposition();
}

/**
 * @brief 舵向电机依照轮向电机目标角速度就近转位
 *
 */
void Class_Chassis::_Steer_Motor_Kinematics_Nearest_Transposition()
{
    for (int i = 0; i < 4; i++)
    {
        float tmp_delta_angle = Target_Steer_Angle[i] - Now_Steer_Angle[i];
        tmp_delta_angle = fmod(tmp_delta_angle, 2.0f * PI);
        tmp_delta_angle = Math_Modulus_Normalization(tmp_delta_angle, 2.0f * PI);

        // 根据转动角度范围决定是否需要就近转位
        if (-PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= PI / 2.0f)
        {
            // ±PI / 2之间无需反向就近转位
            Target_Steer_Angle[i] = tmp_delta_angle + Now_Steer_Angle[i];
        }
        else
        {
            // 需要反转扣圈情况
            Target_Steer_Angle[i] = Math_Modulus_Normalization(tmp_delta_angle + PI, 2.0f * PI) + Now_Steer_Angle[i];
            Target_Wheel_Omega[i] *= -1.0f;
        }
    }
}

/**
 * @brief 输出到动力学状态
 *
 */
void Class_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
            // 未标定状态和失能状态下不进行控制
        case (Chassis_Control_Type_DISABLE):
        {
            // 底盘失能
            for (int i = 0; i < 4; i++)
            {
                PID_Velocity_X.Set_Integral_Error(0.0f);
                PID_Velocity_Y.Set_Integral_Error(0.0f);
                PID_Omega.Set_Integral_Error(0.0f);
            }

            break;
        }
        case (Chassis_Control_Type_NORMAL):
        {

#ifdef CHASSIS_SLOPE_ENABLE
            PID_Velocity_X.Set_Target(Slope_Velocity_X.Get_Out());
            PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
            PID_Omega.Set_Target(Slope_Omega.Get_Out());
#else
            PID_Velocity_X.Set_Target(Target_Velocity_X);
            PID_Velocity_Y.Set_Target(Target_Velocity_Y);
            PID_Omega.Set_Target(Target_Omega);
#endif
            PID_Velocity_X.Set_Now(Now_Velocity_X);
            PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

            PID_Velocity_Y.Set_Now(Now_Velocity_Y);
            PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

            PID_Omega.Set_Now(Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            break;
        }
    }
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega;

    force_x = PID_Velocity_X.Get_Out();
    force_y = PID_Velocity_Y.Get_Out();
    torque_omega = PID_Omega.Get_Out();

    // 每个轮的扭力
    float tmp_force[4];
    for (int i = 0; i < 4; i++)
    {
        // 解算到每个轮组的具体摩擦力
        tmp_force[i] = force_x * arm_cos_f32(Now_Steer_Angle[i]) + force_y * arm_sin_f32(Now_Steer_Angle[i]) - torque_omega / Wheel_To_Core_Distance[i] * arm_sin_f32(Steer_Azimuth[i] - Now_Steer_Angle[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        // 摩擦力转换至扭矩
        Target_Wheel_Current[i] = tmp_force[i] * Wheel_Radius + Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega());

        //            // 普通控制模式，应用原有的静摩擦和动摩擦前馈
        //            if (Target_Wheel_Omega[i] < 40.0f && Target_Wheel_Omega[i] > 3.0f)
        //            {
        //                Target_Wheel_Current[i] += Static_Resistance_Wheel_Current[i];
        //            }
        //            else if (Target_Wheel_Omega[i] > -40.0f && Target_Wheel_Omega[i] < -3.0f)
        //            {
        //                Target_Wheel_Current[i] -= Static_Resistance_Wheel_Current[i];
        //            }

        // 动摩擦阻力前馈
        if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] += Dynamic_Resistance_Wheel_Current[i];
        }
        else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] -= Dynamic_Resistance_Wheel_Current[i];
        }
        else
        {
            Target_Wheel_Current[i] += Motor_Wheel[i].Get_Now_Omega() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
        }

        // 低电流前馈控制模式
        if (Math_Abs(Target_Wheel_Current[i]) < Low_Current_Deadzone)
            Target_Wheel_Current[i] = 0.0f;

        else if (Math_Abs(Target_Wheel_Current[i]) < Low_Current_Threshold)
        {
            // 如果电流小于阈值，添加前馈
            if (Target_Wheel_Current[i] > 0)
            {
                Target_Wheel_Current[i] += Low_Current_Feedforward[i];
            }
            else if (Target_Wheel_Current[i] < 0)
            {
                Target_Wheel_Current[i] -= Low_Current_Feedforward[i];
            }
        }
    }

    // 根据斜坡与压力进行电流限幅防止贴地打滑
    // TODO
}

/**
 * @brief 输出到电机
 *
 */
float omega_calibrate = 0.0f;
void Class_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_UNCALIBRATED):
        {
            for (int i = 0; i < 4; i++)
            {
                // 对舵向电机单独校准
                if (!Steer_Calibration_Status[i])
                {
                    Motor_Steer[i].Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
                    Motor_Steer[i].Set_Target_Speed(7.5f);
                }
                else
                {
                    Motor_Steer[i].Set_Target_Speed(0.0f);
                }
            }

            break;
        }
        case (Chassis_Control_Type_DISABLE):
        {
            // 底盘失能
            for (int i = 0; i < 4; i++)
            {
                Motor_Steer[i].Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);
                Motor_Wheel[i].Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);

                Motor_Steer[i].PID_Position.Set_Integral_Error(0.0f);
                Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);

                Motor_Steer[i].Set_Target_Current(0.0f);
                Motor_Wheel[i].Set_Control_Current(0.0f);
            }

            break;
        }
        case (Chassis_Control_Type_NORMAL):
        {
            // 舵轮模型
            for (int i = 0; i < 4; i++)
            {
                Motor_Steer[i].Set_Control_Method(MOTOR_CONTROL_METHOD_POSITION);
                Motor_Wheel[i].Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);
            }
            // Target_Wheel_Current[1] = - Target_Wheel_Current[1];
            // Target_Wheel_Current[2] = - Target_Wheel_Current[2];
            for (int i = 0; i < 4; i++)
            {

                Motor_Steer[i].Set_Target_Position(Target_Steer_Angle[i]);

                Motor_Steer[i].Set_Feedback_Position(Steer_Encoder[i].Get_Total_Angle() * DEG_TO_RAD);

                if (Math_Abs(Target_Wheel_Current[i]) >= Wheel_Current_Limit)
                {
                    Motor_Wheel[i].Set_Control_Current(Target_Wheel_Current[i]);
                }
                else
                {
                    Motor_Wheel[i].Set_Control_Current(0.0f);
                }
            }

            break;
        }
    }

    // 舵向电机数据发送
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer[i].Calculate();
    }
    //  Motor::DJI_TIM_Send_Group(&hfdcan3, Motor::CAN_Tx_ID_Both);

    // 轮向电机数据发送
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Calculate();
    }
}
/************************ COPYRIGHT(C) ROBOPIONEER **************************/
