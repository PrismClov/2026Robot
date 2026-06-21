// #ifndef CRT_LIFT_H
// #define CRT_LIFT_H

// #include "alg_pid.h"
// #include "dvc_motor_base.h"
// #include <array>

// template <uint8_t motor_num>
// class Class_Lift
// {
//     /**
//      * @brief 控制类型
//      */
//     enum class Enum_Lift_Control_Type
//     {
//         Lift_Control_Type_DISABLE = 0,
//         Lift_Control_Type_MOVE
//     };

//     enum class Enum_Lift_Calibrate_Method
//     {
//         Lift_Calibrate_Method_DISABLE = 0, // 不校准
//         Lift_Calibrate_Method_Speed,       // 以恒定速度运动直到堵转
//         Lift_Calibrate_Method_Current    // 以恒定电流运动直到堵转
//     };

//     struct Parameters
//     {
//         PID_Parameters PID_Distance[motor_num];    // 路程环PID参数
//         bool Need_Calibrate = false;               // 是否需要堵转校准
//         float Calibrate_Speed = 0.3f;              // 堵转校准速度, m/s (正负决定方向)
//         float Calibrate_Threshold_Current = 5.0f;  // 堵转电流阈值, A
//         float Distance_Approach_Threshold = 0.01f; // 距离接近目标的阈值, m
//         float Max_Velocity = 5.0f;                 // 速度开环模式下的最大速度, m/s
//         float Angle_To_Distance = 0.0f;            // 电机 rad → 同步带 m 转换系数
//     };

// public:
//     Enum_Lift_Control_Type Lift_Control_Type = Enum_Lift_Control_Type::Lift_Control_Type_DISABLE;

//     Class_PID Distance_PID[motor_num];
//     Class_Motor_Base *Motor_Lift[motor_num] = {};

//     void Init(std::array<Class_Motor_Base *, motor_num> motors, const Parameters &parameters);

//     void Calibrate();

//     void Set_Target_Position(float target_position);

//     float Get_Now_Distance(uint8_t i) const;

//     float Get_Target_Position() const;

//     bool Get_Is_Calibrated() const;

//     void TIM_Control_PeriodElapsedCallback();

// private:
//     Parameters Param = {};

//     float Now_Distance[motor_num] = {0.0f};

//     float Offset[motor_num] = {0.0f};

//     float Target_Position = 0.0f;

//     bool Is_Calibrated = false;
// };

// /**
//  * @brief 初始化
//  */
// template <uint8_t motor_num>
// void Class_Lift<motor_num>::Init(std::array<Class_Motor_Base *, motor_num> motors, const Parameters &parameters)
// {
//     Param = parameters;
//     for (uint8_t i = 0; i < motor_num; i++)
//     {
//         Motor_Lift[i] = motors[i];
//         const auto &p = parameters.PID_Distance[i];
//         Distance_PID[i].Init(
//             p.K_P, p.K_I, p.K_D, p.K_F,
//             p.I_Out_Max, p.Out_Max, p.D_T, p.Dead_Zone,
//             p.I_Variable_Speed_A, p.I_Variable_Speed_B,
//             p.I_Separate_Threshold, p.D_First);
//     }
// }

// /**
//  * @brief 堵转校准
//  */
// template <uint8_t motor_num>
// void Class_Lift<motor_num>::Calibrate()
// {
//     bool all_done = true;
//     for (uint8_t i = 0; i < motor_num; i++)
//     {
//         if (!Calibrate)
//         {
//             float offset;
//             if (Motor_Lift[i]->Calibrate(Param.Calibrate_Speed, Param.Calibrate_Threshold_Current, offset))
//             {
//                 Offset[i] = offset;
//                 Motor_Lift[i]->Set_Target_Speed(0.0f);
//                 Calibrate_Done[i] = true;
//             }
//             else
//             {
//                 all_done = false;
//             }
//         }
//     }

//     if (all_done)
//     {
//         Is_Calibrated = true;

//         // 校准完成后, 将当前位置记为零点
//         for (uint8_t i = 0; i < motor_num; i++)
//         {
//             Motor_Lift[i]->Update_Feedback();
//             float raw_pos = Motor_Lift[i]->Get_Position();
//             Now_Distance[i] = (raw_pos - Offset[i]) * Param.Angle_To_Distance;
//         }
//         Lift_Control_Type = Enum_Lift_Control_Type::Lift_Control_Type_MOVE;
//     }
// }

// /**
//  * @brief 设置目标位置，单位 m
//  */
// template <uint8_t motor_num>
// void Class_Lift<motor_num>::Set_Target_Position(float target_position)
// {
//     Target_Position = target_position;
// }

// /**
//  * @brief 定时器周期回调函数
//  */
// template <uint8_t motor_num>
// void Class_Lift<motor_num>::TIM_Control_PeriodElapsedCallback()
// {
//     if (Lift_Control_Type != Enum_Lift_Control_Type::Lift_Control_Type_MOVE)
//     {
//         return;
//     }

//     // 更新各电机当前距离
//     for (uint8_t i = 0; i < motor_num; i++)
//     {
//         float raw_pos = Motor_Lift[i]->Get_Position();
//         Now_Distance[i] = (raw_pos - Offset[i]) * Param.Angle_To_Distance;
//         Distance_PID[i].Set_Now(Now_Distance[i]);
//     }

//     // 大误差速度开环, 小误差位置闭环
//     for (uint8_t i = 0; i < motor_num; i++)
//     {
//         float error = Target_Position - Now_Distance[i];

//         if (Math_Abs(error) > Param.Distance_Approach_Threshold)
//         {
//             // 定速度运动
//             float speed = (error > 0) ? Param.Max_Velocity : -Param.Max_Velocity;
//             Motor_Lift[i]->Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
//             Motor_Lift[i]->Set_Target_Speed(speed);
//         }
//         else
//         {
//             // 位置闭环: PID 路程环输出速度目标
//             Distance_PID[i].Set_Target(Target_Position);
//             Distance_PID[i].TIM_Calculate_PeriodElapsedCallback();
//             Motor_Lift[i]->Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
//             Motor_Lift[i]->Set_Target_Speed(Distance_PID[i].Get_Out());
//         }
//     }

//     // 3. 执行电机 PID 计算
//     for (uint8_t i = 0; i < motor_num; i++)
//     {
//         Motor_Lift[i]->Calculate();
//     }
// }

// template <uint8_t motor_num>
// float Class_Lift<motor_num>::Get_Now_Distance(uint8_t i) const
// {
//     return (i < motor_num) ? Now_Distance[i] : 0.0f;
// }

// template <uint8_t motor_num>
// float Class_Lift<motor_num>::Get_Target_Position() const
// {
//     return Target_Position;
// }

// template <uint8_t motor_num>
// bool Class_Lift<motor_num>::Get_Is_Calibrated() const
// {
//     return Is_Calibrated;
// }

// #endif
