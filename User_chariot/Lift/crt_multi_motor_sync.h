#ifndef CRT_MULTI_MOTOR_SYNC_H
#define CRT_MULTI_MOTOR_SYNC_H

#include "alg_pid.h"
#include "dvc_motor_base.h"
#include <array>

/**
 * @brief 多电机同步运动基类
 *
 * 每个电机独立路程环PID，支持开环→PID闭环双层控制。
 * 外部只需通过 Set_Target_Position / Target_Position[i] 设定目标，
 * 每周期调用 Move_To_Position() 即可自动运行。
 */
template <uint8_t motor_num>
class Class_MultiMotorSync_Base
{
public:
    struct Parameters
    {
        PID_Parameters PID_Distance[motor_num];    // 路程环PID参数
        float Distance_Approach_Threshold = 0.01f; // 开环→闭环切换阈值, 行程单位
        float Max_Velocity = 5.0f;                 // 开环速度, 行程单位/s
        float Angle_To_Distance = 1.0f;            // 电机 rad → 行程 转换系数
        int8_t Direction_Sign[motor_num] = {};     // ±1, -1=电机转向反向(镜像安装)
        Calibrate_Params Calibrate;                // 堵转校准参数
    };

    Class_PID Distance_PID[motor_num];
    Class_Motor_Base *Motor[motor_num] = {};

    void Init(std::array<Class_Motor_Base *, motor_num> motors, const Parameters &parameters);

    void Set_Target_Position(float target_position);          // 所有电机设同一目标
    float Get_Target_Position() const;
    float Get_Now_Distance(uint8_t i) const;
    bool Get_Is_Calibrated() const;

    bool Calibrate_Update();                                  // 堵转校准，完成后自动计算Offset
    void Distance_Update();                                   // 更新 Now_Distance 并喂给路程PID
    void Move_To_Position();                                  // 核心控制: 开环/闭环切换 

protected:
    Parameters Param = {};

    float Now_Distance[motor_num] = {0.0f};                   // 当前行程, = (电机角度 - Offset) * Angle_To_Distance
    float Offset[motor_num] = {0.0f};                         // 机械零点补偿(弧度), 由校准或Set_Offset设定
    float Target_Distance[motor_num] = {0.0f};                // 目标行程
    bool Is_Calibrated = false;
    bool Calibrated_Motor[motor_num] = {};                    // 单电机校准完成标记
};

template <uint8_t motor_num>
void Class_MultiMotorSync_Base<motor_num>::Init(std::array<Class_Motor_Base *, motor_num> motors,const Parameters &parameters)
{
    Param = parameters;
    for (uint8_t i = 0; i < motor_num; i++)
    {
        if (Param.Direction_Sign[i] >= 0.0f)
        {
            Param.Direction_Sign[i] = 1.0f;
        }
        else
        {
            Param.Direction_Sign[i] = -1.0f;
        }
        Calibrated_Motor[i] = false;
    }
    for (uint8_t i = 0; i < motor_num; i++)
    {
        Motor[i] = motors[i];
        const auto &p = parameters.PID_Distance[i];
        Distance_PID[i].Init(
            p.K_P, p.K_I, p.K_D, p.K_F,
            p.I_Out_Max, p.Out_Max, p.D_T, p.Dead_Zone,
            p.I_Variable_Speed_A, p.I_Variable_Speed_B,
            p.I_Separate_Threshold, p.D_First);
    }
}

template <uint8_t motor_num>
void Class_MultiMotorSync_Base<motor_num>::Set_Target_Position(float target_position)
{
    for (uint8_t i = 0; i < motor_num; i++)
        Target_Distance[i] = target_position;
}

template <uint8_t motor_num>
float Class_MultiMotorSync_Base<motor_num>::Get_Target_Position() const
{
    return motor_num > 0 ? Target_Distance[0] : 0.0f;
}

template <uint8_t motor_num>
float Class_MultiMotorSync_Base<motor_num>::Get_Now_Distance(uint8_t i) const
{
    return (i < motor_num) ? Now_Distance[i] : 0.0f;
}

template <uint8_t motor_num>
bool Class_MultiMotorSync_Base<motor_num>::Get_Is_Calibrated() const
{
    return Is_Calibrated;
}

template <uint8_t motor_num>
bool Class_MultiMotorSync_Base<motor_num>::Calibrate_Update()
{
    if (Is_Calibrated)
        return true;

    bool all_done = true;
    for (uint8_t i = 0; i < motor_num; i++)
    {
        if (Calibrated_Motor[i])
        {
            Motor[i]->Calculate();
            continue;
        }

        float offset;
        auto calib = Param.Calibrate;
        calib.motion_value *= (float)Param.Direction_Sign[i]; // 镜像电机取反运动方向
        if (Motor[i]->Calibrate(calib, offset))
        {
            Offset[i] = offset;
            Calibrated_Motor[i] = true;
        }
        else
        {
            all_done = false;
        }
        Motor[i]->Calculate();
    }

    if (all_done)
    {
        for (uint8_t i = 0; i < motor_num; i++)
        {
            Motor[i]->Update_Feedback();
            float raw = Motor[i]->Get_Position();
            Now_Distance[i] = (raw - Offset[i]) * Param.Angle_To_Distance * Param.Direction_Sign[i];
        }
        Is_Calibrated = true;
    }
    return all_done;
}

template <uint8_t motor_num>
void Class_MultiMotorSync_Base<motor_num>::Distance_Update()
{
    for (uint8_t i = 0; i < motor_num; i++)
    {
        float raw = Motor[i]->Get_Position();
        Now_Distance[i] = (raw - Offset[i]) * Param.Angle_To_Distance * Param.Direction_Sign[i];
        Distance_PID[i].Set_Now(Now_Distance[i]);
    }
}

template <uint8_t motor_num>
void Class_MultiMotorSync_Base<motor_num>::Move_To_Position()
{
    for (uint8_t i = 0; i < motor_num; i++)
    {
        float target = Target_Distance[i];
        float error = target - Now_Distance[i];
        float sign = Param.Direction_Sign[i];

        if (Math_Abs(error) > Param.Distance_Approach_Threshold)
        {
            // 远距离 → 开环恒速逼近
            Motor[i]->Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
            Motor[i]->Set_Target_Speed(sign * ((error > 0) ? Param.Max_Velocity : -Param.Max_Velocity));
        }
        else
        {
            // 近距离 → 路程PID闭环精确到位
            Distance_PID[i].Set_Target(target);
            Distance_PID[i].TIM_Calculate_PeriodElapsedCallback();
            Motor[i]->Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
            Motor[i]->Set_Target_Speed(sign * Distance_PID[i].Get_Out());
        }
    }
    // Calculate() 由外部 TIM 回调统一调用，此处不调用
}

// 显式实例化，确保代码生成
template class Class_MultiMotorSync_Base<2>;

#endif
