#ifndef DVC_MOTOR_DJI_C610_BASE_H
#define DVC_MOTOR_DJI_C610_BASE_H

#include <stdint.h>
#include <math.h>

#include "alg_pid.h"
#include "dvc_motor_base.h"
#include "drv_can.h"
#include "drv_math.h"

/**
 * @brief DJI 电机 CAN ID
 *
 * C610/C620 常用 ID:
 * 0x201 ~ 0x208
 */
enum Enum_Motor_DJI_C610_ID
{
    Motor_DJI_C610_ID_0x201 = 1,
    Motor_DJI_C610_ID_0x202,
    Motor_DJI_C610_ID_0x203,
    Motor_DJI_C610_ID_0x204,
    Motor_DJI_C610_ID_0x205,
    Motor_DJI_C610_ID_0x206,
    Motor_DJI_C610_ID_0x207,
    Motor_DJI_C610_ID_0x208,
};
/**
 * @brief DJI 电机CAN发送ID
 */
enum Enum_CAN_Tx_ID
{
    CAN_Tx_ID_0x200_Only = 0,
    CAN_Tx_ID_0x1FF_Only,
    CAN_Tx_ID_Both
};

/**
 * @brief DJI 电机 CAN 原始反馈数据
 */
struct Struct_Motor_DJI_C610_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief DJI C610 处理后的反馈数据
 */
struct Struct_Motor_DJI_C610_Rx_Data
{
    float Now_Angle = 0.0f;       // rad, 减速箱输出端角度
    float Now_Omega = 0.0f;       // rad/s, 减速箱输出端角速度
    float Now_Current = 0.0f;     // A
    float Now_Temperature = 0.0f; // K

    uint32_t Pre_Encoder = 0;
    int32_t Total_Encoder = 0;
    int32_t Total_Round = 0;
};
namespace Motor
{

    /**
     * @brief DJI C610 电机类，直接继承 Class_Motor_Base
     *
     * 设计目标：
     * 1. 只使用 Class_Motor_Base 的控制模式 Enum_Motor_Control_Mode
     * 2. 不再保留 Enum_Motor_DJI_Control_Method，避免两套模式同步
     * 3. CURRENT 模式：直接下发电流目标
     * 4. SPEED 模式：速度 PID 输出电流目标
     * 5. POSITION 模式：外部位置环输出速度目标，再由速度 PID 输出电流目标
     *
     * 舵轮舵向建议：
     * - POSITION 模式使用外部绝对编码器反馈
     * - 通过 Set_Feedback_Position(absolute_angle_rad) 输入舵向真实角度
     */
    struct PID_Parameters
    {
        float K_P = 0.0f;
        float K_I = 0.0f;
        float K_D = 0.0f;
        float K_F = 0.0f;

        float I_Out_Max = 0.0f;
        float Out_Max = 0.0f;
        float D_T = 0.001f;
        float Dead_Zone = 0.0f;

        float I_Variable_Speed_A = 0.0f;
        float I_Variable_Speed_B = 0.0f;
        float I_Separate_Threshold = 0.0f;

        Enum_PID_D_First D_First = PID_D_First_DISABLE;
    };

    uint8_t *Allocate_Tx_Data(
        FDCAN_HandleTypeDef *hfdcan,
        Enum_Motor_DJI_C610_ID fdcan_rx_id);
    void DJI_TIM_Send_Group(FDCAN_HandleTypeDef *hfdcan, Enum_CAN_Tx_ID __Enum_CAN_Tx_ID);

    class Class_Motor_DJI_C610 : public Class_Motor_Base
    {
    public:
        struct Parameters
        {
            PID_Parameters PID_Position;

            PID_Parameters PID_Omega;

            /*
             * true:
             *   POSITION 模式使用外部绝对编码器反馈 Base_Feedback_Position
             * false:
             *   POSITION 模式使用 C610 自身编码器反馈 Rx_Data.Now_Angle
             */
            bool Use_External_Position_Feedback = false;
        };

    
        Class_PID PID_Omega;
        Class_PID PID_Position;


        Class_Motor_DJI_C610() = default;

        /**
         * @brief 硬件初始化
         *
         * @param hfdcan FDCAN 句柄
         * @param fdcan_rx_id 电机反馈 ID，0x201 ~ 0x208
         * @param gearbox_rate 减速比，舵向若无减速箱可设 1
         * @param current_max 最大输出电流，A
         * @param parameters Base 控制参数
         * @return true 初始化成功
         * @return false 初始化失败
         */
        bool Init(
            FDCAN_HandleTypeDef *hfdcan,
            Enum_Motor_DJI_C610_ID fdcan_rx_id,
            float gearbox_rate = 36.0f,
            float current_max = 10.0f);

        bool Init(
            FDCAN_HandleTypeDef *hfdcan,
            Enum_Motor_DJI_C610_ID fdcan_rx_id,
            const Parameters &parameters,
            float gearbox_rate = 36.0f,
            float current_max = 10.0f
        );

        /**
         * @brief 兼容 Class_Motor_Base 的无参 Init
         *
         * 真正硬件初始化请调用带参数 Init()。
         */

        inline void Set_Control_Method(Enum_Motor_Control_Method __Method) override;

        inline void Set_Target_Current(float __Target_Current) override;
        inline void Set_Target_Speed(float __Target_Speed) override;
        inline void Set_Target_Position(float __Target_Position) override;

        inline void Set_Feedback_Current(float __Feedback_Current) override;
        inline void Set_Feedback_Speed(float __Feedback_Speed) override;
        inline void Set_Feedback_Position(float __Feedback_Position) override;

        inline float Get_Current() const override;
        inline float Get_Speed() const override;
        inline float Get_Position() const override;

        /**
         * @brief 从 CAN 帧更新 Class_Motor_Base 统一反馈值
         */
        void Update_Feedback() override;

        /**
         * @brief 控制周期主入口：反馈更新 → PID 计算 → 输出限幅
         */
        void Calculate() override;

        /**
         * @brief 将计算结果写入 CAN 发送缓冲区
         */
        void Output() override;

        /**
         * @brief CAN 接收回调
         *
         * 外部 FDCAN 分发时，在收到对应电机反馈后调用。
         */
        void FDCAN_RxCpltCallback(uint8_t *rx_data);

        /**
         * @brief 100ms 心跳检测定时器回调
         *
         * 丢帧时判定电机离线，清零状态和积分。
         */
        void TIM_100ms_Alive_PeriodElapsedCallback();

        bool Is_Initialized() const;
        Enum_Motor_Status Get_Status() const;

        /* ===== DJI C610 原生反馈值 ===== */
        inline float Get_Now_Angle() const;
        inline float Get_Now_Omega() const;
        inline float Get_Now_Current() const;
        inline float Get_Now_Temperature() const;

        /* ===== 内部状态读取 ===== */
        inline float Get_Target_Current_Internal() const;
        inline float Get_Target_Omega_Internal() const;
        inline float Get_Output() const;

        /**
         * @brief 速度前馈 — 叠加到速度环目标，Limit_Output() 中自动清零
         */
        inline void Set_Feedforward_Omega(float feedforward_omega);
        inline void Set_Feedforward_Current(float feedforward_current);

    private:
        /*
         * CAN / DJI 底层相关
         */
        Struct_FDCAN_Manage_Object *FDCAN_Manage_Object = nullptr;
        Enum_Motor_DJI_C610_ID FDCAN_Rx_ID = Motor_DJI_C610_ID_0x201;
        uint8_t *Tx_Data = nullptr;

        float Gearbox_Rate = 36.0f;
        float Current_Max = 10.0f;

        uint16_t Encoder_Num_Per_Round = 8192;
        float Current_To_Out = 10000.0f / 10.0f;
        float Theoretical_Output_Current_Max = 10.0f;

        /*
         * 心跳检测：Flag 在每次收到反馈时自增，
         * Pre_Flag 在 100ms 定时器中对比判断是否丢帧。
         */
        uint32_t Flag = 0;
        uint32_t Pre_Flag = 0;

        /* DJI CAN 协议待发送的电流值（已转换为协议格式） */
        float Out = 0.0f;

        Struct_Motor_DJI_C610_Rx_Data Rx_Data;

        /*
         * Class_Motor_Base 统一控制层
         */
        Parameters Param;
        Enum_Motor_Status Motor_Status = Motor_Status_DISABLE;
        Enum_Motor_Control_Method Control_Method = MOTOR_CONTROL_METHOD_CURRENT;

        float Target_Current = 0.0f;  // A, 上层目标电流
        float Target_Speed = 0.0f;    // rad/s, 上层目标速度
        float Target_Position = 0.0f; // rad, 上层目标位置

        /* 电机接受到的反馈值，单位同 Target_* 单位 */
        float Feedback_Current = 0.0f;  // A
        float Feedback_Speed = 0.0f;    // rad/s
        float Feedback_Position = 0.0f; // rad

        bool Initialized = false;

        /* 前馈叠加量：由上层在 Calculate() 前写入，Limit_Output() 中清零 */
        float Feedforward_Speed = 0.0f;   // rad/s
        float Feedforward_Current = 0.0f; // A

    private:
        bool Check_Parameters(const Parameters &parameters) const;

        void Data_Process();

        /**
         * @brief 使用 Class_Motor_Base 的 Control_Mode 直接计算 C610 PID
         */
        void PID_Calculate();

        void Limit_Output();
        void Output_CAN_Data();

        /**
         * @brief 将角度归一化到 [-2π, 2π] 区间
         */
        static float Normalize_Angle(float angle);
        static bool Is_Finite(float value);
    };

    /* ===== inline 实现：状态读取 ===== */

    inline Enum_Motor_Status Class_Motor_DJI_C610::Get_Status() const
    {
        return Motor_Status;
    }

    inline float Class_Motor_DJI_C610::Get_Now_Angle() const
    {
        return Rx_Data.Now_Angle;
    }

    inline float Class_Motor_DJI_C610::Get_Now_Omega() const
    {
        return Rx_Data.Now_Omega;
    }

    inline float Class_Motor_DJI_C610::Get_Now_Current() const
    {
        return Rx_Data.Now_Current;
    }

    inline float Class_Motor_DJI_C610::Get_Now_Temperature() const
    {
        return Rx_Data.Now_Temperature;
    }

    inline float Class_Motor_DJI_C610::Get_Target_Current_Internal() const
    {
        return Target_Current;
    }

    inline float Class_Motor_DJI_C610::Get_Target_Omega_Internal() const
    {
        return Target_Speed;
    }

    inline float Class_Motor_DJI_C610::Get_Output() const
    {
        return Out;
    }

    /* ===== inline 实现：前馈写入（含 NaN/Inf 防护） ===== */

    inline void Class_Motor_DJI_C610::Set_Feedforward_Omega(float feedforward_omega)
    {
        if (Is_Finite(feedforward_omega))
        {
            Feedforward_Speed = feedforward_omega;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Feedforward_Current(float feedforward_current)
    {
        if (Is_Finite(feedforward_current))
        {
            Feedforward_Current = feedforward_current;
        }
    }

    /* ===== inline 实现：Class_Motor_Base 接口 ===== */

    inline void Class_Motor_DJI_C610::Set_Control_Method(Enum_Motor_Control_Method __Method)
    {
        Control_Method = __Method;
    }

    inline void Class_Motor_DJI_C610::Set_Target_Current(float __Target_Current)
    {
        if (Is_Finite(__Target_Current))
        {
            Target_Current = __Target_Current;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Target_Speed(float __Target_Speed)
    {
        if (Is_Finite(__Target_Speed))
        {
            /* C610 对外速度单位：rad/s */
            Target_Speed = __Target_Speed;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Target_Position(float __Target_Position)
    {
        if (Is_Finite(__Target_Position))
        {
            /* C610 对外位置单位：rad */
            Target_Position = __Target_Position;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Feedback_Current(float __Feedback_Current)
    {
        if (Is_Finite(__Feedback_Current))
        {
            Feedback_Current = __Feedback_Current;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Feedback_Speed(float __Feedback_Speed)
    {
        if (Is_Finite(__Feedback_Speed))
        {
            Feedback_Speed = __Feedback_Speed;
        }
    }

    inline void Class_Motor_DJI_C610::Set_Feedback_Position(float __Feedback_Position)
    {
        if (Is_Finite(__Feedback_Position))
        {
            /* 外部位置反馈写入前归一化到 [-π, π] */
            Feedback_Position = __Feedback_Position;
        }
    }

    inline float Class_Motor_DJI_C610::Get_Current() const
    {
        return Feedback_Current;
    }

    inline float Class_Motor_DJI_C610::Get_Speed() const
    {
        return Feedback_Speed;
    }

    inline float Class_Motor_DJI_C610::Get_Position() const
    {
        return Feedback_Position;
    }
}
#endif
