#include "dvc_motor_dji.h"

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
}
bool Class_Motor_DJI_C610::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_C610_ID fdcan_rx_id,
    float gearbox_rate,
    float current_max
)
{
    Parameters default_parameters;
    return Init(
        hfdcan,
        fdcan_rx_id,
        gearbox_rate,
        current_max,
        default_parameters
    );
}
bool Class_Motor_DJI_C610::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_C610_ID fdcan_rx_id,
    float gearbox_rate,
    float current_max,
    const Parameters& parameters
)
{
    Initialized = false;

    FDCAN_Manage_Object = nullptr;
    Tx_Data = nullptr;

    if (hfdcan == nullptr)
    {
        return false;
    }

    if (!Is_Finite(gearbox_rate) ||
        !Is_Finite(current_max) ||
        gearbox_rate <= 0.0f ||
        current_max <= 0.0f)
    {
        return false;
    }

    if (!Check_Parameters(parameters))
    {
        return false;
    }

    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }
    else
    {
        return false;
    }

    Tx_Data = Allocate_Tx_Data(hfdcan, fdcan_rx_id);

    if (Tx_Data == nullptr)
    {
        return false;
    }

    FDCAN_Rx_ID = fdcan_rx_id;
    Gearbox_Rate = gearbox_rate;
    Current_Max = current_max;
    Param = parameters;

    Control_Mode = MOTOR_CONTROL_MODE_DISABLE;

    Target_Current = 0.0f;
    Target_Speed = 0.0f;
    Target_Position = 0.0f;

    Feedback_Current = 0.0f;
    Feedback_Speed = 0.0f;
    Feedback_Position = 0.0f;

    Target_Speed = 0.0f;

    Feedforward_Speed = 0.0f;
    Feedforward_Current = 0.0f;

    Out = 0.0f;

    Flag = 0;
    Pre_Flag = 0;
    Motor_Status = Motor_Status_DISABLE;
    Rx_Data = {};


    Initialized = true;

    return true;
}

void Class_Motor_DJI_C610::Init()
{
    /*
     * 兼容 Class_Motor_Base 的无参 Init。
     * 真正硬件初始化请调用带参数 Init()。
     */
}

bool Class_Motor_DJI_C610::Check_Parameters(const Parameters& parameters) const
{
    if (!Is_Finite(parameters.Position_Kp) ||
        !Is_Finite(parameters.Position_Ki) ||
        !Is_Finite(parameters.Position_Kd) ||
        !Is_Finite(parameters.Position_Integral_Limit) ||
        !Is_Finite(parameters.Position_Output_Limit))
    {
        return false;
    }

    if (parameters.Position_Integral_Limit < 0.0f ||
        parameters.Position_Output_Limit <= 0.0f)
    {
        return false;
    }

    return true;
}

uint8_t* Class_Motor_DJI_C610::Allocate_Tx_Data(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_C610_ID fdcan_rx_id
)
{
    if (hfdcan == nullptr)
    {
        return nullptr;
    }

    if (hfdcan == &hfdcan1)
    {
        switch (fdcan_rx_id)
        {
        case Motor_DJI_C610_ID_0x201: return &FDCAN1_0x200_Tx_Data[0];
        case Motor_DJI_C610_ID_0x202: return &FDCAN1_0x200_Tx_Data[2];
        case Motor_DJI_C610_ID_0x203: return &FDCAN1_0x200_Tx_Data[4];
        case Motor_DJI_C610_ID_0x204: return &FDCAN1_0x200_Tx_Data[6];

        case Motor_DJI_C610_ID_0x205: return &FDCAN1_0x1ff_Tx_Data[0];
        case Motor_DJI_C610_ID_0x206: return &FDCAN1_0x1ff_Tx_Data[2];
        case Motor_DJI_C610_ID_0x207: return &FDCAN1_0x1ff_Tx_Data[4];
        case Motor_DJI_C610_ID_0x208: return &FDCAN1_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan2)
    {
        switch (fdcan_rx_id)
        {
        case Motor_DJI_C610_ID_0x201: return &FDCAN2_0x200_Tx_Data[0];
        case Motor_DJI_C610_ID_0x202: return &FDCAN2_0x200_Tx_Data[2];
        case Motor_DJI_C610_ID_0x203: return &FDCAN2_0x200_Tx_Data[4];
        case Motor_DJI_C610_ID_0x204: return &FDCAN2_0x200_Tx_Data[6];

        case Motor_DJI_C610_ID_0x205: return &FDCAN2_0x1ff_Tx_Data[0];
        case Motor_DJI_C610_ID_0x206: return &FDCAN2_0x1ff_Tx_Data[2];
        case Motor_DJI_C610_ID_0x207: return &FDCAN2_0x1ff_Tx_Data[4];
        case Motor_DJI_C610_ID_0x208: return &FDCAN2_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan3)
    {
        switch (fdcan_rx_id)
        {
        case Motor_DJI_C610_ID_0x201: return &FDCAN3_0x200_Tx_Data[0];
        case Motor_DJI_C610_ID_0x202: return &FDCAN3_0x200_Tx_Data[2];
        case Motor_DJI_C610_ID_0x203: return &FDCAN3_0x200_Tx_Data[4];
        case Motor_DJI_C610_ID_0x204: return &FDCAN3_0x200_Tx_Data[6];

        case Motor_DJI_C610_ID_0x205: return &FDCAN3_0x1ff_Tx_Data[0];
        case Motor_DJI_C610_ID_0x206: return &FDCAN3_0x1ff_Tx_Data[2];
        case Motor_DJI_C610_ID_0x207: return &FDCAN3_0x1ff_Tx_Data[4];
        case Motor_DJI_C610_ID_0x208: return &FDCAN3_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }

    return nullptr;
}



void Class_Motor_DJI_C610::Update_Feedback()
{
    if (!Initialized)
    {
        return;
    }

    Feedback_Current = Rx_Data.Now_Current;
    Feedback_Speed = Rx_Data.Now_Omega;

    /*
     * 舵轮建议使用外置绝对编码器位置反馈。
     * 如果 Use_External_Position_Feedback = true，
     * 则 Base_Feedback_Position 由 Set_Feedback_Position() 写入，不在这里覆盖。
     */
    if (!Param.Use_External_Position_Feedback)
    {
        Feedback_Position = Normalize_Angle(Rx_Data.Now_Angle);
    }
}

void Class_Motor_DJI_C610::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    Update_Feedback();

    PID_Calculate();

    Limit_Output();
}

void Class_Motor_DJI_C610::PID_Calculate()
{
    switch (Control_Mode)
    {
    case MOTOR_CONTROL_MODE_CURRENT:
    {
        /*
         * C610 电调内部有电流环，这里直接使用上层电流目标。
         */
        Target_Current = Target_Current + Feedforward_Current;
        Target_Speed = 0.0f;
        break;
    }

    case MOTOR_CONTROL_MODE_SPEED:
    {
        /*
         * 速度 PID 输出电流目标。
         */
        Feedback_Speed = Rx_Data.Now_Omega;
        PID_Omega.Set_Target(Target_Speed + Feedforward_Speed);
        PID_Omega.Set_Now(Feedback_Speed);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();
        break;
    }

    case MOTOR_CONTROL_MODE_POSITION:
    {
        /*
         * 舵向位置模式：
         * 位置外环使用 Feedback_Position。
         * 若 Use_External_Position_Feedback = true，
         * 该值来自外置绝对编码器。
         */
        if (!Param.Use_External_Position_Feedback)
        {
            Feedback_Position = Rx_Data.Now_Angle;
        }
        Feedback_Speed = Rx_Data.Now_Omega;


        
        PID_Position.Set_Target(Target_Position);
        PID_Position.Set_Now(Feedback_Position);
        PID_Position.TIM_Calculate_PeriodElapsedCallback();

        Target_Speed = PID_Position.Get_Out();

        PID_Omega.Set_Target(Target_Speed + Feedforward_Speed);
        PID_Omega.Set_Now(Feedback_Speed);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();
        break;
    }

    case MOTOR_CONTROL_MODE_DISABLE:
    default:
    {
        Target_Current = 0.0f;
        Target_Speed = 0.0f;
        break;
    }
    }
}

void Class_Motor_DJI_C610::Limit_Output()
{
    float tmp_value = Target_Current + Feedforward_Current;

    Math_Constrain(&tmp_value, -Current_Max, Current_Max);

    Out = tmp_value * Current_To_Out;

    Feedforward_Current = 0.0f;
    Feedforward_Speed = 0.0f;
}

void Class_Motor_DJI_C610::Output()
{
    if (!Initialized)
    {
        return;
    }

    Output_CAN_Data();
}

void Class_Motor_DJI_C610::Output_CAN_Data()
{
    if (Tx_Data == nullptr)
    {
        return;
    }

    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;
}

void Class_Motor_DJI_C610::FDCAN_RxCpltCallback(uint8_t* rx_data)
{
    (void)rx_data;

    if (!Initialized)
    {
        return;
    }

    Flag += 1;

    Data_Process();
}

void Class_Motor_DJI_C610::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (!Initialized)
    {
        return;
    }

    if (Flag == Pre_Flag)
    {
        Motor_Status = Motor_Status_DISABLE;

        PID_Position.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);

        Target_Current = 0.0f;
        Target_Speed = 0.0f;
        Out = 0.0f;
    }
    else
    {
        Motor_Status = Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_Motor_DJI_C610::Data_Process()
{
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega;
    int16_t tmp_current;

    Struct_Motor_DJI_C610_CAN_Rx_Data* tmp_buffer =
        (Struct_Motor_DJI_C610_CAN_Rx_Data*)FDCAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16(
        (void*)&tmp_buffer->Encoder_Reverse,
        (void*)&tmp_encoder
    );

    Math_Endian_Reverse_16(
        (void*)&tmp_buffer->Omega_Reverse,
        (void*)&tmp_omega
    );

    Math_Endian_Reverse_16(
        (void*)&tmp_buffer->Current_Reverse,
        (void*)&tmp_current
    );

    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;

    if (delta_encoder < -(int16_t)(Encoder_Num_Per_Round / 2))
    {
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > (int16_t)(Encoder_Num_Per_Round / 2))
    {
        Rx_Data.Total_Round--;
    }

    Rx_Data.Total_Encoder =
        Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;

    Rx_Data.Now_Angle =
        (float)Rx_Data.Total_Encoder /
        (float)Encoder_Num_Per_Round *
        kTwoPi /
        Gearbox_Rate;

    Rx_Data.Now_Omega =
        (float)tmp_omega *
        RPM_TO_RADPS /
        Gearbox_Rate;

    Rx_Data.Now_Current =
        (float)tmp_current /
        Current_To_Out;

    Rx_Data.Now_Temperature =
        (float)tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

    Rx_Data.Pre_Encoder = tmp_encoder;
}



bool Class_Motor_DJI_C610::Is_Initialized() const
{
    return Initialized;
}



float Class_Motor_DJI_C610::Normalize_Angle(float angle)
{
    if (!Is_Finite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, kTwoPi);

    if (angle > kPi)
    {
        angle -= kTwoPi;
    }
    else if (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}

float Class_Motor_DJI_C610::Limit(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

bool Class_Motor_DJI_C610::Is_Finite(float value)
{
    return isfinite(value);
}
