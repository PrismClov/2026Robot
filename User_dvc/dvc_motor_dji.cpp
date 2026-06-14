#include "dvc_motor_dji.h"

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
}

namespace Motor
{
    
/**
 * @brief 重载 Init() — 默认控制参数版本
 */
bool Class_Motor_DJI_C610::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_ID fdcan_rx_id,
    float gearbox_rate,
    float current_max
)
{
    Parameters default_parameters;
    return Init(
        hfdcan,
        fdcan_rx_id,
        default_parameters,
        gearbox_rate,
        current_max
    );
}
/**
 * @brief 完整 Init() — 带控制参数版本
 *
 * 工作流程：
 * 1. 校验参数合法性（gearbox_rate > 0、current_max > 0、PID 参数有限）
 * 2. 绑定 FDCAN 句柄到对应的 Manage_Object
 * 3. 从 FDCAN 共享发送缓冲区分配 2 字节 Tx_Data
 * 4. 清零所有状态和目标值
 */
bool Class_Motor_DJI_C610::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_ID fdcan_rx_id,
    const Parameters& parameters,
    float gearbox_rate,
    float current_max
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

    Control_Method = MOTOR_CONTROL_METHOD_CURRENT;

    Target_Current = 0.0f;
    Target_Speed = 0.0f;
    Target_Position = 0.0f;

    Feedback_Current = 0.0f;
    Feedback_Speed = 0.0f;
    Feedback_Position = 0.0f;

    Feedforward_Speed = 0.0f;
    Feedforward_Current = 0.0f;

    Out = 0.0f;

    Flag = 0;
    Pre_Flag = 0;
    Motor_Status = Motor_Status_DISABLE;
    Rx_Data = {};

    // PID 初始化
    PID_Position.Init(parameters.PID_Position.K_P,
                      parameters.PID_Position.K_I,
                      parameters.PID_Position.K_D,
                      parameters.PID_Position.K_F,
                      parameters.PID_Position.I_Out_Max,
                      parameters.PID_Position.Out_Max,
                      parameters.PID_Position.D_T,
                      parameters.PID_Position.Dead_Zone,
                      parameters.PID_Position.I_Variable_Speed_A,
                      parameters.PID_Position.I_Variable_Speed_B,
                      parameters.PID_Position.I_Separate_Threshold,
                      parameters.PID_Position.D_First);

    PID_Omega.Init(parameters.PID_Omega.K_P,
                   parameters.PID_Omega.K_I,
                   parameters.PID_Omega.K_D,
                   parameters.PID_Omega.K_F,
                   parameters.PID_Omega.I_Out_Max,
                   parameters.PID_Omega.Out_Max,
                   parameters.PID_Omega.D_T,
                   parameters.PID_Omega.Dead_Zone,
                   parameters.PID_Omega.I_Variable_Speed_A,
                   parameters.PID_Omega.I_Variable_Speed_B,
                   parameters.PID_Omega.I_Separate_Threshold,
                   parameters.PID_Omega.D_First);

    Initialized = true;

    return true;
}

/**
 * @brief 校验控制参数合法性
 *
 * 检查项：PID 参数必须为有限值，积分限幅非负，输出限幅为正。
 */
bool Class_Motor_DJI_C610::Check_Parameters(const Parameters& parameters) const
{
    if (!Is_Finite(parameters.PID_Position.K_P) ||
        !Is_Finite(parameters.PID_Position.K_I) ||
        !Is_Finite(parameters.PID_Position.K_D) ||
        !Is_Finite(parameters.PID_Position.I_Out_Max) ||
        !Is_Finite(parameters.PID_Position.Out_Max))
    {
        return false;
    }

    if (parameters.PID_Position.I_Out_Max < 0.0f ||
        parameters.PID_Position.Out_Max <= 0.0f)
    {
        return false;
    }

    return true;
}

/**
 * @brief 为指定 CAN 接口和电机 ID 分配 Tx_Data 缓冲区
 *
 * DJI C610 每个电机占用 2 字节（电流目标高/低各 1 字节），
 * 同一 ID 组（0x200 / 0x1FF）的 4 个电机共享一个 CAN 帧。
 */
uint8_t* Allocate_Tx_Data(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_ID fdcan_rx_id
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
        case Motor_DJI_ID_0x201: return &FDCAN1_0x200_Tx_Data[0];
        case Motor_DJI_ID_0x202: return &FDCAN1_0x200_Tx_Data[2];
        case Motor_DJI_ID_0x203: return &FDCAN1_0x200_Tx_Data[4];
        case Motor_DJI_ID_0x204: return &FDCAN1_0x200_Tx_Data[6];

        case Motor_DJI_ID_0x205: return &FDCAN1_0x1ff_Tx_Data[0];
        case Motor_DJI_ID_0x206: return &FDCAN1_0x1ff_Tx_Data[2];
        case Motor_DJI_ID_0x207: return &FDCAN1_0x1ff_Tx_Data[4];
        case Motor_DJI_ID_0x208: return &FDCAN1_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan2)
    {
        switch (fdcan_rx_id)
        {
        case Motor_DJI_ID_0x201: return &FDCAN2_0x200_Tx_Data[0];
        case Motor_DJI_ID_0x202: return &FDCAN2_0x200_Tx_Data[2];
        case Motor_DJI_ID_0x203: return &FDCAN2_0x200_Tx_Data[4];
        case Motor_DJI_ID_0x204: return &FDCAN2_0x200_Tx_Data[6];

        case Motor_DJI_ID_0x205: return &FDCAN2_0x1ff_Tx_Data[0];
        case Motor_DJI_ID_0x206: return &FDCAN2_0x1ff_Tx_Data[2];
        case Motor_DJI_ID_0x207: return &FDCAN2_0x1ff_Tx_Data[4];
        case Motor_DJI_ID_0x208: return &FDCAN2_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }
    else if (hfdcan == &hfdcan3)
    {
        switch (fdcan_rx_id)
        {
        case Motor_DJI_ID_0x201: return &FDCAN3_0x200_Tx_Data[0];
        case Motor_DJI_ID_0x202: return &FDCAN3_0x200_Tx_Data[2];
        case Motor_DJI_ID_0x203: return &FDCAN3_0x200_Tx_Data[4];
        case Motor_DJI_ID_0x204: return &FDCAN3_0x200_Tx_Data[6];

        case Motor_DJI_ID_0x205: return &FDCAN3_0x1ff_Tx_Data[0];
        case Motor_DJI_ID_0x206: return &FDCAN3_0x1ff_Tx_Data[2];
        case Motor_DJI_ID_0x207: return &FDCAN3_0x1ff_Tx_Data[4];
        case Motor_DJI_ID_0x208: return &FDCAN3_0x1ff_Tx_Data[6];

        default: return nullptr;
        }
    }

    return nullptr;
}



/**
 * @brief 更新反馈值到 Class_Motor_Base 统一接口
 *
 * - 电流和速度始终从 C610 内部编码器反馈读取
 * - 位置根据 Use_External_Position_Feedback 决定来自 C610 还是外部绝对编码器
 */
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
        Feedback_Position = Rx_Data.Now_Angle;
    }
}

/**
 * @brief 控制周期主入口：反馈更新 → PID 计算 → 输出限幅
 */
void Class_Motor_DJI_C610::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    Update_Feedback();

    PID_Calculate();

    Limit_Output();

    Output();
}

/**
 * @brief 根据控制模式执行对应 PID 计算
 *
 * 三种控制模式：
 * - CURRENT：直接使用上层电流目标（+ 前馈电流），不经 PID
 * - SPEED：速度 PID 输出电流目标（+ 速度前馈）
 * - POSITION：位置外环 → 速度目标 → 速度内环 → 电流目标（双环串级 PID）
 *
 * 前馈叠加：
 * - Feedforward_Speed 叠加到速度环目标
 * - Feedforward_Current 叠加到最终电流输出
 */
void Class_Motor_DJI_C610::PID_Calculate()
{
    switch (Control_Method)
    {
    case MOTOR_CONTROL_METHOD_CURRENT:
    {
        /*
         * C610 电调内部有电流环，这里直接使用上层电流目标。
         */
        Target_Current = Target_Current + Feedforward_Current;
        Target_Speed = 0.0f;
        break;
    }

    case MOTOR_CONTROL_METHOD_SPEED:
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

    case MOTOR_CONTROL_METHOD_POSITION:
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

    case MOTOR_CONTROL_METHOD_DISABLE:
    default:
    {
        Target_Current = 0.0f;
        Target_Speed = 0.0f;
        break;
    }
    }
}

/**
 * @brief 输出限幅与单位转换
 *
 * CURRENT 模式下 PID_Calculate() 已叠加 Feedforward_Current，
 * 此处再次叠加确保所有模式最终都能接受前馈。
 * 限幅后清零前馈值，避免重复叠加。
 */
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

/**
 * @brief 写入 CAN 发送缓冲区（2 字节，大端字节序）
 *
 * DJI 协议：Tx_Data[0] = 高字节，Tx_Data[1] = 低字节
 * 实际 CAN 帧由 FDCAN 定期发送任务统一打包发出。
 */
void Class_Motor_DJI_C610::Output_CAN_Data()
{
    if (Tx_Data == nullptr)
    {
        return;
    }

    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;

}

/**
 * @brief FDCAN 接收回调 — 更新电机状态标志和反馈数据
 *
 * 每次收到反馈帧后 Flag 自增，用于后续心跳检测。
 * rx_data 参数未使用，直接从 FDCAN_Manage_Object 读取最新帧。
 */
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

/**
 * @brief 100ms 心跳检测定时器回调
 *
 * 如果连续 100ms 内 Flag 未变化（未收到新反馈帧），
 * 判定电机离线：禁用状态、清零 PID 积分、清零输出。
 * 在线时重置 PID 积分可防止恢复连接后积分突跳。
 */
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

/**
 * @brief 解析 CAN 原始反馈数据
 *
 * 处理流程：
 * 1. 字节序反转（DJI 为大端）
 * 2. 多圈追踪：通过相邻帧编码器差值判断是否过零，更新 Total_Round
 * 3. 计算减速箱输出端角度：Total_Encoder / 分辨率 * 2π / 减速比
 * 4. 计算角速度：RPM → rad/s / 减速比
 * 5. 计算电流和温度
 *
 * DJI C610 编码器分辨率：8192 线/圈（13 bit 绝对值编码器）
 */
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

    Struct_Motor_DJI_CAN_Rx_Data* tmp_buffer =
        (Struct_Motor_DJI_CAN_Rx_Data*)FDCAN_Manage_Object->Rx_Buffer.Data;

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


/**
 * @brief 判断输入值是否为有限数
 *
 * @param value 输入值
 * @return true 输入值为有限数
 * @return false 输入值是无限数
 */
bool Class_Motor_DJI_C610::Is_Finite(float value)
{
    return isfinite(value);
}
/**
 * @brief 大疆电机统一发送接口
 * C610 C620
 * @return
 */
void DJI_TIM_Send_Group(FDCAN_HandleTypeDef *hfdcan, Enum_CAN_Tx_ID __Enum_CAN_Tx_ID)
{
    if (hfdcan == nullptr)
    {
        return;
    }

    // C610 C620共用 0x200 和 0x1FF 两组 ID，分别对应 FDCAN1_0x200_Tx_Data 和 FDCAN1_0x1ff_Tx_Data 共享缓冲区
    // 0x201 ~ 0x204 使用 0x200 组，0x205 ~ 0x208 使用 0x1FF 组
    // DJI电调使用标准帧 ID，数据长度固定为 8 字节，每个电机占用 2 字节（高字节 + 低字节）
    uint8_t *tx_data = nullptr;
    switch (__Enum_CAN_Tx_ID)
    {
    case CAN_Tx_ID_0x200_Only:
    {
        //绑定0x201~0x204的电机ID到0x200组发送缓冲区
        tx_data = Allocate_Tx_Data(hfdcan, Motor_DJI_ID_0x201);
        if (tx_data != nullptr)
        FDCAN_Send_Data(hfdcan, 0x200, tx_data, FDCAN_ID_Standard);
        break;
    }

    case CAN_Tx_ID_0x1FF_Only:
    {
        //绑定0x205~0x208的电机ID到0x1FF组发送缓冲区
        tx_data = Allocate_Tx_Data(hfdcan, Motor_DJI_ID_0x205);
        if (tx_data != nullptr)
        FDCAN_Send_Data(hfdcan, 0x1FF, tx_data, FDCAN_ID_Standard);
        break;
    }

    case CAN_Tx_ID_Both:
    {
        //绑定0x201~0x204和0x205~0x208的电机ID到0x200组和0x1FF组发送缓冲区
        tx_data = Allocate_Tx_Data(hfdcan, Motor_DJI_ID_0x201);

        if (tx_data != nullptr)
        FDCAN_Send_Data(hfdcan, 0x200, tx_data, FDCAN_ID_Standard);

        tx_data = Allocate_Tx_Data(hfdcan, Motor_DJI_ID_0x205);

        if (tx_data != nullptr)
        FDCAN_Send_Data(hfdcan, 0x1FF, tx_data, FDCAN_ID_Standard);
        break;
    }

    default:
        break;
    }
}

/* ====================================================================== */
/*  Class_Motor_DJI_C620 实现                                             */
/* ====================================================================== */

namespace
{
    float power_calculate_c620(float K_0, float K_1, float K_2, float A, float Current, float Omega)
    {
        return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
    }
}

bool Class_Motor_DJI_C620::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_ID fdcan_rx_id,
    float gearbox_rate,
    float current_max
)
{
    Parameters default_parameters;
    return Init(
        hfdcan,
        fdcan_rx_id,
        default_parameters,
        gearbox_rate,
        current_max
    );
}

bool Class_Motor_DJI_C620::Init(
    FDCAN_HandleTypeDef* hfdcan,
    Enum_Motor_DJI_ID fdcan_rx_id,
    const Parameters& parameters,
    float gearbox_rate,
    float current_max
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

    Control_Method = MOTOR_CONTROL_METHOD_CURRENT;

    Target_Current = 0.0f;
    Target_Speed = 0.0f;
    Target_Position = 0.0f;

    Feedback_Current = 0.0f;
    Feedback_Speed = 0.0f;
    Feedback_Position = 0.0f;

    Feedforward_Speed = 0.0f;
    Feedforward_Current = 0.0f;

    Out = 0.0f;
    Power_Estimate = 0.0f;
    Power_Factor = 1.0f;

    Flag = 0;
    Pre_Flag = 0;
    Motor_Status = Motor_Status_DISABLE;
    Rx_Data = {};

    PID_Position.Init(parameters.PID_Position.K_P,
                      parameters.PID_Position.K_I,
                      parameters.PID_Position.K_D,
                      parameters.PID_Position.K_F,
                      parameters.PID_Position.I_Out_Max,
                      parameters.PID_Position.Out_Max,
                      parameters.PID_Position.D_T,
                      parameters.PID_Position.Dead_Zone,
                      parameters.PID_Position.I_Variable_Speed_A,
                      parameters.PID_Position.I_Variable_Speed_B,
                      parameters.PID_Position.I_Separate_Threshold,
                      parameters.PID_Position.D_First);

    PID_Omega.Init(parameters.PID_Omega.K_P,
                   parameters.PID_Omega.K_I,
                   parameters.PID_Omega.K_D,
                   parameters.PID_Omega.K_F,
                   parameters.PID_Omega.I_Out_Max,
                   parameters.PID_Omega.Out_Max,
                   parameters.PID_Omega.D_T,
                   parameters.PID_Omega.Dead_Zone,
                   parameters.PID_Omega.I_Variable_Speed_A,
                   parameters.PID_Omega.I_Variable_Speed_B,
                   parameters.PID_Omega.I_Separate_Threshold,
                   parameters.PID_Omega.D_First);

    Initialized = true;

    return true;
}

bool Class_Motor_DJI_C620::Check_Parameters(const Parameters& parameters) const
{
    if (!Is_Finite(parameters.PID_Position.K_P) ||
        !Is_Finite(parameters.PID_Position.K_I) ||
        !Is_Finite(parameters.PID_Position.K_D) ||
        !Is_Finite(parameters.PID_Position.I_Out_Max) ||
        !Is_Finite(parameters.PID_Position.Out_Max))
    {
        return false;
    }

    if (parameters.PID_Position.I_Out_Max < 0.0f ||
        parameters.PID_Position.Out_Max <= 0.0f)
    {
        return false;
    }

    if (!Is_Finite(parameters.Power_K_0) ||
        !Is_Finite(parameters.Power_K_1) ||
        !Is_Finite(parameters.Power_K_2) ||
        !Is_Finite(parameters.Power_A))
    {
        return false;
    }

    return true;
}

void Class_Motor_DJI_C620::Update_Feedback()
{
    if (!Initialized)
    {
        return;
    }

    Feedback_Current = Rx_Data.Now_Current;
    Feedback_Speed = Rx_Data.Now_Omega;

    if (!Param.Use_External_Position_Feedback)
    {
        Feedback_Position = Rx_Data.Now_Angle;
    }
}

void Class_Motor_DJI_C620::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    Update_Feedback();

    PID_Calculate();

    Limit_Output();

    Output();

    if (Param.Power_Limit_Status == Motor_DJI_C620_Power_Limit_Status_DISABLE)
    {
        Feedforward_Current = 0.0f;
        Feedforward_Speed = 0.0f;
    }
}

void Class_Motor_DJI_C620::PID_Calculate()
{
    switch (Control_Method)
    {
    case MOTOR_CONTROL_METHOD_CURRENT:
    {
        Target_Current = Target_Current + Feedforward_Current;
        Target_Speed = 0.0f;
        break;
    }

    case MOTOR_CONTROL_METHOD_SPEED:
    {
        PID_Omega.Set_Target(Target_Speed + Feedforward_Speed);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();
        break;
    }

    case MOTOR_CONTROL_METHOD_POSITION:
    {
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

    case MOTOR_CONTROL_METHOD_DISABLE:
    default:
    {
        Target_Current = 0.0f;
        Target_Speed = 0.0f;
        break;
    }
    }
}

void Class_Motor_DJI_C620::Limit_Output()
{
    float tmp_value = Target_Current + Feedforward_Current;

    Math_Constrain(&tmp_value, -Current_Max, Current_Max);

    Out = tmp_value * Current_To_Out;
}

void Class_Motor_DJI_C620::Output()
{
    if (!Initialized)
    {
        return;
    }

    Output_CAN_Data();
}

void Class_Motor_DJI_C620::Output_CAN_Data()
{
    if (Tx_Data == nullptr)
    {
        return;
    }

    Tx_Data[0] = (int16_t)Out >> 8;
    Tx_Data[1] = (int16_t)Out;
}

void Class_Motor_DJI_C620::FDCAN_RxCpltCallback(uint8_t* rx_data)
{
    (void)rx_data;

    if (!Initialized)
    {
        return;
    }

    Flag += 1;

    Data_Process();
}

void Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback()
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

void Class_Motor_DJI_C620::Data_Process()
{
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega;
    int16_t tmp_current;

    Struct_Motor_DJI_CAN_Rx_Data* tmp_buffer =
        (Struct_Motor_DJI_CAN_Rx_Data*)FDCAN_Manage_Object->Rx_Buffer.Data;

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

    Rx_Data.Now_Power = power_calculate_c620(
        Param.Power_K_0, Param.Power_K_1, Param.Power_K_2, Param.Power_A,
        Rx_Data.Now_Current, Rx_Data.Now_Omega);

    Rx_Data.Pre_Encoder = tmp_encoder;
}

void Class_Motor_DJI_C620::TIM_Power_Limit_After_Calculate_PeriodElapsedCallback()
{
    if (!Initialized)
    {
        return;
    }

    if (Param.Power_Limit_Status == Motor_DJI_C620_Power_Limit_Status_ENABLE)
    {
        Power_Limit_Control();
    }

    Math_Constrain(&Target_Current, -Current_Max, Current_Max);
    Out = Target_Current * Current_To_Out;

    Output();

    Feedforward_Current = 0.0f;
    Feedforward_Speed = 0.0f;
}

void Class_Motor_DJI_C620::Power_Limit_Control()
{
    Power_Estimate = power_calculate_c620(
        Param.Power_K_0, Param.Power_K_1, Param.Power_K_2, Param.Power_A,
        Target_Current, Rx_Data.Now_Omega);

    if (Power_Estimate > 0.0f)
    {
        if (Power_Factor >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            float a = Param.Power_K_2;
            float b = Param.Power_K_0 * Rx_Data.Now_Omega;
            float c = Param.Power_A + Param.Power_K_1 * Rx_Data.Now_Omega * Rx_Data.Now_Omega - Power_Factor * Power_Estimate;
            float delta = b * b - 4.0f * a * c;

            if (delta < 0.0f)
            {
                Target_Current = 0.0f;
            }
            else
            {
                float h = sqrtf(delta);
                float result_1 = (-b + h) / (2.0f * a);
                float result_2 = (-b - h) / (2.0f * a);

                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((Target_Current > 0.0f && result_1 > 0.0f) || (Target_Current < 0.0f && result_1 < 0.0f))
                    {
                        Target_Current = result_1;
                    }
                    else
                    {
                        Target_Current = result_2;
                    }
                }
                else
                {
                    if (Math_Abs(result_1) < Math_Abs(result_2))
                    {
                        Target_Current = result_1;
                    }
                    else
                    {
                        Target_Current = result_2;
                    }
                }
            }
        }
    }
}

bool Class_Motor_DJI_C620::Is_Initialized() const
{
    return Initialized;
}

float Class_Motor_DJI_C620::Normalize_Angle(float angle)
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

bool Class_Motor_DJI_C620::Is_Finite(float value)
{
    return isfinite(value);
}

}


