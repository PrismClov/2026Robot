/**
 * @file dvc_motor_mksesc.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief N5065电机配置与操作,由于MKSESC电调有自己的PID算法，支持多个环控制，无需在本机计算PID，但需返回角度速度等数据
 * @version 0.1
 * @date 2024-10-19 0.1 24-25赛季定稿
 *
 * @copyright NEUQ-RoboPioneer (c) 2024-2025
 *
 */

 #ifndef DVC_MOTOR_MKSESC_CPP
 #define DVC_MOTOR_MKSESC_CPP
 
 /* Includes ------------------------------------------------------------------*/
 
 #include <math.h>

 #include "alg_pid.h"
 #include "dvc_motor_base.h"
 #include "drv_can.h"
 #include "drv_math.h"
 
 /* Exported macros -----------------------------------------------------------*/
 
 /* Exported types ------------------------------------------------------------*/

 

 
 /**
  * @brief MKSESC电调控制指令，放在CANID拓展部分执行
  *
  */
 enum FDCAN_PACKET_ID
 {
     FDCAN_PACKET_SET_DUTY = 0,
     FDCAN_PACKET_SET_CURRENT,
     FDCAN_PACKET_SET_CURRENT_BRAKE,
     FDCAN_PACKET_SET_RPM,
     FDCAN_PACKET_SET_POS,
     FDCAN_PACKET_FILL_RX_BUFFER,
     FDCAN_PACKET_FILL_RX_BUFFER_LONG,
     FDCAN_PACKET_PROCESS_RX_BUFFER,
     FDCAN_PACKET_PROCESS_SHORT_BUFFER,
     FDCAN_PACKET_STATUS,
     FDCAN_PACKET_SET_CURRENT_REL,
     FDCAN_PACKET_SET_CURRENT_BRAKE_REL,
     FDCAN_PACKET_SET_CURRENT_HANDBRAKE,
     FDCAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
     FDCAN_PACKET_STATUS_2,
     FDCAN_PACKET_STATUS_3,
     FDCAN_PACKET_STATUS_4,
     FDCAN_PACKET_STATUS_5,
 };
 
 
 /**
  * @brief MKSESC电调错误代码
  *
  */
 enum mc_fault_code{
 FAULT_CODE_NONE = 0,
 FAULT_CODE_OVER_VOLTAGE,
 FAULT_CODE_UNDER_VOLTAGE,
 FAULT_CODE_DRV8302,
 FAULT_CODE_ABS_OVER_CURRENT,
 FAULT_CODE_OVER_TEMP_FET,
 FAULT_CODE_OVER_TEMP_MOTOR,
 FAULT_CODE_END
 };
 
 /**
  * @brief MKSESC电机  接受各项原始参数结构体数据包1
  *
  */
 struct Struct_Motor_MKSESC_Rx_Data_Origin_Packet_1{
     int32_t Erpm_Reverse_Origin;
         int16_t Current_Reverse_Origin;
         int16_t Duty_Reverse_Origin;
 };
 
 /**
  * @brief MKSESC电机  接受各项原始参数结构体数据包4
  *
  */
 struct Struct_Motor_MKSESC_Rx_Data_Origin_Packet_4{
     int16_t FET_Temp_Reverse_Origin;
         int16_t Motor_Temp_Reverse_Origin;
         int16_t Total_Current_In_Reverse_Origin;
         int16_t PID_Pos_Reverse_Origin;
 };
 
 /**
  * @brief MKSESC电调返回电机各项实际参数
  *
  */
 struct Struct_Motor_MKSESC_Rx_Data{
     float Now_Omega;
     float Now_Current;
     float Now_Duty;
     float Now_Angle;
 };
 
 /**
  * @brief MKSESC类初始化，具体电机各项参数需要在上位机软件上进行调试
  *
  */
 class Class_Motor_MKSESC : public Class_Motor_Base
 {
 public:
     struct Base_Parameters
     {
         float Wheel_Radius_M = 0.05f;
         float Reduction_Ratio = 1.0f;
         bool Output_Use_TIM_Send_Callback = true;
     };
 
     void Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __FDCAN_Motor_ID,uint8_t __Motor_Pole_Pairs, float __Angle_Max = 12.5f, float __Omega_Max = 25.0f, float __Duty_Max = 10.0f, float __Current_Max = 10.261194f, Enum_Motor_Control_Method __Control_Method = MOTOR_CONTROL_METHOD_CURRENT);

     void Set_Base_Parameters(const Base_Parameters& parameters);
 
     inline float Get_Angle_Max();
 
     inline float Get_Omega_Max();
 
     inline float Get_Duty_Max();
 
     inline float Get_Current_Max();
 
     inline Enum_Motor_Status Get_Status();
 
     inline float Get_Now_Angle();
 
     inline float Get_Now_Omega();
 
     inline float Get_Now_Duty();
 
     inline float Get_Now_Current();
 
     inline float Get_Control_Angle();
 
     inline float Get_Control_Omega();
 
     inline float Get_Control_Duty();
 
     inline float Get_Control_Current();
 
     inline Enum_Motor_Control_Method Get_Control_Method();
 
     inline void Set_Control_Angle(float __Control_Angle);
 
     inline void Set_Control_Omega(float __Control_Omega);
 
     inline void Set_Control_Duty(float __Control_Duty);
 
     inline void Set_Control_Current(float __Control_Current);
 
     inline void Set_Control_Method(Enum_Motor_Control_Method __Method);


     inline void Set_Target_Current(float current) override;

     inline void Set_Target_Speed(float speed) override;

     inline void Set_Target_Position(float position) override;

     inline void Set_Feedback_Current(float current) override;

     inline void Set_Feedback_Speed(float speed) override;

     inline void Set_Feedback_Position(float position) override;

     inline float Get_Current() const override;

     inline float Get_Speed() const override;

     inline float Get_Position() const override;
 
     void FDCAN_RxCpltCallback(uint8_t *Rx_Data);
 
     void TIM_100ms_Alive_PeriodElapsedCallback();
 
     void TIM_Send_PeriodElapsedCallback();

     void Update_Feedback() override;

     void Calculate() override;

     void Output() override;
 
 protected:
     // 初始化相关变量
 
     // 绑定的CAN
     Struct_FDCAN_Manage_Object *FDCAN_Manage_Object = nullptr;
     // 收发数据绑定的CAN ID
     uint32_t FDCAN_Motor_ID;
     // 最大位置
     float Angle_Max;
     // 最大速度
     float Omega_Max;
     // 最大占空比
     float Duty_Max;
     // 最大电流
     float Current_Max;
     //电机极对数
     uint8_t Pole_Pairs;
     // 常量
 
     // 内部变量
 
     // 当前时刻的电机接收flag
     uint32_t Flag = 0;
     // 前一时刻的电机接收flag
     uint32_t Pre_Flag = 0;
 
     // 读变量
 
     // 电机状态
     Enum_Motor_Status Motor_Status = Motor_Status_DISABLE;
     // 电机对外接口信息
     Struct_Motor_MKSESC_Rx_Data Rx_Data;
 
     // 写变量
 
     // 读写变量
 
     //电机控制方式
     Enum_Motor_Control_Method Control_Method = MOTOR_CONTROL_METHOD_CURRENT;
 
     //占空比,-100~100%
     float Control_Duty = 0.0f;
     // 角度, rad, 目标角度
     float Control_Angle = 0.0f;
     // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
     float Control_Omega = 0.0f;
     // 电流, A, EMIT模式是限幅, 其余模式无效
     float Control_Current = 0.0f;
        // 刹车电流用于电机锁零刹车
    float Brake_Current = 3.0f;

     Base_Parameters Base_Param;
     float Base_Target_Current = 0.0f;
     float Base_Target_Speed = 0.0f;
     float Base_Target_Position = 0.0f;
     float Base_Feedback_Current = 0.0f;
     float Base_Feedback_Speed = 0.0f;
     float Base_Feedback_Position = 0.0f;
     bool Base_Initialized = false;
     // 内部函数
 
     void Data_Process();
 
     float Speed_Mps_To_Motor_Radps(float speed_mps) const;

     float Motor_Radps_To_Speed_Mps(float motor_radps) const;

     float Wheel_Rad_To_Motor_Rad(float wheel_rad) const;

     float Motor_Rad_To_Wheel_Rad(float motor_rad) const;

     static bool Is_Finite(float value);

     void Output_CAN_Data();
 };
 
 /* Exported variables --------------------------------------------------------*/
 
 /* Exported function declarations --------------------------------------------*/
 
 /**
  * @brief 获取角度最大值
  *
  * @return float 角度最大值
  */
 inline float Class_Motor_MKSESC::Get_Angle_Max()
 {
     return (Angle_Max);
 }
 
 /**
  * @brief 获取角速度最大值
  *
  * @return float 角速度最大值
  */
 inline float Class_Motor_MKSESC::Get_Omega_Max()
 {
     return (Omega_Max);
 }
 
 /**
  * @brief 获取占空比最大值
  *
  * @return float 占空比最大值
  */
 inline float Class_Motor_MKSESC::Get_Duty_Max()
 {
     return (Duty_Max);
 }
 
 /**
  * @brief 获取电流最大值
  *
  * @return float 电流最大值
  */
 inline float Class_Motor_MKSESC::Get_Current_Max()
 {
     return (Current_Max);
 }
 
 /**
  * @brief 获取电机状态
  *
  * @return Enum_Motor_Status 电机状态
  */
 inline Enum_Motor_Status Class_Motor_MKSESC::Get_Status()
 {
     return (Motor_Status);
 }
 
 /**
  * @brief 获取当前角度
  *
  * @return float 当前角度
  */
 inline float Class_Motor_MKSESC::Get_Now_Angle()
 {
     return (Rx_Data.Now_Angle);
 }
 
 /**
  * @brief 获取当前角速度
  *
  * @return float 当前角速度
 */
 inline float Class_Motor_MKSESC::Get_Now_Omega()
 {
 return (Rx_Data.Now_Omega);
 }
 
 /**
  * @brief 获取当前占空比
  *
  * @return float 当前占空比
  */
 inline float Class_Motor_MKSESC::Get_Now_Duty()
 {
     return (Rx_Data.Now_Duty);
 }
 
 /**
  * @brief 获取当前电流
  *
  * @return float 当前电流
  */
 inline float Class_Motor_MKSESC::Get_Now_Current()
 { 
     return (Rx_Data.Now_Current);
 }
 
 /**
  * @brief 获取电机控制方式
  *
  * @return Enum_Motor_Control_Method 电机控制方式
  */
 inline Enum_Motor_Control_Method Class_Motor_MKSESC::Get_Control_Method()
 {
     return (Control_Method);
 }
 
 /**
  * @brief 获取角度, rad, 目标角度
  *
  * @return float 角度, rad, 目标角度
  */
 inline float Class_Motor_MKSESC::Get_Control_Angle()
 {
     return (Control_Angle);
 }
 
 /**
  * @brief 获取角速度, rad/s
  *
  * @return float 角速度, rad/s
  */
 inline float Class_Motor_MKSESC::Get_Control_Omega()
 {
     return (Control_Omega);
 }
 
 /**
  * @brief 获取占空比
  *
  * @return float 占空比
  */
 inline float Class_Motor_MKSESC::Get_Control_Duty()
 {
     return (Control_Duty);
 }
 
 /**
  * @brief 获取电流, A
  *
  * @return float 电流, A
  */
 inline float Class_Motor_MKSESC::Get_Control_Current()
 {
     return (Control_Current);
 }
 
 /**
  * @brief 设定角度, rad, 目标角度
  *
  * @param __Control_Angle 角度, rad, 目标角度
  */
 inline void Class_Motor_MKSESC::Set_Control_Angle(float __Control_Angle)
 {
     Control_Angle = __Control_Angle;
 }
 
 /**
  * @brief 设定角速度, rad/s
  *
  * @param __Control_Omega 角速度, rad/s
  */
 inline void Class_Motor_MKSESC::Set_Control_Omega(float __Control_Omega)
 {
     Control_Omega = __Control_Omega;
 }
 
 /**
  * @brief 设定占空比
  *
  * @param __Control_Duty 占空比
  */
 inline void Class_Motor_MKSESC::Set_Control_Duty(float __Control_Duty)
 {
     Control_Duty = __Control_Duty;
 }
 
 /**
  * @brief 设定电流, A
  *
  * @param __Control_Current 电流, A
  */
 inline void Class_Motor_MKSESC::Set_Control_Current(float __Control_Current)
 {
     Control_Current = __Control_Current;
 }
 
 /**
  * @brief 设定电机控制方法
  *
  * @param 
  */
inline void Class_Motor_MKSESC::Set_Control_Method(Enum_Motor_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}


inline void Class_Motor_MKSESC::Set_Target_Current(float __Target_Current)
{
    if (Is_Finite(__Target_Current))
    {
        Base_Target_Current = __Target_Current;
        Control_Current = __Target_Current;
    }
}

inline void Class_Motor_MKSESC::Set_Target_Speed(float __Target_Speed)
{
    if (Is_Finite(__Target_Speed))
    {
        Base_Target_Speed = __Target_Speed;
    }
}

inline void Class_Motor_MKSESC::Set_Target_Position(float __Target_Position)
{
    if (Is_Finite(__Target_Position))
    {
        Base_Target_Position = __Target_Position;
    }
}

inline void Class_Motor_MKSESC::Set_Feedback_Current(float __Feedback_Current)
{
    if (Is_Finite(__Feedback_Current))
    {
        Base_Feedback_Current = __Feedback_Current;
    }
}

inline void Class_Motor_MKSESC::Set_Feedback_Speed(float __Feedback_Speed)
{
    if (Is_Finite(__Feedback_Speed))
    {
        Base_Feedback_Speed = __Feedback_Speed;
    }
}

inline void Class_Motor_MKSESC::Set_Feedback_Position(float __Feedback_Position)
{
    if (Is_Finite(__Feedback_Position))
    {
        Base_Feedback_Position = __Feedback_Position;
    }
}

inline float Class_Motor_MKSESC::Get_Current() const
{
    return Base_Feedback_Current;
}

inline float Class_Motor_MKSESC::Get_Speed() const
{
    return Base_Feedback_Speed;
}

inline float Class_Motor_MKSESC::Get_Position() const
{
    return Base_Feedback_Position;
}
 
 #endif
/************************ COPYRIGHT(C) ROBOPIONEER **************************/
