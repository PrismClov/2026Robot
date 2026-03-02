#ifndef CRT_LIFT_H
#define CRT_LIFT_H

#include "dvc_motor_dji.h"
enum class LiftState {
    CALIBRATION, // 寻找机械零点/归零
    CONTROL      // 闭环位置控制
};
enum DR16_Status
{
    Caliberate,
    Lift_Status
};
class Class_Lift
{ 
    public:
    //3508抬升轮
    Class_Motor_DJI_C620 Motor_Lift_Left;
    Class_Motor_DJI_C620 Motor_Lift_Right;
    //2006爬行轮
    Class_Motor_DJI_C610 Motor_Move_Left;
    Class_Motor_DJI_C610 Motor_Move_Right;
    Class_PID PID_Distance_Left;
    Class_PID PID_Distance_Right;

    DR16_Status DR16_Status = Caliberate;

    void Init();

    void Caliberation();
    
    void Distance_Caculate();

    void PID_Switch();

    void TIM_Calculate_PeriodElapsedCallback();

    inline void Set_Target_Distance(float Distance_Limit);

    inline float Get_Target_Distance_Limit();

    inline float Get_Now_Distance_Left();
    inline float Get_Now_Distance_Right();
    inline float Get_Target_Distance_Left();
    inline float Get_Target_Distance_Right();
    inline bool Get_Caliberate_Flag();
    void StateMachine();

    private:

    uint8_t cnt0 = 0, cnt1 = 0;
    LiftState current_state = LiftState::CALIBRATION; // 默认先校准
    // ... 其他变量
   
    //同步带行程
    float Now_Distance_Left = 0.0f;
    float Now_Distance_Right = 0.0f;

    float Target_Distance_Left = 0.0f;
    float Target_Distance_Right = 0.0f;

    float Target_Distance_Limit = 0.40f;
    //
    float Switch_Distance = 0.02;
    
    //校准距离 贴地时剩余行程3cm
    float Distance_Caliberate = 0.03f;

    //校准标志位
    bool Left_Caliberate_Flag = false;
    bool Right_Caliberate_Flag = false;
    //校准速度
    float Left_Caliberate_Speed = - 5.0f;
    float Right_Caliberate_Speed = - 5.0f;
    
    float Left_Caliberate_Torque = 6.5f;
    float Right_Caliberate_Torque = 6.5f;

    //齿轮减速比
    float Gearbox_Rate = 1.5;
    //同步带轮直径
    float Diameter = 0.05f;
    //同步带轮齿数
    float Tooth_Number = 32.0f;
    //同步带正方向
    int8_t Belt_Sign = -1;
    //同步带节距
    float Step = 0.005;  
    //角度转换为同步带行程
    float Angle_to_Distance = Belt_Sign*Tooth_Number*Step/(2*PI*Gearbox_Rate);

    float Left_Offset = 0.0f; 
    float Right_Offset = 0.0f;

    float Control_Omega = 1.0f;

};
inline void Class_Lift::Set_Target_Distance(float __distance)
{
    Target_Distance_Left = __distance;
    Target_Distance_Right = __distance;
}

inline float Class_Lift::Get_Target_Distance_Limit()
{
    return Target_Distance_Limit;
}

inline bool Class_Lift::Get_Caliberate_Flag()
{
    return (Left_Caliberate_Flag&&Right_Caliberate_Flag);
}

inline float Class_Lift::Get_Now_Distance_Left()
{
    return Now_Distance_Left;
}
inline float Class_Lift::Get_Now_Distance_Right()
{
    return Now_Distance_Right;
}
inline float Class_Lift::Get_Target_Distance_Left()
{
    return Target_Distance_Left;
}
inline float 
Class_Lift::Get_Target_Distance_Right()
{
    return Target_Distance_Right;
}
#endif //  CRT_LIFT_H