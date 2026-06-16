#ifndef CRT_LIFT_H
#define CRT_LIFT_H
/*---------------------includes-------------------------*/
#include "dvc_motor_dji.h"
#include "alg_fsm.h"
#include "dvc_dr16.h"
#include "dvc_dwt.h"
/*---------------------变量声明-------------------------*/
class Class_Lift;
/*---------------------枚举-------------------------*/
/**
 * @brief 抬升状态
 * 
 */

enum Enum_Lift_Status
{
    Lift_Status_INIT = 0,   //初始状态
    Lift_Status_Wait_R2,       //等待R2
    Lift_Status_Lift_R2,         //抬起R2
    Lift_Status_Down_R2,          //放下R2 ：是否要与等待R2状态合并
    
};

/**
 * @brief 抬升控制模式
 * 
 */
enum Enum_Lift_Control_Mode 
{
    MODE_SPEED_OPEN,   // 速度开环模式
    MODE_POSITION_CLOSE // 位置闭环模式
};

/**
 * @brief 抬升状态机
 * 
 */

class Class_FSM_Lift : public Class_FSM
{
    public:
    
    Class_Lift *Lift;

    void Lift_TIM_Status_PeriodElapsedCallback();

    Enum_Lift_Status Lift_Status =  Lift_Status_INIT;

};



/**
 * @brief 抬升
 * 
 */
class Class_Lift
{ 
    public:

    //控制状态
    Enum_Lift_Control_Mode Lift_Control_Mode_L = MODE_POSITION_CLOSE;
    Enum_Lift_Control_Mode Lift_Control_Mode_R = MODE_POSITION_CLOSE;
    //抬升状态机
    Class_FSM_Lift FSM_Lift;
    friend class Class_FSM_Lift;
    //行程环
    Class_PID PID_Distance_L;
    Class_PID PID_Distance_R;
    
    //3508同步带抬升
    Motor::Class_Motor_DJI_C620 Motor_Lift_L;
    Motor::Class_Motor_DJI_C620 Motor_Lift_R;

/*-----------------------全流程函数---------------------------*/
    //行程计算
    void Distance_Caculate();

    void TIM_Calculate_PeriodElapsedCallback();

/*-----------------------状态函数---------------------------*/
    void Init();

    //校准
    void Calibrate();
    
    //通用抬升函数
    void Up(float Target_Distance_L, float Target_Distance_R);

    //下降
    //void Down();

/*-----------------------接口---------------------------*/
    inline void Set_Target_Distance(float Distance_Limit);

    inline float Get_Target_Distance_Limit();

    //获取左侧同步带当前及目标距离
    inline float Get_Now_Distance_L();

    inline float Get_Target_Distance_L();
    
    //获取右侧同步带当前及目标距离
    inline float Get_Now_Distance_R();

    inline float Get_Target_Distance_R();

    //是否校准完成
    bool Is_Calibrate_Finished_Open();

    //是否上升完成
    bool Is_Wait_R2_Finished_step();

    //是否抬起R2完成
    bool Is_Lift_R2_Finished_step();

    //是否放下R2完成
    bool Is_Down_R2_Finished_step();

    //是否下降完成
    //bool Is_Down_Finished();

/*-----------------------状态注销---------------------------*/
    //校准状态注销
    void Calibrate_Cancel();    

    //通用抬升注销函数
    void UP_Cancel();
    
    //下降状态注销

    //void Down_Cancel();

    //设置同步带校准偏差
    inline void Set_Offset(float __offset_l, float __offset_r);

    inline void Yaw_Flag_True();

    inline float Get_Velocity_Max();

    //抬升存活检测
    void TIM_100ms_Alive_PeriodElapsedCallback();
    //抬升电机堵转检测函数
    bool Check_Motor_Block();

    
    private:

    bool Yaw_Flag = false; //是否进行yaw控制的标志位，true为进行，false为不进行
    bool Block_Flag = false; //电机是否堵转的标志位，true为堵转，false为不堵转

/*-----------------------电控参数---------------------------*/ 
    float Max_Velocity = 200.0f; 
    //同步带行程 单位m
    float Now_Distance[2] = { 0.0f, 0.0f };

    //同步带上升目标距离，下降时加了负号
    float Target_Distance_Wait_R2[2] = { - 0.40f, - 0.40f };

    float Target_Distance_Lift_R2[2] = { - 0.15f, - 0.15f };

    float Target_Distance_Down_R2[2] = { - 0.40f, - 0.40f };

    float Target_Speed_Open[2] = { 5.0f, 5.0f };

    float Target_Distance_Limit = 0.55f;

    float Distance_Error = 0.008f;

    // 位置误差阈值1cm(用于位置环与速度环的切换)
    float Position_Threshold = 0.01f;

    //校准距离 贴地时剩余行程3cm
    float Distance_Caliberate = 0.03f;

    //校准标志位
    bool Caliberate_Flag[2] = { false, false };

    bool Lift_Block_Flag [2] = { false, false };

    //开环校准输出
    float Caliberate_Output = -2800.0f;

    //闭环校准输出

    float Calibrate_Output[2] = { -5.0f, -5.0f };
    float Calibrate_Out[2] = { 1.0f, 1.0f };

    //偏移量
    float Offset[2] = { 0.0f, 0.0f };

    //闭环校准行程:该行程内使用闭环校准 数值待确定
    float Caliberate_Distance_Close = 0.400f;//设置成0.00f表示不使用闭环校准 >0.500表示全程使用闭环校准 



/*-----------------------机械参数---------------------------*/
   //齿轮减速比
     float Gearbox_Rate = 2.50f/1.0f;
    //同步带轮直径
    float Diameter = 0.05f; 
    //同步带轮齿数
    float Tooth_Number = 34.0f;//下同步轮
    //同步带正方向
    int8_t Belt_Sign = -1;
    //同步带节距
    float Step = 0.005f;  
    //角度转换为同步带行程
    float Angle_to_Distance = Belt_Sign * Tooth_Number * Step / ( 2 * PI * Gearbox_Rate);
};

/*
* @brief 设置目标距离
*
*/

inline void Class_Lift::Set_Target_Distance(float __distance)
{
    //Target_Distance[0] = __distance;
    //Target_Distance[1] = __distance;
}

/*
* @brief 获取目标距离
*
*/

inline float Class_Lift::Get_Target_Distance_Limit()
{
    return Target_Distance_Limit;
}



/*
* @brief 获取左侧同步带当前距离
*/

inline float Class_Lift::Get_Now_Distance_L()
{
    return Now_Distance[0];
}

/*
* @brief 获取右侧同步带当前距离
*/

inline float Class_Lift::Get_Now_Distance_R()
{
    return Now_Distance[1];
}

/*
* @brief 获取左侧同步带目标距离
*/

inline float Class_Lift::Get_Target_Distance_L()
{
    //return Target_Distance[0];
}
inline float 
Class_Lift::Get_Target_Distance_R()
{
    //return Target_Distance[1];
}


/*
* @brief 设置同步带校准偏差
*/
inline void Class_Lift::Set_Offset(float __offset_l, float __offset_r)
{
    Offset[0] = __offset_l;
    Offset[1] = __offset_r;
}


/**
 * @brief 设置yaw控制标志位
 * 
 */
inline void Class_Lift::Yaw_Flag_True()
{
    Yaw_Flag = true;
}


/*. * @brief 获取最大速度
 * 
 */
inline float Class_Lift::Get_Velocity_Max()
{
    return Max_Velocity;
}
#endif //  CRT_LIFT_H