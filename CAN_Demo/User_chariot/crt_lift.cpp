#include "crt_lift.h"
void Class_Lift::Init()
{
	Motor_Lift_Left.Init(&hcan2,Motor_DJI_ID_0x201);
    Motor_Lift_Right.Init(&hcan2,Motor_DJI_ID_0x202);
    Motor_Move_Left.Init(&hcan2,Motor_DJI_ID_0x203);
    Motor_Move_Right.Init(&hcan2,Motor_DJI_ID_0x204);

    //PID初始化
    Motor_Lift_Left.PID_Omega.Init(5.0f, 0.0,0.0);
    Motor_Lift_Right.PID_Omega.Init(5.0f, 0.0,0.0);

    PID_Distance_Left.Init(-160.0f, 0.0f, 0.0f, 0.0f, 0.0f, 14.0);
    PID_Distance_Right.Init(-160.0f, 0.0f, 0.0f, 0.0f, 0.0f, 14.0 );

    Motor_Move_Left.PID_Omega.Init(0.5,0.0,0.0);
    Motor_Move_Right.PID_Omega.Init(0.5,0.0,0.0);
}
void Class_Lift::Distance_Caculate()
{
    //计算同步带行程 齿数*节距
    Now_Distance_Left = Motor_Lift_Left.Get_Now_Angle()*Angle_to_Distance - Left_Offset;

    Now_Distance_Right = Motor_Lift_Right.Get_Now_Angle()*Angle_to_Distance - Right_Offset;
}

void Class_Lift::Caliberation()
{
    //速度环校准
    if(!Left_Caliberate_Flag)
    {
        Motor_Lift_Left.Set_Target_Omega(Left_Caliberate_Speed);
    }
    if(!Right_Caliberate_Flag)
    {
        Motor_Lift_Right.Set_Target_Omega(Right_Caliberate_Speed);
    }

    //校准检测
    //
    if(Math_Abs(Now_Distance_Left) >= Distance_Caliberate && Math_Abs(Motor_Lift_Left.Get_Now_Current()) >= Left_Caliberate_Torque)
    {
        Left_Caliberate_Flag=true;
    }
    if(Math_Abs(Now_Distance_Right) >= Distance_Caliberate && Math_Abs(Motor_Lift_Right.Get_Now_Current()) >= Right_Caliberate_Torque)
    {
        Right_Caliberate_Flag=true;
    }
    
    if(Left_Caliberate_Flag)
    {
        Motor_Lift_Left.Set_Target_Omega(0.0);
        if(cnt0 == 0)
        {
            cnt0 = 1;
            Left_Offset = Now_Distance_Left;
        }
    }
    if(Right_Caliberate_Flag)
    {
        
        if(cnt1 == 0)
        {
            cnt1 = 1;
            Right_Offset = Now_Distance_Right;

        }
        Motor_Lift_Right.Set_Target_Omega(0.0);
        
    }

}

 /**
  * @brief 符号函数
  * 
  */
 inline int8_t Sign(float __value)
 {
    int8_t res = 0;
    if(__value>0)
    {
        res = 1;
    }
    else if(__value<0)
    {
        res = -1;
    }
    return res;
 }
/**
 * @brief 角度环速度环的逻辑切换
 * 
 */
 void Class_Lift::PID_Switch()
 {
    float Difference_Left = Math_Abs(Now_Distance_Left - Target_Distance_Left);
    float Difference_Right = Math_Abs(Now_Distance_Right - Target_Distance_Right);

    if(Difference_Left<Switch_Distance)
    {
        Motor_Lift_Left.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Motor_Lift_Left.Set_Target_Angle(Target_Distance_Left/Angle_to_Distance);
    }
    else
    {
        Motor_Lift_Left.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Lift_Left.Set_Target_Omega(Sign(Difference_Left)*Left_Caliberate_Speed);
    }
    if(Difference_Right<Switch_Distance)
    {
        Motor_Lift_Right.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Motor_Lift_Right.Set_Target_Angle(Target_Distance_Right/Angle_to_Distance);
    }
    else
    {
        Motor_Lift_Right.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Lift_Right.Set_Target_Omega(Sign(Difference_Right)*Right_Caliberate_Speed);
    }

    Motor_Lift_Left.TIM_Calculate_PeriodElapsedCallback();
    Motor_Lift_Right.TIM_Calculate_PeriodElapsedCallback();
 }
//暂时不用
 void Class_Lift::TIM_Calculate_PeriodElapsedCallback()
 {
    if(!Left_Caliberate_Flag)
    {
        Caliberation();
    }

    Distance_Caculate();

    PID_Switch();
 }

 void Class_Lift::StateMachine() {
    // 1. 计算当前反馈位移
    Distance_Caculate();
    
    switch(current_state) {
        case LiftState::CALIBRATION:
            // 1. 执行原有的校准逻辑（寻找限位或电流突变）
            Caliberation();
            
            // 2. 检查是否校准完成
            if (Left_Caliberate_Flag && Right_Caliberate_Flag && DR16_Status == Lift_Status) {
                // 校准完成后，重置PID积分项并切换状态
                current_state = LiftState::CONTROL;
            }
            break;

         case LiftState::CONTROL:

            PID_Distance_Left.Set_Now(Now_Distance_Left);
            PID_Distance_Left.Set_Target(Target_Distance_Left);
            PID_Distance_Left.TIM_Adjust_PeriodElapsedCallback();
            Motor_Lift_Left.Set_Target_Omega(PID_Distance_Left.Get_Out());

            PID_Distance_Right.Set_Now(Now_Distance_Right);
            PID_Distance_Right.Set_Target(Target_Distance_Right);
            PID_Distance_Right.TIM_Adjust_PeriodElapsedCallback();
            Motor_Lift_Right.Set_Target_Omega(PID_Distance_Right.Get_Out());
            
            break;
    }
    Motor_Lift_Left.TIM_Calculate_PeriodElapsedCallback();
	Motor_Lift_Right.TIM_Calculate_PeriodElapsedCallback();
}