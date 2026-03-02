#include "crt_lift.h"
<<<<<<< HEAD
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

=======


/**
 * @brief 初始化函数
 */
void Class_Lift::Init()
{
	Motor_Lift_L.Init(&hcan2,Motor_DJI_ID_0x201);
    Motor_Lift_R.Init(&hcan2,Motor_DJI_ID_0x202);
    Motor_Move_L.Init(&hcan2,Motor_DJI_ID_0x203);
    Motor_Move_R.Init(&hcan2,Motor_DJI_ID_0x204);

    //PID初始化
    Motor_Lift_L.PID_Omega.Init(5.0f, 0.0,0.0);
    Motor_Lift_R.PID_Omega.Init(5.0f, 0.0,0.0);

    PID_Distance_L.Init(-160.0f, 0.0f, 0.0f, 0.0f, 0.0f, 14.0);
    PID_Distance_R.Init(-160.0f, 0.0f, 0.0f, 0.0f, 0.0f, 14.0 );

    Motor_Move_L.PID_Omega.Init(0.5,0.0,0.0);
    Motor_Move_R.PID_Omega.Init(0.5,0.0,0.0);
}

/*
* @brief 获取同步带行程
*/

void Class_Lift::Distance_Caculate()
{
    //计算同步带行程 齿数*节距
    Now_Distance[0] = Motor_Lift_L.Get_Now_Angle()*Angle_to_Distance - Offset[0];

    Now_Distance[1] = Motor_Lift_R.Get_Now_Angle()*Angle_to_Distance - Offset[1];
}

/*-----------------------校准状态---------------------------*/

/**
 * @brief 校准
 */

void Class_Lift::Caliberate()
{
    //速度环校准
    if(!Caliberate_Flag[0])
    {
        Motor_Lift_L.Set_Target_Omega(Caliberate_Speed[0]);
    }
    if(!Caliberate_Flag[1])
    {
        Motor_Lift_R.Set_Target_Omega(Caliberate_Speed[1]);
    }

    //校准检测
    if(Math_Abs(Now_Distance[0]) >= Distance_Caliberate && Math_Abs(Motor_Lift_L.Get_Now_Current()) >= Caliberate_Torque[0])
    {
        Caliberate_Flag[0] = true;
    }
    if(Math_Abs(Now_Distance[1]) >= Distance_Caliberate && Math_Abs(Motor_Lift_R.Get_Now_Current()) >= Caliberate_Torque[1])
    {
        Caliberate_Flag[1] = true;
    }


}

/*
* @brief 获取校准完成标志
*/
bool Class_Lift::Is_Caliberate_Finished()
{
    return Caliberate_Flag[0] && Caliberate_Flag[1];
}

/*
*  @brief 校准注销
*/
void Class_Lift::Caliberate_Cancel()
 {
    //速度环锁死
    Motor_Lift_L.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);

    Motor_Lift_R.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);


    Motor_Lift_L.Set_Target_Omega(0.0f);
    
    Motor_Lift_R.Set_Target_Omega(0.0f);

 }

/*-----------------------上升状态---------------------------*/

/*
* @brief 上升
*/
void Class_Lift::Up()
{
    

}


/**
 * @brief 查看上升是否完成
 */
bool Class_Lift::Is_Up_Finished()
{
    return false;
}

/**
 * @brief 上升注销
 */
 void Class_Lift::Up_Cancel()
 {
    
 }


/*-----------------------移动状态---------------------------*/
/** 
* @brief 移动
*/
void Class_Lift::Move()
{

}

/**
* @brief 查看移动是否完成
*/
bool Class_Lift::Is_Move_Finished()
{
   return false;
}

/**
* @brief 移动注销
*/
void Class_Lift::Move_Cancel()
{

}


/*-----------------------下降状态---------------------------*/
/**
* @brief 下降
*/
void Class_Lift::Down()
{

}

/**
* @brief 查看下降是否完成
*/
bool Class_Lift::Is_Down_Finished()
{
    return false;
}


/**
* @brief 下降注销
*/

void Class_Lift::Down_Cancel()
{

}


/*-----------------------定时器回调函数---------------------------*/
/**
 * @brief 定时器回调函数
 */
 void Class_Lift::TIM_Calculate_PeriodElapsedCallback()
 {
    //实时计算同步带行程
    Distance_Caculate();

    //状态机回调
    FSM_Lift.Lift_TIM_Status_PeriodElapsedCallback();

    //PID回调函数
    

 }

 void Class_Lift_FSM::Lift_TIM_Status_PeriodElapsedCallback()
 {
    Status[Now_Status_Serial].Count_Time++;

    switch(Lift_Status)
    {
        case Lift_Status_INIT :
        {
            Lift->Caliberate();

            //查看校准是否完成
            if(Lift->Is_Caliberate_Finished())
            {
                //设置同步带偏移
                Lift->Set_Offset(Lift->Get_Now_Distance_L(),Lift->Get_Now_Distance_R());
                
                Lift->Caliberate_Cancel();

                Lift_Status = Lift_Status_UP;
                
                Set_Status(1);
            }
        }

        break;

        case Lift_Status_UP :
        {
            Lift->Up();

            if(Lift->Is_Up_Finished())
            {   
                Lift->Up_Cancel();

                Lift_Status = Lift_Status_MOVING;

                Set_Status(2);
            }
        }

        break;

        case Lift_Status_MOVING :
        {
            Lift->Move();

            if(Lift->Is_Move_Finished())
            {
                Lift->Move_Cancel();

                Lift_Status = Lift_Status_DOWN;

                Set_Status(3);
            }
        }

        break;
        
        case Lift_Status_DOWN :
        {
            Lift->Down();

            if(Lift->Is_Down_Finished())
            {
                Lift->Down_Cancel();

                Lift_Status = Lift_Status_UP;

                Set_Status(1);
            }

        }

        break;

    }
}
 
>>>>>>> d6d6a17 (R2_Code)
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
<<<<<<< HEAD
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
=======
 }
>>>>>>> d6d6a17 (R2_Code)
