#include "crt_lift.h"


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
