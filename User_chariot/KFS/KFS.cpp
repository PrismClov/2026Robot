#include "KFS.h"

void Class_KFS::Init()
{
    Motor_KFS_Rotate.Init(&hfdcan1, Motor_DJI_ID_0x205, Motor_DJI_Control_Method_ANGLE);
    // 设定PID参数 角度环kp 6.0f 角速度环kd 6.0f
    Motor_KFS_Rotate.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    Motor_KFS_Rotate.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    AIRPUMP_KFS.Init(GPIOC, GPIO_PIN_13);

}

void Class_KFS::TIM_KFS_PeriodElapsedCallback()
{
    Motor_KFS_Rotate.TIM_Calculate_PeriodElapsedCallback();
    Motor_KFS_Rotate.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
    
    switch (KFS_Control_Type)
    {
        case (KFS_Control_Type_Init):
        {
            Motor_KFS_Rotate.Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
            Motor_KFS_Rotate.Set_Target_Current(0.0f);
            AIRPUMP_KFS.AIRPUMP_Close();           
            if(KFS_Yaw_Flag)
            {
                Set_KFS_Control_Type(KFS_Control_Type_CLAMP);
            }
        }
        break;

        case (KFS_Control_Type_CLAMP):
        {
            Motor_KFS_Rotate.Set_Target_Angle(Clamp_Angle);
            float Angle_Error = Math_Abs(Motor_KFS_Rotate.Get_Now_Angle() - Clamp_Angle);
            if(Angle_Error < 0.2f && Motor_KFS_Rotate.Get_Now_Omega() < 0.2f)
            {
                AIRPUMP_KFS.AIRPUMP_Open();
                if(KFS_Yaw_Flag)
                {
                    Set_KFS_Control_Type(KFS_Control_Type_ROTATE);
                }
            }
        }
        break;

        case (KFS_Control_Type_ROTATE):
        {
            Motor_KFS_Rotate.Set_Target_Angle(Rotate_Angle);
            if(KFS_Yaw_Flag)
            {
                Set_KFS_Control_Type(KFS_Control_Type_Init);
            }
        }
        break;

        KFS_Yaw_Flag = false;


    }

    FDCAN_Send_Data(&hfdcan1, 0x1FF, FDCAN1_0x1ff_Tx_Data);
}