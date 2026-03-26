#include "KFS.h"

void Class_KFS::Init()
{
    Motor_KFS_Rotate.Init(&hfdcan1, Motor_DJI_ID_0x205, Motor_DJI_Control_Method_ANGLE);

    Motor_KFS_Rotate.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);
    
    AIRPUMP_KFS.Init(GPIOC, GPIO_PIN_13);

}

void Class_KFS::TIM_KFS_PeriodElapsedCallback()
{
    switch (KFS_Control_Type)
    {
        case (KFS_Control_Type_DISABLE):
        {
            Motor_KFS_Rotate.Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
            Motor_KFS_Rotate.Set_Target_Current(0.0f);
            AIRPUMP_KFS.AIRPUMP_Close();           
        }
        break;

        case (KFS_Control_Type_CLAMP):
        {
            Motor_KFS_Rotate.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
            Motor_KFS_Rotate.Set_Target_Angle(Target_Angle_KFS);
            AIRPUMP_KFS.AIRPUMP_Open();
            
        }
        break;

        default:
            break;
        }
}