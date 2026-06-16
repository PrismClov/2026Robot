#include "dvc_motor_base.h"
#include "drv_math.h"

bool Class_Motor_Base::Calibrate(float speed, float current_threshold, float &offset)
{
    Update_Feedback();
    Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
    Set_Target_Speed(speed);

    if (Math_Abs(Get_Current()) >= current_threshold)
    {
        Set_Target_Speed(0.0f);
        offset = Get_Position();
        return true;
    }
    return false;
}
