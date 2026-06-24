#include "dvc_motor_base.h"
#include "drv_math.h"

bool Class_Motor_Base::Calibrate(const Calibrate_Params &params, float &offset)
{
    Update_Feedback();

    uint32_t now = DWT_GetCurrentTimeUs();

    // 设置运动模式
    switch (params.motion_mode)
    {
        case CALIBRATE_MOTION_NONE:
            offset = Get_Position();
            Stall_Debounce_Start_Time = 0;
            return true;

        case CALIBRATE_MOTION_SPEED:
            Set_Control_Method(MOTOR_CONTROL_METHOD_SPEED);
            Set_Target_Speed(params.motion_value);
            break;

        case CALIBRATE_MOTION_CURRENT:
            Set_Control_Method(MOTOR_CONTROL_METHOD_CURRENT);
            Set_Target_Current(params.motion_value);
            break;
    }

    // 堵转条件检测
    bool condition_met = false;
    switch (params.detect_mode)
    {
        case CALIBRATE_DETECT_CURRENT:
            condition_met = Math_Abs(Get_Current()) >= params.detect_threshold;
            break;
        case CALIBRATE_DETECT_SPEED:
            condition_met = Math_Abs(Get_Speed()) <= params.detect_threshold;
            break;
    }

    // 消抖: 条件需持续满足 debounce_us 时长
    if (condition_met)
    {
        if (Stall_Debounce_Start_Time == 0)
            Stall_Debounce_Start_Time = now;

        if (now - Stall_Debounce_Start_Time >= params.debounce_us)
        {
            offset = Get_Position();
            Stall_Debounce_Start_Time = 0;
            return true;
        }
    }
    else
    {
        Stall_Debounce_Start_Time = 0;
    }

    return false;
}
