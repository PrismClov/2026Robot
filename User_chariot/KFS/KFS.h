#ifndef KFS_H
#define KFS_H

/*--------------------includes---------------------*/

#include "dvc_motor_dji.h"
#include "dvc_airtool.h"
#include "drv_can.h"

/*--------------------enums----------------------*/

enum Enum_KFS_Control_Type
{
    KFS_Control_Type_Init = 0,
    KFS_Control_Type_CLAMP,
    KFS_Control_Type_ROTATE,
};
 
class Class_KFS
{
    public: 
    
    void Init();
    
    //翻转KFS
    Class_Motor_DJI_C620 Motor_KFS_Rotate;
    //气缸吸取KFS
    Class_AIRPUMP AIRPUMP_KFS;

    void TIM_KFS_PeriodElapsedCallback();

    inline void KFS_Yaw_Flag_True();


    inline void Set_KFS_Control_Type(Enum_KFS_Control_Type __Control_Type);

    private:

    bool KFS_Yaw_Flag = false;

    Enum_KFS_Control_Type KFS_Control_Type = KFS_Control_Type_Init;
    
    //KFS目标翻转角度
    float Clamp_Angle = 0.0f;
    
    //KFS目标旋转角度
    float Rotate_Angle = 0.0f;
};
/*--------------------macros-----------------------*/

/*--------------------types------------------------*/

/*--------------------variables--------------------*/

inline void Class_KFS::Set_KFS_Control_Type(Enum_KFS_Control_Type __Control_Type)
{
    KFS_Control_Type = __Control_Type;
}


inline void Class_KFS::KFS_Yaw_Flag_True()
{
    KFS_Yaw_Flag = true;
}
/******************* Wulin *****END OF FILE****/

#endif
