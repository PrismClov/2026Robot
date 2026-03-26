#ifndef KFS_H
#define KFS_H

/*--------------------includes---------------------*/

#include "dvc_motor_dji.h"
#include "dvc_airtool.h"
#include "drv_can.h"

/*--------------------enums----------------------*/

enum Enum_KFS_Control_Type
{
    KFS_Control_Type_DISABLE = 0,
    KFS_Control_Type_CLAMP,
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

    inline void Set_KFS_Control_Type(Enum_KFS_Control_Type __Control_Type);

    private:

    Enum_KFS_Control_Type KFS_Control_Type = KFS_Control_Type_DISABLE;
    
    //KFS目标翻转角度
    float Target_Angle_KFS = 0.0f;
    

};
/*--------------------macros-----------------------*/

/*--------------------types------------------------*/

/*--------------------variables--------------------*/

inline void Class_KFS::Set_KFS_Control_Type(Enum_KFS_Control_Type __Control_Type)
{
    KFS_Control_Type = __Control_Type;
}
/******************* Wulin *****END OF FILE****/

#endif
