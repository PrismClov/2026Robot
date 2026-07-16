/**
 * @file dvc_serialscreen.h
 * @brief 串口屏幕驱动头文件
 * @date 2026-05-04
 */

#ifndef DVC_SERIALSCREEN_H
#define DVC_SERIALSCREEN_H

#include "drv_uart.h"
#include "dvc_dwt.h"
#include "dvc_sbus.h"

/* 串口屏状态 --------------------------------------------------------------*/
enum Enum_Screen_Status
{
    SCREEN_STATUS_DISABLE = 0,
    SCREEN_STATUS_ENABLE
};

/* 页面ID定义 --------------------------------------------------------------*/
enum Enum_Screen_Page_ID
{
    SCREEN_PAGE_0 = 0, // 页面0
    SCREEN_PAGE_1,     // 页面1
    SCREEN_PAGE_2,     // 页面2
    SCREEN_PAGE_3,     // 页面3
    SCREEN_PAGE_4,     // 页面4
    SCREEN_PAGE_MAX    // 最大页面数 最多16页，根据需求设置
};

/* 指令类型枚举 ------------------------------------------------------------*/
enum Enum_Screen_Cmd_Type
{
    SCREEN_CMD_JUMP = 0, // 跳转页面指令
    SCREEN_CMD_BOX,      // 画矩形指令
    SCREEN_CMD_SET_NUM,  // 设置数字指令
    SCREEN_CMD_SET_TXT,  // 设置文本指令
    SCREEN_CMD_MAX       // 指令总数
};

/* HFD串口屏驱动类 ---------------------------------------------------------*/
class Class_SerialScreen
{
public:
    void Init(UART_HandleTypeDef *huart);

    inline Enum_Screen_Status Get_Screen_Status();
    inline Enum_Screen_Page_ID Get_Current_Page_ID();
    inline void Set_Screen_Status(Enum_Screen_Status state);
    inline bool Is_Initialized();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void Jump_To_Page(Enum_Screen_Page_ID page_id);

private:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;  // UART管理对象
    Enum_Screen_Status Screen_Status = SCREEN_STATUS_DISABLE; // 串口屏使能状态
    Enum_Screen_Cmd_Type Current_Cmd_Type = SCREEN_CMD_JUMP;  // 当前指令类型
    Enum_Screen_Page_ID Current_Page_ID = SCREEN_PAGE_0;      // 当前页面ID
    bool Initialized = false;                                 // 初始化标志
    bool Power_On_Delay_Complete = false;                     // 上电延时完成标志
    uint32_t Power_On_Time = 0;                               // 上电时间

    char cmd_Jump[32];

    // 检查上电延时
    void Check_Power_On_Delay();
};

/**
 * @brief 获取串口屏使能状态
 *
 * @return Enum_Screen_Status 使能状态
 */
Enum_Screen_Status Class_SerialScreen::Get_Screen_Status()
{
    return Screen_Status;
}

/**
 * @brief 获取当前页面ID
 *
 *
 * @return Enum_Screen_Page_ID 页面ID
 */
Enum_Screen_Page_ID Class_SerialScreen::Get_Current_Page_ID()
{
    return Current_Page_ID;
}

/**
 * @brief
 *
 * @param state 使能状态
 */
void Class_SerialScreen::Set_Screen_Status(Enum_Screen_Status state)
{
    Screen_Status = state;
}

/**
 * @brief 检查是否已初始化
 *
 * @return true 已初始化
 * @return false 未初始化
 */
bool Class_SerialScreen::Is_Initialized()
{
    return Initialized;
}

#endif /* DVC_SERIALSCREEN_H */
