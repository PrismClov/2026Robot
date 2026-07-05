/**
 * @file dvc_serialscreen.cpp
 * @brief 串口屏幕驱动源文件
 * @date 2026-05-04
 */

#include "dvc_serialscreen.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief 初始化串口屏幕驱动
 *
 * @param huart UART句柄
 */
void Class_SerialScreen::Init(UART_HandleTypeDef *huart) 
{
    if (huart == nullptr) {
        Initialized = false;
        return;
    }
    
    // 绑定UART管理对象
    if (huart->Instance == USART1) {
        UART_Manage_Object = &UART1_Manage_Object;
    } else if (huart->Instance == USART2) {
        UART_Manage_Object = &UART2_Manage_Object;
    } else if (huart->Instance == USART3) {
        UART_Manage_Object = &UART3_Manage_Object;
    } else if (huart->Instance == UART4) {
        UART_Manage_Object = &UART4_Manage_Object;
    } else if (huart->Instance == UART5) {
        UART_Manage_Object = &UART5_Manage_Object;
    } else if (huart->Instance == USART6) {
        UART_Manage_Object = &UART6_Manage_Object;
    } else if (huart->Instance == UART7) {
        UART_Manage_Object = &UART7_Manage_Object;
    } else if (huart->Instance == UART8) {
        UART_Manage_Object = &UART8_Manage_Object;
    } else if (huart->Instance == UART9) {
        UART_Manage_Object = &UART9_Manage_Object;
    } else if (huart->Instance == USART10) {
        UART_Manage_Object = &UART10_Manage_Object;
    } else {
        Initialized = false;
        return;
    }
    
    Power_On_Time = DWT_GetCurrentTimeMs();
    Power_On_Delay_Complete = false;
    Initialized = true;
}

/**
 * @brief  检查上电延时
 *
 */
void Class_SerialScreen::Check_Power_On_Delay() 
{
    if (!Power_On_Delay_Complete) {
        uint32_t current_time = DWT_GetCurrentTimeMs();
        if (current_time - Power_On_Time >= 1000)  // 等待1秒
        {  
            Power_On_Delay_Complete = true;
        }
    }
}

/**
 * @brief 检测串口屏存活
 *
 */
void Class_SerialScreen::TIM_100ms_Alive_PeriodElapsedCallback() 
{
    if (!Initialized) return;
    
    // 检查上电延时
    Check_Power_On_Delay();
    if (!Power_On_Delay_Complete) return;

    Screen_Status = SCREEN_STATUS_ENABLE;
}

/**
 * @brief 跳转到指定页面
 *
 * @param page_id 页面ID
 */
void Class_SerialScreen::Jump_To_Page(Enum_Screen_Page_ID page_id) 
{
    memset(cmd_Jump, 0, sizeof(cmd_Jump)); // 清空命令缓冲区

    if (!Initialized || UART_Manage_Object == nullptr) return;

    snprintf(cmd_Jump , sizeof(cmd_Jump) , "JUMP(%d);\r\n" , page_id);

    UART_Send_Char_Data(UART_Manage_Object->UART_Handler , cmd_Jump , strlen(cmd_Jump));

    Current_Page_ID = page_id;
}
 


 