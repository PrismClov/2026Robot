/**
 * @file dvc_sbus.cpp
 */

#include "dvc_sbus.h"
#include "tsk_config_and_callback.h"
#include "drv_math.h"

#include <string.h>

#define SBUS_FRAME_HEADER (0x0FU)
#define SBUS_FRAMELOST_MASK (0x04U)
#define SBUS_FAILSAFE_MASK (0x08U)

#define SBUS_CHANNEL_MID (992U)
#define SBUS_CHANNEL_MAX (1811U)
#define SBUS_CHANNEL_ABSOLUTE_MAX (2047U)
#define SBUS_SWITCH_LOW_THRESHOLD (700U)
#define SBUS_SWITCH_HIGH_THRESHOLD (1300U)

#define SBUS_SIGN_RIGHT_X (1.0f)
#define SBUS_SIGN_RIGHT_Y (1.0f)
#define SBUS_SIGN_LEFT_X (1.0f)
#define SBUS_SIGN_LEFT_Y (1.0f)
#define SBUS_SIGN_VRA (1.0f)
#define SBUS_SIGN_VRB (1.0f)

bool Is_Likely_SBUS_Footer(uint8_t footer)
{
    return (footer == 0x00U) || ((footer & 0x0FU) == 0x04U);
}

uint16_t Extract_SBUS_Channel(Struct_SBUS_UART_Rx_Data *raw_frame, uint8_t channel_index)
{
    uint8_t *frame = (uint8_t *)raw_frame;
    uint32_t bit_index = (uint32_t)channel_index * 11U;
    uint32_t byte_index = 1U + (bit_index / 8U);
    uint32_t shift = bit_index % 8U;
    uint32_t packed = (uint32_t)frame[byte_index] |
                      ((uint32_t)frame[byte_index + 1U] << 8U) |
                      ((uint32_t)frame[byte_index + 2U] << 16U);

    return (uint16_t)((packed >> shift) & 0x07FFU);
}

void Class_SBUS::Init(UART_HandleTypeDef *huart_1)
{
    if (huart_1->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart_1->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart_1->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart_1->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart_1->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart_1->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }

    Rocker_Offset = (float)SBUS_CHANNEL_MID;
    Rocker_Num = (float)(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MID);
    Reset_Control_Data();
}

void Class_SBUS::Reset_Control_Data()
{
    memset(&Now_UART_Rx_Data, 0, sizeof(Now_UART_Rx_Data));
    memset(&Pre_UART_Rx_Data, 0, sizeof(Pre_UART_Rx_Data));
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(&Pre_Rx_Data, 0, sizeof(Pre_Rx_Data));

    Switch_A_Status = SWITCH_UP;
    Switch_B_Status = SWITCH_UP;
    Switch_C_Status = SWITCH_UP;
    Switch_D_Status = SWITCH_UP;
    Pre_Switch_A_Status = SWITCH_UP;
    Pre_Switch_B_Status = SWITCH_UP;
    Pre_Switch_C_Status = SWITCH_UP;
    Pre_Switch_D_Status = SWITCH_UP;

    Rx_Data.Switch_A = SBUS_Switch_Status_UP;
    Rx_Data.Switch_B = SBUS_Switch_Status_UP;
    Rx_Data.Switch_C = SBUS_Switch_Status_UP;
    Rx_Data.Switch_D = SBUS_Switch_Status_UP;
    Pre_Rx_Data.Switch_A = SBUS_Switch_Status_UP;
    Pre_Rx_Data.Switch_B = SBUS_Switch_Status_UP;
    Pre_Rx_Data.Switch_C = SBUS_Switch_Status_UP;
    Pre_Rx_Data.Switch_D = SBUS_Switch_Status_UP;
    SBUS_Failsafe_Active = false;
}

void Class_SBUS::Invalidate_Control_Data()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    Switch_A_Status = SWITCH_UP;
    Switch_B_Status = SWITCH_UP;
    Switch_C_Status = SWITCH_UP;
    Switch_D_Status = SWITCH_UP;
    Rx_Data.Switch_A = SBUS_Switch_Status_UP;
    Rx_Data.Switch_B = SBUS_Switch_Status_UP;
    Rx_Data.Switch_C = SBUS_Switch_Status_UP;
    Rx_Data.Switch_D = SBUS_Switch_Status_UP;
    SBUS_Update_Status = SBUS_Status_DisUpdate;
}

bool Class_SBUS::Is_Channel_Valid(uint16_t value) const
{
    return (value <= SBUS_CHANNEL_ABSOLUTE_MAX);
}

uint16_t Class_SBUS::Get_Channel_Value(uint8_t channel_index) const
{
    if (channel_index == 0U || channel_index > SBUS_MAX_CHANNELS)
    {
        return SBUS_CHANNEL_MID;
    }

    uint16_t value = Rx_Data.Channel[channel_index - 1U];
    if (Is_Channel_Valid(value) == false)
    {
        return SBUS_CHANNEL_MID;
    }

    return value;
}

float Class_SBUS::Normalize_Channel(uint16_t value, float sign) const
{
    float output = ((float)value - Rocker_Offset) / Rocker_Num;
    Math_Constrain(&output, -1.0f, 1.0f);
    return (output * sign);
}

uint8_t Class_SBUS::Decode_Two_Position_Switch_Channel(uint16_t value, uint8_t fallback_status) const
{
    if (Is_Channel_Valid(value) == false)
    {
        return fallback_status;
    }

    if (value >= SBUS_SWITCH_HIGH_THRESHOLD)
    {
        return SWITCH_UP;
    }
    if (value <= SBUS_SWITCH_LOW_THRESHOLD)
    {
        return SWITCH_DOWN;
    }

    return fallback_status;
}

uint8_t Class_SBUS::Decode_Three_Position_Switch_Channel(uint16_t value, uint8_t fallback_status) const
{
    if (Is_Channel_Valid(value) == false)
    {
        return fallback_status;
    }

    if (value >= SBUS_SWITCH_HIGH_THRESHOLD)
    {
        return SWITCH_UP;
    }
    if (value <= SBUS_SWITCH_LOW_THRESHOLD)
    {
        return SWITCH_DOWN;
    }

    return SWITCH_MIDDLE;
}

void Class_SBUS::Judge_Three_Position_Switch(Enum_SBUS_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    switch (Pre_Status)
    {
    case SWITCH_UP:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = SBUS_Switch_Status_UP;
            break;
        case SWITCH_DOWN:
            *Switch = SBUS_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case SWITCH_MIDDLE:
            *Switch = SBUS_Switch_Status_TRIG_UP_MIDDLE;
            break;
        }
        break;

    case SWITCH_DOWN:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = SBUS_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case SWITCH_DOWN:
            *Switch = SBUS_Switch_Status_DOWN;
            break;
        case SWITCH_MIDDLE:
            *Switch = SBUS_Switch_Status_TRIG_DOWN_MIDDLE;
            break;
        }
        break;

    case SWITCH_MIDDLE:
    default:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = SBUS_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case SWITCH_DOWN:
            *Switch = SBUS_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case SWITCH_MIDDLE:
        default:
            *Switch = SBUS_Switch_Status_MIDDLE;
            break;
        }
        break;
    }
}

void Class_SBUS::Judge_Update(const Struct_SBUS_Rx_Data &Pre_Data, const Struct_SBUS_Rx_Data &Now_Data)
{
    if (Pre_Data.Channel[SBUS_CH_RIGHT_X - 1U] == Now_Data.Channel[SBUS_CH_RIGHT_X - 1U] &&
        Pre_Data.Channel[SBUS_CH_RIGHT_Y - 1U] == Now_Data.Channel[SBUS_CH_RIGHT_Y - 1U] &&
        Pre_Data.Channel[SBUS_CH_LEFT_X - 1U] == Now_Data.Channel[SBUS_CH_LEFT_X - 1U] &&
        Pre_Data.Channel[SBUS_CH_LEFT_Y - 1U] == Now_Data.Channel[SBUS_CH_LEFT_Y - 1U] &&
        Pre_Data.Channel[SBUS_CH_YAW - 1U] == Now_Data.Channel[SBUS_CH_YAW - 1U] &&
        Pre_Data.Channel[SBUS_CH_VRB - 1U] == Now_Data.Channel[SBUS_CH_VRB - 1U] &&
        Pre_Switch_A_Status == Switch_A_Status &&
        Pre_Switch_B_Status == Switch_B_Status &&
        Pre_Switch_C_Status == Switch_C_Status &&
        Pre_Switch_D_Status == Switch_D_Status)
    {
        SBUS_Update_Status = SBUS_Status_DisUpdate;
    }
    else
    {
        SBUS_Update_Status = SBUS_Status_Update;
    }
}

bool Class_SBUS::Decode_SBUS_Frame(uint8_t *Rx_Buffer, uint16_t Length)
{
    if (Length < SBUS_FRAME_LENGTH)
    {
        return false;
    }

    int latest_valid_offset = -1;

    for (uint16_t offset = 0; offset + SBUS_FRAME_LENGTH <= Length; offset++)
    {
        uint8_t *frame = &Rx_Buffer[offset];
        if (frame[0] != SBUS_FRAME_HEADER)
        {
            continue;
        }
        if (Is_Likely_SBUS_Footer(frame[24]) == false)
        {
            continue;
        }

        latest_valid_offset = (int)offset;
    }

    if (latest_valid_offset < 0)
    {
        return false;
    }

    memcpy(&Now_UART_Rx_Data, &Rx_Buffer[latest_valid_offset], sizeof(Now_UART_Rx_Data));

    return true;
}

void Class_SBUS::SBUS_Data_Process()
{
    for (uint8_t i = 0U; i < 16U; i++)
    {
        Rx_Data.Channel[i] = Extract_SBUS_Channel(&Now_UART_Rx_Data, i);
    }

    Rx_Data.Flags = Now_UART_Rx_Data.Flags;
    Rx_Data.Channel[16] = (Rx_Data.Flags & 0x01U) ? SBUS_CHANNEL_ABSOLUTE_MAX : 0U;
    Rx_Data.Channel[17] = (Rx_Data.Flags & 0x02U) ? SBUS_CHANNEL_ABSOLUTE_MAX : 0U;
    Rx_Data.Frame_Lost = ((Rx_Data.Flags & SBUS_FRAMELOST_MASK) != 0U);
    Rx_Data.Failsafe = ((Rx_Data.Flags & SBUS_FAILSAFE_MASK) != 0U);

    Rx_Data.Right_X = Normalize_Channel(Get_Channel_Value(SBUS_CH_RIGHT_X), SBUS_SIGN_RIGHT_X);
    Rx_Data.Right_Y = Normalize_Channel(Get_Channel_Value(SBUS_CH_RIGHT_Y), SBUS_SIGN_RIGHT_Y);
    Rx_Data.Left_X = Normalize_Channel(Get_Channel_Value(SBUS_CH_LEFT_X), SBUS_SIGN_LEFT_X);
    Rx_Data.Left_Y = Normalize_Channel(Get_Channel_Value(SBUS_CH_LEFT_Y), SBUS_SIGN_LEFT_Y);
    Rx_Data.VRA = Normalize_Channel(Get_Channel_Value(SBUS_CH_VRA), SBUS_SIGN_VRA);
    Rx_Data.VRB = Normalize_Channel(Get_Channel_Value(SBUS_CH_VRB), SBUS_SIGN_VRB);
    // Keep the legacy yaw semantic on CH5/VRA for upper-layer compatibility.
    Rx_Data.Yaw = Rx_Data.VRA;

    Switch_A_Status = Decode_Two_Position_Switch_Channel(Get_Channel_Value(SBUS_CH_SWITCH_A), Pre_Switch_A_Status);
    Switch_B_Status = Decode_Two_Position_Switch_Channel(Get_Channel_Value(SBUS_CH_SWITCH_B), Pre_Switch_B_Status);
    Switch_C_Status = Decode_Three_Position_Switch_Channel(Get_Channel_Value(SBUS_CH_SWITCH_C), Pre_Switch_C_Status);
    Switch_D_Status = Decode_Two_Position_Switch_Channel(Get_Channel_Value(SBUS_CH_SWITCH_D), Pre_Switch_D_Status);

    Rx_Data.Switch_A = (Switch_A_Status == SWITCH_UP) ? SBUS_Switch_Status_UP : SBUS_Switch_Status_DOWN;
    Rx_Data.Switch_B = (Switch_B_Status == SWITCH_UP) ? SBUS_Switch_Status_UP : SBUS_Switch_Status_DOWN;
    Judge_Three_Position_Switch(&Rx_Data.Switch_C, Switch_C_Status, Pre_Switch_C_Status);
    Rx_Data.Switch_D = (Switch_D_Status == SWITCH_UP) ? SBUS_Switch_Status_UP : SBUS_Switch_Status_DOWN;
    Judge_Update(Pre_Rx_Data, Rx_Data);
}

void Class_SBUS::SBUS_UART_RxCpltCallback(uint8_t *Rx_Buffer, uint16_t Length)
{
    if (Decode_SBUS_Frame(Rx_Buffer, Length) == false)
    {
        Error_Cnt++;
        return;
    }

    SBUS_Data_Process();

    if (Rx_Data.Frame_Lost == true || Rx_Data.Failsafe == true)
    {
        SBUS_Failsafe_Active = true;
        SBUS_Status = SBUS_Status_DISABLE;
        Invalidate_Control_Data();
        Pre_UART_Rx_Data = Now_UART_Rx_Data;
        Pre_Rx_Data = Rx_Data;
        Pre_Switch_A_Status = Switch_A_Status;
        Pre_Switch_B_Status = Switch_B_Status;
        Pre_Switch_C_Status = Switch_C_Status;
        Pre_Switch_D_Status = Switch_D_Status;
        return;
    }

    SBUS_Failsafe_Active = false;
    SBUS_Flag += 1U;

    Pre_UART_Rx_Data = Now_UART_Rx_Data;
    Pre_Rx_Data = Rx_Data;
    Pre_Switch_A_Status = Switch_A_Status;
    Pre_Switch_B_Status = Switch_B_Status;
    Pre_Switch_C_Status = Switch_C_Status;
    Pre_Switch_D_Status = Switch_D_Status;
}

void Class_SBUS::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (SBUS_Failsafe_Active == true)
    {
        SBUS_Status = SBUS_Status_DISABLE;
    }
    else if (SBUS_Flag == Pre_SBUS_Flag)
    {
        SBUS_Status = SBUS_Status_DISABLE;
        Offline_Cnt++;
    }
    else
    {
        SBUS_Status = SBUS_Status_ENABLE;
    }

    Pre_SBUS_Flag = SBUS_Flag;
}
