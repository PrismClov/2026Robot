/*
 * @Author: <hzy>
 * @Date: 2026-5-30 13:50
 * @LastEditTime: 2026-5-30 13:50
 * @LastEditors: <hzy>
 * @Description: alg_crc.cpp
 * @FilePath: \User_alg\alg_crc.cpp
 * @
 */
#include "alg_crc.h"
void CRC8_UpdateByte(uint8_t &crc, uint8_t data)
{
    crc = crc8_table[crc ^ data];
}
void CRC8_Update_Buffer(uint8_t &crc, const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        CRC8_UpdateByte(crc, data[i]);
    }
}