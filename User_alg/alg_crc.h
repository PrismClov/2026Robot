#ifndef ALG_CRC_H
#define ALG_CRC_H

#include <cstdint>
#include <cstddef>
#include <array>
#include <string_view>
#include <type_traits>

namespace Algorithm
{
namespace CRC_Lib
{

/**
 * @brief 根据CRC位宽获取类型（同时用于表项和返回值）
 */
template <uint8_t Width>
using CRCType = std::conditional_t<Width <= 8, uint8_t,
                std::conditional_t<Width <= 16, uint16_t, 
                uint32_t>>;

/**
 * @brief CRC 编译期核心计算工具
 */
struct CRC_Core 
{
    static constexpr uint32_t reflect(uint32_t value, uint8_t bit_width) 
    {
        uint32_t result = 0;
        for (uint8_t i = 0; i < bit_width; i++) 
        {
            if (value & (1u << i)) 
            {
                result |= 1u << (bit_width - 1 - i);
            }
        }
        return result;
    }

    template <uint8_t Width>
    static constexpr auto generate_table(uint32_t poly, bool refin) 
    {
        using Type = CRCType<Width>;
        std::array<Type, 256> table{};
        
        uint32_t mask = (Width >= 32) ? 0xFFFFFFFFu : ((1u << Width) - 1);

        if (refin) 
        {
            uint32_t reflected_poly = reflect(poly, Width);
            for (uint32_t d = 0; d < 256; d++) 
            {
                uint32_t entry = d;
                for (uint8_t i = 0; i < 8; i++) 
                {
                    bool lsb = (entry & 1u) != 0u;
                    entry >>= 1;
                    if (lsb) entry ^= reflected_poly;
                    entry &= mask;
                }
                table[d] = static_cast<Type>(entry);
            }
        } 
        else 
        {
            for (uint32_t d = 0; d < 256; d++) 
            {
                uint32_t entry = d << (Width - 8);
                for (uint8_t i = 0; i < 8; i++) 
                {
                    bool msb = ((entry >> (Width - 1)) & 1u) != 0u;
                    entry <<= 1;
                    if (msb) entry ^= poly;
                    entry &= mask;
                }
                table[d] = static_cast<Type>(entry);
            }
        }
        return table;
    }

    template <uint32_t Poly, uint32_t Init, uint32_t Xorout, bool Refin, bool Refout, uint8_t Width, typename TableType>
    static constexpr auto process(const std::array<TableType, 256>& table, const uint8_t* data, size_t length)
    {
        using Type = CRCType<Width>;
        
        uint32_t crc_value = Init;
        uint32_t mask = (Width >= 32) ? 0xFFFFFFFFu : ((1u << Width) - 1u);

        if constexpr (Refin)
        {
            for (size_t i = 0; i < length; i++)
            {
                uint8_t idx = static_cast<uint8_t>(crc_value & 0xFF) ^ data[i];
                crc_value = (crc_value >> 8) ^ table[idx];
            }
        }
        else
        {
            uint8_t shift_bits = static_cast<uint8_t>(Width - 8);
            for (size_t i = 0; i < length; i++)
            {
                uint8_t idx = static_cast<uint8_t>(crc_value >> shift_bits) ^ data[i];
                crc_value = (crc_value << 8) ^ table[idx];
            }
            crc_value &= mask;
        }

        if constexpr (Refout)
        {
            crc_value = reflect(crc_value, Width);
        }
        crc_value ^= Xorout;
        
        return static_cast<Type>(crc_value & mask);
    }
};

/**
 * @brief CRC 静态类模板
 */
template <uint32_t Poly, uint32_t Init, uint32_t Xorout, bool Refin, bool Refout, uint8_t Width>
class Class_CRC_Static 
{
private:
    static_assert(Width >= 8 && Width <= 32, "CRC width must be 8..32");

    static constexpr auto LUT = CRC_Core::generate_table<Width>(Poly, Refin);

public:
    using Type = CRCType<Width>;
    
    // 返回值版本
    static Type compute(const void *data, size_t length) noexcept
    {
        // process内部会处理空指针和零长度的情况，返回Init ^ Xorout
        return CRC_Core::process<Poly, Init, Xorout, Refin, Refout, Width>(LUT, static_cast<const uint8_t*>(data), length);
    }

    static Type compute(std::string_view str) noexcept
    {
        return compute(str.data(), str.size());
    }
};

// ============================================================================
// CRC 类型定义
// ============================================================================

// CRC-8 (uint8_t)
using CRC8              = Class_CRC_Static<0x07, 0x00, 0x00, false, false, 8>;
using CRC8_ITU          = Class_CRC_Static<0x07, 0x00, 0x55, false, false, 8>;
using CRC8_ROHC         = Class_CRC_Static<0x07, 0xFF, 0x00, true, true, 8>;
using CRC8_MAXIM        = Class_CRC_Static<0x31, 0x00, 0x00, true, true, 8>;

// CRC-16 (uint16_t)
using CRC16_IBM         = Class_CRC_Static<0x8005, 0x0000, 0x0000, true, true, 16>;
using CRC16_MODBUS      = Class_CRC_Static<0x8005, 0xFFFF, 0x0000, true, true, 16>;
using CRC16_CCITT       = Class_CRC_Static<0x1021, 0x0000, 0x0000, true, true, 16>;

// CRC-32 (uint32_t)
using CRC32             = Class_CRC_Static<0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true, 32>;

} // namespace CRC_Lib

} // namespace Algorithm

/**
 * 使用示例：
 * uint8_t crc_value;
 * uint8_t data[8] = {0,1,2,3,4,5,6,7};
 * crc_value = Algorithm::CRC_Lib::CRC8::compute(data, sizeof(data));
 * crc_value = Algorithm::CRC_Lib::CRC8::compute("Hello, World!");
 */
#endif // ALG_CRC_H