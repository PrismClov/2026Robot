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
 * @brief 根据CRC位宽获取类型（同时用于表项、多项式、初始值和返回值）
 * 扩展支持了 Width <= 64
 */
template <uint8_t Width>
using CRCType = std::conditional_t<Width <= 8, uint8_t,
                std::conditional_t<Width <= 16, uint16_t, 
                std::conditional_t<Width <= 32, uint32_t, 
                uint64_t>>>;

/**
 * @brief CRC 编译期核心计算工具
 */
struct CRC_Core 
{
    /**
     * @brief 安全计算编译期掩码，完美规避移位溢出的未定义行为 (UB)
     */
    template <uint8_t Width>
    static constexpr CRCType<Width> get_mask() noexcept
    {
        using Type = CRCType<Width>;
        // 既然是编译期常量，if constexpr 会在编译时直接丢弃不符合条件的分支
        if constexpr (Width >= sizeof(Type) * 8) 
        {
            return ~static_cast<Type>(0);
        } 
        else 
        {
            return (static_cast<Type>(1) << Width) - 1;
        }
    }

    /**
     * @brief 位反转函数
     * @tparam Width CRC位宽
     */
    template <uint8_t Width>
    static constexpr CRCType<Width> reflect(CRCType<Width> value) 
    {
        using Type = CRCType<Width>;
        Type result = 0;
        for (uint8_t i = 0; i < Width; i++) 
        {
            if (value & (static_cast<Type>(1) << i)) 
            {
                result |= static_cast<Type>(1) << (Width - 1 - i);
            }
        }
        return result;
    }

    /**
     * @brief 生成CRC查找表
     * @tparam Width CRC位宽
     */
    template <uint8_t Width>
    static constexpr auto generate_table(CRCType<Width> poly, bool refin) 
    {
        using Type = CRCType<Width>;
        std::array<Type, 256> table{};
        
        // 动态计算掩码，防止位移溢出
        constexpr Type mask = CRC_Core::get_mask<Width>();

        if (refin) 
        {
            Type reflected_poly = reflect<Width>(poly);
            for (uint32_t d = 0; d < 256; d++) 
            {
                Type entry = static_cast<Type>(d);
                for (uint8_t i = 0; i < 8; i++) 
                {
                    bool lsb = (entry & 1u) != 0u;
                    entry >>= 1;
                    if (lsb) entry ^= reflected_poly;
                    entry &= mask;
                }
                table[d] = entry;
            }
        } 
        else 
        {
            for (uint32_t d = 0; d < 256; d++) 
            {
                Type entry = static_cast<Type>(d) << (Width - 8);
                for (uint8_t i = 0; i < 8; i++) 
                {
                    bool msb = ((entry >> (Width - 1)) & 1u) != 0u;
                    entry <<= 1;
                    if (msb) entry ^= poly;
                    entry &= mask;
                }
                table[d] = entry;
            }
        }
        return table;
    }

    /**
     * @brief 计算CRC值
      * @tparam Width CRC位宽
      * @tparam Poly CRC多项式
      * @tparam Init CRC初始值
      * @tparam Xorout CRC输出异或值
      * @tparam Refin 输入数据是否反转
      * @tparam Refout 输出CRC是否反转
      * @param table 预生成的CRC查找表
      * @param data 输入数据指针
      * @param length 输入数据长度
     */
    template <uint8_t Width, CRCType<Width> Poly, CRCType<Width> Init, CRCType<Width> Xorout, bool Refin, bool Refout, typename TableType>
    static constexpr auto process(const std::array<TableType, 256>& table, const uint8_t* data, size_t length)
    {
        using Type = CRCType<Width>;
        
        Type crc_value = Init;
        constexpr Type mask = CRC_Core::get_mask<Width>();

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
            constexpr uint8_t shift_bits = static_cast<uint8_t>(Width - 8);
            for (size_t i = 0; i < length; i++)
            {
                uint8_t idx = static_cast<uint8_t>(crc_value >> shift_bits) ^ data[i];
                crc_value = (crc_value << 8) ^ table[idx];
            }
            crc_value &= mask;
        }

        if constexpr (Refout)
        {
            crc_value = reflect<Width>(crc_value);
        }
        crc_value ^= Xorout;
        
        return static_cast<Type>(crc_value & mask);
    }
};

/**
 * @brief CRC 静态类模板
 */
template <uint8_t Width, CRCType<Width> Poly, CRCType<Width> Init, CRCType<Width> Xorout, bool Refin, bool Refout>
class Class_CRC_Static 
{
private:
    // 扩展支持 8 到 64 位
    static_assert(Width >= 8 && Width <= 64, "CRC width must be 8..64");
    static_assert(Width % 8 == 0, "CRC width must be a multiple of 8");

    static constexpr auto LUT = CRC_Core::generate_table<Width>(Poly, Refin);

public:
    using Type = CRCType<Width>;
    
    static Type compute(const void *data, size_t length) noexcept
    {
        return CRC_Core::process<Width, Poly, Init, Xorout, Refin, Refout>(LUT, static_cast<const uint8_t*>(data), length);
    }

    static Type compute(std::string_view str) noexcept
    {
        return compute(str.data(), str.size());
    }
};

// ============================================================================
// CRC 类型定义（参数顺序已变更为：Width, Poly, Init, Xorout, Refin, Refout）
// ============================================================================

// CRC-8 (uint8_t)
using CRC8              = Class_CRC_Static<8, 0x07, 0x00, 0x00, false, false>;
using CRC8_ITU          = Class_CRC_Static<8, 0x07, 0x00, 0x55, false, false>;
using CRC8_ROHC         = Class_CRC_Static<8, 0x07, 0xFF, 0x00, true, true>;
using CRC8_MAXIM        = Class_CRC_Static<8, 0x31, 0x00, 0x00, true, true>;

// CRC-16 (uint16_t)
using CRC16_IBM         = Class_CRC_Static<16, 0x8005, 0x0000, 0x0000, true, true>;
using CRC16_MODBUS      = Class_CRC_Static<16, 0x8005, 0xFFFF, 0x0000, true, true>;
using CRC16_CCITT       = Class_CRC_Static<16, 0x1021, 0x0000, 0x0000, true, true>;

// CRC-32 (uint32_t)
using CRC32             = Class_CRC_Static<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true>;

// CRC-64 (uint64_t)
using CRC64_ECMA        = Class_CRC_Static<64, 0x42F0E1EBA9EA3693ULL, 0x0000000000000000ULL, 0x0000000000000000ULL, false, false>;
using CRC64_ISO         = Class_CRC_Static<64, 0x000000000000001BULL, 0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL, true, true>;

} // namespace CRC_Lib
} // namespace Algorithm

/**
 * 使用示例：
 * uint32_t crc32_value = Algorithm::CRC_Lib::CRC32::compute(data, length);
 * uint8_t crc8_value = Algorithm::CRC_Lib::CRC8_ITU::compute("Hello, World!");
 */
#endif // ALG_CRC_H