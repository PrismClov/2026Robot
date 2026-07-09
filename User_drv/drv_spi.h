/**
 * @file drv_spi.h
 * @author clover
 * @brief 模板化SPI驱动，头文件无任何HAL依赖
 * @version 1.0
 *
 * @copyright RoboPioneer (c) 2024
 */

#ifndef DRV_SPI_H
#define DRV_SPI_H

#include <cstdint>
#include <functional>
#include "stm32h7xx_hal.h"

#define SPI_BUFFER_SIZE 256

/**
 * @brief SPI传输完成回调函数类型
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针
 * @param len 数据长度
 */
using SPI_CallBack = std::function<void(uint8_t*, uint8_t*, uint16_t)>;

/**
 * @brief SPI配置模板 — 持有HAL句柄和回调
 */
template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
struct Struct_SPI_Config
{
    HAL_Handle* hal_handle{nullptr};
    SPI_CallBack callback{nullptr};
    CS_GPIO* cs_gpio{nullptr};
    CS_PIN cs_pin{0};
};

/**
 * @brief 通用SPI驱动类（模板化设计）
 * 
 * @tparam HAL_Handle HAL SPI句柄类型（如 SPI_HandleTypeDef）
 * @tparam CS_GPIO GPIO端口类型（如 GPIO_TypeDef）
 * @tparam CS_PIN GPIO引脚类型（如 uint16_t）
 * 
 * 使用例:
 *   using SPI1_Device = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
 *   SPI1_Device spi1;
 *   spi1.Init(&hspi1, GPIOA, GPIO_PIN_4);
 */
template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
class Class_SPI_Driver
{
public:
    Class_SPI_Driver() = default;
    ~Class_SPI_Driver() = default;

    // 禁止拷贝
    Class_SPI_Driver(const Class_SPI_Driver&) = delete;
    Class_SPI_Driver& operator=(const Class_SPI_Driver&) = delete;

    // --- 初始化 ---
    void Init(HAL_Handle* hal_handle, 
              CS_GPIO* cs_gpio, 
              CS_PIN cs_pin,
              SPI_CallBack callback = nullptr);

    // --- 阻塞式发送（全双工） ---
    bool Transmit(uint8_t* data, uint16_t len, uint32_t timeout = 1000);

    // --- 阻塞式收发（全双工） ---
    bool TransmitReceive(uint8_t* tx_data, 
                         uint8_t* rx_data, 
                         uint16_t len, 
                         uint32_t timeout = 1000);

    // --- DMA发送接收（异步） ---
    bool TransmitReceive_DMA(uint8_t* tx_data, 
                             uint8_t* rx_data, 
                             uint16_t len);

    // --- 缓冲区访问（用于DMA模式） ---
    uint8_t* GetTxBuffer() { return tx_buffer_; }
    uint8_t* GetRxBuffer() { return rx_buffer_; }
    uint16_t GetTxLength() const { return tx_len_; }
    uint16_t GetRxLength() const { return rx_len_; }

    // --- 中断回调（由HAL中断调用） ---
    void OnTxRxCplt();
    void OnError();

    // --- 片选引脚修改（运行时可调） ---
    void SetCsPin(CS_GPIO* cs_gpio, CS_PIN cs_pin);

    // --- 状态查询 ---
    bool IsInitialized() const { return initialized_; }
    HAL_Handle* GetHALHandle() const { return config_.hal_handle; }
    bool IsDMAInProgress() const { return dma_in_progress_; }

private:
    void CsSelect();
    void CsDeselect();

    Struct_SPI_Config<HAL_Handle, CS_GPIO, CS_PIN> config_;
    bool initialized_{false};
    bool dma_in_progress_{false};

    uint8_t tx_buffer_[SPI_BUFFER_SIZE]{0};
    uint8_t rx_buffer_[SPI_BUFFER_SIZE]{0};
    uint16_t tx_len_{0};
    uint16_t rx_len_{0};
};

// 声明HAL操作辅助类（在cpp中特化）
template <typename HAL_Handle>
struct Class_SPI_HAL;

#include "drv_spi_impl.h"  // 模板实现包含




// #ifdef __cplusplus
// extern "C" {
// #endif
// extern SPI_HandleTypeDef hspi1;
// #ifdef __cplusplus
// }
// #endif
// /* ========== SPI设备类型别名 & 全局实例 ========== */
// using SPI1_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
// using SPI2_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
// using SPI3_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
// using SPI4_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
// using SPI5_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
// using SPI6_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;

// inline SPI1_Device_T SPI1_Device;
// inline SPI2_Device_T SPI2_Device;
// inline SPI3_Device_T SPI3_Device;
// inline SPI4_Device_T SPI4_Device;
// inline SPI5_Device_T SPI5_Device;
// inline SPI6_Device_T SPI6_Device;

// void RegisterSPIDevice(SPI_HandleTypeDef* handle, SPI1_Device_T* device);

#endif

/************************ COPYRIGHT(C) ROBOPIONEER **************************/