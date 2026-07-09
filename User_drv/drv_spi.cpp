/**
 * @file drv_spi.cpp
 * @author clover
 * @brief 模板化SPI驱动实现，HAL相关全部在此文件中
 * @version 1.0
 *
 * @copyright RoboPioneer (c) 2024
 */

#include "drv_config.h"

/* ========== HAL操作特化 ========== */

template <>
struct Class_SPI_HAL<SPI_HandleTypeDef>
{
    // 阻塞发送
    static uint8_t Transmit(SPI_HandleTypeDef* h, uint8_t* data, uint16_t len, uint32_t timeout)
    {
        return HAL_SPI_Transmit(h, data, len, timeout);
    }

    // 阻塞收发
    static uint8_t TransmitReceive(SPI_HandleTypeDef* h, uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout)
    {
        return HAL_SPI_TransmitReceive(h, tx, rx, len, timeout);
    }

    // DMA收发
    static uint8_t TransmitReceive_DMA(SPI_HandleTypeDef* h, uint8_t* tx, uint8_t* rx, uint16_t len)
    {
        return HAL_SPI_TransmitReceive_DMA(h, tx, rx, len);
    }

    // GPIO写引脚（适配不同HAL版本）
    static void WritePin(GPIO_TypeDef* gpio, uint16_t pin, uint8_t state)
    {
        HAL_GPIO_WritePin(gpio, static_cast<uint16_t>(pin), 
                         state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
};

/* ========== 显式实例化 ========== */
// inline 变量在 h 头中定义，此处实例化模板类
template class Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;

/* ========== HAL中断回调 ========== */

// 使用注册表方式，避免硬编码
struct SPICallbackRegistry
{
    SPI_HandleTypeDef* handle;
    SPI1_Device_T* device;
};

// 静态注册表（最大支持8个SPI）
static SPICallbackRegistry spi_registry[8] = {nullptr};
static uint8_t spi_registry_count = 0;

// 注册函数（由初始化时调用）
void RegisterSPIDevice(SPI_HandleTypeDef* handle,
                       SPI1_Device_T* device)
{
    for (uint8_t i = 0; i < spi_registry_count; i++)
    {
        if (spi_registry[i].handle == handle)
            return;  // 已注册
    }
    
    if (spi_registry_count < 8)
    {
        spi_registry[spi_registry_count].handle = handle;
        spi_registry[spi_registry_count].device = device;
        spi_registry_count++;
    }
}
/**
 * @brief 获取SPI句柄对应的设备对象指针
 * @note 使用直接比较 + 编译期优化
 */
static inline SPI1_Device_T*
    GetSPIDevice(SPI_HandleTypeDef* hspi)
{
    // 方法1：直接比较（最快，但硬编码）
    #if 1
    if (hspi == &hspi1) return &SPI1_Device;
    // if (hspi == &hspi2) return &SPI2_Device;
    // if (hspi == &hspi3) return &SPI3_Device;
    // if (hspi == &hspi4) return &SPI4_Device;
    // if (hspi == &hspi5) return &SPI5_Device;
    // if (hspi == &hspi6) return &SPI6_Device;
    return nullptr;
    #endif
    
    // 方法2：switch-case（编译器生成跳转表）
    #if 0
    switch ((uintptr_t)hspi)
    {
        case (uintptr_t)&hspi1: return &SPI1_Device;
        case (uintptr_t)&hspi2: return &SPI2_Device;
        case (uintptr_t)&hspi3: return &SPI3_Device;
        case (uintptr_t)&hspi4: return &SPI4_Device;
        case (uintptr_t)&hspi5: return &SPI5_Device;
        case (uintptr_t)&hspi6: return &SPI6_Device;
        default: return nullptr;
    }
    #endif
}

/**
 * @brief SPI传输完成中断回调 - 混合优化版本
 * @note 结合直接比较和跳转表优点
 */
__attribute__((hot))
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    // 快速获取设备对象
    auto* device = GetSPIDevice(hspi);
    
    if (device)
    {
        device->OnTxRxCplt();
    }
}

/**
 * @brief SPI错误中断回调 - 混合优化版本
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    auto* device = GetSPIDevice(hspi);
    
    if (device)
    {
        // 处理错误：强制释放CS
        device->OnError();  // 需要添加此方法
    }
}

/* ========== 需要添加到类中的方法 ========== */

// 在 drv_spi.h 的 Class_SPI_Driver 中添加：
/*
public:
    void OnError()
    {
        dma_in_progress_ = false;
        CsDeselect();
        // 可添加错误回调
    }
*/
/* ========== 使用示例（注释） ========== */

/*
// 初始化示例
void SPI_InitExample()
{
    // 初始化SPI1
    SPI1_Device.Init(&hspi1, GPIOA, GPIO_PIN_4, 
        [](uint8_t* tx, uint8_t* rx, uint16_t len) {
            // DMA传输完成回调
            // 处理接收数据
        });
    
    // 注册到中断系统
    RegisterSPIDevice(&hspi1, &SPI1_Device);
    
    // 初始化SPI2
    SPI2_Device.Init(&hspi2, GPIOB, GPIO_PIN_12);
    RegisterSPIDevice(&hspi2, &SPI2_Device);
}

// 阻塞传输示例
void SPI_BlockingExample()
{
    uint8_t tx_data[10] = {0x01, 0x02, 0x03};
    uint8_t rx_data[10] = {0};
    
    // 发送
    SPI1_Device.Transmit(tx_data, 3, 1000);
    
    // 收发
    SPI1_Device.TransmitReceive(tx_data, rx_data, 3, 1000);
}

// DMA传输示例
void SPI_DMAExample()
{
    uint8_t tx_data[10] = {0x01, 0x02, 0x03};
    uint8_t rx_data[10] = {0};
    
    // 启动DMA传输（异步）
    SPI1_Device.TransmitReceive_DMA(tx_data, rx_data, 3);
    
    // 传输完成后自动调用回调
}
*/

/************************ COPYRIGHT(C) ROBOPIONEER **************************/