/**
 * @file drv_spi_impl.h
 * @brief SPI驱动模板实现（与头文件分离，但需要包含）
 */

#ifndef DRV_SPI_IMPL_H
#define DRV_SPI_IMPL_H

#include <cstring>
#include "drv_spi.h"

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::Init(
    HAL_Handle* hal_handle,
    CS_GPIO* cs_gpio,
    CS_PIN cs_pin,
    SPI_CallBack callback)
{
    config_.hal_handle = hal_handle;
    config_.cs_gpio = cs_gpio;
    config_.cs_pin = cs_pin;
    config_.callback = callback;
    initialized_ = true;
    dma_in_progress_ = false;
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
bool Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::Transmit(
    uint8_t* data, 
    uint16_t len, 
    uint32_t timeout)
{
    if (!initialized_ || !config_.hal_handle || data == nullptr || len == 0)
        return false;

    if (dma_in_progress_)
        return false;  // DMA忙

    CsSelect();
    auto result = Class_SPI_HAL<HAL_Handle>::Transmit(
        config_.hal_handle, data, len, timeout);
    CsDeselect();

    return (result == 0);
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
bool Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::TransmitReceive(
    uint8_t* tx_data,
    uint8_t* rx_data,
    uint16_t len,
    uint32_t timeout)
{
    if (!initialized_ || !config_.hal_handle || tx_data == nullptr || 
        rx_data == nullptr || len == 0)
        return false;

    if (dma_in_progress_)
        return false;

    CsSelect();
    auto result = Class_SPI_HAL<HAL_Handle>::TransmitReceive(
        config_.hal_handle, tx_data, rx_data, len, timeout);
    CsDeselect();

    return (result == 0);
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
bool Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::TransmitReceive_DMA(
    uint8_t* tx_data,
    uint8_t* rx_data,
    uint16_t len)
{
    if (!initialized_ || !config_.hal_handle || tx_data == nullptr || 
        rx_data == nullptr || len == 0 || len > SPI_BUFFER_SIZE)
        return false;

    if (dma_in_progress_)
        return false;

    // 复制数据到内部缓冲区（便于回调使用）
    memcpy(tx_buffer_, tx_data, len);
    tx_len_ = len;
    rx_len_ = len;

    CsSelect();
    dma_in_progress_ = true;

    auto result = Class_SPI_HAL<HAL_Handle>::TransmitReceive_DMA(
        config_.hal_handle, tx_buffer_, rx_buffer_, len);

    if (result != 0)
    {
        dma_in_progress_ = false;
        CsDeselect();
        return false;
    }

    // 如果DMA启动成功，回调中会处理CS释放
    // 但需要把接收数据复制到用户缓冲区（在回调中完成）
    return true;
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::OnTxRxCplt()
{
    dma_in_progress_ = false;
    CsDeselect();

    if (config_.callback)
    {
        config_.callback(tx_buffer_, rx_buffer_, tx_len_);
    }
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::OnError()
{
    dma_in_progress_ = false;
    CsDeselect();
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::SetCsPin(
    CS_GPIO* cs_gpio, 
    CS_PIN cs_pin)
{
    if (dma_in_progress_)
        return;  // DMA传输中不允许修改

    config_.cs_gpio = cs_gpio;
    config_.cs_pin = cs_pin;
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::CsSelect()
{
    if (config_.cs_gpio)
    {
        Class_SPI_HAL<HAL_Handle>::WritePin(
            config_.cs_gpio, config_.cs_pin, 0);  // 低电平选中
    }
}

template <typename HAL_Handle, typename CS_GPIO, typename CS_PIN>
void Class_SPI_Driver<HAL_Handle, CS_GPIO, CS_PIN>::CsDeselect()
{
    if (config_.cs_gpio)
    {
        Class_SPI_HAL<HAL_Handle>::WritePin(
            config_.cs_gpio, config_.cs_pin, 1);  // 高电平释放
    }
}

#endif