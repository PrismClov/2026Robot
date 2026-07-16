#include "drv_spi.h"



#ifdef __cplusplus
extern "C" {
#endif
extern SPI_HandleTypeDef hspi1;
#ifdef __cplusplus
}
#endif
/* ========== SPI设备类型别名 & 全局实例 ========== */
using SPI1_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
using SPI2_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
using SPI3_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
using SPI4_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
using SPI5_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;
using SPI6_Device_T = Class_SPI_Driver<SPI_HandleTypeDef, GPIO_TypeDef, uint16_t>;

inline SPI1_Device_T SPI1_Device;
inline SPI2_Device_T SPI2_Device;
inline SPI3_Device_T SPI3_Device;
inline SPI4_Device_T SPI4_Device;
inline SPI5_Device_T SPI5_Device;
inline SPI6_Device_T SPI6_Device;

void RegisterSPIDevice(SPI_HandleTypeDef* handle, SPI1_Device_T* device);
