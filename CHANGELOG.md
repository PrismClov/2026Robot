# Changelog
---
# 2026年5月30日
### 加入CRC校验
- 添加 CRC 校验，防止数据传输错误导致数据错误。
- 接口为
```cpp
void CRC8_UpdateByte(uint8_t &crc, uint8_t data);

void void CRC8_Update_Buffer(uint8_t &crc, const uint8_t *data, uint32_t length);
```

### 引入命名空间
- 添加命名空间 `namespace Motor`，避免命名冲突。
---

## 2026-05-29

### CAN 模块 MessageRAMOffset 配置

为确保 CAN1、CAN2、CAN3 共享 Message RAM 时不发生重叠，手动设置了各实例的偏移量（单位：32-bit word）：

- **CAN1**：`hfdcan1.Init.MessageRAMOffset = 0`
- **CAN2**：`hfdcan2.Init.MessageRAMOffset = 850`
- **CAN3**：`hfdcan3.Init.MessageRAMOffset = 1700`

> **注意**：STM32H723 的 FDCAN 总 Message RAM 大小为 2560 字（`0x24000000` 起始）。当前分配（0 ~ 850 ~ 1700）相互不重叠，但请确保每个 CAN 实例实际使用的 RAM 大小（由 FIFO/过滤器数量决定）未超过预留区间。

### 链接脚本 `STM32H723VGTx_FLASH.ld` 错误定位与修正

CubeMX 生成的原始 `.ld` 文件存在多处语法错误，导致链接失败。以下是详细修正说明。

#### 1. 第 55 行：栈顶地址定义错误

**错误代码：**

```ld
_estack = ORIGIN() + LENGTH();    /* end of RAM */
```

**问题**：`ORIGIN()` 和 `LENGTH()` 必须指定内存区域名称。

**修正为：**

```ld
_estack = ORIGIN(RAM) + LENGTH(RAM);   /* 0x24000000 + 128K */
```

#### 2. 第 149 行：`.data` 段输出区域语法错误

**错误代码：**

```ld
} > AT> FLASH
```

**问题**：缺少虚拟内存区域（VMA），标准语法应为 `} >VMA AT> LMA`。

**修正为：**

```ld
} >RAM AT> FLASH
```

#### 3. 第 166 行：`.bss` 段未指定内存区域

**错误代码：**

```ld
} >
```

**问题**：没有指定 `.bss` 段应放入哪个 RAM。

**修正为：**

```ld
} >RAM
```

#### 4. 第 177 行：`._user_heap_stack` 段未指定内存区域

**错误代码：**

```ld
} >
```

**修正为：**

```ld
} >RAM
```

---

## 2026年5月28日 — 代码新框架最终整理

### 架构调整
- **`Class_Motor_DJI_C610`** - 直接继承 `Class_Motor_Base`，在类内部实现统一电机接口、CAN 反馈读取、PID 计算、舵向位置外环和输出逻辑。
- **`Class_Motor_MKSESC`** - 直接继承 `Class_Motor_Base`，在类内部实现统一电机接口、CAN 反馈读取、轮向速度/电流/位置目标转换和输出逻辑。
- **`Class_Swerve_Module`** - 继续只依赖 `Class_Motor_Base`，不依赖具体电机型号，也不再依赖 Adapter 或多继承包装类。
- **`User_tsk/tsk_config_and_callback.cpp`** - 舵轮初始化改为直接创建 `Steer_Motor`、`Drive_Motor`、`Swerve_Module`，FDCAN 回调直接分发给具体电机对象。

### 删除过渡方案
- **`User_dvc/dvc_motor_swerve_adapter.h/.cpp`** - 删除过渡用 Adapter 方案。
- **`User_dvc/dvc_motor_swerve_inherit.h/.cpp`** - 删除多继承包装方案。
- **`User_dvc/dvc_motor_swerve_base.h`** - 删除临时兼容头文件。

### 编译验证
- 已通过 CMake Debug 构建：`cmake --build --preset Debug`
- 生成文件：`build/Debug/H723VGT6_Demo.elf`

---

## 2026年5月27日 — 代码新框架搭建

### 框架目标
- 本次修改不是单个驱动补丁，而是为后续舵轮底盘开发搭建新的代码框架。
- 新框架将底层电机驱动、舵轮模块控制、舵向编码器反馈和工程构建系统拆分开，方便后续扩展不同型号电机和不同舵轮模块。
- 上层控制逻辑尽量依赖统一接口，减少直接绑定某一种电机驱动类带来的耦合。

### 新增文件
- **`CMakeLists.txt`** - CMake 主工程文件，接入 CubeMX 生成代码和用户层代码。
- **`CMakePresets.json`** - CMake 构建预设，支持 `Debug/Release` 配置。
- **`cmake/gcc-arm-none-eabi.cmake`** - ARM GCC 工具链配置。
- **`cmake/stm32cubemx/CMakeLists.txt`** - CubeMX 代码子工程配置。
- **`STM32H723VGTx_FLASH.ld`** - GCC 链接脚本。
- **`startup_stm32h723xx.s`** - GCC 启动文件。
- **`Core/Src/syscalls.c`** - newlib 系统调用适配。
- **`Core/Src/sysmem.c`** - 堆内存适配。
- **`User_dvc/dvc_motor_base.h`** - 通用电机抽象基类，统一电流、速度、位置、反馈更新、控制计算和输出接口。
- **`User_dvc/dvc_motor_swerve_adapter.h/.cpp`** - 舵轮电机适配器，将 DJI C610 和 MKSESC 接入统一电机接口。
- **`User_dvc/dvc_swerve_module.h/.cpp`** - 单舵轮模块控制框架，负责舵向角优化、轮向速度/力控目标分配和输出调用。
- **`User_dvc/dvc_swerve_steer_encoder.h/.cpp`** - 舵向编码器角度转换模块，兼容原舵轮编码器 CAN 数据处理逻辑。
- **`User_dvc/dvc_encoder_rudder.cpp/.h`** - 舵轮编码器驱动（后续调整为 `dvc_swerve_steer_encoder.*`）。

### 代码调整
- **工程构建系统** - 从 Keil 单工程补充为 CMake + ARM GCC 构建方式，生成 `compile_commands.json`，便于 VSCode/clangd 索引。
- **`CMakeLists.txt`** - 加入 `User_alg/User_drv/User_dvc/User_chariot/User_ita/User_tsk` 用户源码和头文件目录。
- **`CMakeLists.txt`** - 加入 CMSIS-DSP include 路径，解决 `arm_math.h` 在 CMake 下找不到的问题。
- **`CMakeLists.txt`** - CubeMX 未生成 `spi.h/spi.c`，CMake 中临时排除 `drv_spi.cpp`，后续启用 SPI 后可恢复。
- **`cmake/gcc-arm-none-eabi.cmake`** - 为 Cortex-M7 增加 `-mthumb` 编译参数。
- **`STM32H723VGTx_FLASH.ld`** - 修复链接脚本中缺失的 RAM 区域标识。
- **`tsk_config_and_callback.cpp`** - 大量删减冗余代码，优化结构。

### 编译验证
- 生成文件：`build/Debug/H723VGT6_Demo.elf`、`build/Debug/compile_commands.json`
- 内存占用：FLASH 56216 B / 1 MB（~5.36%），RAM 13136 B / 128 KB（~10.02%）

### 框架说明
- `Class_Motor_Base` — 统一电机抽象层，上层舵轮模块只依赖通用接口。
- `Class_Swerve_Module` — 单个舵轮模块控制层，负责舵向目标、轮向速度/力控目标和舵向优化。
- `dvc_motor_swerve_adapter` — 适配层，将 DJI/MKSESC 电机驱动接入新框架。
- `dvc_swerve_steer_encoder` — 舵向编码器转换层，原始编码器值转换为 rad 角度。

---

## 历史提交记录（git commit）

| 日期 | 提交 | 说明 |
|------|------|------|
| 近期 | e37daa2 | DJI 电机框架重写 |
| | 9b6c70c | Robocon 2027 新代码框架 |
| | aee9a26 | Merge branch 'Wulin_R1' |
| | 2f7c3aa | 新版舵轮引入舵向磁编，以及自制 VESC 电调 |
| | f0c5bbe | 优化舵轮力控底盘力学前馈代码逻辑 |
| | 312dac4 | KFS 中期完成版 |
| | c606f46 | 更新夹取机械臂参数 |
| | 2d960a2 | KFS 调试和夹取里基于达秒电机 MIT 模式的控制优化 |
| | 483e767 | 灵足映射问题 |
| | 2c2b400 | 灵足电机位置模式 |
| | 6901e4c | CAN 通信 bug 修改 |
| | 842d8de | KFS 夹取 |
| | a944c44 | 添加重力补偿前馈并完成调参 |
| | c098f15 | 添加夹取重力补偿前馈并完成调参 |
| | 9fd26c8 | 调整函数架构并写状态机，更新参数 |
| | 0883d91 | 夹取测试 |
| | a386c72 | 夹取状态机 |
| | 6d399e7 | 例程 bug 修改 |
| | 1f172ce | 灵足电机例程改写 |
| | 58b4ec6 | 关节电机测试 |
| | e35695c | Wulin_R1_Origin |
