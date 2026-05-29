# Wulin_R1

STM32H723 舵轮机器人底盘控制固件，支持 DJI C610 / MKSESC 等电机驱动，基于 CMake + ARM GCC 构建。

## 项目概览

- **MCU**: STM32H723VGT6
- **开发平台**: STM32CubeMX + CMake + ARM GCC
- **Keil MDK**: 同步支持 Keil MDK-ARM 工程
- **语言**: C11 + C++17

## 目录结构

```
Wulin_R1/
├── CMakeLists.txt              # CMake 主工程文件
├── CMakePresets.json           # CMake 构建预设
├── cmake/                      # 工具链 & CubeMX 子工程
│   ├── gcc-arm-none-eabi.cmake
│   └── stm32cubemx/
├── Core/                       # CubeMX 生成的驱动 & 启动文件
├── Drivers/                    # HAL 库 & CMSIS
├── Middlewares/                # 中间件
├── User_dvc/                   # 设备驱动层（电机、舵轮模块、编码器…）
├── User_drv/                   # 硬件驱动层（CAN、UART、SPI、TIM、ADC…）
├── User_alg/                   # 算法层（PID、滤波器、状态机、坡度补偿…）
├── User_chariot/               # 整车业务层（底盘、KFS 机械臂、武器…）
├── User_ita/                   # 机器人高层接口
├── User_tsk/                   # 任务调度 & 回调
├── MDK-ARM/                    # Keil MDK 工程
├── build/                      # CMake 构建输出
├── 修改日志.md
├── 舵轮磁编数据发送格式.md
└── CHANGELOG.md
```

## 代码架构

### 电机驱动层

| 类 | 说明 |
|---|---|
| `Class_Motor_Base` | 通用电机抽象基类，定义电流/速度/位置控制和反馈接口 |
| `Class_Motor_DJI_C610` | DJI C610 电机驱动，直接继承 `Class_Motor_Base` |
| `Class_Motor_MKSESC` | MKSESC/VESC 电机驱动，直接继承 `Class_Motor_Base` |
| `Class_Motor_DM` | 达秒（D-Motor）驱动 |
| `Class_Motor_RMD` | RMD 电机驱动 |
| `Class_Motor_RS` | RS 系列电机驱动 |

### 舵轮模块

- **`Class_Swerve_Module`** — 单舵轮模块控制，负责舵向优化、轮向速度/力控目标分配
- **`Class_Swerve_Steer_Encoder`** — 舵向绝对编码器角度转换

### 算法层

- **PID 控制器** (`alg_pid`)
- **滤波器** (`alg_filter`)
- **有限状态机** (`alg_fsm`)
- **坡度补偿** (`alg_slope`)
- **定位** (`alg_location`)
- **队列** (`alg_queue`)
- **定时器** (`alg_timer`)

### 硬件驱动层

- CAN、UART、SPI、ADC、TIM、GPIO、看门狗

## 构建方式

### CMake + ARM GCC

```bash
# 配置
cmake --preset Debug

# 构建
cmake --build --preset Debug

# 输出
# build/Debug/H723VGT6_Demo.elf
# build/Debug/compile_commands.json
```

### Keil MDK

打开 `MDK-ARM/H723VGT6_Demo.uvprojx` 即可编译。

## 框架说明

1. **`Class_Motor_Base`** — 统一电机抽象层，上层只依赖通用接口，不绑定具体电机型号
2. **`Class_Swerve_Module`** — 舵轮控制层，仅依赖 `Class_Motor_Base`，不依赖 Adapter 或多继承
3. **`dvc_motor_swerve_adapter`**（已删除） — 过渡方案，已由电机直接继承 `Class_Motor_Base` 替代
4. **`dvc_swerve_steer_encoder`** — 舵向编码器转换层，将原始编码器值转换为 rad 角度

详细架构演进见 [CHANGELOG.md](CHANGELOG.md)。