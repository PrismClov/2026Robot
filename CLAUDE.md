# Code Style

## File Naming

| Prefix | Layer | Example |
|--------|-------|---------|
| `drv_` | Driver | `drv_can.cpp` |
| `dvc_` | Device | `dvc_motor_dji.cpp` |
| `alg_` | Algorithm | `alg_pid.cpp` |
| `crt_` | Chariot module | `crt_chassis.cpp` |
| `ita_` | Interaction | `ita_robot.cpp` |
| `tsk_` | Task | `tsk_config_and_callback.cpp` |

## Class & Enum Naming

- `Class_<Name>` for all user classes
- DJI motors inside `Motor::` namespace: `Motor::Class_Motor_DJI_C620`
- FSM paired class: `Class_FSM_<Module>` + `friend class Class_FSM_<Module>`
- Enum type: `Enum_<Name>`, values: `SCREAMING_SNAKE_CASE`
- Flat enum (not `enum class`) for FSM int conversion
- `PID_Parameters` struct: `K_P`, `K_I`, `K_D`, `K_F`, `I_Out_Max`, `Out_Max`, `D_T`, `Dead_Zone`, `I_Variable_Speed_A/B`, `I_Separate_Threshold`, `D_First`

## Variable Conventions

- Members: `PascalCase` or `Snake_Case`
- Locals: `snake_case`
- Constants: `Pascal_Case` with `constexpr` or `UPPER_SNAKE_CASE` for macros

## Motor Init via Parameters Struct

```cpp
Motor_Move.Init(&hfdcan3, Motor::Motor_DJI_ID_0x201,
    Motor::Class_Motor_DJI_C620::Parameters{
        .PID_Position = PID_Parameters{
            .K_P = 8.0f, .K_I = 0.0f, .K_D = 0.0f, .Out_Max = 7.0f,
        },
        .PID_Omega = PID_Parameters{
            .K_P = 2.0f, .K_I = 0.0f, .K_D = 0.0f, .Out_Max = 20.0f,
        },
    });
```

Rules: designated initializers with field labels, zero fields can be omitted.

## Multi-Motor Sync (`Class_MultiMotorSync_Base<N>`)

```cpp
Class_MultiMotorSync_Base::Init({&Motor_L, &Motor_R},
    Class_MultiMotorSync_Base::Parameters{
        .PID_Distance = {
            PID_Parameters{.K_P = -60.0f, .K_I = 0.07f, .K_D = 0.02f, .Out_Max = 3.0f},
            PID_Parameters{.K_P = -60.0f, .K_I = 0.07f, .K_D = 0.02f, .Out_Max = 3.0f},
        },
        .Distance_Approach_Threshold = 0.01f,
        .Max_Velocity = 5.0f,
        .Angle_To_Distance = angle_to_dist,
        .Calibrate = {.motion_mode = CALIBRATE_MOTION_NONE},
    });
```

Control flow: `Distance_Update()` → `Move_To_Position()` → `Motor[i].Calculate()`.

**Inheritance** (Lift): `class Class_Lift : public Class_MultiMotorSync_Base<2>` — direct access to protected `Target_Distance`, `Now_Distance`, `Offset`.
**Composition** (KFS, Weapon): `Class_MultiMotorSync_Base<2> Lift` as member — access via public methods.



每次对话前必须重新读取相关代码文件，而不是基于上一次的记忆开始
