# Mechanical Era

STM32F103 麦克纳姆轮小车与四路舵机控制。控制代码使用 C++14，CubeMX 外设初始化和 ST 驱动保持生成的 C 代码。

## 分层

每个层自己一个目录、自带 `inc/` + `src/`；头文件用裸文件名（`#include "arm_pose.hpp"`），CMake 按层加 `-I`。

| 层 | 目录 | 职责 | 不负责 | 本工程文件 |
| --- | --- | --- | --- | --- |
| **HAL** | `Core/`、`Drivers/`（CubeMX 生成，手改只动 `USER CODE`） | 操作寄存器、启动外设、ISR 入口 | 业务逻辑、数据解析 | `MX_TIM8_Init`、`HAL_SPI_TransmitReceive`；`Core/Src/main.c` 只调一次 `main_task()` |
| **Driver** | `Driver/inc` + `Driver/src` | 片选/字节间隙等时序、方向脚与 PWM、编码器计数 | 解析协议、做闭环、认业务 | `ps2`（SPI 帧交换、模拟模式协商、轮询超时）、`outputs`（电机方向+PWM、换向等待、舵机脉宽）、`encoder`（四路 AB 相） |
| **Algorithm** | `Algorithm/inc` + `Algorithm/src` | 纯数学：IK/FK、PID；不碰硬件、无全局状态、不读时钟 | 硬件操作、状态管理 | `arm_kinematics`（平面 2R）、`pid`（纯 PID + 抗饱和） |
| **Device** | `Device/inc` + `Device/src` | 协议解析、设备状态机、闭环控制接口 | 底层通信细节 | `ps2_protocol`（报文解码）、`motor`（四路速度闭环）、`arm_joint`（行程状态机）、`arm_pose`（机械臂入口）、`drive_control`（麦轮混控）、`arm_control`（摇杆换算） |
| **Task** | `Task/inc` + `Task/src` | 业务编排、控制循环、模式状态机 | 底层协议、寄存器 | `main`（1 ms 循环与板载急停）、`mode_control`（模式/按键）、`diagnostics`（串口上报） |
| **公共** | `Include/` | 参数真值与 C 入口声明 | 任何逻辑 | `car_config.hpp`（底盘与摇杆）、`arm_config.hpp`（机械臂）、`app.h`（`Core/Src/main.c` 包含的 `main_task()` 声明，必须带 `extern "C"`） |

- 依赖方向：Task → Device → Algorithm，Task → Driver → HAL；Device 不 include Driver 的实现细节。
- **已知的两处跨层调用**（迁移只搬代码，没改接口）：`Driver/src/ps2.cpp` 调 `decode_ps2_frame()`（Driver → Device 解码）、`Driver/src/outputs.cpp` 调 `motor_update()`（Driver → Device 闭环）。彻底切开就是「驱动只回原始字节、Device 解码」和「闭环调度移出 `apply_motors`」。

## 阅读顺序

1. `Task/src/main.cpp`：主循环，按键去抖与急停，采集输入、计算指令、执行输出、定时报告。
2. `Driver/src/ps2.cpp`：PS2 接收器的 SPI 通信、模拟模式配置、轮询及超时；`Device/src/ps2_protocol.cpp` 负责唯一一份报文解码。
3. `Task/src/mode_control.cpp`：手柄操作逻辑，两个模式入口、按键边沿位图、`L3+R3` 急停组合、预设绑定表。
4. `Device/src/arm_pose.cpp`：机械臂编排层，预设/笛卡尔入口 → 解算 → 关节层，以及反馈聚合。
5. `Algorithm/src/arm_kinematics.cpp`（平面 2R 逆解/正解）与 `Device/src/arm_joint.cpp`（四路行程状态机）。
6. `Device/src/motor.cpp`（速度闭环）+ `Algorithm/src/pid.cpp`（算法）+ `Driver/src/encoder.cpp`（编码器计数）：一个电机被拆在三层里的示例。
7. `Device/src/drive_control.cpp`：底盘麦轮混合与按键平移/自转；`Driver/src/outputs.cpp`：方向脚、PWM、换向等待与舵机脉宽。
8. 参数：`Include/car_config.hpp`（轮位、极性、麦轮布局、摇杆死区、自转占空比）与 `Include/arm_config.hpp`（几何、限位、工作区、速度、8 条预设的唯一真值）。

数据按 `ps2_update → ps2_read → mode_control_update → apply_motors / apply_servos` 流动。PS2 报文只解码一次，程序中的按键位统一为 1 表示按下。调试统计不参与控制决策。


## 编译

固件需要 CMake、Ninja 和 PATH 中的 Arm GNU Toolchain：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

输出为 `build/Debug/mechanical_era.elf` 或 `build/Release/mechanical_era.elf`。

标定结构限位时需要另一份固件：在 `CMakeLists.txt` 的 `target_compile_options` 里临时加 `-DARM_BRINGUP`，标完必须去掉并重烧正常版。

## 代码约定

- 公共类型用明确名称，如 `Ps2Frame`、`DriveCommand`、`ArmCommand`；函数用 `snake_case`。
- 配置集中在 `car_config` / `arm_config` 命名空间；文件内部细节使用匿名命名空间。无需为每个模块再包一层通用 `Command` / `from_ps2`。
- 使用普通结构体和函数，无动态分配、继承层次或通用硬件抽象框架。
- 纯计算与 HAL 操作分开，`.hpp` 声明接口，`.cpp` 实现行为。`Include/app.h` 是供生成的 C 入口调用的薄桥接声明。
- 注释统一用 doxygen 模板：文件头 80 列 banner + `@file` / `@brief`；
  类型和函数用 `@brief` / `@details` / `@note` / `@param` / `@return`，VSCode 悬停即可读；
  单行字段说明仍写行尾 `//`。格式见任一 `Driver/inc/*.hpp`。
- 使用 `.clang-format` 统一自编代码格式；不要批量格式化 `Core/` 和 `Drivers/` 的生成代码。
- 生成文件中的手工入口保留在 `USER CODE` 区域。重新生成 CubeMX 后检查片选初值，并重新编译固件。
- `build/` 是可重新生成的本机输出，不纳入版本控制。

## 操作与接线

采用 O 型麦轮布局。M1 左前、M2 右前、M3 右后、M4 左后，M1/M2 反向。轮位和方向在 `Include/car_config.hpp` 中配置。

**模式**：`Start` 进模式一（底盘，上电默认），`Select` 进模式二（机械臂）。两个键都是直接入口、各自幂等，不是 toggle；急停锁定时不响应。

**模式一：底盘**

| 输入 | 行为 |
| --- | --- |
| 左摇杆 Y / X | 前进后退 / 左右平移，死区 16，满偏 100% |
| `L2` / `R2` | 逆时针 / 顺时针原地自转（`kRotateDutyPercent`，默认 80；同按 = 相抵） |
| 方向键 ↑↓←→ | 平移按钮备份（80%），覆盖左摇杆 |
| 右摇杆 / `L1` / `R1` / △○╳□ | 无功能（右摇杆整条通道已删） |

**模式二：机械臂精调**

| 输入 | 行为 |
| --- | --- |
| 左摇杆 X / Y | 末端笛卡尔速度：X 前伸/回收，Y 抬升/下压；平方律，满杆 50 mm/s，死区 8，松手即停 |
| `L2` / `R2` | 夹爪 松 / 紧（servo[2]，±200 %/s；正行程 = 张开） |
| `L1` / `R1` | 腕 roll 逆 / 顺（servo[3]，±150 %/s） |
| ↑ / ↓ / ← / → | 预设 P0 / P1 / P2 / P3 |
| △ / ○ / ╳ / □ | 预设 P4 / P5 / P6 / P7 |
| `R3` 单按 | 无功能（它只是急停组合的另一半） |

扳机是开关量（报文只解按键位，没有压力字节），夹爪和腕只能恒速动，行程靠按住时长。摇杆与扳机**可以同帧并行**：左手摆位的同时右手开合夹爪。

**急停**

- `L3`+`R3` 同按 ≥ 200 ms 锁定急停，再按一次解除；两键必须先都松开才能再次触发。**`╳` 不再是急停**，它现在是预设 P6。
- 板上 Key 提供同样行为的硬件级第二道保险。
- 手柄断线或有效报文超时 100 ms 时电机停止，机械臂保持在当前姿态（不回中）。

| PS2 信号 | STM32 引脚 |
| --- | --- |
| DAT | PB14 / SPI2_MISO |
| CMD | PB15 / SPI2_MOSI |
| ATT / CS | PB12 |
| CLK | PB13 / SPI2_SCK |
| GND | 与控制板共地 |

SPI2 使用 Mode 3、LSB 优先、256 分频，20 ms 轮询。接收器按模块规格供电，信号电平匹配 3.3 V。

TIM1 输出四路电机 PWM，TIM8 的 PC6～PC9 输出四路舵机 PWM。USART3_TX 使用 PC10，115200、8N1，每 500 ms 输出两行状态：

```
PS2 online=1 mode=73 op=ARM keys=0000 LX=128 LY=128 M=0,0,0,0 STOP=0 KEY=0 bad=0
ARM S1=-19 S2=-51 S3=0 S4=0 xy=(190.0,10.0) tgt=(190.0,10.0) th=(-17.2,43.8) ok=1 lim=0 wall=0 poses=0x00 bu=0
```

`mode=` 是 PS2 协议模式字（0x41/0x73/0x79），`op=` 才是操作模式。`xy` 是 FK 回算的当前末端，`tgt` 是本帧解算用的目标，`th` 是连杆角，`ok`/`lim`/`wall` 分别是「够不着 / 关节超限 / 工作区钳位」三个独立标志，`poses` 是上电自检失败的预设位掩码（`0x00` = 八条全好），`bu=1` 表示这是 `ARM_BRINGUP` 标定版（正常版恒 0，标完看到 1 就是烧错了）。

## 机械臂标定

**没有单关节 jog 了**：轴 0/1 只能由笛卡尔解算驱动，手动入口只剩夹爪(2) 和腕(3)。所有数值在 `Include/arm_config.hpp`（`kGeom` / `kJointLimits` / `kWorkspace` / `kMotion` / `kPoses`），改完重烧。

顺序不能乱：

1. **量连杆**：卡尺量轴心距 → `kGeom.upper_mm` / `fore_mm`。
2. **定零点**：`-DARM_BRINGUP` 版把该路设 0，量角器量绝对角 → `zero_deg`。
3. **定比例**：`ARM_BRINGUP` 推到 +30 / +60，两点量角度差 → `deg_per_percent`（含符号）。
4. **定限位**：`ARM_BRINGUP` 逐 10% 加，找到顶住的百分比后回退 3，换成角度写进 `range_min_deg` / `range_max_deg`。**只能这样测**：正常版的笛卡尔出口本来就被角度限位钳着，「用限位去测限位」量不出真实边界。
5. **校验模型**：烧正常版，笛卡尔推到 (150,120)，腕点与目标目测偏差应 < 5 mm；超了先查 `deg_per_percent`，再查下垂（`kMotion.sag_mm_per_100mm`），不要先改连杆长度。
6. **填预设**：笛卡尔摆到 8 个工位，抄串口 `xy=` 与 `S1..S4` → `kPoses[8]`。
7. **上电全表校验**：不碰手柄，串口应打 `poses=0x00`。

`ARM_BRINGUP` 版把四路直接设成 `Device/src/arm_pose.cpp` 里的 `kBringupPercent[4]`，跳过 IK、工作区、角度限位，**没有任何软件限位**，必须人工盯住、听到堵转立刻拔电。它不接任何手柄键，标完去掉宏重烧，否则等于把全部限位关掉上赛场。

限位只存角度，百分比由 `percent_of()` 在编译期派生（`Include/arm_config.hpp`），不要再手填一套百分比限位。
