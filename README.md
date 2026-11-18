# Mechanical Era

STM32F103（256 KB Flash / 48 KB RAM）上的麦轮底盘 + 平面 2R 机械臂控制器：
一个 1 ms 主循环、一个 PS2 手柄、四路有刷电机（编码器速度闭环）、四路舵机。
控制代码是 C++14；CubeMX 生成的外设初始化与 ST 驱动保持 C 代码不动。

- 芯片：STM32F103xE（`mechanical_era.ioc`），HSE 8 MHz × 9 = **72 MHz**
- 工具链：CMake + Ninja + PATH 里的 `arm-none-eabi-`（GCC 15.3.1），`-Wall -Wextra -Werror`
- 当前构建：Debug `FLASH 40260 B (15.4%)` / `RAM 3104 B (6.3%)`；
  Release `FLASH 29172 B (11.1%)` / `RAM 3104 B`；两者均 0 error / 0 warning
- 设计文档：`plan/arm_pose_v0.3.md`（机械臂）、`plan/ps2_control_v0.2.md`（输入与模式）
- 二维仿真：`sim/index.html`（双击即开，无后端；几何/标定/预设/导出都在里面，见 `sim/README.md`）

---

## 1. 分层

每个层自己一个目录、自带 `inc/` + `src/`；头文件用裸文件名（`#include "arm_pose.hpp"`），
CMake 按层加 `-I`。

| 层 | 目录 | 职责 | 不负责 | 本工程文件 |
| --- | --- | --- | --- | --- |
| **HAL** | `Core/`、`Drivers/`（CubeMX 生成，手改只动 `USER CODE`） | 操作寄存器、启动外设、ISR 入口 | 业务逻辑、数据解析 | `MX_TIM8_Init`、`HAL_SPI_TransmitReceive`；`Core/Src/main.c` 只调一次 `main_task()` |
| **Driver** | `Driver/inc` + `Driver/src` | 片选/字节间隙等时序、方向脚与 PWM、编码器计数 | 解析协议、做闭环、认业务 | `ps2`（SPI 帧交换、模拟模式协商、轮询超时）、`outputs`（电机方向+PWM、换向等待、舵机脉宽）、`encoder`（四路 AB 相） |
| **Algorithm** | `Algorithm/inc` + `Algorithm/src` | 纯数学：IK/FK、摇杆限位、PID；不碰硬件、无全局状态、不读时钟 | 硬件操作、状态管理 | `arm_kinematics`（平面 2R）、`arm_limit`（摇杆限位：位置法 + 关节域贴边滑）、`pid`（纯 PID + 抗饱和） |
| **Device** | `Device/inc` + `Device/src` | 协议解析、设备状态机、闭环控制接口 | 底层通信细节 | `ps2_protocol`（报文解码）、`motor`（四路速度闭环）、`arm_joint`（行程状态机）、`arm_pose`（机械臂编排）、`drive_control`（麦轮混控）、`arm_control`（摇杆换算） |
| **Task** | `Task/inc` + `Task/src` | 业务编排、控制循环、模式状态机 | 底层协议、寄存器 | `main`（1 ms 循环与板载急停）、`mode_control`（模式/按键）、`diagnostics`（串口上报） |
| **公共** | `Include/` | 参数真值与 C 入口声明 | 任何逻辑 | `car_config.hpp`（底盘与摇杆）、`arm_config.hpp`（机械臂）、`app.h`（`main_task()` 声明，必须 `extern "C"`） |

依赖方向：Task → Device → Algorithm，Task → Driver → HAL；Device 不 include Driver 的实现细节。

两处已存在的跨层调用（历史遗留，接口未改）：`Driver/src/ps2.cpp` 调 `decode_ps2_frame()`
（Driver → Device 解码）、`Driver/src/outputs.cpp` 调 `motor_update()`（Driver → Device 闭环）。

> `CMakeLists.txt` 里的源文件是**逐条列出**的（不是 `GLOB`）：新增 `.cpp` 必须手工登记到
> `target_sources`，否则静默不参与编译。

### 阅读顺序

1. `Task/src/main.cpp`：主循环、板载 Key 去抖与急停、定时上报。
2. `Driver/src/ps2.cpp`：SPI 帧交换、模拟模式协商、轮询与超时；解码在 `Device/src/ps2_protocol.cpp`。
3. `Task/src/mode_control.cpp`：两个模式入口、按键边沿位图、`L3+R3` 急停组合、预设绑定表。
4. `Device/src/arm_pose.cpp`：预设/笛卡尔入口 → 解算 → 关节层，以及反馈聚合。
5. `Algorithm/src/arm_kinematics.cpp`（平面 2R 逆解/正解）+ `Device/src/arm_joint.cpp`（四路行程状态机）。
6. `Device/src/motor.cpp` + `Algorithm/src/pid.cpp` + `Driver/src/encoder.cpp`：一个电机拆在三层里的示例。
7. `Device/src/drive_control.cpp`（麦轮混控）+ `Driver/src/outputs.cpp`（方向脚、PWM、换向等待、舵机脉宽）。

---

## 2. 硬件接线

| 功能 | 引脚 | 外设 / 备注 |
| --- | --- | --- |
| PS2 ATT(CS) | PB12 | GPIO 推挽输出，空闲高 |
| PS2 CLK | PB13 | SPI2_SCK |
| PS2 DAT | PB14 | SPI2_MISO（上拉） |
| PS2 CMD | PB15 | SPI2_MOSI |
| 电机 M1 PWM | PA8 | TIM1_CH1 |
| 电机 M2 PWM | PA9 | TIM1_CH2 |
| 电机 M3 PWM | PA10 | TIM1_CH3 |
| 电机 M4 PWM | PA11 | TIM1_CH4（PA11 是 USB_DM，本工程不用 USB） |
| M1 方向 A/B | PC0 / PC1 | GPIO 推挽 |
| M2 方向 A/B | PC2 / PC3 | 同上 |
| M3 方向 A/B | PC4 / PC5 | 同上 |
| M4 方向 A/B | PB10 / PB11 | 同上 |
| M1 编码器 | PA15 (TIM2_CH1) / PB3 (CH2) | 需 `SWJ_NOJTAG` + TIM2 部分重映射 2，SWD 调试保留 |
| M2 编码器 | PA6 (TIM3_CH1) / PA7 (CH2) | |
| M3 编码器 | PA0 (TIM5_CH1) / PA1 (CH2) | |
| M4 编码器 | PB6 (TIM4_CH1) / PB7 (CH2) | |
| 舵机 S1～S4 | PC6 / PC7 / PC8 / PC9 | TIM8_CH1～CH4 |
| 串口调试 | PC10 = USART3_TX / PC11 = RX | 115200 8N1，部分重映射 |
| 板载停车键 | PA12 | 输入上拉，低电平有效 |
| 晶振 | PD0 / PD1 | HSE |

定时器与外设实配：

| 项目 | 配置 | 结果 |
| --- | --- | --- |
| TIM1（电机 PWM） | PSC 0、ARR 65535 | 72 MHz / 65536 ≈ **1.10 kHz**，占空比 = CCR/65536 |
| TIM8（舵机 PWM） | PSC 71（1 MHz 计数）、ARR 20000 | **50 Hz**，1 个计数 = 1 µs |
| TIM2/3/4/5 | 编码器模式 TI12，IC 滤波 4 | 16 位计数，`encoder::delta_ticks()` 取增量 |
| SPI2 | Master、Mode 3、LSB first、256 分频 | APB1 36 MHz / 256 ≈ 141 kHz，20 ms 一轮询 |
| USART3 | 115200 8N1 | 每 500 ms 四行 |

电机是「一路 PWM + 两个方向脚」的 H 桥接法：`duty > 0` 拉 A、`duty < 0` 拉 B，幅值写 CCR。
换向时先断该路 H 桥 2 ms 再反向（`Driver/src/outputs.cpp`），避免反电动势灌进驱动。

---

## 3. 构建与烧录

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

输出 `build/Debug/mechanical_era.elf` / `build/Release/mechanical_era.elf`。

烧录（ST-Link + OpenOCD）：

```powershell
openocd -f flash.cfg
```

调试用 VS Code 的 Cortex-Debug 配置 `Debug (OpenOCD)`（`.vscode/launch.json`，Live Watch 4 Hz）。

**标定版固件**：在 `CMakeLists.txt` 的 `target_compile_options` 里临时加 `-DARM_BRINGUP` 后重编重烧。
标完必须去掉并重烧正常版 —— 标定版把四路限位放开到 ±100、绕过 IK 与工作区，等于没有任何软件限位。

---

## 4. 运行时数据流

```
HAL_GPIO_ReadPin(Key) ─┐
ps2_update() ─► ps2_read() ─► mode_control_update(frame, now) ─┬─► output.drive ─► apply_motors() ─► motor_update()（PID）─► TIM1 + 方向脚
                    （20 ms 一帧 / 100 ms 断线）                └─► output.arm   ─► apply_servos() ──────────────────────► TIM8
                                                                      └─► report_status() / report_arm()（500 ms）─► USART3
```

- 主循环 1 ms；PS2 报文每 20 ms 换一帧，其余 19 轮沿用同一帧（边沿检测靠 `mode_control` 里的位图）。
- PS2 报文只解码一次（`ps2_protocol.cpp`），程序里按键位统一为 **1 = 按下**。
- 时间由外部传入（`now_ms`）：`arm_pose` / `arm_joint` / `motor` 内部不读 `HAL_GetTick`。
- 调试统计（`ps2_debug`、`motor_status`）不参与控制决策。

---

## 5. 操作

采用 O 型麦轮布局。M1 左前、M2 右前、M3 右后、M4 左后；轮位与极性都在 `Include/car_config.hpp`
（`kMotorWheel` / `kMotorPolarity`，四路极性经实车单轮校准，现全为 +1）。

**模式**：`Start` 进模式一（底盘，上电默认），`Select` 进模式二（机械臂）。
两个键都是直接入口、各自幂等，不是 toggle；急停锁定时不响应。

**模式一：底盘**

| 输入 | 行为 |
| --- | --- |
| 左摇杆 Y / X | 前进后退 / 左右平移，死区 16，满偏 100% |
| `L2` / `R2` | 逆时针 / 顺时针原地自转（`kRotateDutyPercent` = 80；同按 = 相抵） |
| 方向键 ↑↓←→ | 平移按钮备份（80%），覆盖左摇杆 |
| 右摇杆 / `L1` / `R1` / △○╳□ | 无功能（右摇杆整条通道已删） |

**模式二：机械臂精调**

| 输入 | 行为 |
| --- | --- |
| 左摇杆 X / Y | 末端笛卡尔速度：X 前伸/回收，Y 抬升/下压；平方律，满杆 50 mm/s，死区 8，松手即停。顶到边界不是停死，而是**贴边切向滑**（`Algorithm/src/arm_limit.cpp`，算式与实测见 `sim/README.md` §8） |
| `L2` / `R2` | 夹爪 松 / 紧（servo[2]，200 %/s；正行程 = 张开） |
| `L1` / `R1` | 腕 roll 逆 / 顺（servo[3]，150 %/s） |
| ↑ / ↓ / ← / → | 预设 P0 / P1 / P2 / P3 |
| △ / ○ / ╳ / □ | 预设 P4 / P5 / P6 / P7 |
| `R3` 单按 | 无功能（它只是急停组合的另一半） |

预设 P0~P7 的具体坐标见 `arm_config.hpp` 的 `kPoses`。扳机是开关量（报文只解按键位，没有压力字节），夹爪与腕只能恒速动，行程靠按住时长。
摇杆与扳机**可以同帧并行**：左手摆位的同时右手开合夹爪。

**急停与失效**

- `L3` + `R3` 同按 ≥ 200 ms 锁定急停，再按一次解除；两键必须先都松开才能再次触发。
  **`╳` 不再是急停**，它是预设 P6。
- 急停期间：底盘停转、机械臂冻结在当前姿态，`Start`/`Select`/预设全部不响应；
  停机期间按下的键不会被记住，解除后不会补执行。
- 手柄断线或有效报文超时 100 ms：电机停止，机械臂保持当前姿态（不回中）。
- 板上 Key（PA12）去抖 20 ms，按一次锁定、再按一次解除，锁定时底盘停止输出。

**注释里没写、但必须知道的两条机械事实**

- 轴 0（大臂）/ 轴 1（小臂）没有手动 jog 入口：只能由笛卡尔解算驱动，
  单路标定走 `-DARM_BRINGUP` 编译开关（见 §3、§7）。
- 舵机没有编码器 = 开环：`S1..S4` 与串口的 `xy` 都是**命令值**，不是实测角度/实测末端。

---

## 6. 串口输出

USART3（PC10），115200 8N1，每 500 ms 四行：

```
PS2 online=1 mode=73 op=ARM keys=0000 LX=128 LY=128 M=0,0,0,0 STOP=0 KEY=0 bad=0
PID4 target=0 ticks=0 pwm=0 stalled=0
M4HW-v1 req=0 ccr=1500 arr=65535 run=1 en=1 moe=1 oc=6 pa11=0 pb10=0 pb11=0 out=0,0 map=1A045
ARM S1=-19 S2=-51 S3=0 S4=0 xy=(190.0,10.0) tgt=(190.0,10.0) th=(-17.2,43.8) ok=1 lim=0 wall=0 poses=0x00 bu=0
```

- `mode=` 是 PS2 协议模式字（0x41 绿灯数字 / 0x73 红灯模拟 / 0x79 双震动），`op=` 才是操作模式（DRV/ARM）。
- `keys=` 是按键位图（低 16 位，1 = 按下），`bad=` 是校验失败的帧数，`STOP=` 是停车状态（急停 latch 或断线），
  `KEY=` 是板载 Key 去抖后的电平。
- `PID4` 是 M4 的闭环状态；`M4HW-v1` 是 TIM1/GPIO 的实配寄存器，用来对照「请求的占空比 vs 寄存器里真的是多少」。
- `S1..S4` 位置固定（标定手法依赖它）；`xy` 是 FK 回算的当前末端，`tgt` 是本帧解算用的目标，
  `th` 是连杆角；`ok`/`lim`/`wall` 三个**独立**标志：`ok=1` 精确命中（没缩步没裁角）、
  `lim=1` 一步都走不动（正对墙 / 墙角，整帧未下发）、`wall=1` 顶住边界但还在走
  （缩步 / 沿边切向滑，已下发）；
  `poses` 是上电自检失败的预设位掩码（`0x00` = 八条全好）；
  `bu=1` 表示这是 `ARM_BRINGUP` 标定版（正常版恒 0，标完看到 1 就是烧错了）。

---

## 7. 参数与标定清单（要填 / 要采集的值）

下面每个数都是「标定后该改的地方」；标着 TBD 的是占位值，顺序不能乱。

### 7.1 机械臂几何与标定（`Include/arm_config.hpp`）

| 项 | 现值 | 位置 | 怎么拿 |
| --- | --- | --- | --- |
| `upper_mm` / `fore_mm` | 76 / 40.08 | `kGeom` | 卡尺量两段**轴心距**（11.18 已测） |
| `shoulder.zero_deg` | 90.0 | `kGeom` | **必须是行程中点**：θ1∈[0,180] ⇒ 竖直向上 = 0% |
| `elbow.zero_deg` | 45.0 | `kGeom` | **必须是行程中点**：θ2∈[−45,+135] ⇒ 半折 = 0% |
| `shoulder/elbow.deg_per_percent` | 0.9 | `kGeom` | `ARM_BRINGUP` 推到两个点（如 +30 / +60），角度差 = k，带符号 |
| `shoulder.range_min/max_deg` | 0 / 180 | `kGeom` | `ARM_BRINGUP` 逐 10% 加到顶住，回退 3° |
| `elbow.range_min/max_deg` | −45 / 135 | `kGeom` | B 方案：前超伸 45°／后折叠 135°；内半径按侧不同 |
| `elbow_sign` | +1 | `kGeom` | 已定：高肘（上吊）；+1 取肘点更高的一支，−1 整体镜像 |
| `kWorkspace` | 70/116, −20/85 | `arm_config.hpp` | **软边界**：框内靠圆环钳位投影到弧带上 |
| `kPoses[8]` | 8 条占位坐标 | `arm_config.hpp` | 笛卡尔摆到工位，抄串口 `xy=` 与 `S1..S4` |
| 夹爪正负方向 | 正 = 张开 | `kPoses` | 标定第一件事：点 `L2`/`R2` 看方向，反了 8 条表全反号 |
| `sag_mm_per_100mm` | 0（关闭） | `kMotion` | 水平位测下垂：`下垂 mm / (x/100)` |
| `cart_speed_mm_per_second` | 50 | `kMotion` | 手感，满杆末端速度 |
| `slew_percent_per_second` | 300 | `kMotion` | 关节插值限速（编译期要过 4 倍余量自检） |
| `settle_percent` | 1.0 | `kMotion` | `at_target` 容差 |
| `jog_percent_per_second[4]` | 150/150/200/150 | `kMotion` | 肩/肘（占位）/夹爪/腕 |
| `stick_damping_mm_per_percent` | 0.2 | `kMotion` | 关节兜底阻尼 λ：只影响「顶住后沿边滑」。越小越跟手，奇异区越抖（0.02 时抖动 3.08 mm/帧²，0.2 时 0.061） |
| `stick_box_tol_mm` | 0.02 | `kMotion` | 矩形当墙的线性化残差容差：舵机一个步进 2.6 mm 的 1%。调 0 会被自己的残差判死、贴边整段不动 |

`kJointLimits` 不手填：由 `percent_of()` 从角度限位在编译期派生。改角度限位即可，别维护第二套百分比。

标定顺序：**量连杆 → 定零点 → 定比例 → 定限位 → 校验模型（推到 (105.2,38.3) 目测偏差 < 5 mm）
→ 填预设 → 上电不碰手柄看 `poses=0x00`**。

`arm_config.hpp` 末尾有一整套 `static_assert`：连杆为正、`|L1−L2| > 10`、限位不越 ±100、
工作区边界合理、`x_max ≤ L1+L2`、`slew_ok()`、预设全在工作区内。
**改了几何或工作区，8 条预设要一起改**，否则直接编不过 —— 这是保护，不是麻烦。

### 7.2 底盘速度闭环（`Device/src/motor.cpp:17-27`）

这几个数在 `motor.cpp` 的匿名命名空间顶部，不在 `Include/` 里：

| 项 | 现值 | 含义 / 怎么拿 |
| --- | --- | --- |
| `kTicksAtFullSpeed` | 20.0 | 100% 目标对应「每个 10 ms 控制周期的编码器计数」。空载给 100% 跑 1 s，读 `PID4 ticks=` 反算 |
| `kProportionalGain` | 3.0 | 速度环 Kp（%/tick） |
| `kIntegralGain` | 8.0 | 速度环 Ki（%/tick/s） |
| `kIntegralLimit` | 12.5 | 积分限幅（抗饱和），与 Kp/Ki 一起整定 |
| `kOutputMaxPercent` | 100.0 | PID 输出上限 |
| `kUpdateMs` / `kMaxDtMs` / `kEncoderStallMs` | 10 / 50 / 300 | 控制周期 / dt 钳位 / 堵转判定窗口 |

### 7.3 舵机脉宽（`Driver/src/outputs.cpp:50-51`）

| 项 | 现值 | 含义 |
| --- | --- | --- |
| `kNeutralUs` | 1500 | 0% 对应的脉宽（四路共用） |
| `kSpanUs` | 1000 | ±100% 对应 ±1000 µs，即 **500～2500 µs** |

四路共用一张映射表，没有单路零点/限幅。上电前先确认舵机允许的脉宽范围，
再确认「0% = 装配中位」对四路都成立。

### 7.4 手感与安全（`Include/car_config.hpp`）

| 项 | 现值 | 含义 |
| --- | --- | --- |
| `kMecanumLayout` | O | 滚子 O 型；改 X 型会翻平移符号 |
| `kMotorWheel[4]` / `kMotorPolarity[4]` | 见文件 | 轮位与极性；`mix_drive` 有编译期校验（每个轮位恰好一次、极性 ±1） |
| `kMaxDutyPercent` | 100 | 摇杆满偏占空比 |
| `kButtonDutyPercent` | 80 | 方向键备份平移占空比 |
| `kAxisDeadzone` | 16 | 底盘摇杆死区（原始 0..255 计数） |
| `kArmDeadzone` | 8 | 机械臂摇杆死区（要更精细） |
| `kRotateDutyPercent` | 80 | L2/R2 自转占空比 |
| `kEstopHoldMs` | 200 | L3+R3 同按判定的时间窗 |

---

## 8. 代码约定

- 公共类型用明确名称（`Ps2Frame`、`DriveCommand`、`ArmCommand`），函数用 `snake_case`。
- 配置集中在 `car_config` / `arm_config` 命名空间；文件内部细节用匿名命名空间。
  纯算法/对象级参数（PID 增益、满速计数）留在各自的 `.cpp` 里，不进几何配置文件。
- 普通结构体 + 自由函数：无动态分配、无继承、无通用硬件抽象框架。
- `.hpp` 声明接口，`.cpp` 实现行为；`Include/app.h` 是给 C 入口的薄桥接（必须 `extern "C"`）。
- 注释统一 doxygen：文件头 80 列 banner + `@file` / `@brief`；类型与函数用
  `@brief` / `@details` / `@note` / `@param` / `@return`；单行字段说明写行尾 `//`。
- `.clang-format` 只格式化自编代码；`Core/`、`Drivers/` 的生成代码不要批量格式化。
- 手工入口只写在 `USER CODE` 区域。重新 CubeMX 生成后要复查：SPI2 的 Mode 3 / LSB / 256 分频
  （`main.c` 的 `USER CODE SPI2_Init 2` 有强制纠正段）、PS2_CS 初值、Key 上拉。
- `build/` 是可重新生成的本机输出，不进版本库。

### 主机侧数值自检（不上板）

`Algorithm/`、`Device/arm_joint`、`Device/arm_pose`、`Task/mode_control` 都不 include HAL，
可以直接用 PC 的 g++ 编成 exe 跑：

```bash
g++ -std=c++14 -I Include -I Algorithm/inc -I Device/inc -I Driver/inc -I Task/inc \
    your_probe.cpp \
    Task/src/mode_control.cpp Device/src/arm_pose.cpp Device/src/arm_joint.cpp \
    Device/src/drive_control.cpp Device/src/ps2_protocol.cpp Device/src/arm_control.cpp \
    Algorithm/src/arm_kinematics.cpp Algorithm/src/arm_limit.cpp Algorithm/src/pid.cpp -o probe.exe
```

用它验：正逆解互逆误差、8 条预设可达性、限位与 jog 会不会被绕过、急停/断线时是否真的冻结。
比上车试便宜得多。
