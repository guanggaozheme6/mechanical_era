# arm_pose v0.1 整合匹配度报告

> 复查对象：`arm_pose_v0.1.md` 的设计与骨架代码 × `mechanical_era` 现有工程。
> 复查方式：静态逐文件对照 + **在 PC 上用 g++ 实际编译运行候选实现** + **用 arm-none-eabi 交叉编译验证目标平台可编译可链接**。
> 复现物：`C:\Users\guang\AppData\Local\hermes\cache\scratch\ik_probe\`（`arm_kinematics.hpp/.cpp`、`test_ik.cpp`）
> 结论：**可以动工，但有 2 个必须先修的设计缺陷、1 个必须砍掉的范围项、4 个需要你拍板的决策点。**

---

## 0. 一句话结论

设计方向和现有代码**没有结构性冲突**——不需要改 `mode_control`、`outputs`、`ps2`、`drive_control`，IK 真能做成一个只依赖 `<cmath>` 的纯计算模块（已在 PC 上编过、跑过、数值全过）。但设计稿里的骨架有 **2 个会直接编译失败 / 产生 NaN 的硬缺陷**，另有一个"串口命令"项会突破"不动生成代码"的边界，建议从 v0.1 砍掉。

| 维度 | 匹配度 | 说明 |
|---|---|---|
| 模块边界 | ✅ 100% | 新模块只依赖 `<cstdint>` + `<cmath>`，不碰 HAL |
| 现有接口签名 | ✅ 100% | `arm_pose_request/update/hold/value` 全部不需要改签名 |
| `mode_control.cpp` | ✅ 不动 | 唯一要动的是 `arm_pose.cpp` 内部 |
| `outputs.cpp` | ✅ 不动 | 百分比 → 脉宽那一层完全复用，且自带 ±100 钳位 |
| C++ 标准 | ❌ 需改 | C++14 不支持设计稿里的 designated initializers，**编不过** |
| 数值健壮性 | ❌ 需改 | `(0,0)` 目标会产生 NaN → percent 溢出 INT_MIN（已实测复现） |
| 串口命令 | ⚠️ 越界 | 需要 USART3 RX 中断，会改 `Core/Src/main.c` + `stm32f1xx_it.c` |
| 资源占用 | ✅ 无压力 | flash 29144 B / 512 KB（5.6%），IK 增量约 2~4 KB |

---

## 1. 已实测验证的硬事实

不是推演，是真跑出来的：

### 1.1 目标平台可编译、可链接

```
arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O2 -std=c++14 -Wall -Wextra -Werror \
  -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -c arm_kinematics.cpp
→ COMPILE OK，text = 1364 B，data = 0，bss = 0
```

未解析符号只有两类，全部有库对应：

| 类别 | 符号 | 提供方 |
|---|---|---|
| libm | `acosf` `atan2f` `sqrtf` `sinf` `cosf` `fabsf` | `libm.a`（已用 `nm` 确认 `T acosf` 等全部存在） |
| libgcc 软浮点 | `__aeabi_fadd/fmul/fdiv/fcmp*/f2iz/i2f` | 内置，M3 无 FPU 的正常结果 |

链接行里 **`-lm` 已经在**（`build/Debug/build.ninja` 的 `LINK_LIBRARIES = -lm`，来自 `cmake/gcc-arm-none-eabi.cmake:47`），不需要额外配置。toolchain 的 `-fno-rtti -fno-exceptions -fno-threadsafe-statics`（`cmake/gcc-arm-none-eabi.cmake:40`）与本模块无关——不用异常、不用 RTTI、没有函数内静态变量。

### 1.2 同一份代码在 PC 上可以跑单元测试

因为 IK 模块**零 HAL 依赖**，`g++ -std=c++14 -Wall -Wextra -Werror` 直接编译通过并跑完全部用例：**ALL PASS，0 failures**。这是个额外收益：以后标定值一变，不用烧板子就能回归。

### 1.3 完整用例输出（可直接当验收基准，L1=110 / L2=95）

| 请求 (mm) | θ1 | θ2 | S1 | S2 | FK 回算 | 误差 | reach | lim |
|---|---|---|---|---|---|---|---|---|
| (205, 0) | 0.00° | 0.00° | 0% | −100% | (205.00, 0.00) | 0.00 | 1 | 0 |
| (180, 0) | −26.38° | 57.35° | −29% | −36% | (179.78, 1.24) | 1.26 | 1 | 0 |
| (145, 0) | −40.93° | 90.27° | −45% | 0% | (145.34, 0.80) | 0.87 | 1 | 0 |
| (100, 0) | −53.54° | 122.16° | −59% | 36% | (99.63, 0.90) | 0.98 | 1 | 0 |
| (60, 0) | −59.62° | 146.98° | −66% | 63% | (60.47, 0.21) | 0.52 | 1 | 0 |
| (15, 0) | 0.00° | 180.00° | 0% | 100% | (15.00, 0.00) | 0.00 | 1 | 0 |
| (150, 120) | 19.73° | 40.99° | 22% | −54% | (149.26, 120.51) | 0.90 | 1 | 0 |
| (100, 150) | 30.07° | 57.03° | 33% | −37% | (101.51, 149.31) | 1.66 | 1 | 0 |
| (0, 205) | 90.00° | 0.00° | 100% | −100% | (0.00, 205.00) | 0.00 | 1 | 0 |
| (150, 140) | 43.03° | 0.00° | 48% | −100% | (149.44, 140.33) | 0.65 | **0** | 0 |
| (−80, 0) | 90.00° | 134.79° | 100% | 50% | (−67.18, 42.82) | 44.70 | 1 | **1** |
| (300, 0) | 0.00° | 0.00° | 0% | −100% | (205.00, 0.00) | 95.00 | **0** | 0 |
| (0, 0) | 0.00° | 180.00° | 0% | 100% | (15.00, 0.00) | 15.00 | 0 | 0 |

三条从这张表里读出来的结论，比数字本身重要：

1. **FK 回算误差 0.5~1.7 mm 全部来自 percent 取整**（±0.5% = ±0.45° = 末端 ±1.6 mm），不是算法误差。所以 5 mm 验收阈值合理，不必追求更低——除非把 `g_target[]` 改成 0.1% 分辨率（但 `ArmCommand.servo[]` 是 int，下游无法表达）。
2. **`(150,140)` 不可达，但 FK 误差只有 0.65 mm**——差 0.18 mm 够不着，钳位后几乎重合。**绝不能靠"回算误差小"来判断可达**，必须看 `reachable`。
3. **`(−80,0)` 在工作空间内（reach=1）但关节超限（lim=1），末端偏 44.7 mm**。两个标志必须独立，这条也被实测证明了。

---

## 2. 必须修的缺陷（按严重度）

### D1 —— C++14 编不过：designated initializers【致命，已实测】

设计稿第 4 节和第 8 节写的是：

```cpp
constexpr ArmModel kArm = {
    .upper_mm = 110.0F, .fore_mm = 95.0F, ... };   // ← C++20 语法
```

工程是 **C++14**（`CMakeLists.txt:7`，`CMAKE_CXX_STANDARD 14` + `CXX_EXTENSIONS OFF`），`-Werror` 下直接报错。必须改成按声明顺序的位置初始化：

```cpp
const ArmModel kArm = {
    110.0F, 95.0F,                      // upper_mm, fore_mm
    {0.0F, 0.9F, -90.0F, 90.0F},        // shoulder: zero_deg, deg_per_percent, min, max
    {90.0F, 0.9F, 0.0F, 180.0F},        // elbow
    +1, 2U, 3U};                        // elbow_sign, gripper_servo, wrist_servo
```

副作用：字段顺序从此变成 ABI 约定，**以后在结构体中间插字段会静默错位**。缓解办法是在 `ArmModel` 定义上方写一行注释"字段顺序即初始化顺序，只在末尾追加"，或者在 `arm_kinematics.cpp` 里给每个初值加行尾注释（如上）。

### D2 —— 目标落在肩轴上时除以 0，NaN 一路传到 TIM8【致命，已实测复现】

设计稿骨架的钳位分支是：

```cpp
} else if (r < std::fabs(l1 - l2)) {
    const float k = std::fabs(l1 - l2) / r;   // r == 0 → inf → 0*inf = NaN
```

实测 `solve_xy(0, 0)` 的输出：

```
th1=nan th2=nan  S1=-2147483648% S2=-2147483648%  FK(-87.26,121.81)
```

`-2147483648` 就是 `INT_MIN`，是 NaN 转 int 的 UB 结果。它会进 `apply_servos()`：`1500 + 1000 * (-2147483648) / 100` 整数溢出 → 写进 TIM8 CCR 的脉宽离谱 → 舵机乱抽甚至堵转。

**修法（已验证）**：r 小于 1 mm 时不做除法，直接把目标固定放到 `+x` 轴上的 `|L1−L2|` 处，置 `reachable=false`：

```cpp
constexpr float kMinRadiusMm = 1.0e-3F;
if (r < kMinRadiusMm) { tx = rmin; ty = 0.0F; }
else { const float k = rmin / r; tx = x_mm * k; ty = y_mm * k; }
```

另外在 `deg_to_percent()` 里加 `!std::isfinite(deg)` 兜底返回 0，双保险。

### D3 —— 限位边界点误报 `limited`【低，已实测】

`d1 != d1_raw` 这种浮点相等比较在边界（`θ1` 正好 90°、`θ2` 正好 180°）会误报。实测 `(0,15)` 原本报 `lim=1`，实际正好在上限内。改成带容差：

```cpp
constexpr float kLimitEpsDeg = 1.0e-3F;
s.limited = (std::fabs(d1 - d1_raw) > kLimitEpsDeg) || (std::fabs(d2 - d2_raw) > kLimitEpsDeg);
```

### D4 —— `Solution` 字段与文档不一致【低】

设计稿第 8 节的 `Solution` 只有 `shoulder_percent / elbow_percent / x_mm / y_mm / reachable / limited`，但串口打印和标定核对需要**角度**。候选实现里补了两个字段：

```cpp
float shoulder_deg;
float elbow_deg;
```

它们不增加下游负担（只是返回值多两个 float），但能让 `report_arm` 直接打 `θ1/θ2`。

### D5 —— `kGripperServo` 与 `kArm.gripper_servo` 双真值【中，设计债】

`src/arm_pose.cpp:40` 已有 `constexpr unsigned kGripperServo = 2U;`，设计稿又在 `kArm` 里放了 `gripper_servo`。两处都写 2 时没事，一旦改动就是"标定改了 A、逻辑还读 B"。

**建议**：删 `kGripperServo`，`arm_pose_gripper_servo()`（`src/arm_pose.cpp:160`）改为 `return arm_geometry::kArm.gripper_servo;`。调用点只有 `src/mode_control.cpp:93` 一处，零风险。

### D6 —— 姿态表不可达时上层无感知【中，行为变化】

现在 `arm_pose_request()` 从表里取四个 int 写进 `g_target[]`，永远"成功"。改成坐标 + IK 后，`kPoseTable` 某个点可能 `reachable=false`（例如你以后把 Ready 点写到 210 mm）。

**若不给上层信号，表现是"按了 ○ 但臂只走到边界就不动了"**，操作员无法区分"到边界"和"卡住"。

两个方案，建议选 B：

| 方案 | 改动 | 代价 |
|---|---|---|
| A. `ControlOutput` 加 `arm_reachable` 字段 | `mode_control.hpp` + `mode_control.cpp:145-149` + `main.cpp` | 改 3 个文件，破坏"不动 mode_control"的承诺 |
| **B. 加查询函数 `arm_pose_last_solution()`** | 只改 `arm_pose.cpp/.hpp` + `diagnostics.cpp` | 0 个已承诺不改的文件 |

B 的用法：`arm_pose_request()` 里保存最近一次 `Solution`，`report_arm()` 每 500 ms 打一次 `reach=.. lim=..`。串口看得见就够了，不需要进控制流。

---

## 3. 必须砍掉的范围项

### R1 —— 串口 `P x y` / `F` 命令不属于 v0.1

设计稿第 10 节提了串口命令，但现状是：

- `MX_USART3_UART_Init()`（`Core/Src/main.c`）配的是 `UART_MODE_TX_RX`，但**从未启动过接收**（没有 `HAL_UART_Receive_IT`，`stm32f1xx_it.c` 里也没有 `USART3_IRQHandler` 的 HAL 调用）。
- 要做命令解析，必须：改 `Core/Src/main.c` 加 `HAL_UART_Receive_IT` → 改 `Core/Src/stm32f1xx_it.c` 加中断服务 → 新增环形缓冲 → 在 `main.cpp` 主循环里轮询解析。

**这会动两处 CubeMX 生成代码**，而 README 明确约定"生成文件中的手工入口保留在 USER CODE 区域"，改中断文件有被重新生成覆盖的风险。

**建议**：v0.1 不做串口命令。验证手段改为：
1. `kPoseTable` 里四个点直接填坐标，手柄 △○□L2 触发，看动作对不对；
2. `report_arm` 增加 `θ1/θ2/reach/lim/FK` 打印，肉眼核对；
3. 需要任意坐标时，临时改 `kPoseTable` 一行重新烧录（迭代一次 30 秒）。

串口命令挪到 v0.1.1，单独排期，那时再认真做 RX 中断 + 缓冲。

---

## 4. 接口兼容性逐项核对

### 4.1 不需要改动的文件（零修改）

| 文件 | 为什么不用动 |
|---|---|
| `src/mode_control.cpp` | 四条姿态键（`:67-78`）只调 `arm_pose_request(ArmPose)`，签名不变；夹爪 jog（`:85-94`）走 `arm_pose_set_jog`，不变；急停分发（`:143`）不变 |
| `src/outputs.cpp` | `apply_servos()`（`:29-39`）拿 int percent 换算脉宽，还自带 ±100 钳位（`:34-35`）——IK 层即使出界，这里也能兜一道 |
| `src/ps2.cpp` / `ps2_protocol.cpp` | 与机械臂无关 |
| `src/drive_control.cpp` / `motor_pid.cpp` | 与机械臂无关 |
| `src/main.cpp` | 主循环调用顺序不变；`report_arm` 签名不变 |
| `Core/Src/main.c` | TIM8 四路 PWM（`:228` 起）本来就是 20 ms 周期、比较值=微秒，不用动 |
| `inc/car_config.hpp` | `kArmDeadzone`、`kJogPercentPerSecond`、`kSlewPercentPerSecond` 全部沿用 |

### 4.2 需要改动的文件（4 个）

| 文件 | 改动 | 风险 |
|---|---|---|
| `CMakeLists.txt` | `target_sources` 加 `src/arm_kinematics.cpp`（第 22-32 行列表内） | 无 |
| `inc/arm_kinematics.hpp` + `src/arm_kinematics.cpp` | 新增 | 无，已在两套编译器上验证 |
| `inc/arm_pose.hpp` | 加 `arm_pose_request_xy()`、`arm_pose_last_solution()`、`arm_pose_deg()` 三个声明 | 无（纯追加） |
| `src/arm_pose.cpp` | `kPoseTable` 改坐标表；`arm_pose_request` 内调 IK；删 `kGripperServo`；`kLimitMin/Max` 由角度派生 | **主要工作量** |
| `src/diagnostics.cpp` | `report_arm` 增打印 θ1/θ2/reach/lim | 无（只加字段，不改前 8 个字段位置，标定手法不受影响） |
| `README.md` | 机械臂标定一节改写 | 无 |

### 4.3 签名兼容性

| 现有接口 | v0.1 后 | 兼容 |
|---|---|---|
| `void arm_pose_init(uint32_t)` | 不变 | ✅ |
| `void arm_pose_request(ArmPose)` | 签名不变，**语义变为"坐标+IK"**；不可达时钳位到边界 | ⚠️ 语义变化，需要 D6 的信号 |
| `void arm_pose_set_jog(unsigned, int)` | 不变，仍直接改百分比、绕过 IK | ✅ 这是特性不是漏洞 |
| `ArmCommand arm_pose_update(uint32_t)` | 不变 | ✅ |
| `ArmCommand arm_pose_hold(uint32_t)` | 不变 | ✅ |
| `int arm_pose_value(unsigned)` | 不变（百分比） | ✅ |
| `bool arm_pose_at_target()` | 不变（比百分比差） | ✅ |
| `unsigned arm_pose_gripper_servo()` | 内部改为读 `kArm.gripper_servo` | ✅ |

**关键设计确认**：`solve_xy()` **只在 `arm_pose_request()` / `arm_pose_request_xy()` 里调用一次**，不进 `arm_pose_update()` 的每帧循环。所以：

- 摇杆 jog 与 IK 不会互相打架（时间上互斥，jog 是每帧消费的 `g_jog[]`，IK 是事件驱动的目标写入）；
- 20 ms 控制周期里没有三角函数开销。

### 4.4 限位域的一致性（必须留意）

目前 `kLimitMin/Max = ±100`（`src/arm_pose.cpp:36-37`）恰好等价于角度限位，因为：

| 关节 | 角度限位 | `zero_deg` | 换算 percent |
|---|---|---|---|
| 大臂 | −90°…+90° | 0° | −100…+100 |
| 小臂 | 0°…+180° | 90° | −100…+100 |

**但一旦你把结构限位收紧**（必然要收，小臂 ±180 会叠回大臂），两者就脱钩。例如小臂实测只能到 `θ2 ∈ [10°, 170°]`：

```
percent = (deg − 90) / 0.9  →  −88.9 … +88.9
```

此时 `kLimitMin/Max` 必须同步改成 `{−89, 89}`（小臂那一路），否则 IK 输出的角度虽然合法，`clamp_joint` 仍放行到 ±100，舵机会顶到结构。

**建议实现**：在 `arm_pose.cpp` 里用 constexpr 函数从 `kArm` 派生，只保留一套真值：

```cpp
constexpr int percent_of(const arm_geometry::JointCalib &c, float deg) {
    return static_cast<int>((deg - c.zero_deg) / c.deg_per_percent + (deg >= c.zero_deg ? 0.5F : -0.5F));
}
constexpr int kLimitMin[4] = {percent_of(kArm.shoulder, kArm.shoulder.min_deg),
                              percent_of(kArm.elbow, kArm.elbow.min_deg), -100, -100};
```

（C++14 的 constexpr 函数允许 if/三元，但不能调 `std::` 数学函数——上面只用四则运算，合法。）

---

## 5. 资源与性能

| 项 | 数值 | 判断 |
|---|---|---|
| flash 现状 | text 29144 B / 512 KB = 5.6% | 宽裕 |
| IK 模块本体 | `arm_kinematics.o` text 1364 B | 可忽略 |
| libm 引入（acosf/atan2f/sinf/cosf/sqrtf） | 估 2~3 KB（未做完整链接实测） | 仍然宽裕，预计总占用 < 7% |
| RAM | `kArm` 在 `.rodata`（flash），`Solution` 约 28 B 栈上 | 无影响 |
| 单次 `solve_xy` 耗时 | 未在目标板实测；含 2×sqrt + 1×acos + 2×atan2 + 2×sin/cos + 软浮点除法，M3 @72 MHz 估算 **十微秒量级** | 每帧不调用，20 ms 周期内无感。要精确值就用 DWT 计数器包一层测 |

> 注意 `-Os` 的 Release 构建下 `arm_kinematics.o` 会更小，上面是 `-O2` 的结果。

---

## 6. 需要你拍板的 4 个决策点

| # | 决策 | 我的建议 | 影响 |
|---|---|---|---|
| **P1** | 串口 `P x y` / `F` 命令：v0.1 做还是砍？ | **砍**，挪 v0.1.1 | 做就要动 `Core/Src/main.c` + `stm32f1xx_it.c` |
| **P2** | 不可达姿态的反馈方式：改 `ControlOutput` 还是加查询函数？ | **加 `arm_pose_last_solution()`**，不动 `mode_control` | 选改结构体就要改 3 个文件 |
| **P3** | `kLimitMin/Max`：保持手填 ±100，还是从角度 constexpr 派生？ | **派生**（一套真值） | 手填会在你收紧限位时漏改 |
| **P4** | `kArm` 放哪：`arm_kinematics.cpp` 私有定义，还是进 `car_config`？ | 放 **`arm_kinematics.cpp` 顶部**，和现在 `kPoseTable` 放 `arm_pose.cpp` 顶部的风格一致 | 进 `car_config` 会让那个文件变成"什么都装" |

---

## 7. 建议的落地顺序（每步可独立编译验证）

1. **加 `arm_kinematics.hpp/.cpp` + CMakeLists 一行**，此时没人调用，编译应零警告通过。
2. **`.cpp` 里 `kArm` 先按当前的 ±180/±90 填**，跑一次 PC 侧的 `test_ik`（我可以把测试文件也放进仓库 `tests/`，用 g++ 跑，不参与固件构建）。
3. **改 `arm_pose.cpp` 的 `kPoseTable` → 坐标表**，`arm_pose_request` 内接 IK，删 `kGripperServo`，派生 `kLimitMin/Max`。
4. **`diagnostics.cpp` 加 θ/reach/lim 打印**，烧板子空转（不上负载），用手柄触发四个姿态看数值和动作。
5. **标定 TBD-1~4**（连杆长度、零点、0.9°/%、结构限位），每填一项重算一次 PC 测试的基准表。
6. 最后改 `README.md`，写清"坐标指腕点、`+x` 车头、`zero_deg` 小臂取 90°"。

第 1~2 步完全不影响现有固件行为，可以放心先合。

---

## 8. 附带发现（不影响动工，但值得知道）

1. **`build/Debug/build.ninja` 里的路径是 `D:/proj/mc_area_Group_proj/mechanical_era`**，不是当前目录。这份 build 是从旧位置拷过来的缓存，重新 `cmake --preset Debug` 会重写。不影响任何结论，但别用旧的 build 目录判断"编译通过"。
2. **`src/diagnostics.cpp:32` 里 `"PID4 ...\\\\r\\\\n"`**——`.cpp` 源码里写成了 `\\r\\n`（双反斜杠），实际输出的是字面 `\r\n` 四个字符而不是回车换行。这是既有小 bug，与本次改动无关，顺手可以修（改成 `"\r\n"`）。
3. **`arm_pose_request()` 目前会把四路全写一遍**（`src/arm_pose.cpp:91-94`）。改坐标表后记得保留 `wrist` 那一列的写入，否则 `servo[3]` 会停在上一姿态的残留值——虽然 v0.1 的 `PoseTarget.wrist` 全是 0，但字段要接上。
