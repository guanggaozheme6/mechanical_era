# arm_pose v0.2 —— 笛卡尔精调 + 顶层配置结构体

> 本文是 `arm_pose_v0.1.md` 的增量。v0.1 的几何模型、逆解公式、六条硬防护、四条实测结论**全部继续有效**，本文只写变更。
> 整合报告：`arm_pose_v0.1_integration_report.md`（本次全部按其中的修正版骨架执行）。
> 基准数值：L1=110 / L2=95（示例值，你填真值后重算）。

---

## 0. 相对 v0.1 的变更总览

| 项 | v0.1 | v0.2 | 来源 |
|---|---|---|---|
| 精调方式 | 左摇杆 Y → 单关节百分比 jog | **左摇杆 X/Y → 末端目标点 (x,y)，臂自己追** | P1 |
| 串口 `P x y` / `F` 命令 | v0.1 不做 | **仍然不做**（理由不变：要动 CubeMX 生成的中断文件） | — |
| 不可达反馈 | 查询函数，不进控制流 | **`ControlOutput` 增加 `ArmFeedback`** | P2 |
| 限位真值 | 角度限位 + 百分比派生（两处） | **只有角度限位，全部进一个顶层结构体** | P3 |
| 配置位置 | `arm_kinematics.cpp` 私有 | **`inc/arm_config.hpp` 顶层 `constexpr`** | P4 |
| percent 精度 | `int` | **`float`（精调必须，否则会卡顿跳变，见第 4 节）** | 新发现 |

---

## 1. 顶层配置：`inc/arm_config.hpp`（P3 + P4）

一个结构体装完全部：连杆长度、每关节零位与比例、**舵机允许的连杆角度范围**、通道绑定、笛卡尔工作区、满杆速度。

```cpp
// inc/arm_config.hpp
#pragma once

namespace arm_config {

// 单个关节的机械标定 + 舵机允许的连杆角度范围（唯一真值）
struct Joint {
    float zero_deg;         // percent == 0 时连杆相对 +x 的角度（大臂 0=水平，小臂 90=垂直大臂）
    float deg_per_percent;  // 带符号；0.9 表示 1% 行程 = 0.9°
    float range_min_deg;    // 舵机/结构允许的最小连杆角
    float range_max_deg;    // 舵机/结构允许的最大连杆角
};

struct Model {
    // —— 连杆 ——
    float upper_mm;         // L1 肩轴心 → 肘轴心
    float fore_mm;          // L2 肘轴心 → 腕点
    // —— 关节 ——
    Joint shoulder;         // servo[0]
    Joint elbow;            // servo[1]
    int   elbow_sign;       // +1 = 小臂向上兜
    // —— 通道 ——
    unsigned gripper_servo; // servo[2] 松紧
    unsigned wrist_servo;   // servo[3] roll，不参与解算
    // —— 笛卡尔精调工作区（矩形，超出即钳位，防止撞奇异区/穿车体）——
    float area_x_min_mm;
    float area_x_max_mm;
    float area_y_min_mm;
    float area_y_max_mm;
    // —— 手感 ——
    float jog_speed_mm_per_second;  // 摇杆满偏时的末端速度
};

// ⚠️ C++14：只能位置初始化，字段顺序 = 初始化顺序，以后只在末尾追加
constexpr Model kArm = {
    110.0F, 95.0F,                              // upper_mm, fore_mm           TBD-1
    {  0.0F, 0.9F, -90.0F,  90.0F},             // shoulder  zero,deg/%,min,max TBD-2/3/4
    { 90.0F, 0.9F,   0.0F, 180.0F},             // elbow     zero 必须 90°      TBD-2/3/4
    +1,                                         // elbow_sign = 向上兜
    2U, 3U,                                     // gripper_servo, wrist_servo
    70.0F, 205.0F, -20.0F, 120.0F,              // 工作区矩形 mm
    50.0F,                                      // 满杆 50 mm/s
};

// 编译期自检：工作区不能超出连杆总长，也不能落进折叠死区
static_assert(kArm.area_x_max_mm <= kArm.upper_mm + kArm.fore_mm,
              "工作区 x_max 超出 L1+L2，永远走不到");
static_assert(kArm.area_x_min_mm > 0.0F, "工作区 x_min 必须为正，否则末端会穿过肩轴");
static_assert(kArm.jog_speed_mm_per_second > 0.0F, "满杆速度必须为正");

// 角度范围必须落在舵机 ±100% 行程内，否则 range_* 与 percent 钳位互相打架
constexpr bool servo_range_ok(const Joint &j) {
    return (j.range_max_deg - j.zero_deg) / j.deg_per_percent <= 100.0F &&
           (j.range_min_deg - j.zero_deg) / j.deg_per_percent >= -100.0F;
}
static_assert(servo_range_ok(kArm.shoulder), "大臂角度范围超出舵机行程");
static_assert(servo_range_ok(kArm.elbow), "小臂角度范围超出舵机行程");

} // namespace arm_config
```

### 1.1 为什么限位只放这一处（P3）

百分比限位不再手填，全部 `constexpr` 派生，杜绝"改了一处忘另一处"：

```cpp
// src/arm_pose.cpp
constexpr int percent_of(const arm_config::Joint &j, float deg) {
    const float raw = (deg - j.zero_deg) / j.deg_per_percent;
    return static_cast<int>(raw >= 0.0F ? raw + 0.5F : raw - 0.5F);
}
constexpr int kLimitMin[4] = {percent_of(arm_config::kArm.shoulder, arm_config::kArm.shoulder.range_min_deg),
                              percent_of(arm_config::kArm.elbow,    arm_config::kArm.elbow.range_min_deg),
                              -100, -100};
constexpr int kLimitMax[4] = {percent_of(arm_config::kArm.shoulder, arm_config::kArm.shoulder.range_max_deg),
                              percent_of(arm_config::kArm.elbow,    arm_config::kArm.elbow.range_max_deg),
                              100, 100};
```

`clamp_joint()`（`src/arm_pose.cpp:54-62`）继续用 `kLimitMin/Max`，**一个字节都不用改**。夹爪两路保持 ±100。

### 1.2 工作区矩形的取值依据

`area_x_min_mm` 就是"小臂最多能折多少"的直接后果，按实测 `range_max_deg` 查表：

| 小臂 `range_max_deg` | 对应的 `area_x_min_mm` |
|---|---|
| 90° | 145.3 |
| 120° | 103.3 |
| 150° | 55.0 |
| 170° | 23.3 |
| 180° | 15.0 |

推导见 v0.1 的 12.2 节。默认给的 70 mm 对应 θ2 ≈ 143°，是"留了余量、又不至于完全够不着近处"的折中值，你标定后按上表改。

`area_y_max_mm = 120` 是保守值（大臂竖起来能到 205，但那样力矩全压在舵机上），按实际任务调。

---

## 2. 笛卡尔精调（P1）—— 核心

### 2.1 交互定义

```
        操作手站在车侧面（正好是车体纵剖面 = x-y 平面）
                          ↑ y (上)
                          |
                    ┌─────┼─────┐   area_y_max
                    │     │     │
                    │   ● 末端   │       左摇杆：
                    │  ↖ ↑ ↗    │         ↑ 抬高末端（+y）
     area_x_min ────┼───────────┼──── area_x_max → x
                    │  ← · →    │         → 前伸（+x）
                    │  ↙ ↓ ↘    │         ← 回收（−x）
                    └─────┼─────┘         ↓ 降低(−y)
                          |（肩轴）
```

- **左摇杆 = 末端速度矢量**，不是位置。推着走，松手停（目标点冻结在当前位置）。
- 满杆速度 `jog_speed_mm_per_second`（默认 50 mm/s），**平方律**映射，小偏移自动变成精调。
- 右摇杆不参与机械臂（保持 v0.1 的约定，避免与底盘抢）。

### 2.2 平方律速度映射（含死区）

```cpp
// -100..100 的摇杆百分比（已扣 kArmDeadzone=8）→ 速度
const float f = static_cast<float>(stick) / 100.0F;
const float v = arm_config::kArm.jog_speed_mm_per_second * f * std::fabs(f);  // 保号平方
```

实算（v_max=50）：

| 摇杆偏移 | 末端速度 | 20 ms 步长 |
|---|---|---|
| 10% | 0.50 mm/s | 0.010 mm |
| 20% | 2.00 mm/s | 0.040 mm |
| 35% | 6.12 mm/s | 0.122 mm |
| 50% | 12.50 mm/s | 0.250 mm |
| 70% | 24.50 mm/s | 0.490 mm |
| 100% | 50.00 mm/s | 1.000 mm |

平方律的意义：操作手"轻推"就落在 2 mm/s（肉眼几乎看不出动，适合对准），"推到底"才是 50 mm/s（大范围转移）。一个摇杆兼顾粗调与精调，不需要额外的速度档按键。

> 注意 20% 时步长只有 **0.04 mm** —— 这正是第 4 节"必须用 float percent"的直接原因。

### 2.3 积分方式：闭环于「当前实际末端」，不是开环目标点

这是整个 v0.2 最关键的一个决策。

```
每帧：
  (fx, fy) = FK(g_current[0], g_current[1])      ← 从「当前实际输出」反算，不是上一帧的目标
  tx = clamp(fx + vx*dt, area_x_min, area_x_max)
  ty = clamp(fy + vy*dt, area_y_min, area_y_max)
  sol = solve_xy(tx, ty)
  g_target[0] = sol.shoulder_percent_f            ← float
  g_target[1] = sol.elbow_percent_f
```

**为什么不用 `g_cart_target += v*dt` 的开环积分：**

1. **slew 限速造成的滞后会累积**。`g_current` 以 300%/s 追 `g_target`，若目标是开环累加的，摇杆推久一点目标就会"跑在"实际位置前面几毫米，松手后臂还会自己再走一段（追尾），操作手会认为"松手不停"。
2. **目标被钳位后误差不会自愈**。顶到工作区边界时开环目标继续累加，往回推要先"消化"掉这段虚假距离，手感像有回弹。
3. 闭环每帧都从实际位置出发，**永远只走一个步长**，上面两个问题同时消失。

顺带一个好处：闭环意味着末端速度由摇杆决定，**与 slew 参数无关**，不会出现"改了 slew 速度就变了"的耦合。

slew 是否够用？最坏情况实算：

| 末端速度 | 到肩距离 r | 需要的关节角速度 | 现有 slew |
|---|---|---|---|
| 50 mm/s | 60 mm | 47.7 °/s = **53 %/s** | 300 %/s ✅ |
| 50 mm/s | 150 mm | 19.1 °/s = 21 %/s | 300 %/s ✅ |
| 20 mm/s | 60 mm | 19.1 °/s = 21 %/s | 300 %/s ✅ |

**余量 5 倍以上，`kSlewPercentPerSecond` 保持 300 不用动。**

### 2.4 钳位：先矩形，后圆环

两级钳位，顺序不能反：

1. **矩形**（`arm_config` 的 `area_*`）：挡住穿肩轴（负 x）、挡住过高（力矩）、挡住折叠死区。命中时置 `at_wall = true`。
2. **圆环**（`solve_xy` 内部）：`|L1−L2| ≤ r ≤ L1+L2`，挡住够不着。命中时置 `reachable = false`。

**命中任一级时，不要更新 `g_target`**（保持上一帧合法值），只置标志。理由：钳位后的点往往在奇异区边缘（比如 r 刚好 = L1+L2 时 θ2 = 0，雅可比退化），照着走会出现"推摇杆不动、一推就弹"的抖动。停住更稳。

### 2.5 模式切换：标定模式自动回到关节 jog

现有按键语义**一个都不改**，用已有的 `g_calibrating` 做开关：

| 状态 | 左摇杆 | ↑ ↓ | L1/R1 |
|---|---|---|---|
| 机械臂模式（`cal=0`） | **笛卡尔：控制末端 (x,y)** | 无作用（保留给别的功能） | 夹爪松紧 |
| 机械臂模式（`cal=1`，START 进入） | **关节 jog：微调选中的那一路** | 选关节 1→4 循环 | 夹爪松紧 |

标定流程完全复用现有手法（推到碰结构 → 抄 `S1..S4`），一行都不用改。这是"不动标定链路"的关键设计。

切换瞬间必须**重新对齐基准**，否则会跳：

- 进标定（笛卡尔 → 关节）：无需动作，笛卡尔的目标已经等于当前末端。
- 出标定（关节 → 笛卡尔）：`(fx, fy) = FK(g_current)` 就是基准，天然连续 ✅（闭环积分的副产品）
- `arm_pose_request(ArmPose)` 切姿态后：也要把内部基准刷新到该姿态坐标，否则下一次推摇杆会从旧位置算起。

### 2.6 撞墙反馈

`at_wall = true` 或 `reachable = false` 时，串口打标记（见第 3 节 `ArmFeedback`）。操作手在侧面看到"臂不动了"时，串口同时在报 `wall=1`，能区分"顶到边界"和"卡住/没电"。

---

## 3. `ControlOutput` 改动（P2）

新增一个聚合结构，只加**一个**字段，避免把 `ControlOutput` 撑散：

```cpp
// inc/mode_control.hpp
struct ArmFeedback {
    float x_mm = 0.0F;          // 当前末端（FK 回算，基于 g_current）
    float y_mm = 0.0F;
    float target_x_mm = 0.0F;   // 本帧下发的目标点
    float target_y_mm = 0.0F;
    float shoulder_deg = 0.0F;
    float elbow_deg = 0.0F;
    bool  reachable = true;     // 工作空间（圆环）内
    bool  limited = false;      // 关节角被 range_*_deg 裁剪
    bool  at_wall = false;      // 目标被工作区矩形钳位
    bool  cartesian = true;     // true=笛卡尔精调，false=标定时的关节 jog
};

struct ControlOutput {
    DriveCommand drive;
    ArmCommand arm;
    ArmFeedback arm_fb;          // ★ 新增
    OperationMode mode = OperationMode::Drive;
    bool stopped = false;
    bool calibrating = false;
    unsigned selected_joint = 0;
};
```

对应改动（3 处，都是机械性的）：

| 文件 | 改动 |
|---|---|
| `inc/mode_control.hpp` | 加 `ArmFeedback` + `ControlOutput::arm_fb` |
| `src/mode_control.cpp:139-149` | 把 `g_calibrating` 传给 `arm_pose_set_jog_mode()`；`output.arm_fb = arm_pose_feedback();` |
| `inc/diagnostics.hpp` / `src/diagnostics.cpp:63-73` | `report_arm` 签名加 `const ArmFeedback &`，在现有 8 个字段**之后**追加打印 |
| `src/main.cpp:49` | 多传一个 `output.arm_fb` |

**打印格式必须保持前缀不变**（标定手法依赖它）：

```
ARM S1=-45 S2=0 S3=0 S4=0 joint=0 cal=0 xy=(145.3,0.8) tgt=(145.3,0.8) th=(-40.9,90.3) ok=1 lim=0 wall=0 cart=1
```

---

## 4. 精度：`Solution` 必须给出 float percent

v0.1 的 `Solution` 只有 `int shoulder_percent / elbow_percent`。在笛卡尔精调下这是**不够的**，实算：

| 摇杆偏移 | 每帧末端步长 | 换算成 percent | 若用 int（取整到 0.25%） |
|---|---|---|---|
| 20% | 0.040 mm | **0.017 %** | 四舍五入到 0 → 目标不更新 |
| 35% | 0.122 mm | 0.052 % | 还是 0 → 不动 |
| 50% | 0.250 mm | 0.106 % | 偶尔跳到 0.25% = **1.18 mm 的突跳** |

现象就是：**轻推摇杆，臂先纹丝不动，攒够几帧后"啪"地跳一下**。这不是算法错，是量化。

修法：`Solution` 增加两个 float 字段，`g_target[]`（本来就是 `float`）直接吃它，只在 `make_command()`（`src/arm_pose.cpp:64-73`）输出给 `ArmCommand.servo[]` 时才取整——那一层原本就有四舍五入。

```cpp
struct Solution {
    bool reachable;
    bool limited;
    float x_mm, y_mm;
    float shoulder_deg, elbow_deg;
    float shoulder_percent_f;   // ★ 精调用这个
    float elbow_percent_f;
    int   shoulder_percent;     // 打印 / 标定核对用（= 四舍五入后的值）
    int   elbow_percent;
};
```

FK 也要有 float 版本（内部闭环用），int 版本留给串口打印：

```cpp
void solve_forward_f(float s1_percent, float s2_percent, float &x_mm, float &y_mm);
void solve_forward(int s1_percent, int s2_percent, float &x_mm, float &y_mm);  // 保留
```

0.01% 的分辨率对应末端 0.024 mm（@r=150），肉眼完全不可见 ✅。

---

## 5. 接口清单

### 5.1 新增（`inc/arm_pose.hpp`）

```cpp
// 每帧给一次速度；松手传 0。只在笛卡尔模式下生效。
void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s);

// 绝对坐标目标（姿态表 / 动作序列用），不可达时钳位并保持标志。
void arm_pose_request_xy(float x_mm, float y_mm);

// 切换精调模式：true = 笛卡尔（正常操作），false = 关节 jog（标定）
void arm_pose_set_jog_mode(bool cartesian);

// 本帧的完整反馈，供 ControlOutput 收集
ArmFeedback arm_pose_feedback();
```

### 5.2 保持不变（一行不改）

`arm_pose_init` / `arm_pose_request(ArmPose)` / `arm_pose_set_jog` / `arm_pose_update` / `arm_pose_hold` / `arm_pose_value` / `arm_pose_at_target` / `arm_pose_gripper_servo` 签名全部不变。

`arm_pose_set_jog()` 继续有效——它在关节 jog 模式下是主通道，在笛卡尔模式下被忽略（不清零 target）。

### 5.3 内部新增状态

```cpp
bool  g_cartesian = true;      // 当前精调模式
float g_cart_vx = 0.0F;        // 本帧速度（一次性，用完清零，与 g_jog 同语义）
float g_cart_vy = 0.0F;
ArmFeedback g_fb;              // 最近一次反馈
```

---

## 6. 每帧时序

```
main loop (HAL_Delay(1))
  ├─ ps2_update()                          20 ms 才刷新，1 ms 循环里多数是空转
  ├─ mode_control_update()
  │    ├─ 按键边沿 / 模式切换
  │    ├─ 机械臂模式：左摇杆 → arm_axis_percent(LX/LY) → 平方律 → arm_pose_set_cart_velocity()
  │    │                                      或标定模式：arm_pose_set_jog()
  │    └─ output.arm_fb = arm_pose_feedback()
  └─ arm_pose_update(now)
       ├─ 笛卡尔：FK(g_current) → 钳矩形 → solve_xy() → g_target[0..1] = float percent
       ├─ 关节 jog：g_target += jog*dt（原逻辑）
       ├─ slew 逼近（原逻辑，300 %/s）
       └─ make_command() 四舍五入成 int
```

性能：`solve_xy` 在目标板上约十几微秒，1 kHz 循环里占 ~1%。若实测有压力，把 IK 降频到 `now - last_ik >= 10 ms` 或直接复用 PS2 的 20 ms 节拍——**但不要低于 20 ms**，否则摇杆动作会明显滞后。

---

## 7. 验证清单

上电后按顺序做，每步不过就别进下一步：

| # | 动作 | 期望 |
|---|---|---|
| 1 | 编译（Debug + Release） | 零警告，`--print-memory-usage` 里 flash 增量 < 5 KB |
| 2 | 上电，串口看 `ARM` 行 | `xy` ≈ 填的末端的 FK；`cart=1`；`ok=1` |
| 3 | 轻推左摇杆 → 侧看末端 | 开始就动（无卡顿跳变）；速度随偏移平滑增大 |
| 4 | 推到底 | 末端约 50 mm/s；推到边界时停住且 `wall=1` |
| 5 | 松手 | **立即停**，不再继续走（验证闭环积分没追尾） |
| 6 | 推过 x 边界 | 停住，`wall=1`；往回推立刻恢复 |
| 7 | 按 START 进标定 | `cart=0`，左摇杆变成单关节微调，↑↓ 选关节照常 |
| 8 | 标定模式推摇杆到碰结构 | 记 `S1..S4`，填进 `arm_config` |
| 9 | 退出标定，再推摇杆 | 无跳变（验证基准刷新） |
| 10 | 按 ○ 切 Ready 姿态 | 走插值到目标，`xy` 收敛到表里的坐标 |

---

## 8. 风险（v0.2 新增）

| 风险 | 后果 | 应对 |
|---|---|---|
| 人眼只能看一个投影 | 目标深度（到车的距离）判断偏差，抓取偏前/偏后 | 无 yaw 的固有代价；操作手需正对侧面、车先摆正。要求更高就走摄像头视觉伺服（不在本届范围） |
| 笛卡尔速度与实物下垂叠加 | 水平位伸出去时末端比目标低 | 已在 v0.1 风险表；v0.2 可在 `solve_xy` 前加 `y -= sag()` 预补偿（v0.3） |
| 闭环依赖 FK | 标定填错时，`xy` 显示的值本身是错的 | FK 回算只证自洽，仍需人工目测一次（v0.1 结论 4） |
| 摇杆双轴同时满偏 | 末端斜向速度 √2×50 = 70 mm/s，可能超 slew | 实算仍远低于 300 %/s；若要严格恒定速率，对 (vx,vy) 做归一化（`if |v|>v_max 则等比缩放`），v0.2 建议加上 |
| 目标被钳位后不更新 target | 边缘小区域手感"粘" | 已设计为只停不弹；若粘滞明显，改成"沿边界切向滑动"（v0.3） |

---

## 9. 改动文件清单

| 文件 | 动作 | 量级 |
|---|---|---|
| `inc/arm_config.hpp` | **新增**：顶层 `constexpr Model kArm` + static_assert | ~60 行 |
| `inc/arm_kinematics.hpp` | 新增：`Solution` 带 float percent、`solve_forward_f` | ~40 行 |
| `src/arm_kinematics.cpp` | 新增：IK/FK/换算（v0.1 已验证的修正版 + float 输出） | ~90 行 |
| `inc/arm_pose.hpp` | 新增 4 个接口 + `ArmFeedback` | +15 行 |
| `src/arm_pose.cpp` | 笛卡尔积分、模式切换、反馈收集、坐标姿态表、派生限位 | 主要工作量 |
| `inc/mode_control.hpp` | 加 `ArmFeedback` + `ControlOutput::arm_fb` | +12 行 |
| `src/mode_control.cpp` | 摇杆分发改成"按 `g_calibrating` 分笛卡尔/关节"；填 `arm_fb` | 局部改 `:80-82`、`:139-149` |
| `src/diagnostics.cpp` / `inc/diagnostics.hpp` | `report_arm` 多收 `ArmFeedback`，追加打印 | 局部 |
| `src/main.cpp` | `report_arm` 多传一个参数 | 1 行 |
| `CMakeLists.txt` | `target_sources` 加 `src/arm_kinematics.cpp` | 1 行 |
| `README.md` | 机械臂操作一节改写（摇杆 = 末端位移） | — |

**仍然零改动**：`src/outputs.cpp`、`src/ps2*.cpp`、`src/drive_control.cpp`、`src/motor_pid.cpp`、`Core/Src/*`、`Drivers/*`。

---

## 10. 我采用的默认值（要改说一声）

| 参数 | 默认 | 理由 |
|---|---|---|
| `jog_speed_mm_per_second` | 50 | 横向 1 米行程 20 s 走完，精调靠平方律 |
| 工作区 | x ∈ [70, 205]，y ∈ [−20, 120] | x_min 对应 θ2≈143°，y_max 避开高力矩区 |
| 标定模式自动切关节 jog | 是 | 复用现有 START 语义，不加按键 |
| `kSlewPercentPerSecond` | 300 不变 | 实测余量 5 倍 |
| 笛卡尔 IK 频率 | 每帧 | 约 1% CPU，比降频简单可靠 |
| 摇杆归一化 | 加 | 双轴满偏时限制合速度不超过 `jog_speed` |

---

## 11. 落地顺序

1. `inc/arm_config.hpp` + `inc/arm_kinematics.hpp` + `src/arm_kinematics.cpp` + CMakeLists 一行 → 编译零警告（没人调用）
2. PC 侧 `test_ik` 跑基准表（float 版本，误差应 < 0.05 mm）
3. `arm_pose.cpp`：坐标姿态表 + `arm_pose_request_xy` + 派生限位（先不接笛卡尔）
4. `mode_control.hpp/cpp` + `diagnostics` + `main.cpp` 接 `ArmFeedback`，烧板子验证姿态切换和串口格式
5. 加笛卡尔积分 + 模式切换 → 按第 7 节 1~10 步验收
6. 标定 TBD-1~4，填 `arm_config`，重算基准表
7. README
