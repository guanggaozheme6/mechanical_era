# arm_pose v0.3 —— 机械臂算法独立成模块 + 笛卡尔精调定稿

> 状态：**设计定稿 + 模块骨架已真实编译并通过 PC 单测**（不是推演）。仓库里只新增本文，不改任何现有 `src/` `inc/` `Core/` `Drivers/` 文件。
> 复现物：`C:\Users\guang\AppData\Local\hermes\cache\scratch\arm_v03\`（4 个头 + 3 个实现 + 3 个测试 + 1 个 bench + `build.sh`）
> 前序文档：`arm_pose_v0.1.md`（几何/2R IK/标定顺序）、`arm_pose_v0.1_integration_report.md`（D1~D6 / R1 / P1~P4 / 实测基准）、`arm_pose_v0.2.md`（笛卡尔精调 / 闭环 FK / 平方律 / 两级钳位 / float percent）
> 按键来源：`ps2_control_v0.2.md`（**以此为准**；本文 §7 与它曾有 2 处符号/语义冲突，已在 §7.5 记录裁决结果）

---

## 0. 结论摘要

| 项 | v0.3 的答案 |
|---|---|
| 算法归属 | 拆成三个纯计算模块 + 一层编排，**全部可脱离 HAL / 手柄 / 时钟编译运行** |
| 单关节手动入口 | **全部删除**（甲方变更）。轴 0/1 只能由笛卡尔解算驱动，`arm_joint_jog` 只对轴 2/3 开放 |
| 结构限位（TBD-4） | 现场**无法用手柄量**。改为 `#if ARM_BRINGUP` 编译开关，绕过解算直写单路百分比，标完关掉重烧 |
| 姿态表 | 4 条百分比 → **8 条坐标表**（`arm_config::kPoses[8]`），索引即按键绑定表的 `pose` 字段 |
| 限位真值 | 只保留角度；百分比限位由 `constexpr` 派生（`arm_config.hpp:58-66`） |
| 复合误差防护 | D1~D6 全部落实，另加「非有限输入」与「非法 dt」两道兜底 |
| 真实编译 | g++ 16.2.0：3 个 TU，`-Wall -Wextra -Werror`，**0 警告**；arm-none-eabi-g++ 15.3.1：**4 个 .o，0 警告** |
| 真实测试 | **486 条断言，0 失败**（test_ik 232 / test_joint 56 / test_pose 198） |
| 体积 | `arm_kinematics.o` text 1996 B、`arm_joint.o` 1072 B、`arm_pose.o` 1836 B；合计 **text 4904 B / data 32 B / bss 96 B** |
| 单次 IK | PC 上 **83.2 ns/call**（目标板**未实测**，估算十微秒量级） |

---

## 1. 甲方变更：删掉单关节 jog，以及它连带推翻的东西

这一节放在最前面，因为**它改变了标定路线**，不是"风险表里加一行"能交代的。

### 1.1 变更内容

| # | 变更 | 直接后果 |
|---|---|---|
| 1 | 取消「单关节 jog 模式」（原：按住 R3 → 左摇杆变 jog、↑↓ 选关节） | 轴 0/1 没有任何手动入口；`arm_pose_set_cartesian()` / `g_calibrating` / `selected_joint` 全部消失 |
| 2 | `↑↓←→` 不再选关节，与 `△○╳□` 一起给 8 条预设 | 关节选择状态机删除，预设变成纯表索引 |
| 3 | `ControlOutput` 删 `calibrating` / `selected_joint` | `report_arm` 签名改 `(const ArmCommand&, const ArmFeedback&)`；`src/main.cpp:46` 的 200 ms 刷新分支删除 |
| 4 | 夹爪(2)/腕(3) 仍要 `joint_jog`，且必须能与笛卡尔速度同帧共存 | 关节层不能"模式互斥"，只能按 axis 白名单放行 |

### 1.2 为什么这条变更危险：TBD-4 失去测量手段

原来的测量闭环是：

```
推摇杆 → 关节百分比自己涨 → 顶到结构 → 抄 S1..S4 → 换算成 range_min_deg/range_max_deg
```

删掉 jog 之后这条链**断在第一步**：轴 0/1 的百分比只能由 `solve_xy()` 写，而 `solve_xy()` 的输入本来就被 `range_min_deg/range_max_deg` 钳着 —— 用限位去测限位，逻辑上是循环的。具体表现：

- 摇杆推到结构附近时，`apply_xy()` 会先判 `solution.limited`，命中就**不下发**（`arm_pose.cpp:99-102`）。所以臂永远停在你填的限位值上，量不到"再往前还有多少余量"。
- 你会看到「推到某个位置就不动了」，但分不清是**结构挡住**、**限位填小了**、还是**够不着**（三个标志 `wall/ok/lim` 能区分后两者，区分不了"结构实际还能再走 3°"）。

**这不是可以靠操作熟练度绕过去的**，是入口被删掉的必然结果。

### 1.3 给出的路径：`ARM_BRINGUP` 编译开关

不接任何手柄键，纯编译期：

```cpp
// src/arm_pose.cpp:18-24（已实现并编过）
#if defined(ARM_BRINGUP)
constexpr bool kBringup = true;
#else
constexpr bool kBringup = false;
#endif
// 标定期初值：改一个数 → 重烧 → 读串口 → 记下来
constexpr int kBringupPercent[kServoCount] = {0, 0, 0, 0};   // src/arm_pose.cpp:37
```

打开后 `arm_pose_init()` 直接把四路设成 `kBringupPercent`，**跳过 IK、工作区、角度限位**（`arm_pose.cpp:139-148`）；`update()` 只按 slew 走到该值，不做任何解算（`arm_pose.cpp:253-257`）；`set_cart_velocity` / `joint_jog` / `request` 全部直接 return（`arm_pose.cpp:164,207,234`）。

| 为什么这么设计 | 不这么做会怎样 |
|---|---|
| 编译期开关，不是运行时命令 | 运行时命令要串口 RX 中断 → 动 `Core/Src/main.c` + `stm32f1xx_it.c`（R1 的结论仍然有效） |
| 不接任何按键 | 甲方明确要求"不接任何手柄键"，避免标定版被误当正常版烧进比赛 |
| 关闭时零成本 | `if (kBringup)` 是 `constexpr false`，`-O2` 直接消除；已实测 bringup 版 `text` 只有 712 B（正常版 1836 B） |
| 仍然走 slew | 一步跳到结构挡块会撞出冲击电流；300 %/s 让接触是"顶上"而不是"撞上" |

**TBD-4 的新操作流程**（每一步约 30 s）：

1. 把 `kBringupPercent[0]` 从 0 每次 +10 往上加，重烧，看大臂走到哪里停。
2. 找到"顶住/发抖"的那个百分比 `p_stop`。回退 3（约 2.7°）。
3. `range_max_deg = zero_deg + (p_stop - 3) * deg_per_percent`（`zero_deg`、`deg_per_percent` 用 §11 的顺序先定）。
4. 负方向同样做一遍得 `range_min_deg`。小臂换 `kBringupPercent[1]`，其余两路保持 0。
5. 关掉 `ARM_BRINGUP` 重烧正常版。

> 为什么标定版要用一套独立的 `kBringupLimits`（`arm_pose.cpp:39-43`，全 ±100）而不是 `kJointLimits`：`arm_joint::configure()` 会把初值钳进 limits（`arm_joint.cpp:22-28`），用 `kJointLimits` 的话「用限位去测限位」的循环还在，只不过程序从 `solve_xy()` 换成了 `bank_configure()` —— 你依然量不出真实边界。代价是标定版**没有任何软件限位**，必须人工盯住，听到堵转立刻拔电。

### 1.4 TBD-2 / TBD-3 的新测量手法（甲方指定：笛卡尔法 + 串口反推）

| TBD | 旧手法（已失效） | 新手法 | 这个方法能证明什么、不能证明什么 |
|---|---|---|---|
| TBD-2 `zero_deg` | jog 到 percent=0，量角器量连杆角 | **两法并用**：① 笛卡尔推到已知坐标 → 读串口 `th1` 与 `S1`，`zero_deg = th1 - S1*deg_per_percent`；② `ARM_BRINGUP` 把该路设 0，量角器量绝对角 | ① 只能证明**模型自洽**（`th` 是算出来的，`S` 是下发的，两边都不含编码器）；它**证明不了"臂实际到了那个角度"**。要绝对角只有 ② 或目测 |
| TBD-3 `deg_per_percent` | jog 到 ±50，量角度差 | 笛卡尔推两个不同高度的已知坐标（例如 (120,60) 与 (120,120)），抄两组 `S1/th1`，`k = Δth1/ΔS1`，再取倒数 | 同上。**必须两点以上**，单点任何 k 都自洽 |
| TBD-1 `upper_mm/fore_mm` | 卡尺 | 不变（卡尺量轴心距） | 这是纯几何量，不受入口影响 |
| TBD-8 预设坐标 | 抄百分比 | 笛卡尔摆到位 → 抄 `xy=` 与 `S1..S4` → 填 `kPoses[]` | 直接可信（`xy` 就是下发目标） |

**必须诚实写在这里**：没有编码器时，笛卡尔法反推出来的 angle 只是"我们命令了多少度"，不是"臂实际到了多少度"。下垂（sag）会系统性污染这组数，而 v0.3 默认 `sag_mm_per_100mm = 0`（见 §15 风险 R-1）。**所以 TBD-2/3 完成后必须再做一次目测：让末端去 (150,120)，实际腕点与目标点偏差 > 5 mm 就说明 deg_per_percent 或下垂有问题。**

---

## 2. 分层：谁负责什么，为什么这样切

### 2.1 四层职责

| 层 | 文件 | 输入 | 输出 | 允许知道 | **禁止**知道 |
|---|---|---|---|---|---|
| 参数 | `inc/arm_config.hpp` (188 行) | — | `constexpr` 全部数值 | 几何类型、关节类型 | 任何逻辑、任何状态 |
| 几何 | `inc/arm_kinematics.hpp` (69) + `src/arm_kinematics.cpp` (133) | `Model const&`, `(x,y)` | `Solution` | `<cmath>` | HAL、tick、`car_config`、`ArmCommand`、全局状态 |
| 行程 | `inc/arm_joint.hpp` (64) + `src/arm_joint.cpp` (116) | `dt_s`, limit, `%/s` | `current` 百分比 | 纯结构体 + 自己 | 角度、坐标、IK、HAL、tick |
| 编排 | `inc/arm_pose.hpp` (72) + `src/arm_pose.cpp` (288) | 速度 / 预设索引 / `now_ms` | `ArmCommand` | 上面三层 + `ArmCommand` | 手柄、SPI、TIM8 |
| 输入 | `src/mode_control.cpp`（改造） | `Ps2Frame` | 速度 / 索引 | 按键语义 | **任何几何、任何积分、任何限位** |

### 2.2 为什么这样切（每条都有反例）

| 切法 | 理由 | 不这么切会怎样 |
|---|---|---|
| `Model` 以 `const&` 传入，不做命名空间全局常量 | 几何模块可以被 PC 工具、上位机、另一个工程原样复用；单测可以拿"假模型"验证 `elbow_sign=-1` 分支（`test_ik.cpp:147-176` 就是这么做的） | 用 `namespace` 级 `constexpr kArm` 会让几何模块和本车标定值焊死，换一台车就得 fork 一份 |
| 关节层吃 `dt_s` 而不是 `HAL_GetTick` | slew 时序、限位、hold 全部能在 PC 上跑时序测试（`test_joint.cpp` 的 56 条断言全靠这个） | 吃 tick 就必须在 PC 上 mock 时钟，测试要么写不出来，要么测的是一层假时钟 |
| 夹爪/腕与两个解算关节同一套 `Joint` | 一处改动四处同时生效；gripper 的 jog/限速/限位不再有第二份实现 | 给夹爪单写一份 jog+限速 = 两份 slew 代码，改 slew 要改两处（v0.2 曾想这么干，v0.3 否决） |
| 角度限位只存一份，百分比 `constexpr` 派生 | 收限位只改 2 个数（`range_min/max_deg`），不会"角度改了、百分比忘了" | 手填 ±100 与角度限位脱钩：小臂收到 `[10°,170°]` 时百分比必须同步成 ±89，漏改就顶结构 |
| 预设姿态 = 数据表 + 索引，不是枚举分支 | 8 条预设只需要 `kPresetBinds[]` 一张表；加第 9 条只加一行 | 8 个 `if (button == X) request(Y)` → `mode_control` 里 8 个分支 + `ArmPose` 枚举 + 姿态表三处同步改 |
| 反馈结构 `ArmFeedback` 放在 `arm_pose.hpp` | 它是 `arm_pose` 的产物，`mode_control` 只是搬运工 | 放 `mode_control.hpp`（v0.2 做法）会让"生产者→消费者"反向依赖，`arm_pose.hpp` 还得 include `mode_control.hpp` |

### 2.3 这次切分**删掉了哪些重复真值**

| 被删的东西 | 原位置 | 之后唯一真值 | 删掉的理由 |
|---|---|---|---|
| `kLimitMin[4] / kLimitMax[4]` 手填 ±100 | `src/arm_pose.cpp:36-37` | `arm_config::kJointLimits[]`（由角度派生，`arm_config.hpp:58-66`） | 两套限位必然漂移（P3） |
| `kGripperServo = 2U` | `src/arm_pose.cpp:40` | `arm_config::kGripperServo`（`arm_config.hpp:45`） | 双真值（D5）；调用点只有 `mode_control.cpp:93` 一处 |
| `kPoseTable[4][4]` 百分比 | `src/arm_pose.cpp:21-27` | `arm_config::kPoses[8]`（坐标 + 夹爪/腕） | 百分比表无法表达"末端在哪"，且只有 4 条 |
| `kSlewPercentPerSecond` | `car_config.hpp:27` | `arm_config::kMotion.slew_percent_per_second` | 机械臂参数不该和轮位/极性混在一个文件 |
| `kJogPercentPerSecond` / `kGripperJogPercentPerSecond` | `car_config.hpp:23,25` | `arm_config::kMotion.jog_percent_per_second[4]` | 同上；且现在是"每路一个"，腕 roll 才有独立速度 |
| `g_target[4] / g_current[4] / g_jog[4]` | `src/arm_pose.cpp:45-47` | `arm_joint::Bank`（`arm_pose.cpp:39`） | 三个平行数组 = 三处越界风险，且夹爪要单独处理时必然分叉 |
| `kMaxElapsedMs` | `src/arm_pose.cpp:43` | `arm_config::kMotion.max_elapsed_ms` | 参数集中 |
| `clamp_percent()` 局部实现 | `src/arm_pose.cpp:50-52` | `arm_kinematics::clamp_percent` | 三份 ±100 钳位（`arm_pose`、`arm_kinematics`、`outputs.cpp:34-35`）→ 现在前两份合成一份 |
| `clamp_joint()` 局部实现 | `src/arm_pose.cpp:54-62` | `arm_joint::set_target` 内部的 `clamp_to_limits` | 限位钳位只在一处执行 |

`outputs.cpp:29-39` 的第三道 ±100 钳位**保留**（硬件层最后一道保险，不要删）。

---

## 3. `inc/arm_config.hpp`：唯一真值

### 3.1 内容清单

| 段落 | 行 | 内容 |
|---|---|---|
| 几何 | 22-28 | `kGeom`：L1/L2 + 两个 `JointCalib` + `elbow_sign` |
| 通道 | 42-50 | `kShoulderServo=0` / `kElbowServo=1` / `kGripperServo=2` / `kWristServo=3` |
| 限位 | 52-76 | `percent_of()` + `kJointLimits[4]`（由角度派生） |
| 工作区 | 78-92 | `kWorkspace{x,y_min,max}` |
| 手感 | 96-124 | `kMotion`：末端速度 / slew / settle / sag / max_elapsed / 每路 jog 速度 |
| 预设 | 127-186 | `Pose` 索引枚举 + `kPoses[8]` |

### 3.2 编译期自检清单（21 条 `static_assert`，全部通过）

| 断言 | 拦住的错误 |
|---|---|
| 连杆长度 > 0；`elbow_sign ∈ {±1}` | 手误填负数 / 写 0 |
| `|L1-L2| > 10 mm` | 内半径太小 → 折叠区数值病态（**注意：原写 20 mm，本车 15 mm 直接编不过，这是阈值填错，不是设计错，见 §9.3**） |
| 通道编号 < 4 | 抄错通道 |
| 大臂/小臂百分比限位在 ±100 内 | `deg_per_percent` 填错（例如 270° 舵机填 0.9） |
| 上下界不颠倒 | `min > max` |
| 工作区 `x_min > 0`、上下界有序、`x_max ≤ L1+L2` | 工作区写进够不着的区域 |
| `cart_speed > 0`、`slew > 0`、`sag ≥ 0`、夹爪 jog > 0 | 速度为 0 → 摇杆没反应，排查会绕远路 |
| slew 余量 ≥ 4 倍（实算 53 %/s 需求 vs 300 供给） | 有人把 slew 调到 50 之后手感变"跟不上"却查不出原因 |
| 8 条预设在工作区内、夹爪/腕 ±100 内 | 上电才炸（现在编不过） |
| `kPoseCount == 8` 且表行数一致 | 表加了一行忘了改枚举 |

### 3.3 位置初始化（D1）

C++14 没有 designated initializers，`kGeom` / `kMotion` / `kPoses` 全部**按声明顺序位置初始化**，每行末尾必须有注释标明字段。规则写死在文件头：

> 字段顺序 = 初始化顺序，以后只准在末尾追加。

不这么做的代价：在结构体中间插一个字段，所有初值静默错位一格（`slew` 吃到 `settle` 的值），编译器**不会报错**。

---

## 4. `arm_kinematics`：纯几何

### 4.1 接口

```cpp
Solution solve_xy(const Model &, float x_mm, float y_mm);          // 逆解 + 两级钳位 + FK 自检
void solve_forward(const Model &, float s1, float s2, float &x, float &y);  // 正解（float 版）
float percent_to_deg(const JointCalib &, float percent);
float deg_to_percent(const JointCalib &, float deg);
float clamp_percent(float);
```

`Solution`（`arm_kinematics.hpp:36-46`）带 **8 个字段**：`reachable / limited / x_mm / y_mm / shoulder_deg / elbow_deg / shoulder_percent / elbow_percent`。角度是 D4 要求的；两个 percent 是 `float`，v0.2 §4 的量化结论继续有效。

### 4.2 五条数值硬防护（对应 D1~D4）

| 处 | 代码位置 | 做法 | 不做会怎样 |
|---|---|---|---|
| D2-a | `arm_kinematics.cpp:78-86` | `r < 1e-3` 时不做 `r_min/r` 除法，直接落到 `(r_min, 0)` | 实测过：`(0,0)` 输出 `th=nan, S1=-2147483648%` → 整数溢出 → TIM8 CCR 乱写 → 舵机抽 |
| D2-b | `arm_kinematics.cpp:26-35` | `deg_to_percent` 里 `!isfinite(deg) \|\| deg_per_percent == 0` → 返回 0 | NaN→int 是 UB，`static_cast<int>(NaN)` 得到 `INT_MIN` |
| D2-c | `arm_kinematics.cpp:53-60` | 非有限 `x/y` 直接当作 `(0,0)` 并置 `reachable=false` | 上游一个 NaN 会污染整条链路（v0.1 只防了 `r==0`，没防 NaN 输入） |
| D3 | `arm_kinematics.cpp:113-116` | 限位比较用 `kLimitEpsDeg = 1e-3` 容差 | 边界点（θ 正好 90°/180°）误报 `limited`，`apply_xy` 会因此拒绝下发 → 臂差一点点到不了 |
| acos 定义域 | `arm_kinematics.cpp:93-97` | `cos_elbow` 先判有限再钳 `[-1,1]` | `acos` 输入 1+1e-7 → NaN |

另外：`r_min` 若 `< 1e-3`（L1≈L2 的退化结构）会被抬到 1 mm，保证函数对**所有** `Model` 都是全定义的 —— 复用给别人时不会踩到。

### 4.3 两级钳位顺序（v0.2 §2.4 的落实位置）

| 级 | 在哪 | 命中标志 | 语义 |
|---|---|---|---|
| 矩形（工作区） | `arm_pose.cpp:67-104` | `at_wall` | 穿肩轴 / 力矩过大 / 折叠死区 |
| 圆环（可达域） | `arm_kinematics.cpp:62-95` | `reachable=false` | 够不着 |
| 关节限位 | `arm_kinematics.cpp:115` | `limited` | 几何可达但转不过去 |

**三个标志必须独立**，实测过反例：`(-80,0)` 在圆环内（`reach=1`）但关节超限（`lim=1`），末端偏 44.87 mm（见 §10 表）。合并成一个 `ok` 就分不清该让操作手"往回拉"还是"改限位"。

---

## 5. `arm_joint`：行程层状态机

### 5.1 状态机

```
        set_jog(rate)              set_target(pct)              update(dt, slew)
   ┌──────────────────┐      ┌────────────────────┐      ┌──────────────────────┐
   │ jog (一次性)      │─────►│ target（限位内）    │─────►│ current 以 slew 逼近   │
   └──────────────────┘      └────────────────────┘      └──────────────────────┘
             ▲                        ▲                              │
             │ hold() 清 jog           │ set_target 再钳位             ▼
             └────────────────────────┴──────────────────────  output_percent()
```

| 语义 | 实现 | 为什么 |
|---|---|---|
| jog 是"本帧速度" | `update` 后强制清零（`arm_joint.cpp:51`） | 松手即停不需要额外的"停止"命令，手柄断线也不会继续走 |
| 限位钳位只在 `set_target` 里 | `clamp_to_limits`（`arm_joint.cpp:8-21`） | 唯一入口，jog 累加也走它，不会绕过 |
| 非法目标回退到**上一个合法 target** | `clamp_to_limits(..., fallback)` | 回到 `current` 会让"目标突然缩回原位"，回到旧 target 才是"忽略这次输入" |
| `dt ≤ 0 / NaN / Inf` 时不动但消费 jog | `arm_joint.cpp:43-46` | 主循环偶发回绕/卡顿时不许出现负步长或 NaN 步长 |
| `slew ≤ 0` 视为冻结 | `arm_joint.cpp:53-56` | 配置写错时"不动"比"一步跳到位"安全 |
| `output_percent` 四舍五入 + 非有限回落 0 | `arm_joint.cpp:73-83` | `-2147483648` 不许出现；`-0.6` 不许截断成 0 |

### 5.2 为什么夹爪/腕不特殊

`Bank.channel[4]` 里四路是同一种类型、同一套函数。夹爪"特殊"的地方只有两处，都在参数里而不是代码里：

1. `kJointLimits[2] = {-100, +100}`（无角度限位）
2. `kMotion.jog_percent_per_second[2] = 200`（比解算关节快）

v0.1 曾把"夹爪 jog"当独立通道（`mode_control.cpp:85-94` 直接调 `arm_pose_set_jog(2, ±200)`），那条路径现在没了 —— 夹爪只是一个 `axis=2` 的 jog 调用。

---

## 6. `arm_pose`：编排层

### 6.1 每帧时序（`arm_pose.cpp:246-264`）

```
mode_control_update()                     ← 只产出"意图"
  ├─ 左摇杆 → 平方律 → arm_pose_set_cart_velocity(vx, vy)   （一次性）
  ├─ L2/R2  → arm_pose_joint_jog(2, ±100)                    （一次性）
  ├─ L1/R1  → arm_pose_joint_jog(3, ±100)                    （一次性）
  └─ 8 键   → arm_pose_request(index)                        （事件）
        │
        ▼
arm_pose_update(now_ms)
  ├─ dt = clamp(now - last, 0 .. 100ms) / 1000
  ├─ 若 vx/vy 非 0：cartesian_step(dt)
  │     FK(current) → (fx,fy) + v*dt → 矩形钳位 → solve_xy → 失败则不改 target
  ├─ 清零 vx/vy
  ├─ bank_update(dt, 300 %/s)      ← 四路统一 slew
  └─ refresh_feedback()            ← FK + 角度，供串口
```

### 6.2 闭环 FK 积分（v0.2 §2.3 的落实）

`cartesian_step()`（`arm_pose.cpp:110-125`）每帧从 `current` 反算末端，再加一个步长。三个后果都实测过（`test_pose.cpp`）：

| 性质 | 实测证据 | 若改成开环 `cart_target += v*dt` |
|---|---|---|
| 松手不追尾 | 推 0.4 s 到位 (130.00, 95.00)，松手再跑 30 帧，位移 **< 0.3 mm** | 目标会跑在实际位置前面几毫米，松手后臂继续走一段 |
| 到边界不蠕动 | 推到 (181.00, 95.00) 后继续推 20 帧，位移 **< 0.2 mm** | 目标继续累加，往回推要先"消化"虚假距离 |
| 扳回立即恢复 | 从边界往回拉 10 帧，退回 > 8 mm 且 `reachable` 恢复 1 | 同上，手感像有回弹 |

### 6.3 平方律 + 双轴归一化（`arm_pose.cpp:214-231`）

`arm_pose_stick_velocity(stick_x, stick_y, vx, vy)`：`v = v_max * f * |f|`，然后若合速度 > `v_max` 则等比缩放。

| 摇杆 | 末端速度 | 20 ms 步长 |
|---|---|---|
| 10% | 0.50 mm/s | 0.010 mm |
| 20% | 2.00 mm/s | 0.040 mm |
| 35% | 6.13 mm/s | 0.123 mm |
| 50% | 12.50 mm/s | 0.250 mm |
| 70% | 24.50 mm/s | 0.490 mm |
| 100% | 50.00 mm/s | 1.000 mm |
| **100% + 100%** | **50.00 mm/s（合速度）**，各 35.36 | 1.000 mm |

最后一行就是 v0.2 遗留的"双轴满偏合速度 70 mm/s"风险项，**已解决**（`test_pose.cpp:146-151` 断言合速度 = 50.000 ± 0.001）。

### 6.4 `arm_pose_request(unsigned)` 必须清速度

`arm_pose.cpp:166-169`：进函数第一件事清零 `g_cart_vx/vy`。不清的话，同一帧里"摇杆速度 + 预设请求"会让臂先沿摇杆方向挪一格再被预设拽回 —— 现象是"按了 ╳ 它先抖一下"。`test_pose.cpp:251-260` 断言同一帧内 `target == 预设坐标`。

### 6.5 上电默认位 = Home 预设

`kPoses[0] = (110, 95)` 就是 `FK(0,0,0,0)` 的结果（`test_ik.cpp:92-97` 断言误差 < 0.05 mm）。所以**上电不需要动**，按 ↑ 也不会跳。这是选 Home 坐标时的硬约束：任何别的值都会在上电第一帧产生一次可见跳动。

---

## 7. 按键定稿映射（与 `ps2_control_v0.1.md` 的一致性）

### 7.1 模式与全局

| 输入 | 行为 | 备注 |
|---|---|---|
| `Start` | 进入模式一（底盘，上电默认） | 直接进入，非 toggle；急停锁定时不响应 |
| `Select` | 进入模式二（机械臂精调） | 同上 |
| `L3`+`R3` | 急停 latch 取反 | 两键同按 ≥200 ms，避免 20 ms 帧对齐抖动 |
| 手柄断线 | `stopped=true`，机械臂 hold | 板载 Key 仍是独立的第二道保险（`src/main.cpp:24-33` 不动） |

### 7.2 模式一（底盘）—— 本文只引用，不定义

左摇杆 X/Y 平移；`L2/R2` 左/右自转；方向键平移备份 80%。**单电机测试、右摇杆通道、`×`/`Select` 按住停车已全部删除**（`ps2_control_v0.2.md` §1）。本文不写模式一的改动清单。

### 7.3 模式二（机械臂精调）—— 本文负责的语义

| 输入 | 行为 | 走哪个接口 | 速度 |
|---|---|---|---|
| 左摇杆 X | 前伸 / 回收（±x） | `arm_pose_stick_velocity()` → `arm_pose_set_cart_velocity()` | 平方律，满杆 50 mm/s |
| 左摇杆 Y | 抬升 / 下压（±y） | 同上 | 同上 |
| `L2` / `R2` | 夹爪 松 / 紧 | `arm_pose_joint_jog(2, ∓100)` | 200 %/s（`kMotion.jog[2]`） |
| `L1` / `R1` | 腕 roll 逆 / 顺 | `arm_pose_joint_jog(3, ∓100)` | 150 %/s（`kMotion.jog[3]`） |
| `↑` `↓` `←` `→` | P1 / P2 / P3 / P4 | `arm_pose_request(i)` | 300 %/s 插值 |
| `△` `○` `╳` `□` | P5 / P6 / P7 / P8 | 同上 | 同上 |
| `R3` | **无功能**（见 §7.5） | — | — |

**扳机是开关量**：`ps2_protocol.cpp` 的解码只取按键位，没有压力字节。所以夹爪/腕的速度是**恒定的**，行程只能靠**按住时长**控制 —— 按住 0.5 s 走 100%，推到底和按一半没有任何区别。文档里不能写成"按压力度控制夹爪"。

### 7.4 预设表（`kPoses[8]`，索引即绑定）

| 槽 | 键 | 名称 | 坐标 (mm) | 夹爪 | 腕 | 实测 θ1 / θ2 | 实测 S1 / S2 |
|---|---|---|---|---|---|---|---|
| P0 | ↑ | Home 收纳 | (110, 95) | 0 | 0 | −0.00° / 90.00° | −0.0 / 0.0 |
| P1 | ↓ | Ready 准备 | (150, 60) | −20 | 0 | −13.03° / 76.23° | −14.5 / −15.3 |
| P2 | ← | Reach 前伸 | (190, 10) | −40 | 0 | −17.22° / 43.84° | −19.1 / −51.3 |
| P3 | → | Retract 回收 | (75, 15) | 0 | 0 | −46.66° / 136.96° | −51.8 / 52.2 |
| P4 | △ | Lift 抬起 | (120, 110) | −40 | 0 | 8.18° / 75.10° | 9.1 / −16.6 |
| P5 | ○ | Carry 搬运 | (110, 90) | −40 | 0 | −2.61° / 92.54° | −2.9 / 2.8 |
| P6 | ╳ | Place 放置 | (160, 40) | −30 | 0 | −19.41° / 73.10° | −21.6 / −18.8 |
| P7 | □ | Drop 释放 | (170, 70) | 20 | 0 | −1.88° / 52.67° | −2.1 / −41.5 |

（θ/S 两列是 `test_ik` 的真实输出，见 §10。坐标全是 TBD-8 占位值，标定后替换；`gripper` 正 = 张开、负 = 夹紧，方向未实测。）

### 7.5 与 `ps2_control` 的 2 处冲突 —— **已裁决，见本节末**

| # | ps2_control 写的 | 甲方最新口径 / 本文 | 后果 |
|---|---|---|---|
| **C-1** | 标定 = **按住 R3**（held，松开即退出），§2.1/§2.3 整段按此设计 | 单关节 jog 已取消，`calibrating`/`selected_joint` 已删 → **R3 没有任何剩余功能** | 两份文档必须有一个人改。本文取"R3 空闲，只作为急停组合的另一半"，并把标定改到编译开关（§1.3） |
| **C-2** | §9 声明 v0.3 必须提供 `arm_set_cart_velocity(vx,vy)` / `arm_joint_jog(axis,%/s)` / `arm_pose_request(unsigned)` / `arm_feedback()`，且 `arm_joint_jog` 单位是 **%/s** | 本文提供的是 `arm_pose_set_cart_velocity` / `arm_pose_joint_jog`（单位 **摇杆百分比 −100..100**）/ `arm_pose_request(unsigned)` / `arm_pose_feedback` | 符号名对不上会让实施者停在编译错误上。见下面的对照表；`mode_control` 侧按对照表写即可，两份文档都不必改 |

**裁决（2026-11-18，以 `ps2_control_v0.2.md` 为准，两份文档现已一致）**

- **C-1 → v0.2 已自行改口，无需再改**：`ps2_control_v0.2.md` §1.7 / D-7 明确「`R3` 单按无功能，标定走 `ARM_BRINGUP` 编译开关」，与本文 §1.3 完全一致。
- **C-2 → 采纳本文符号，反过来改了 `ps2_control`**：`ps2_control_v0.2.md` §9 已改为 `arm_pose_stick_velocity` / `arm_pose_set_cart_velocity` / `arm_pose_joint_jog(axis, stick_percent)` / `arm_pose_request(unsigned)` / `arm_pose_feedback()`，并要求平方律只在 `arm_pose` 里存一份。
- **另有一处 v0.3 需修正**：`ps2_control_v0.2.md` 已删除「`L3`+方向键单电机测试」，本文 §7.2 原来照 `v0.1` 抄了这条，已按 v0.2 改写。

**符号对照表（实施时以本表为准）**

| ps2_control 里的名字 | v0.3 实际符号 | 单位差异 |
|---|---|---|
| `arm_set_cart_velocity(vx, vy)` | `arm_pose_set_cart_velocity(float, float)` | 一致（mm/s） |
| `arm_joint_jog(axis, %/s)` | `arm_pose_joint_jog(unsigned axis, int stick_percent)` | v0.3 收摇杆百分比，乘 `kMotion.jog_percent_per_second[axis]` 得到 %/s。这样 `mode_control` 不需要知道任何速度常数 |
| `arm_pose_request(unsigned)` | 同名同签名 | 一致 |
| `arm_feedback()` | `arm_pose_feedback()` | 一致 |

---

## 8. 串口格式（`report_arm`）

签名从 `(const ArmCommand&, unsigned selected_joint, bool calibrating)` 改为：

```cpp
void report_arm(const ArmCommand &command, const ArmFeedback &feedback);
```

输出（**前 4 个字段的位置一个字节都不动**，标定手法依赖它）：

```
ARM S1=-19 S2=-51 S3=0 S4=0 xy=(190.0,10.0) tgt=(190.0,10.0) th=(-17.2,43.8) ok=1 lim=0 wall=0 poses=0x00
```

| 字段 | 来源 | 用途 |
|---|---|---|
| `S1..S4` | `command.servo[]` | 原有标定手法，不动 |
| `xy=` | `feedback.x_mm/y_mm`（FK 回算） | 末端实际在哪（模型意义下） |
| `tgt=` | `feedback.target_x/y` | 本帧解算用的目标（含 sag 补偿） |
| `th=` | `feedback.shoulder_deg/elbow_deg` | 连杆角，TBD-2/3 反推用 |
| `ok=` | `feedback.reachable` | 0 = 够不着（圆环钳位） |
| `lim=` | `feedback.limited` | 1 = 关节被 `range_*_deg` 裁剪 |
| `wall=` | `feedback.at_wall` | 1 = 工作区矩形钳位 或 预设被拒 |
| `poses=` | `feedback.bad_poses` | 上电自检失败的预设位掩码（0x00 = 八条全好） |

**缓冲要加大**：`src/diagnostics.cpp:64` 现在是 `char message[128]`，新格式约 105 字符，加上 `\r\n` 仍在 128 内但已无余量 —— 改成 `char message[160]`，并把 `snprintf` 返回值检查保留（`diagnostics.cpp:69` 那个模式不要动）。

`src/main.cpp:46` 的 `output.calibrating ? 200U : kReportMs` 改为常量 `kReportMs = 500`（`calibrating` 字段已删）。

---

## 9. 真实编译与测试输出

环境：Windows 11 + git-bash；`g++ (MinGW-W64 ucrt) 16.2.0`；`arm-none-eabi-g++ (Arm GNU Toolchain 15.3.Rel1) 15.3.1`。
复现命令：`bash C:/Users/guang/AppData/Local/hermes/cache/scratch/arm_v03/build.sh`

### 9.1 主机编译（`g++ -std=c++14 -Wall -Wextra -Werror`）

```
=== 1. host compile (g++) ===
$ g++ -std=c++14 -Wall -Wextra -Werror -Iinc -I<repo>/inc -c src/arm_kinematics.cpp
$ g++ -std=c++14 -Wall -Wextra -Werror -Iinc -I<repo>/inc -c src/arm_joint.cpp
$ g++ -std=c++14 -Wall -Wextra -Werror -Iinc -I<repo>/inc -c src/arm_pose.cpp
host: 3 translation units OK, 0 warnings
```

**0 警告 0 错误。** `-Werror` 是开着的，所以"0 警告"等价于"没有未用参数、没有符号/无符号比较、没有窄化"。

### 9.2 测试结果（486 条断言）

```
--- ./out/test_ik ---     TOTAL pass=232 fail=0
--- ./out/test_joint ---  TOTAL pass=56  fail=0
--- ./out/test_pose ---   TOTAL pass=198 fail=0
```

| 套件 | 断言 | 覆盖 |
|---|---|---|
| `test_ik` | 232 | 13 条基准表（θ1/θ2/reach/lim 各一条 + 有限性 + 百分比范围 + FK 误差）、FK 往返、原点/NaN/Inf、半径边界、双分支、角度↔百分比、8 条预设可达性 |
| `test_joint` | 56 | slew 逐帧时序（9/10 帧精确到 1e-3）、限位钳位、NaN 目标、jog 一次性、hold、`dt` 的 0/负/NaN/Inf、`slew` 非法值、四舍五入、bank 四路 |
| `test_pose` | 198 | 上电位、8 条预设收敛、闭环积分不追尾、平方律 8 组、双轴归一化、圆环/矩形钳位、边界不蠕动、jog 白名单、同帧并行、预设清速度、hold 冻结、卡顿 5 s 截断 |

### 9.3 测试期望值修改记录（每次改都写明，不放宽容差凑绿）

| # | 改了什么 | 是"实现错"还是"期望错" | 理由 |
|---|---|---|---|
| 1 | `test_ik` 双分支：给 `Model` 的 `elbow.zero_deg` 从 90 → 0、范围 ±90，**不是**把 ±180 范围拉扯大 | **期望/参数错** | 反支 θ2 = −41°，零点取 90° 时会算成 `(−41−90)/0.9 = −145%`，被行程钳死。零点必须与所选分支匹配 —— 这本身是标定结论，值得写进测试 |
| 2 | `test_pose` 边界断言 `fb.at_wall` → `fb.at_wall \|\| !fb.reachable` | **期望错** | 原断言把"矩形钳位"和"圆环钳位"混成一个标志。实现没有错：`at_wall` 只表示工作区矩形，够不着由 `reachable=0` 表示（v0.1 的结论就是两者必须独立）。容差没动 |
| 3 | `test_pose` jog 用例：改成每帧重给 jog | **测试调用序列错** | jog 语义是"本帧速度"，只给一次就只生效一帧（200 %/s × 20 ms = 4%）。原期望 20% 是把 jog 当成了持续速度。改的是调用方式，期望值 20%/15% 没动 |
| 4 | `test_pose` 并行用例：方向从 +x 改成 +y；夹爪期望 20 → 40 | **测试错 ×2** | ① 期望 20 是同一个"jog 只给一次"的错误，10 帧应为 40%；② +x 走 10 mm 时大臂角只变 0.2°（四舍五入后 `S1=0`），用它断言"大臂非零"不成立，改抬升方向后变 5.8° |
| 5 | `test_pose` hold 用例：改成连续 `hold()` 验证输出冻结 | **期望错** | 现有 `arm_pose_hold()`（`src/arm_pose.cpp:137-140`）的语义是"本帧不推进插值"，不是"永久冻结目标"。解除急停后 `update()` 会继续走向原目标 —— 这是要保留的行为 |
| 6 | `test_ik` `for (float eps : {…})` → 显式数组 | **编译错（测试侧）** | 范围 for 直接吃 braced-init-list 要 `<initializer_list>`，`-Werror` 下编不过 |
| 7 | `arm_config` 内半径阈值 20 mm → 10 mm | **参数错** | 本车 `|110−95| = 15`，20 mm 的阈值让 `kGeom` 直接编不过。15 mm 是真实折叠半径（θ2=180°），10 mm 才是"明显填错"的下界 |

**当前没有任何红着的用例**（`build.sh` 退出码 0）。上面的修改全部是"实现/参数/测试调用序列"层面，没有一条是"把容差从 1 mm 放大到 5 mm 来凑绿"。

### 9.4 交叉编译（`arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O2`）

```
=== 4. cross compile (arm-none-eabi-g++) ===
$ arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O2 -std=c++14 -Wall -Wextra -Werror \
    -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -c src/arm_kinematics.cpp
$ ... src/arm_joint.cpp
$ ... src/arm_pose.cpp
cross: 4 objects OK, 0 warnings
```

**零警告。** 另编了一份 `-DARM_BRINGUP` 的 `arm_pose.cpp`，证明标定版路径也能过 `-Werror`（`#if` 两个分支都编过）。

### 9.5 size

```
=== 5. arm-none-eabi-size ===
   text    data     bss     dec     hex filename
   1072       0       0    1072     430 obj/arm_joint.o
   1996       0       0    1996     7cc obj/arm_kinematics.o
   1836      32      96    1964     7ac obj/arm_pose.o
    712      32      88     832     340 obj/arm_pose_bringup.o
```

| 项 | 数值 | 判断 |
|---|---|---|
| 三模块合计 text | **4904 B** | 现固件 text 29144 B（整合报告 §5），增量 ≈ 17%，总量仍 < 7% of 512 KB |
| data + bss | 32 + 96 = **128 B** | `g_bank`（4×16 B）+ `g_fb`(32 B)；RAM 无压力 |
| 标定版 | 712 B text | 关掉 IK，`-O2` 消除了未被引用的解算路径 |
| 相比 v0.1 的 1364 B | 1996 B（+632） | 多了 float percent、非有限输入兜底、`r_min` 退化保护 |

### 9.6 未解析符号

```
U __aeabi_f2iz   U __aeabi_fadd   U __aeabi_fcmpeq  U __aeabi_fcmpge
U __aeabi_fcmpgt U __aeabi_fcmple U __aeabi_fcmplt  U __aeabi_fcmpun
U __aeabi_fdiv   U __aeabi_fmul   U __aeabi_fsub    U __aeabi_i2f
U __aeabi_ui2f
U acosf  U atan2f  U cosf  U sinf  U sqrtf
U memset
U _ZN14arm_kinematics8solve_xy...   （跨 TU，由本模块自己提供）
U _ZN9arm_joint...                   （同上）
```

| 类别 | 符号 | 提供方 |
|---|---|---|
| libm | `acosf atan2f cosf sinf sqrtf` | `libm.a`；链接行已有 `-lm`（`cmake/gcc-arm-none-eabi.cmake:47`），不用改 |
| libgcc 软浮点 | `__aeabi_f*` `__aeabi_i2f` `__aeabi_ui2f` | 内置。M3 无 FPU 的正常结果，不是缺库 |
| libc | `memset` | `Solution` 零初始化被编译器合成成 `memset` 调用（`-O2` 下常见），`newlib` 提供 |

**没有 HAL 符号、没有 `HAL_GetTick`、没有 `printf` 家族。** 这就是"纯计算模块"的机器可验证证据。

### 9.7 IK 单次开销（PC 实测，目标板未实测）

```
solve_xy:      83.2 ns/call
solve_forward: 24.3 ns/call
```

（`g++ -O2`，200 万次循环取平均；`bench_ik.cpp`）

**明确标注**：这是 x86-64 的数字，**不能外推到 Cortex-M3**。M3 上要含 `acosf/atan2f/sinf/cosf` 的软件实现 + `__aeabi_fdiv`，按整合报告 §5 的估算量级是**十微秒**。`solve_xy` 只在"摇杆有输入"或"按预设"时才调用（1 kHz 循环里多数帧不调），`solve_forward` 每帧调一次（24 ns → 估算 1~2 µs）。若要确证，用 DWT 计数器包一层实测，这一项**列为未实测**。

---

## 10. 真实基准数值表（`test_ik` 原样输出，L1=110 / L2=95）

```
  request              th1       th2       S1       S2             FK     err reach/lim
  ( 205.0,  0.0)     0.00°     0.00°     0.0%  -100.0% ( 205.00,   0.00)   0.00  1/0
  ( 180.0,  0.0)   -26.38°    57.35°   -29.3%   -36.3% ( 180.00,   0.00)   0.00  1/0
  ( 145.0,  0.0)   -40.93°    90.27°   -45.5%     0.3% ( 145.00,  -0.00)   0.00  1/0
  ( 100.0,  0.0)   -53.54°   122.16°   -59.5%    35.7% ( 100.00,  -0.00)   0.00  1/0
  (  60.0,  0.0)   -59.62°   146.98°   -66.2%    63.3% (  60.00,   0.00)   0.00  1/0
  (  15.0,  0.0)     0.00°   180.00°     0.0%   100.0% (  15.00,   0.00)   0.00  1/0
  ( 150.0,120.0)    19.73°    40.99°    21.9%   -54.5% ( 150.00, 120.00)   0.00  1/0
  ( 100.0,150.0)    30.07°    57.03°    33.4%   -36.6% ( 100.00, 150.00)   0.00  1/0
  (   0.0,205.0)    90.00°     0.00°   100.0%  -100.0% (  -0.00, 205.00)   0.00  1/0
  ( 150.0,140.0)    43.03°     0.00°    47.8%  -100.0% ( 149.87, 139.88)   0.18  0/0
  ( -80.0,  0.0)    90.00°   134.79°   100.0%    49.8% ( -67.42,  43.07)  44.87  1/1
  ( 300.0,  0.0)     0.00°     0.00°     0.0%  -100.0% ( 205.00,   0.00)  95.00  0/0
  (   0.0,  0.0)     0.00°   180.00°     0.0%   100.0% (  15.00,   0.00)  15.00  0/0
```

四条读法（比数字重要）：

1. **FK 误差 0.00 mm** —— 不再是 v0.1 的 0.5~1.7 mm。原因：`Solution` 用 float percent 回算，整数量化误差被挪到下游 `make_command()`（那里本来就要取整）。这意味着**串口 `xy=` 不再包含取整误差**，5 mm 的验收阈值现在只留给实物误差（下垂、齿隙）。
2. `(150,140)` 差 0.18 mm 够不着 → `reach=0`，但 FK 只偏 0.18 mm。**绝不能靠"回算误差小"判可达**，必须看 `reachable`。
3. `(-80,0)` 在圆环内但关节超限 → `1/1`，末端偏 44.87 mm。两个标志必须独立。
4. `(0,0)` 与 `(300,0)` 都**没有 NaN、没有 INT_MIN**，输出是"钳到边界后的有限值"（D2 全部落实）。

---

## 11. 标定路径（顺序不能乱）

| 步 | 做什么 | 用什么手段 | 产出 |
|---|---|---|---|
| 1 | 量连杆 | 卡尺量轴心距 | `kGeom.upper_mm` / `fore_mm` |
| 2 | 定零点 | `ARM_BRINGUP` 把该路设 0，量角器量绝对角 | `kGeom.shoulder.zero_deg` / `elbow.zero_deg` |
| 3 | 定比例 | `ARM_BRINGUP` 推到 +30 / +60，两点量角度差 | `deg_per_percent`（含符号） |
| 4 | 定限位 | `ARM_BRINGUP` 逐 10% 加，找"顶住"点，回退 3 | `range_min_deg` / `range_max_deg` |
| 5 | 校验模型 | 烧正常版，笛卡尔推到 (150,120)，目测腕点与目标重合 < 5 mm | 通过/重来 |
| 6 | 填预设 | 笛卡尔摆到 8 个工位，抄 `xy=` 与 `S1..S4` | `kPoses[8]` |
| 7 | 上电全表校验 | 看 `poses=0x00` | 八条全可达 |

**第 2~4 步必须用 `ARM_BRINGUP` 版**（原因见 §1.2）。第 5 步回到正常版，此时若目测偏差 > 5 mm，顺序是"先查 `deg_per_percent`，再查下垂"，不要一上来改 L1/L2（卡尺量的长度最可信）。

---

## 12. 逐文件改动清单

### 12.1 新增（5 文件，可直接从 scratch 拷进仓库）

| 文件 | 行数 | 内容 |
|---|---|---|
| `inc/arm_config.hpp` | 188 | 唯一真值 + 21 条 `static_assert` |
| `inc/arm_kinematics.hpp` | 69 | `JointCalib` / `Model` / `Solution` + 5 个接口 |
| `src/arm_kinematics.cpp` | 133 | IK / FK / 换算 + 5 道数值防护 |
| `inc/arm_joint.hpp` | 64 | `Limits` / `Joint` / `Bank` + 9 个接口 |
| `src/arm_joint.cpp` | 116 | 状态机 + 四路 bank |


### 12.2 改动

| 文件 | 动作 | 量级 |
|---|---|---|
| `src/arm_pose.cpp` | **主体重写**：删 `kPoseTable[4][4]`(`:21-27`)、`kLimitMin/Max`(`:36-37`)、`kGripperServo`(`:40`)、`g_target/g_current/g_jog`(`:45-47`)、`clamp_percent`(`:50-52`)、`clamp_joint`(`:54-62`)；改为 `arm_joint::Bank` + `arm_kinematics` + 闭环积分 + 8 条预设 + `ARM_BRINGUP` | 160 → 288 行 |
| `inc/arm_pose.hpp` | `ArmPose` 枚举 → 删除（改用 `arm_config::Pose` 索引）；`arm_pose_request(unsigned)`；新增 `arm_pose_set_cart_velocity` / `stick_velocity` / `joint_jog` / `request_xy` / `feedback` / `bad_pose_mask`；`ArmFeedback` 移到这里 | 35 → 72 行 |
| `inc/mode_control.hpp` | `ControlOutput` 删 `calibrating`(`:16`) / `selected_joint`(`:17`)；加 `ArmFeedback arm_fb`；`#include "arm_pose.hpp"` | +6 / −2 |
| `src/mode_control.cpp` | 删 `g_calibrating`(`:9`)、`g_joint`(`:10`)、`update_arm_mode` 的 8 个分支(`:44-95`)；改为预设绑定表 + `arm_fb` 填装 | 150 → ~150 行（同量级，内容全换） |
| `src/diagnostics.cpp` | `report_arm` 签名改 `(const ArmCommand&, const ArmFeedback&)`；`message[128]` → `[160]`；追加 7 个字段 | `:63-73` 局部 |
| `inc/diagnostics.hpp` | `report_arm` 声明同步 | 1 行 |
| `src/main.cpp` | `:46` 删 `calibrating ? 200 : 500` 分支；`:49` 传 `output.arm_fb` | 2 行 |
| `inc/car_config.hpp` | 删 `kJogPercentPerSecond`(`:23`)、`kGripperJogPercentPerSecond`(`:25`)、`kSlewPercentPerSecond`(`:27`) 及其 `static_assert`(`:36-38`)；`kArmDeadzone`(`:21`) **保留**（它属于摇杆→百分比映射，不属于机械臂算法） | −8 行 |
| `CMakeLists.txt` | `target_sources` 加 `src/arm_kinematics.cpp`、`src/arm_joint.cpp`（`:23` 之后） | 2 行 |
| `README.md` | 「机械臂标定」一节整段重写（现 `:78-87`） | 整节 |

### 12.3 零改动

`src/outputs.cpp`（`apply_servos` 的 ±100 钳位是第三道保险，保留）、`src/ps2.cpp`、`src/ps2_protocol.cpp`、`src/arm_control.cpp`、`src/drive_control.cpp`、`src/motor_pid.cpp`、`Core/Src/*`、`Drivers/*`、`startup_stm32f103xe.s`、`STM32F103xx_FLASH.ld`。

---

## 13. 落地顺序（每步可独立编译验证）

| 步 | 动作 | 验证方式 | 可回滚 |
|---|---|---|---|
| 1 | 拷 5 个新文件 + `CMakeLists.txt` 加 2 行 | `cmake --build`，零警告（此时没人调用） | 是（纯新增） |
| 2 | PC 跑 `build.sh` 的三个测试 | 486 条全绿 | — |
| 3 | 改 `inc/arm_pose.hpp` + `src/arm_pose.cpp` | 编译过；此时 `mode_control` 还调着旧 API → **必然编不过**，所以 3、4 必须同一步做 | — |
| 4 | 改 `mode_control.hpp/.cpp` + `diagnostics` + `main.cpp` | 编译过；烧板看串口格式没破（`S1..S4` 位置不变） | — |
| 5 | 上电空转，按 §14 的 1~6 步 | 姿态切换正确、`xy=` 收敛 | 是 |
| 6 | 笛卡尔摇杆 1~10 步 | 不追尾、不蠕动、扳机并行 | 是 |
| 7 | `ARM_BRINGUP` 标定 TBD-1~4 | 每改一个数重烧一次 | 是（开关关掉即恢复） |
| 8 | 填 `kPoses[8]`，看 `poses=0x00` | 八条全可达 | 是 |
| 9 | 改 `README.md` | — | — |

第 1~2 步完全不影响现有固件行为，可以放心先合。

---

## 14. 上电验证清单

| # | 动作 | 期望 | 不过说明什么 |
|---|---|---|---|
| 1 | 上电，不碰手柄 | 串口 `poses=0x00`；四路 `S=0`；`op=DRV` | 预设表有不可达点 / 上电中位不是 Home |
| 2 | 按 `Select` 进模式二 | `op=ARM`；`S1..S4` 不动；`xy=(110.0,95.0)` | FK 与 `kPoses[0]` 不一致 |
| 3 | 按 8 个预设键逐个 | 每个键对应一条姿态，`xy=` 收敛到表里坐标 ±2 mm | 绑定表索引错 / 该条 `bad_poses` 置位 |
| 4 | 轻推左摇杆 +x | **一开始就动**（不卡顿）；速度随偏移平滑增大 | 平方律或 float percent 退化 |
| 5 | 推到底 | 约 50 mm/s；到边界停住且 `ok=0` 或 `wall=1` | 没钳位（危险，立刻断电） |
| 6 | 松手 | **立即停**，不再继续走 | 闭环积分写成了开环 |
| 7 | 按住 `L2`/`R2` 各 1 s | 夹爪单向往一个方向走，`S3` 变化约 ±200 | jog 白名单/速度常数错 |
| 8 | 按住 `L1`/`R1` 各 1 s | 只有 `S4` 变，`S1/S2` 纹丝不动 | 腕 roll 误接进解算 |
| 9 | 推摇杆同时按 `L2` | 末端和夹爪**同时**动 | 同帧并行被模式互斥挡住 |
| 10 | 推摇杆到边界再往回拉 | 立即反向，无"回弹/粘滞" | 目标被钳位后又继续累加 |
| 11 | `L3`+`R3` 同按 200 ms | `STOP=1`；再按一次解除；锁定期间 `Start/Select` 无效 | 急停组合时序 |
| 12 | 快速连按预设 A→B→A | 每次都不跳变，`tgt=` 立刻切到新预设 | `arm_pose_request` 没清速度 |

---

## 15. 风险表

| # | 风险 | 后果 | v0.3 应对 | 状态 |
|---|---|---|---|---|
| R-1 | **水平位下垂**（v0.2 遗留） | 开环舵机在最大力矩工况下垂，IK 算对也到不了 | 接口做了（`kMotion.sag_mm_per_100mm` + `sag_compensation()`，`arm_pose.cpp:53`），**默认 0 = 关闭**。标定时实测下垂 > 10 mm 就填一个非零系数（线性按前伸距离），不需要改代码 | **接口做、标定未做** |
| R-2 | **边界粘滑**（v0.2 遗留） | 顶到边界时"推不动、一推就弹" | **改为两段式贴边滑**：位置法（沿摇杆方向、解不出对半缩步、只收精确命中）+ 关节域兜底（雅可比 → DLS → 饱和重解 → 法向剔除）。DLS 阻尼 λ=0.2 mm/% 专门把奇异区变软（抖动 0.061 vs 旧末端域 0.620 mm/帧²）。正对墙推时投影为 0 → 判卡死（整帧不下发），这是设计内行为 | **已实现**：`Algorithm/src/arm_limit.cpp`；验收 `node sim/verify/stick_port_test.js` 6/6（debugv0.2 D-27） |
| R-3 | **双轴满偏合速度**（v0.2 遗留） | 斜向 70 mm/s，超设计速度 | `arm_pose_stick_velocity` 末尾等比归一化到 50 mm/s（`arm_pose.cpp:226-230`），已断言合速度 = 50.000 | **已解决** |
| R-4 | **IK 每帧开销**（v0.2 遗留） | 1 kHz 循环里占 CPU | `solve_xy` 只在"有摇杆输入"或"按预设"时调（不是每帧）；`solve_forward` 每帧 1 次。PC 83.2 ns / 24.3 ns，M3 **未实测**（估算 10 µs / 1~2 µs） | **未实测（目标板）** |
| R-5 | **删 jog 后 TBD-4 无手柄入口** | 只能靠编译开关标定，迭代慢（30 s/次） | `ARM_BRINGUP` + 逐 10% 扫（§1.3）；标定期必须人工盯堵转 | **接受** |
| R-6 | 无编码器，FK 只证自洽 | 掉步/堵转/齿隙，`xy=` 显示的是"命令值" | 串口 `xy=` 与目测必须交叉验证一次；不引入编码器就无解 | 固有 |
| R-7 | `deg_per_percent` 假设线性 | 大角度处系统性偏差 | 两点测量校验（TBD-3），偏差 > 3° 改查表 | 未标定 |
| R-8 | 扳机是开关量 | 夹爪/腕只能靠按时长控行程 | 已写入 §7.3；要模拟量需扩 `ps2_protocol` 解析 0x79 压力字节 | 固有 |
| R-9 | `╳` 不再是急停 | 肌肉记忆误按 | 板载 Key 兜底（`src/main.cpp:24-33`）；README 第一行必须写 | 接受 |
| R-10 | `ARM_BRINGUP` 被误烧进比赛版 | 绕过全部限位 | 宏默认不存在；`ArmFeedback.bringup` 会在串口可见（正常版恒 0）；上电自检可以打一行 `BU=1` 警示 | 需在验收清单加一条 |
| R-11 | 车体倾斜 / 悬挂压缩 | `+x` 不再水平，整张表偏移 | 不补偿，操作手保证车体水平 | 固有 |

---

## 16. TBD 清单

| 编号 | 内容 | 怎么定 | 阻塞什么 |
|---|---|---|---|
| TBD-1 | `upper_mm` / `fore_mm` | 卡尺量轴心距 | 全部 IK 精度 |
| TBD-2 | 两个 `zero_deg` | `ARM_BRINGUP` percent=0 + 量角器（§1.4） | 角度绝对正确性 |
| TBD-3 | 两个 `deg_per_percent`（含符号） | `ARM_BRINGUP` 两点法 | 行程百分比↔角度 |
| TBD-4 | 两个 `range_min/max_deg` | `ARM_BRINGUP` 逐 10% 顶到结构，回退 3° | 会不会撞结构 |
| TBD-5 | ~~`elbow_sign`~~ | ✅ 已定 `+1` | — |
| TBD-6 | ~~`servo[3]` 是 roll~~ | ✅ 已定 roll | — |
| TBD-7 | ~~`servo[0]/[1]` 角色~~ | ✅ 0=大臂、1=小臂 | — |
| TBD-8 | 8 条预设坐标 + 夹爪值 | 笛卡尔摆位后抄 `xy=` | 预设能不能用 |
| TBD-9 | `sag_mm_per_100mm` | 水平位实测下垂量，`sag = 下垂mm / (x/100)` | 精调高度准不准 |
| TBD-10 | 夹爪正负方向 | 现场试：`gripper > 0` 是张开还是夹紧 | 预设表符号 |
| TBD-11 | `kWorkspace` 四边界 | 按 TBD-4 结果重算 `x_min`（v0.1 §12.2 表） | 近处够不够得着 |

---

## 17. 决策记录（Q-1 / Q-2 已由甲方定案，Q-3~Q-6 待定）

| # | 问题 | 我的默认 | 不定的后果 |
|---|---|---|---|
| **Q-1 ✅** | **`R3` 现在干什么？**（`ps2_control_v0.1.md` 写"按住 R3 标定"，但标定模式已被删除） | 无功能，只作急停组合的另一半（`L3`+`R3`） | 两份文档冲突（C-1），实施者会写出一个没人用的分支 |
| **Q-2 ✅** | 标定只能靠 `ARM_BRINGUP` 重烧，接受 30 s/次的迭代速度吗？ | 接受 | 若要免重烧，就必须做串口 RX（动 `Core/Src/main.c`，R1 的结论被推翻） |
| Q-3 | 8 条预设的语义与坐标（§7.4 是占位值） | 先按表实现，坐标 TBD-8 | 按下预设走错位置 |
| Q-4 | 夹爪"正 = 张开"这个约定对吗？（TBD-10） | 按正=张开实现 | 8 条预设的夹爪值全部反号 |
| Q-5 | `L1/R1` 腕 roll 的 150 %/s 合适吗？（`ps2_control` 也是 150，但没人实测过手感） | 150 %/s，满行程 1.3 s | 太快会甩飞夹持物 |

---

### 17.1 已定案（甲方 2026-11-18，实施时按这条，不要再讨论）

**Q-1：`R3` 无单独功能。**

- `R3` 唯一的用途是作 `L3+R3` 急停组合的另一半，**不得有任何独立分支**（连 `if (pressed(R3)) {}` 的空壳都不留 —— 空壳会让人以为这里该有功能）。
- 标定不再有手柄入口（见 Q-2），`ps2_control_v0.2.md` §1.7 / D-7 已是这个口径，两份文档一致。
- 以后要给 `R3` 加功能，**先改本节**再动代码。

**Q-2：限位标定就用「改参数 → 重烧」，不做串口 RX。**

- `ARM_BRINGUP` 编译开关 = TBD-2/3/4 的唯一通道：改 `kBringupPercent[4]`（`src/arm_pose.cpp:37`）一个数 → 编译 → 烧录 → 上电看臂停在哪 → 记下来。一条完整 TBD-4 标定按 §1.3 的流程走 20~40 次重烧，**接受**。
- 明确否决，且以后不要再提议：串口 `P x y` / `F` 命令、`HAL_UART_Receive_IT`、`USART3_IRQHandler`、收包环形缓冲。**`Core/Src/main.c` 与 `Core/Src/stm32f1xx_it.c` 全程零改动**，整合报告的 R1 结论继续有效；`arm_pose_v0.1.md` §10 那一整节（串口命令）正式作废。
- 串口的角色只剩**单向输出**：`ARM S1..S4 xy= tgt= th= ok= lim= wall= poses=` 与 `POSES` 自检行。

### 17.2 仍然待定（Q-3~Q-6）

| # | 问题 | 我的建议 | 卡住什么 |
|---|---|---|---|
| Q-3 | 8 条预设的语义与坐标（§7.4 全是占位值） | 先按表实现，坐标 TBD-8 标定后替换 | 按下的姿态对不对 |
| Q-4 | `gripper > 0` = 张开？（TBD-10 未实测） | 标定第一件事就是点 `L2`/`R2` 看方向；反了则 8 条表全反号 | 夹爪动作方向 |
| Q-5 | 腕 roll 150 %/s（满行程 1.3 s） | 先给 150，甩了就降到 100 | 夹持物会不会被甩飞 |
| **Q-6（新增，非 v0.3 原文）** | **预设会把腕 roll 拽回 0**：`arm_pose.cpp:197` 无条件写 `set_target(wrist, pose.wrist_percent)`，而 §7.4 表里 `wrist` 全为 0 → 用 `L1/R1` 转好角度后按任意预设，腕会以 300 %/s 自己转回 0 | **改用哨兵值 `wrist_percent = -101` = 保持不动、不写该轴**（约 20 行改动 + 一条测试），夹爪维持预设写入不变 | 有没有「先转腕再摆位」这个用法 |

---

## 18. 附：本文所有数值的来源标注

| 数值 | 来源 | 是否实测 |
|---|---|---|
| 13 条 IK 基准（θ1/θ2/S/reach/lim） | `test_ik` 真实输出（§10） | ✅ PC 实测 |
| 8 条预设的 θ/S | 同上 | ✅ PC 实测 |
| FK 误差 0.00 mm | 同上 | ✅ PC 实测 |
| slew 逐帧时序（6 %/帧） | `test_joint` 真实输出 | ✅ PC 实测 |
| 闭环积分位移 20 mm / 松手 0.3 mm / 边界 0.2 mm | `test_pose` 真实输出 | ✅ PC 实测 |
| 平方律 8 组 + 归一化 50.000 | 同上 | ✅ PC 实测 |
| 三个 `.o` 的 text/data/bss | `arm-none-eabi-size`（§9.5） | ✅ 交叉编译实测 |
| 未解析符号清单 | `arm-none-eabi-nm -u`（§9.6） | ✅ 实测 |
| 83.2 ns / 24.3 ns | `bench_ik`（x86-64） | ✅ PC 实测，**不可外推** |
| M3 上 IK 耗时（十微秒量级） | 整合报告 §5 的估算 | ❌ **未实测** |
| libm 引入体积（2~3 KB） | 整合报告 §5 | ❌ **未做完整链接实测** |
| 下垂量、结构限位、零点、度每百分比 | TBD-1~4 | ❌ **未标定** |
