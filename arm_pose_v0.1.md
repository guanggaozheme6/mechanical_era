# arm_pose v0.1 —— 二维逆解（IK）设计

> 状态：设计稿，未落地代码。所有标 `TBD` 的数值必须在实车上量出来才能填。
> 前置事实：本机械臂**没有 yaw 轴**，两连杆（大臂 / 小臂）在车体竖直纵剖面内运动，其余两路舵机只做夹爪旋转与松紧。因此解算退化成**平面 2R 逆运动学**，不需要三维坐标、不需要底座回转。

---

## 0. 结论摘要

| 项 | v0.1 的答案 |
|---|---|
| 输入 | 腕点平面坐标 `(x_mm, y_mm)`，原点在肩关节转轴，`x` 指向前方，`y` 指向上方 |
| 输出 | 两路关节角 `(θ1, θ2)`，再仿射映射成 `servo[0] / servo[1]` 的行程百分比 |
| 几何参数 | 一个 `ArmModel` 结构体：`upper_mm`、`fore_mm` 加两个关节的标定项 |
| 不可达 | 沿目标方位角钳到工作空间边界，置 `reachable=false` 供串口报警，仍然下发（不会抖到 NaN） |
| 分支 | 肘部朝向由 `elbow_sign` 固定；实车通常只有一支不撞结构 |
| 夹爪 | `servo[2]` 松紧、`servo[3]` 旋转，**完全不参与 IK**，各走各的通道 |
| 插值 | 仍在关节空间做现有 `kSlewPercentPerSecond` 限速，不改 |
| 不做（留给 v0.2） | 笛卡尔直线插补、夹爪长度补偿、三维/底座回转、力矩/电流保护 |

---

## 1. 现状：现在的 `arm_pose` 是什么

现在的 `arm_pose` 是**纯角度层**——它只知道"某一路舵机应该走到百分之多少"，完全不知道连杆有多长、末端在哪。

调用链（每轮主循环）：

```
ps2_update()            src/ps2.cpp:87
  → mode_control_update()   src/mode_control.cpp:116
      → update_arm_mode()   src/mode_control.cpp:44   ← 按键/摇杆 → 关节百分比增量
      → arm_pose_update()   src/arm_pose.cpp:103      ← 积分、限速、输出
  → apply_servos()          src/outputs.cpp:29        ← 百分比 → 500..2500 µs 脉宽
```

三层各自职责：

| 层 | 文件 | 状态量 | 单位 |
|---|---|---|---|
| 输入 / 手动微调 | `src/mode_control.cpp:81-94` | `g_joint` 选中关节 + 摇杆速度 | %/s |
| 姿态与插值 | `src/arm_pose.cpp:45-48` | `g_target[4]` / `g_current[4]` / `g_jog[4]` | 行程 %，`-100..100` |
| 硬件 | `src/outputs.cpp:29-38` | TIM8 `CCR` | µs（`1500 + 1000*percent/100`） |

关键事实（写文档时逐条核对过）：

1. **四路关节在代码里没有角色定义**（只有约定）。`ArmCommand`（`inc/arm_control.hpp:6-8`）就是个 `int servo[4]`，全工程唯一有物理语义的只有夹爪：`kGripperServo = 2`（`src/arm_pose.cpp:40`）。本设计采用并已确认的约定：**`servo[0]` 大臂、`servo[1]` 小臂、`servo[2]` 夹爪松紧、`servo[3]` 腕部 roll**。
2. **姿态表是四个百分比**，不是坐标：`kPoseTable[4][4]`，`src/arm_pose.cpp:21-27`，注释自己承认"只是能看出动作的占位值"（`src/arm_pose.cpp:17`）。
3. **限位是全行程**，等于没保护：`kLimitMin/Max = ±100`，`src/arm_pose.cpp:36-37`。
4. **1% ≈ 0.9°** 是注释里的经验值（`src/arm_pose.cpp:15`），来源是 `100% → 1000 µs`，`1000 µs → 90°`（0.5~2.5 ms 对 180° 的常规舵机）。这个比例必须实测确认，不同舵机 180°/270° 混用会直接解错。
5. 插值已经是"当前值按 `kSlewPercentPerSecond` 向目标匀速逼近"（`src/arm_pose.cpp:120-132`），**这段逻辑在 v0.1 里原样保留**——IK 只负责把目标从坐标换成百分比，不碰限速。
6. 急停/断线时 `arm_pose_hold()` 冻结输出（`src/arm_pose.cpp:137-140`，`src/mode_control.cpp:143`），IK 引入后这条不变。

**结论：v0.1 不需要推翻任何现有结构，只需要在 `arm_pose` 的目标入口前插一层几何换算。**

---

## 2. 目标与非目标

### 目标

- 给定 `(x, y)` 能算出两路关节百分比，且正解回算误差 < 5 mm。
- 连杆长度、关节零点、方向、安全角度限位全部集中在**一个结构体**里，改数值不用碰算法。
- 新增接口能直接替换 `kPoseTable` 的四列百分比（姿态表改成"坐标 + 夹爪"）。
- 保留现有手动微调（摇杆 jog 百分比）、限速插值、急停保持，行为不回归。
- 不可达 / 超限时**有确定行为**，不产生 NaN、不抖动、不静默出界。

### 非目标（v0.1 明确不做）

- 笛卡尔直线插补（末端走直线）——v0.1 关节空间插值会让末端走弧线。
- 夹爪指尖补偿（IK 解到腕点，不减去夹爪长度）。
- 任意 yaw / 底座回转，三维 IK。
- 舵机位置反馈、力矩限制（当前是开环，没有编码器）。

---

## 3. 坐标与单位约定

```
            y ↑ 上
              |
              |        ● P(x, y)   腕点（小臂末端 / 夹爪安装座交点）
              |      /
              |    /  L2
              |  /
              |/  θ2 (相对大臂延长线，逆时针为正)
              ● 腕关节
             /
          /  L1
        /
      ●───────→ x 前
      肩关节 O(0,0)        θ1 从 +x 轴量起，逆时针为正
```

| 量 | 约定 | 说明 |
|---|---|---|
| 原点 | 肩关节转轴轴心 | 不是车体中心，也不是底板 |
| `+x` | 车体正前方，水平 | 无 yaw，所以这个平面**永远**是车体纵剖面 |
| `+y` | 竖直向上 | 车体倾斜时该平面跟着倾斜，v0.1 不补偿 |
| `θ1` | 大臂相对 `+x` 的角度，逆时针正 | 大臂水平向前 = 0°，竖直向上 = +90° |
| `θ2` | 小臂相对**大臂延长线**的偏角，逆时针正 | 伸直 = 0°；`θ2 > 0` 肘部朝"上/后"折叠 |
| 长度 | mm，浮点 | 建议用卡尺量转轴中心到转轴中心 |
| 末端点 | **腕点**，不含夹爪 | 夹爪长度约 30~60 mm 属于 v0.2 的 `tool_mm` |

> 没有 yaw 的代价要写在操作手册里：**能不能抓到，取决于车先摆正、对准目标**。解算只保证"给定平面内能不能到"，不保证"目标在不在这个平面里"。

---

## 4. 几何模型：一个结构体搞定

```cpp
// inc/arm_kinematics.hpp（草案）

namespace arm_geometry {

// 单路关节的机械标定：把"连杆几何角"和"舵机行程百分比"线性绑起来。
struct JointCalib {
    float zero_deg;         // percent == 0 时，连杆的实际角度（相对 +x，向上为正）
    float deg_per_percent;  // 1% 行程对应的角度；符号即安装方向（+0.9 / -0.9）
    float min_deg;          // 机械安全下限（由限位百分比换算，或直接量）
    float max_deg;          // 机械安全上限
};

struct ArmModel {
    float upper_mm;      // L1 大臂：肩关节轴心 → 肘关节轴心
    float fore_mm;       // L2 小臂：肘关节轴心 → 腕点
    JointCalib shoulder; // servo[0]
    JointCalib elbow;    // servo[1]
    int elbow_sign;      // +1 / -1，固定逆解分支
    unsigned gripper_servo; // 松紧，2
    unsigned wrist_servo;   // 旋转，3（不参与解算）
};

// 实车标定值，全部 TBD
// ⚠️ 工程是 C++14（CMakeLists.txt:7），不能用 .field = 的 designated initializers。
// 只能按声明顺序位置初始化；字段顺序 = 初始化顺序，以后只在末尾追加。
const ArmModel kArm = {
    110.0F, 95.0F,                            // upper_mm(TBD-1), fore_mm(TBD-1)
    {  0.0F, 0.9F, -90.0F,  90.0F},           // shoulder: zero, deg/%, min, max  ✅ θ1∈[-90,+90]
    { 90.0F, 0.9F,   0.0F, 180.0F},           // elbow   : zero, deg/%, min, max  ✅ θ2∈[0,+180]，zero 必须 90°
    +1,                                       // elbow_sign ✅ 向上兜
    2U,                                       // gripper_servo
    3U,                                       // wrist_servo (roll)
};

} // namespace arm_geometry
```

设计取舍：

- **`deg_per_percent` 用带符号的浮点**，而不是 `sign + kDegPerPercent` 两个字段——方向和比例本来就是一件事，分开写只会让两处数字打架。Cortex-M3 没有 FPU，但这层一帧只算一次，软件浮点开销可忽略。
- **`zero_deg` 是"百分比为 0 时的几何角"**，不是"水平角"。这样标定流程是：舵机回中（`percent=0`）→ 用量角器/照片量出连杆实际角度 → 填 `zero_deg`。不需要把机械结构装配到刚好水平。
- **限位用角度表达**，因为 IK 天然产出角度；百分比限位则由 `min_deg/max_deg` 反算得出，**只保留一套真值**。`src/arm_pose.cpp:36-37` 的 `kLimitMin/Max` 应改成从 `kArm` 派生的 `constexpr`，避免"角度限位改了、百分比限位没改"。

---

## 5. 正解与逆解

### 5.1 正解（FK）—— 用于自检和串口回读

$$
\begin{aligned}
x &= L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2)\\
y &= L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)
\end{aligned}
$$

FK 在 v0.1 里不是可选项：**任何一次 IK 输出都要用 FK 回算一遍末端，和输入目标比对**，误差超阈值就在串口打警告。这是没有编码器的开环系统里唯一能发现"标定填错了"的手段。

### 5.2 逆解（IK）—— 余弦定理

设 $r^2 = x^2+y^2$，先解肘角：

$$
\cos\theta_2=\frac{r^2-L_1^2-L_2^2}{2L_1L_2},\qquad
\theta_2=\pm\arccos(\cos\theta_2)
$$

再解肩角。大臂与"肩→目标"连线的夹角：

$$
\beta=\arctan_2\!\left(L_2\sin\theta_2,\; L_1+L_2\cos\theta_2\right),\qquad
\theta_1=\arctan_2(y,x)-\beta
$$

`θ2` 的符号就是 `elbow_sign`：**同一目标点有两个解（肘上 / 肘下），结构上一般只有一个能用**。实车标定时把摇杆推到目标附近，看哪个分支不撞结构、不撞车体，就固定哪个。

### 5.3 三个必须处理的边界

| 情况 | 判据 | 处理 |
|---|---|---|
| 目标太远 | $r > L_1+L_2$ | 沿 $\arctan_2(y,x)$ 方向缩到 $r' = L_1+L_2$，`reachable=false` |
| 目标太近 | $r < \lvert L_1-L_2\rvert$ | 缩到 $r' = \lvert L_1-L_2\rvert$，`reachable=false` |
| 完全伸直 / 折叠（奇异） | $r$ 恰在边界 | 同上钳位；末端在该点只能沿切线方向动，操作员会感觉"推不动" |
| 关节角超限 | $\theta_1 \notin [\text{min},\text{max}]$ | 角度钳位，`limited=true`；末端偏离目标，由 FK 回算如实报告 |

**`acos` 的入参必须先钳到 `[-1, 1]`**，否则浮点误差会让 $r$ 正好等于 $L_1+L_2$ 时算出 `NaN`，一路传到 TIM8 的 `CCR` 就是舵机乱抽。

钳位策略选"缩到半径边界"而不是"直接拒绝"，是为了让摇杆/串口连续推的时候手感连续：推到超界就是顶在最大伸展处，松回来立刻恢复跟随。

### 5.4 数值基准（L1=110, L2=95，供验收用）

| 目标 (mm) | r | θ1 | θ2 | 可达 | FK 回算 |
|---|---|---|---|---|---|
| (150, 120) | 192.09 | +19.73° | +40.99° | ✅ | (150.00, 120.00) |
| (0, 205) | 205.00 | +90.00° | 0.00° | ✅（边界） | (0.00, 205.00) |
| (150, 140) | 205.18 | — | — | ❌ 超 0.18 mm | 钳位后 (149.87, 139.88) |
| (100, 0) | 100.00 | −53.54° | +122.16° | ✅ 几何上；**角度超限** | (100.00, 0.00) |

第 3 行是"差一点点就够不着"的典型，第 4 行是"几何可达但关节转不过去"的典型——**两种情况都必须被上层看到**，所以返回值里 `reachable` 和 `limited` 是两个独立标志，不能合成一个 `ok`。

---

## 6. 角度 ↔ 舵机百分比

正映射（IK 输出 → 下发）：

```cpp
percent = (deg - calib.zero_deg) / calib.deg_per_percent;   // 带符号
```

反映射（手动 jog / 标定打印）：

```cpp
deg = calib.zero_deg + percent * calib.deg_per_percent;
```

`deg_per_percent` 的物理来源：`outputs.cpp:31-32` 里 `100% → 1000 µs`，而常规舵机 `1000 µs → 90°`，所以 `1% → 0.9°`。**这个换算链里有两个假设要实测**：舵机的实际脉宽-角度关系（180° 舵机 ≠ 270° 舵机）、以及"行程百分比到脉宽的线性度"。标定法：

1. 把某关节发 `percent = +50`，量实际角度变化量 Δθ；
2. `deg_per_percent = Δθ / 50`，正负号取实测方向；
3. 在两个不同 percent 点各测一次，确认线性（偏差 > 3° 说明舵机非线性或机械间隙大，标定表要改成查表）。

---

## 7. 与现有模块怎么接

### 7.1 数据流变化（只加一层，不改插值）

```
arm_pose_request_xy(x, y)          ← 新：坐标入口
        │
        ▼
arm_kinematics: IK → (θ1, θ2) → percent   ← 新文件
        │  不可达/超限 → 钳位 + 标志
        ▼
g_target[0..1] 更新；g_target[2..3] 不动    ← arm_pose.cpp 内部，沿用
        │
        ▼
arm_pose_update()  限速逼近（kSlewPercentPerSecond，300 %/s）  ← 不改
        │
        ▼
apply_servos()  1500 ± 1000 µs                                  ← 不改
```

### 7.2 三个入口并存，互不干扰

| 入口 | 谁用 | v0.1 行为 |
|---|---|---|
| `arm_pose_request(ArmPose)` | 手柄 △○□L2 | 姿态表内部改成坐标，走 IK；接口签名不变 |
| `arm_pose_request_xy(x, y)` | 串口 / 上位机 / 动作序列 | 新增 |
| `arm_pose_set_jog(joint, %/s)` | 左摇杆手动微调 | **完全不动**。它是标定和应急手动的手段，不能在引入 IK 后废掉 |

关键：`arm_pose_set_jog` 直接改百分比，绕过 IK。这不是漏洞——手动微调本来就该作用在关节空间。但要让 `at_target` 语义保持一致（`src/arm_pose.cpp:150-158` 比较的是目标与当前的百分比差，仍然有效）。

### 7.3 夹爪两路

- `servo[2]`（松紧）：`L1/R1` 走 `arm_pose_set_jog(arm_pose_gripper_servo(), ±kGripperJogPercentPerSecond)`，`src/mode_control.cpp:85-94`，**一行都不用改**。
- `servo[3]`（旋转）：目前**没有任何按键绑定**，姿态表里第 4 列也恒为 0。v0.1 让它保持"当前值不动"，即 IK 只写 `g_target[0]` 和 `g_target[1]`。
- `servo[3]` 已确认是绕小臂轴线的 **roll**，旋转不改变腕点位置，因此二维 2R 模型成立，不需要升级成 3R（原 TBD-6 已关闭）。

### 7.4 急停与断线

`src/mode_control.cpp:143` 的 `stopped ? arm_pose_hold() : arm_pose_update()` 不动。IK 是"目标生成器"，只在目标被请求时跑一次，不占用急停路径。

---

## 8. 接口草案

```cpp
// inc/arm_kinematics.hpp（草案，v0.1 落地时按此实现）

namespace arm_geometry {

struct JointCalib { float zero_deg, deg_per_percent, min_deg, max_deg; };

struct ArmModel {
    float upper_mm, fore_mm;
    JointCalib shoulder, elbow;
    int elbow_sign;
    unsigned gripper_servo, wrist_servo;
};

extern const ArmModel kArm;

struct Solution {
    bool reachable;     // 目标在工作空间内（未钳位）
    bool limited;       // 关节角被机械限位裁剪
    float x_mm, y_mm;   // 实际末端（钳位后），用 FK 回算填充
    float shoulder_deg, elbow_deg;   // 钳位后的关节角，串口打印用
    int shoulder_percent;
    int elbow_percent;
};

// 平面 2R 逆解：输入腕点，输出两路行程百分比。
Solution solve_xy(float x_mm, float y_mm);

// 正解：给定两路百分比，回算腕点位置（自检 / 串口打印 / 标定核对）。
void solve_forward(int shoulder_percent, int elbow_percent, float &x_mm, float &y_mm);

// 单路换算，标定工具也用。
float percent_to_deg(const JointCalib &c, int percent);
int deg_to_percent(const JointCalib &c, float deg);

} // namespace arm_geometry
```

实现骨架（`src/arm_kinematics.cpp` 草案，注意本工程开 `-Wall -Wextra -Werror`，未用参数会直接编译失败）：

```cpp
Solution solve_xy(float x, float y) {
    const float l1 = kArm.upper_mm, l2 = kArm.fore_mm;
    const float r = std::sqrt(x * x + y * y);

    Solution s{};
    float tx = x, ty = y;
    if (r > l1 + l2) {                       // 太远：径向钳位
        const float k = (l1 + l2) / r;
        tx = x * k; ty = y * k; s.reachable = false;
    } else if (r < std::fabs(l1 - l2)) {     // 太近
        // ⚠️ r 接近 0 时方向未定义，绝不做 rmin / r（会出 inf → NaN → INT_MIN 脉宽）
        if (r < 1.0e-3F) {
            tx = std::fabs(l1 - l2); ty = 0.0F;
        } else {
            const float k = std::fabs(l1 - l2) / r;
            tx = x * k; ty = y * k;
        }
        s.reachable = false;
    } else {
        s.reachable = true;
    }

    const float rr = std::sqrt(tx * tx + ty * ty);
    float c2 = (rr * rr - l1 * l1 - l2 * l2) / (2.0F * l1 * l2);
    c2 = c2 < -1.0F ? -1.0F : (c2 > 1.0F ? 1.0F : c2);   // 必须钳，否则 acos 出 NaN
    const float t2 = static_cast<float>(kArm.elbow_sign) * std::acos(c2);
    const float t1 = std::atan2(ty, tx) -
                     std::atan2(l2 * std::sin(t2), l1 + l2 * std::cos(t2));

    // 角度限位 → 百分比 → 整数
    const float d1 = clampf(t1 * kRadToDeg, kArm.shoulder.min_deg, kArm.shoulder.max_deg);
    const float d2 = clampf(t2 * kRadToDeg, kArm.elbow.min_deg,   kArm.elbow.max_deg);
    s.limited = (std::fabs(d1 - t1 * kRadToDeg) > 1.0e-3F) ||
                (std::fabs(d2 - t2 * kRadToDeg) > 1.0e-3F);   // 带容差，边界点不误报
    s.shoulder_percent = deg_to_percent(kArm.shoulder, d1);
    s.elbow_percent    = deg_to_percent(kArm.elbow, d2);

    solve_forward(s.shoulder_percent, s.elbow_percent, s.x_mm, s.y_mm); // 自检
    return s;
}
```

姿态表也随之从"四列百分比"变成"坐标 + 两路夹爪值"：

```cpp
struct PoseTarget { float x_mm, y_mm; int gripper; int wrist; };
constexpr PoseTarget kPoseTable[kPoseCount] = {
    //  x_mm   y_mm  grip  wrist
    { 140.0F,  60.0F,   0,    0 },   // Home ：收纳，贴近车体
    { 150.0F, 120.0F, -20,    0 },   // Ready：准备抓取
    { 110.0F, 150.0F, -60,    0 },   // Lift ：抓住后抬起（夹紧值 TBD）
    { 120.0F,  90.0F,   0,    0 },   // Place：放置（数值 TBD）
};
```

这四个示例坐标按 L1=110 / L2=95 全部在可达域内（可作 IK 单测基准）：

| 姿态 | r (mm) | θ1 | θ2 |
|---|---|---|---|
| Home (140, 60) | 152.3 | −15.2° | +84.3° |
| Ready (150, 120) | 192.1 | +19.7° | +41.0° |
| Lift (110, 150) | 186.0 | +30.8° | +49.9° |
| Place (120, 90) | 150.0 | −2.3° | +86.2° |

注意 Home / Place 的 `θ2` 已经 84~86°，逼近常见的 90° 结构限位——这正是"限位必须用真实值"的例子，摆位设计时要盯住这一列。

---

## 9. 标定流程（顺序不能乱）

1. **量连杆**：`upper_mm` = 肩轴心→肘轴心，`fore_mm` = 肘轴心→腕点。卡尺，取轴线中点，不是外壳边缘。
2. **定末端点**：在实物上贴个记号标出"腕点"在哪（夹爪安装座的回转中心）。之后所有坐标都以它为准。
3. **定 `elbow_sign`**：随便给个可达目标，分别用 +1 / −1 解出来，看哪个分支舵机能转过去且不撞结构。
4. **定 `zero_deg`**：`arm_pose_set_jog` 把关节推到 `percent = 0`，量连杆实际角度。大臂取"大臂水平"= 0°；**小臂取"小臂相对大臂延长线垂直朝上"= +90°**（取 0° 会让 θ2=180° 时需要 percent=200，超出行程，见 12.3）。
5. **定 `deg_per_percent`**：推到 `percent = ±50`，量角度差，除以 50，带符号。
6. **定 `min_deg / max_deg`**：摇杆慢慢推，推到"刚好碰到机械限位"，记角度，回退 2~3° 留余量。**这一步替代现有 `kLimitMin/Max` 的全行程占位值**（`src/arm_pose.cpp:36-37`）。
7. **验收**：串口下发 `(150, 120)`，FK 回算应得 `(150.0, 120.0)`；实物腕点与目标点目测重合，误差 < 5 mm。再用 `(0, 205)` 试边界、`(150, 140)` 试钳位。

前 3 步决定几何对不对，后 3 步决定会不会撞结构。**先做 4~6 再做 1~3 也行，但不要只做一半就上电跑姿态——半标定的 IK 比纯百分比更危险**，因为它会自信地把舵机推向限位。

---

## 10. 调试与验证

> ⚠️ **范围修正**：串口接收需要 `HAL_UART_Receive_IT` + `USART3_IRQHandler`，会动 `Core/Src/main.c` 和 `Core/Src/stm32f1xx_it.c` 两处 CubeMX 生成代码，**已从 v0.1 砍到 v0.1.1**（见整合报告 R1）。v0.1 的验证手段改为：改 `kPoseTable` 坐标 + 手柄触发 + 串口打印 θ/reach/lim。下表作为 v0.1.1 的目标形态留档。

串口（USART3，115200）加两条命令，`src/diagnostics.cpp` 的 `report_arm` 扩展打印：

| 命令 | 行为 | 打印 |
|---|---|---|
| `P <x> <y>` | 请求解算并下发 | `IK x=150 y=120 -> S1=..% S2=..% th1=.. th2=.. reach=1 lim=0 FK=150.0,120.0` |
| `F` | 只回算，不动舵机 | `FK S1=..% S2=..% -> x=.. y=..` |

`F` 命令是纯只读的自检，标定时用它核对 `zero_deg`：手动把大臂摆到明显水平的位置，看 `F` 返回的 `θ1` 是否接近 0。

当前 `report_arm`（`src/diagnostics.cpp:63-73`）只打四个百分比，v0.1 加两个字段即可，不要重写整个格式（现有标定手法依赖 `S1..S4` 的固定位置）。

---

## 11. 风险清单

| 风险 | 后果 | v0.1 的应对 |
|---|---|---|
| 舵机开环无反馈 | IK 认为到了，实际没到（堵转/掉电/齿隙） | FK 回算只能验证"模型自洽"，验证不了实物。**必须人工目测一次** |
| `deg_per_percent` 假设线性 | 大角度处系统性偏差 | 两点测量校验；偏差大改查表 |
| ~~`servo[3]` 若不是 roll~~ | 已确认是 roll，二维模型成立 | TBD-6 已关闭 |
| 姿态表坐标填在不可达区 | 上电即死顶限位、电流拉满 | 上电首帧对 `kPoseTable` 全表做一次 IK 校验，`reachable=false` 就在串口报警并拒绝切姿态（建议加，v0.1 可选） |
| 关节空间插值的末端轨迹是弧线 | 抓取路径可能扫到障碍 | v0.1 接受；要求直线走 v0.2 笛卡尔插补 |
| 小臂 ±180° 当限位用 | 小臂叠回大臂、撞结构、舵机堵转 | ±180° 只是舵机标称；`min/max_deg` 必须换成实测结构挡块值（TBD-4） |
| **水平位静力矩最大**（本车主要工况） | 开环舵机下垂，实物到不了目标高度，IK 正确也没用 | 标定时在水平位实测下垂量；> 10 mm 则加目标 `y` 预补偿（v0.2） |
| 车体倾斜/悬挂压缩 | 平面倾斜，`+x` 不再水平 | 不补偿；操作员保证车体水平 |
| 软件浮点精度 | 无 FPU，float 有舍入 | 单帧一次 IK，误差 << 1 mm，可忽略；不要用 double（M3 上翻倍开销） |

---

## 12. 待确认清单（TBD）

| 编号 | 内容 | 怎么定 |
|---|---|---|
| TBD-1 | `upper_mm` / `fore_mm` 实际值 | 卡尺量轴线距 |
| TBD-2 | `zero_deg`（两个关节各一个） | percent=0 时量连杆角度 |
| TBD-3 | `deg_per_percent`（含符号） | percent=±50 量角度差 |
| TBD-4 | `min_deg` / `max_deg` 机械限位 | 推到刚碰结构，回退 2~3° |
| ~~TBD-5~~ | `elbow_sign` 可用分支 | ✅ 已确认：**+1**（小臂向上兜） |
| ~~TBD-6~~ | `servo[3]` 是 roll 还是 pitch | ✅ 已确认：roll，二维模型成立 |
| ~~TBD-7~~ | `servo[0]` / `servo[1]` 哪个是大臂 | ✅ 已确认：0=大臂、1=小臂 |
| TBD-8 | 姿态表四个坐标 | 标定完成后按实际抓取点抄 |

TBD-6 / TBD-7 已关闭，剩下的 TBD-1~4 和 TBD-8 都是纯测量项，按第 9 节顺序做即可。**只有 TBD-5 是"选择题"**，因为它的答案取决于实物形状而不是测量值，单独说明如下。

### 12.1 `elbow_sign` 到底是什么

平面 2R 逆解对同一个目标点**永远有两个解**，它们关于「肩→腕」这条连线镜像对称。物理含义是：小臂可以朝上折，也可以朝下折，都能把腕点送到同一个位置。

以 L1=110、L2=95、目标 `(150, 120)` 为例（python 实算）：

| `elbow_sign` | θ1 | θ2 | 肘关节位置 | 连线哪一侧 | 实物形状 |
|---|---|---|---|---|---|
| **+1** | +19.73° | +40.99° | (103.5, 37.1) | 下方 | 大臂低平、小臂上翘，臂弯成一个"向上的兜" |
| **−1** | +57.59° | −40.99° | (59.0, 92.9) | 上方 | 大臂高举、小臂下垂，像吊车/挖掘机的斗杆 |

```
        P●                    [肘]●
       /                     /     \
      /       肘在上方      /       ●P
   [肘]●   ← sign=−1      ●肩     ← sign=+1 的镜像
   /                        \         |
  ●肩                        \        |
   |                          [肘]●  |
   |  sign=+1                  \     |
   |  肘在下方                  \    |
```

（两张图是同一目标点的两个镜像解，不是不同目标点。）

**结构上通常只有一个分支能用**：另一个要么被机械限位挡住，要么小臂会穿过车体/底盘。所以这一步不是"算"，是"选"。

> **已确认（本车）**：`elbow_sign = +1`，即小臂**向上兜**——大臂往下探、小臂向上折。判定依据：在 `y = 0`、`x = 150 mm` 时取到 θ1 = −39.2°、θ2 = +86.2°（大臂低于水平、小臂上翘 86°）。上电后可先做一次快速验证：**从远往近收时，大臂舵机应该先往负方向转、小臂折叠角单调增大**；若实测大臂是向上抬的，那就是 sign = −1，必须改回来。
>
> 因为只取 `θ2 ≥ 0` 这一支，「吊车式」那个镜像解彻底被排除——好处是不存在运行时的分支歧义。

三种判定方法，从快到准：

1. **看形状（最快，一句话）**：把腕点摆在能到的地方，观察小臂相对于"肩→腕连线"是往上折还是往下折。
2. **试两个分支（最可靠）**：算两组的百分比，分别下发（`arm_pose_set_jog` 手动摆到接近位置也行），看哪组能让腕点真的落在目标点，且不撞结构。撞的那个会被限位挡住或明显到不了。
3. **读数反推（不用判断，最省事）**：手动把机械臂摆成一个明确形状，抄串口打印的 `S1 S2`，连同"肘朝上/肘朝下"一起告诉我，我用 FK 反推哪个符号匹配。

第 3 种给用户的回复模板（复制填空即可）：

```
TBD-5 回复：
腕点摆在 (___ mm, ___ mm) 附近时，S1 = ___ ，S2 = ___ ，实物是【肘朝上 / 肘朝下】
```

若两个分支都能到同一位置且都不撞结构（结构对称时会发生），则选**关节角更靠近行程中间**的那支——离限位越远，越不容易堵转发热。

### 12.2 水平工作线（y ≈ 0）：简化了什么、没简化什么

实际任务约束：**腕点大部分时间与肩关节同高**（`y ≈ 0`，只能在车体前方水平线上工作）。这条约束很有用，但它**不能**替我们定 `elbow_sign`。

**不能简化的部分**：在 `y = 0` 上两个解依然完全对称——θ1 反号、θ2 反号。以 L1=110 / L2=95 实算：

| x (mm) | 分支 A（sign=+1）θ1 / θ2 | 分支 B（sign=−1）θ1 / θ2 |
|---|---|---|
| 60 | −59.6° / +147.0° | +59.6° / −147.0° |
| 100 | −53.5° / +122.2° | +53.5° / −122.2° |
| 145 | −40.9° / +90.3° | +40.9° / −90.3° |
| 180 | −26.4° / +57.4° | +26.4° / −57.4° |
| 205 | 0.0° / 0.0° | 0.0° / 0.0° |

所以 TBD-5 仍然得靠"哪支不撞结构"来选，`y = 0` 没有额外信息。

**能简化的部分**（这才是这条约束的价值）：

1. **输入退化成一维**：`arm_solve_x(x)` 等价于 `solve_xy(x, 0.0F)`，可以预计算成 `x → (θ1, θ2)` 定长表，运行时查表 + 线性插值，连 `acos`/`atan2` 都省了。v0.1 建议仍保留通用 `solve_xy`（过渡路径要用），额外提供 `arm_solve_x` 便捷入口。
2. **工作范围完全由小臂折叠限位决定**，而不是由连杆总长决定：

| 小臂限位 | 水平线上可用 x | 可用行程 |
|---|---|---|
| ±90° | ≥ 145.3 mm | 59.7 mm |
| ±100° | ≥ 132.3 mm | 72.7 mm |
| ±120° | ≥ 103.3 mm | 101.7 mm |
| ±150° | ≥ 55.0 mm | 150.0 mm |
| ±180°（本车暂定） | ≥ 15.0 mm | 190.0 mm（几何值，结构做不到） |

   推导：`cos θ2 = (x² − L1² − L2²)/(2L1L2)`，令 `θ2 = θ2_max` 即得 `x_min`。

   **本车暂定的小臂 ±180° 让整条 15~205 mm 在几何上都可达**，但 θ2 = 180° 意味着小臂完全叠回大臂上——机械结构必然先撞。所以 ±180° 只能当"舵机标称上限"填进 `kArm`，真正的 `min_deg / max_deg` 仍然是 TBD-4 的实测值。填实测值时请按上表对照：**你把下限收到多少度，就等价于承认 x 只能靠到多近**。
3. **边界明确**：`x = 205`（`L1+L2`）是伸直奇异点，末端只能沿切线微动；`x` 越靠近 0 折叠越狠，最终撞小臂限位。

**新增的风险**（水平线特有）：

- **静力矩最大**。臂完全水平伸展时，重力对肩关节的力矩是全程最大值（∝ L1·m1/2 + L2·m2 + 负载×x）。开环舵机在这个姿态下垂最明显，**IK 算得再对，实物也到不了目标高度**。标定时必须在水平位实测下垂量；若下垂 > 10 mm，要么换更大扭力舵机，要么在 v0.2 加"目标 y 预补偿"。
- 这也意味着「水平线工作」是**最费舵机的工况**：持续大电流、发热、齿隙累积。姿态设计上应尽量让中间姿态靠近折叠位（θ2 大、x 小），只在抓取瞬间伸展。

若确认全部任务都在 `y = 0`，第 8 节姿态表的示例坐标应把 `y_mm` 全部改成 0 附近（±10 mm 内），并按上表的 `x_min` 约束重新取点。

### 12.3 限位落到舵机行程上：小臂 `zero_deg` 必须取 90°

本车确认的限位是 θ1 ∈ [−90°, +90°]、θ2 ∈ [0°, +180°]。`percent` 的全行程恰好是 180°（`−100..+100` × 0.9°/1% = 180°），和两个关节的行程长度**一样长**，所以两路都能一点不浪费地用满 ±100%——但前提是 `zero_deg` 选在行程中点：

| 关节 | 角度行程 | 长度 | 中点 = `zero_deg` | percent 范围 |
|---|---|---|---|---|
| 大臂 `shoulder`（servo[0]） | −90° … +90° | 180° | 0°（**大臂水平**） | −100 … +100 ✅ |
| 小臂 `elbow`（servo[1]） | 0° … +180° | 180° | **+90°（小臂垂直于大臂）** | −100 … +100 ✅ |

**反例（必须避免）**：小臂若把 `zero_deg` 定在 0°（小臂伸直），那么 θ2 = 180° 时需要 `percent = 180/0.9 = 200`，直接冲出 `[-100, 100]`——`deg_to_percent` 会把结果钳死，臂永远折不到底，而且限位表和标定表会互相打架。

所以 TBD-2 对小臂的具体动作是：**找到"小臂相对大臂延长线正好 90°（垂直朝上）"的那个物理姿态，把它标成 `percent = 0`**；大臂则是"大臂水平"标成 0°。

顺带一个坑：如果用的是 270° 舵机，`deg_per_percent` ≈ 1.35，percent ±100 就对应 ±135°，超出行程——所以 `deg_per_percent` 必须实测（TBD-3），不能照抄 0.9。

---

## 13. v0.2 候选（不在本版）

- 笛卡尔直线插补：每 `dt` 在 `(x,y)` 上按 `move_speed` 前进一点再 IK，末端走直线。
- 夹爪长度补偿：`tool_mm` 沿 `θ1+θ2` 方向外推，坐标改指指尖。
- 目标 `y` 预补偿：按水平位实测下垂量拟合一条补偿曲线，把"想要的高度"减去下垂量再解。
- 一维水平线查表：`x → (θ1, θ2)` 定长表 + 线性插值，省掉运行时 `acos`/`atan2`。
- 姿态间路径规划：先抬后伸，避免两连杆扫过车体。
- `servo[3]` 旋转接入按键（`L2` 已占用，可选 `R2`）。
- 上电 IK 全表校验 + 拒绝非法姿态。

---

## 14. 改动文件清单（v0.1 落地时）

| 文件 | 动作 |
|---|---|
| `inc/arm_kinematics.hpp` | 新增：`ArmModel` / `JointCalib` / `Solution` + 接口 |
| `src/arm_kinematics.cpp` | 新增：FK、IK、限位、百分比换算 |
| `CMakeLists.txt` | `target_sources` 加一行（同 `src/arm_pose.cpp` 附近，第 23-24 行） |
| `inc/arm_pose.hpp` | 加 `arm_pose_request_xy(float, float)`；`ArmPose` 枚举不变 |
| `src/arm_pose.cpp` | `kPoseTable` 改成 `PoseTarget` 坐标表；`kLimitMin/Max` 改为从 `kArm` 派生 |
| `src/diagnostics.cpp` | `report_arm` 增打印 θ1/θ2 与 FK 回算值 |
| `inc/car_config.hpp` | 无需改（`kArmDeadzone`、`kSlewPercentPerSecond` 沿用） |
| `README.md` | "机械臂标定"一节改写为上面第 9 节流程 |

不动：`src/mode_control.cpp`、`src/outputs.cpp`、`src/ps2*.cpp`、`src/drive_control.cpp`、`src/main.cpp`。
