# ps2_control v0.1 —— 两模式按键方案与分发重构

> 状态：**设计稿，未落地代码**。改动集中在 `src/mode_control.cpp` / `src/drive_control.cpp` / `inc/car_config.hpp` / `src/diagnostics.cpp`。
>
> ⚠️ **本文已被 `ps2_control_v0.2.md` 修订**：单电机测试、右摇杆旋转、`×`/`Select` 按住停车、左摇杆单关节 jog、`↑↓` 选关节、`g_calibrating` 标定子态、`R3` 标定入口**全部删除**，左摇杆改为直连笛卡尔解算。按键以 v0.2 第 3 节为准，第 2.2 / 2.3 节作废。
> 机械臂算法层（笛卡尔精调、8 条预设、`ArmFeedback`）由 `arm_pose_v0.3.md` 定义，本文只负责「按键 → 命令」这一层，不含任何几何运算。
> 取证基准：写本文时逐文件核对过现有代码，引用都带 `file:line`。

---

## 0. 结论摘要

| 项 | 结论 |
|---|---|
| 模式枚举 | 不变，仍是 `Drive / Arm`（`inc/mode_control.hpp:9`） |
| 模式入口 | **Select 的 toggle 改成两个直接入口**：`Start` = 进模式一（底盘，上电默认），`Select` = 进模式二（机械臂） |
| 底盘驱动 | 左摇杆平移；**L2/R2 自转**；方向键仍是平移备份；单电机测试从 `START+方向` 搬到 `L3+方向` |
| 机械臂精调 | 左摇杆 = 末端笛卡尔速度；**L2/R2 夹爪松紧**；**L1/R1 腕 roll**；**方向键 + △○╳□ = 8 条预设** |
| 急停 | 手柄侧**必须换键**（`×` 被预设占用）→ 定为 **L3+R3 同按 ≥200 ms**；板载 Key 保留为第二道保险 |
| 标定 | 手柄侧入口从 `START` 改为**按住 R3**（held，不 latch） |
| 关键冲突 | `drive_control.cpp:59-61` 用 `Cross`/`Select` 当全停条件，与新语义直接冲突，**必须删** |

---

## 1. 现状取证：现在的按键映射与模式逻辑

### 1.1 模式状态机（现状）

```
上电 ──► Drive（src/mode_control.cpp:100）
          │
          │ Select 上升沿（:127-132），且 !stopped
          ▼
        Arm ──► 再按 Select 回到 Drive（同一个 toggle 分支）
```

四条现存约束（都要在新方案里明确取舍）：

| # | 现状 | 位置 |
|---|---|---|
| 1 | `Select` 是**双向 toggle**，不是两个入口 | `src/mode_control.cpp:127-132` |
| 2 | **急停锁定时禁止切模式** | `:128` 的 `if (!stopped && select_edge)` |
| 3 | 切模式时 `g_calibrating = false` 并 `resync_arm_edges()`，防止按住不放的键在切进第一帧被当成上升沿 | `:130-131` |
| 4 | `output.calibrating` 只在 Arm 模式为真 | `:147` |

### 1.2 模式一（Drive）现状按键

`src/drive_control.cpp:58-95`：

| 输入 | 行为 | 位置 |
|---|---|---|
| 左摇杆 Y / X | 前进后退 / 左右平移，死区 16，满偏 100% | `:79-80` |
| 方向键 ↑↓←→ | 以 80% 覆盖左摇杆平移 | `:84-88` |
| L1 / R1 | 旋转（顺时针 / 逆时针，80%） | `:89-93` |
| L3 按住 | 右摇杆 X 参与旋转（模拟量旋转） | `:81-82` |
| START + 单方向键 | 单电机测试 35%：上/右/下/左 = M1/M2/M3/M4，R2 反转 | `:67-78` |
| `×` 或 `Select` **按住** | 直接返回全零（第二道停车） | `:59-61` |
| 右摇杆其余 | 不参与 | — |

### 1.3 模式二（Arm）现状按键

`src/mode_control.cpp:44-95`：

| 输入 | 行为 | 位置 |
|---|---|---|
| `START` 上升沿 | 标定模式 latch 开关（串口刷新 500→200 ms） | `:58-60` |
| ↑ / ↓ | 选关节 1→4 循环 | `:61-66` |
| △ / ○ / □ / **L2** | Home / Ready / Lift / Place 四条预设 | `:67-78` |
| 左摇杆 Y | 单关节 jog，150 %/s × 摇杆百分比 | `:81-82` |
| L1 / R1 | 夹爪 **servo[2]** 松紧，±200 %/s | `:85-94` |
| `servo[3]`（腕 roll） | **没有任何绑定**，恒为上个姿态残留值 | `arm_pose_v0.1.md:269` |

### 1.4 全局（两个模式共用）

| 输入 | 行为 | 位置 |
|---|---|---|
| `×` 上升沿 | 手柄侧急停 latch 取反 | `src/mode_control.cpp:120-122` |
| 断线 / 有效帧超时 100 ms | `stopped=true`，电机停、机械臂 hold | `:123-124`、`src/ps2.cpp:8` |
| 板载 Key（PA0，20 ms 去抖） | 硬件级急停 latch，**独立于手柄** | `src/main.cpp:24-33` |

### 1.5 现状里三个必须先知道的坑

1. **`mode=` 这个串口字段不是操作模式**。`src/diagnostics.cpp:13-15` 的 `mode=%02lX` 打的是 **PS2 协议模式字**（0x41/0x73/0x79）。新方案要打操作模式，字段名必须换成 `op=`，否则现场一眼就会读错。
2. **停车判定现在有两处**：`mode_control.cpp:120-124`（latch + 断线）和 `drive_control.cpp:59-61`（Cross/Select 按住）。重复真值，改一处漏一处。v0.1 只保留 `mode_control` 一处。
3. **`Start` 现在身兼三职**：底盘的单电机测试修饰键、机械臂的标定开关、底盘模式下 `Start` 还会屏蔽摇杆（`:67` 的 if 直接 return）。新方案把它变成纯模式键，这三个用途全部要搬家。

---

## 2. 新按键方案（定稿）

### 2.0 一句话更正：麦轮车没有 roll 自由度

你说「左右扳机控制 roll 转向」。**麦轮四轮同平面，车体在平面内的唯一旋转自由度是绕竖轴的 yaw（自转/原地打转），roll（横滚）物理上做不到**。本文按「L2 = 逆时针自转，R2 = 顺时针自转」实现。
如果你要的其实是「斜向横移时车头自动转」，那属于另一种混合逻辑（field-oriented），不在 v0.1。

### 2.1 模式状态机（新）

```
             Start ↑                    Select ↑
   ┌───────────────────────┐   ┌──────────────────────────┐
   ▼                       │   ▼                          │
Drive（默认）  ── Select ↑ ─┴─► Arm(笛卡尔) ── Start ↑ ────┘
   ▲                                 │  ▲
   │                          R3 按住 │  │ R3 松开
   └─────────────────────────────────┴──┴─► Arm(标定, 仅按住期间)
```

| 当前 | 输入 | 结果 | 备注 |
|---|---|---|---|
| Drive | `Select` ↑ | → Arm | 进模式瞬间：清笛卡尔速度、清关节 jog、`resync` 全部边沿、笛卡尔基准 = FK(当前关节值) |
| Drive | `Start` ↑ | 留在 Drive | 幂等，只清边沿 |
| Arm | `Start` ↑ | → Drive | 清笛卡尔速度、清关节 jog、清标定 latch；机械臂**保持姿态不动** |
| Arm | `Select` ↑ | 留在 Arm | 幂等 |
| Arm | 按住 `R3` | 进入标定子态 | 左摇杆改为单关节 jog，↑↓ 选关节；松开立即回笛卡尔 |
| 任意 | `L3`+`R3` 同按 ≥200 ms | 急停 latch 取反 | 两键必须先都松开才能再次触发 |
| 任意 | 手柄断线 | `stopped=true`（不 latch） | 恢复后按原模式继续，机械臂不回中 |
| 急停锁定中 | `Start` / `Select` | **不响应** | 沿用 `mode_control.cpp:128` 的现有约束，避免急停时误操作 |

**为什么标定用「按住 R3」而不是 latch**：latch 的失败模式是「忘了退出」——退出后摇杆本该控制末端，实际仍在单关节 jog，操作手会以为臂卡住。按住式没有这个失败模式，松手自动回正常态，代价是标定时要一直按着 R3（左手拇指按摇杆、右手食指按 R3，桌面上可行）。这是 v0.1 主动做的取舍。

### 2.2 模式一：底盘驱动

| 输入 | 行为 | 相对现状的变化 |
|---|---|---|
| 左摇杆 Y | 前进（推上）/ 后退，死区 16，满偏 100% | 不变 |
| 左摇杆 X | 右 / 左平移 | 不变 |
| **L2 / R2** | **逆时针 / 顺时针自转**，占空比 `kRotateDutyPercent`（默认 80） | ★ 旋转从 L1/R1 搬来；两键同按 = 相抵为 0 |
| 方向键 ↑↓←→ | 平移按钮备份（80%） | 不变 |
| **L3 + 单方向键**（+ **R1** 反转） | 单电机测试 35%：上/右/下/左 = M1/M2/M3/M4 | ★ 修饰键 `Start→L3`、反转键 `R2→R1` |
| `Start` ↑ | 进入/留在模式一 | ★ 不再是单电机测试修饰键 |
| `Select` ↑ | 进模式二 | ★ 不再 toggle |
| 右摇杆 / L1 | 不参与（L1 空闲，留作备用） | ★ 删掉 `L3+右摇杆` 的模拟量旋转 |
| `×` | **不参与**（它的预设语义只在模式二生效） | ★ 不再是全停键 |
| `L3`+`R3` | 急停 latch | ★ 新增 |

### 2.3 模式二：机械臂精调

| 输入 | 行为 | 备注 |
|---|---|---|
| 左摇杆 X / Y | **末端笛卡尔速度**：X 前伸/回收，Y 抬升/下压 | 平方律 + 死区 8，满杆 50 mm/s，规格见 `arm_pose_v0.3.md` |
| **L2 / R2** | **夹爪 松 / 紧**，±200 %/s（`kGripperJogPercentPerSecond`，servo[2]） | 从 L1/R1 搬来 |
| **L1 / R1** | **腕 roll 逆 / 顺**，±150 %/s（`kWristJogPercentPerSecond`，servo[3]） | ★ 全新通道，现状无任何绑定 |
| ↑ / ↓ / ← / → | 预设 P1 / P2 / P3 / P4 | ★ 不再是「选关节」 |
| △ / ○ / ╳ / □ | 预设 P5 / P6 / P7 / P8 | ★ ╳ 从急停改预设 |
| **按住 R3** | 标定子态：左摇杆 = 选中关节 jog（150 %/s）；↑↓ 选关节 1→4 | ★ 替代原 `START` 标定开关 |
| `Start` ↑ | 回模式一 | ★ |
| `L3`+`R3` | 急停 latch | ★ |

**8 条预设槽位怎么分配**（`P1..P8` 是表索引，不是枚举分支）：

| 槽位 | 按键 | 建议语义 | 数值来源 |
|---|---|---|---|
| P1 | ↑ | Home 收纳 / 上电初始 | `arm_pose_v0.3.md` 的预设表 |
| P2 | ↓ | Ready 准备抓取 | 同上 |
| P3 | ← | Place 放置 | 同上 |
| P4 | → | Lift 抬起 | 同上 |
| P5 | △ | 抓取预备（爪张开、贴地） | TBD，未标定则按下无动作 |
| P6 | ○ | 夹紧提升 | TBD |
| P7 | ╳ | 高位转运 | TBD |
| P8 | □ | 松爪释放 | TBD |

**表驱动，不写 8 个 if**：mapping 是数据，不是控制流。

```cpp
// src/mode_control.cpp —— 预设绑定表（唯一真值）
struct PresetBind {
    Ps2Button button;
    unsigned pose;   // kPoseTable 索引
};
constexpr PresetBind kPresetBinds[] = {
    {Ps2Button::Up, 0},      {Ps2Button::Down, 1},
    {Ps2Button::Left, 2},    {Ps2Button::Right, 3},
    {Ps2Button::Triangle, 4},{Ps2Button::Circle, 5},
    {Ps2Button::Cross, 6},   {Ps2Button::Square, 7},
};
constexpr std::size_t kPresetCount = sizeof(kPresetBinds) / sizeof(kPresetBinds[0]);
```

按下判定同样走一个小数组（每个按键一个 `prev` 位），别再为每个键写一个 `g_xxx_prev` 变量——现状 `mode_control.cpp:15-22` 有 9 个独立的 prev 变量，加一个键就要改 4 处。

```cpp
// 边沿检测统一用「上一帧按键位图」，一个变量顶掉所有 g_xxx_prev
uint16_t g_prev_buttons = 0;

bool edge(const Ps2Frame &f, Ps2Button b) {
    return ps2_pressed(f, b) && !(g_prev_buttons & (1u << static_cast<unsigned>(b)));
}
// 每帧末尾 g_prev_buttons = frame.buttons;
```

> 这一条同时解决 `resync_arm_edges()`（`:32-40`）的存在理由：切模式时把 `g_prev_buttons = frame.buttons` 一行就够了。现状那个函数要逐个复制 7 个变量，加键必漏。

### 2.4 急停：L3+R3 的实现语义

```cpp
// 两键同按持续 kEstopHoldMs(200) 才触发；触发后必须两键都松开才能再次触发。
// 200 ms 而不是瞬时，是因为 PS2 20 ms 一帧、两键很少落在同一帧，
// 瞬时判定会时灵时不灵。
bool  g_estop_combo_latched = false;
uint32_t g_combo_since = 0;
```

| 阶段 | 条件 | 动作 |
|---|---|---|
| idle | `L3 && R3` | 记 `g_combo_since = now`，转 arming |
| arming | 仍同按且 `now - g_combo_since >= 200` | `g_latched_stop = !g_latched_stop`，转 fired |
| arming | 任一键松开 | 回 idle（不触发） |
| fired | 两键都松开 | 回 idle（可再次触发） |

**副作用要认**：按住 R3 标定时再按 L3 会触发急停（因为 R3 已在按住状态）。这个方向是安全的（想停就停），反向的误触（想急停却只切了标定）会被 200 ms 窗口挡住——200 ms 内臂不会动，因为标定模式下也要推摇杆才动。

板载 Key 那条（`src/main.cpp:24-33`）**不动**，它是唯一不依赖无线链路的急停。

---

## 3. 分发结构（改完长什么样）

```
main()
 ├─ 板载 Key 去抖 → emergency_stop（不变，主循环内）
 └─ mode_control_update(frame, now)
      ├─ 1. 全局边沿：g_prev_buttons = frame.buttons（每帧末尾更新）
      ├─ 2. 急停组合 L3+R3（第 2.4 节）→ g_latched_stop
      │      stopped = g_latched_stop || !frame.connected
      ├─ 3. 模式入口：Start→Drive / Select→Arm（!stopped；幂等；切换时 clear + resync）
      ├─ 4. 按模式分发：
      │      Drive:  output.drive = drive_from_ps2(frame)
      │      Arm:    output.drive = {}
      │              ├─ 按住 R3 ? 关节 jog（↑↓ 选关节）
      │              │           : 左摇杆 → (vx,vy) → arm_set_cart_velocity()
      │              ├─ L2/R2 → arm_joint_jog(夹爪, ∓/±200)
      │              ├─ L1/R1 → arm_joint_jog(腕,  ∓/±150)
      │              └─ kPresetBinds 遍历 → edge ? arm_pose_request(pose)
      └─ 5. output.arm = stopped ? arm_pose_hold(now) : arm_pose_update(now)
```

**分层约定（低冗余的关键）**：
- `mode_control` 只做「读按键 → 写速度/请求」，**不做积分、不做限位、不做几何**。所有 jog 都是「本帧速度」，下一帧不重新给就自动停 —— 这条语义现状已经成立（`arm_pose.hpp:16-18`），新方案全部照用。
- `arm_pose` / `arm_joint` / `arm_kinematics` 不认识 `Ps2Frame`，也不 include `ps2_protocol.hpp`。手柄换掉、换成串口或上位机，机械臂层一行不改。
- 夹爪与腕 roll 不是特殊通道，就是关节层里的第 3、4 路，和两个解算关节共用一套 jog/限速/限位代码（v0.2 曾想给夹爪单开一条，那是冗余，v0.3 已统一）。

---

## 4. 逐文件改动清单

| 文件 | 动作 | 量级 | 说明 |
|---|---|---|---|
| `inc/car_config.hpp` | 加 `kRotateDutyPercent=80`、`kWristJogPercentPerSecond=150`、`kEstopHoldMs=200` + static_assert | +8 行 | 「手感参数」留在 `car_config`；几何/工作区/预设进 `arm_config`（v0.3） |
| `src/mode_control.cpp` | **主体重写**：直接入口、位图边沿、急停组合、按住 R3 标定、预设查表、按模式分发 | 150 → ~200 行 | 删 9 个 `g_xxx_prev`、删 `resync_arm_edges`、删 4 条硬编码姿态分支 |
| `src/drive_control.cpp` | 删 `:59-61`（重复停车）、`:81-82`（右摇杆旋转）、`:89-93`（L1/R1 旋转）；加 L2/R2 旋转、`L3+方向` 单电机测试、`R1` 反转 | 局部，约 -12/+10 行 | `mix_drive` 一行不动 |
| `inc/mode_control.hpp` | 加 `ArmFeedback arm_fb`（v0.3 定义） | +12 行 | `OperationMode` 枚举不变 |
| `src/diagnostics.cpp` | `report_status` 加 `op=DRV/ARM`；`report_arm` 追加 v0.3 的反馈字段 | 局部 | **`mode=` 前缀不动**（那是 PS2 协议字），新字段取名 `op=` |
| `src/main.cpp` | `report_arm` 多传一个参数 | 1 行 | 板载 Key 逻辑不动 |
| `src/arm_pose.cpp / .hpp` | 由 `arm_pose_v0.3.md` 决定 | — | 本文不碰 |
| `CMakeLists.txt` | 加 `src/arm_joint.cpp`、`src/arm_kinematics.cpp`（v0.3） | 2 行 | — |
| `README.md` | 操作一节整段重写（现 `README.md:45-64` 全部失效） | 整节 | — |

**零改动**：`src/ps2.cpp`、`src/ps2_protocol.cpp`、`src/ps2.hpp`、`src/outputs.cpp`、`src/motor_pid.cpp`、`src/arm_control.cpp`、`Core/Src/*`、`Drivers/*`。

---

## 5. 落地顺序（每步可独立编译）

1. `car_config` 加三个常量 + static_assert → 编译零警告（没人用）。
2. `diagnostics` 加 `op=` 字段（打的是现有 `output.mode`，此时永远是 DRV）→ 烧板确认串口格式没破。
3. `mode_control` 换边沿检测成位图 + 直接入口（先不动按键语义），验证「Start 进 Drive / Select 进 Arm」两向都能走通、`resync` 不再漏键。
4. `drive_control` 改旋转与单电机测试键 → 空转台架上确认四个轮子转向正确、单电机测试仍能逐个点动。
5. `mode_control` 改手臂侧：L2/R2 夹爪、L1/R1 腕 roll、按住 R3 标定、预设查表 → 先不带 IK（`arm_pose_request(ArmPose)` 走老百分比表），确认 8 个键各自触发对了。
6. 接 `arm_pose_v0.3` 的关节层与笛卡尔层，再按 v0.3 的验证清单跑。
7. 最后改 README。

第 1~3 步完全不影响现有动作，可以先合。

---

## 6. 上电验证清单

| # | 动作 | 期望 |
|---|---|---|
| 1 | 上电（不按任何键） | 串口 `op=DRV`；四轮不动；舵机在中位 |
| 2 | 推左摇杆各方向 | 平移四轮方向正确（O 型麦轮，无「斜着走」） |
| 3 | 按住 L2 / R2 | 原地自转，方向正确；两键同按 = 停 |
| 4 | 方向键 ↑↓←→ | 80% 平移，方向与摇杆一致 |
| 5 | `L3`+`↑`（再试 `+R1`） | 只有 M1 动，35%；`R1` 反转 |
| 6 | 按 `Select` | `op=ARM`；四轮立即停；串口 `ARM` 行开始带笛卡尔反馈 |
| 7 | 模式二推左摇杆 | 末端按平方律动，松手立即停（不追尾） |
| 8 | 按住 L2 / R2 | 夹爪松开 / 夹紧，`S3` 单向前进到限位即停 |
| 9 | 按住 L1 / R1 | `S4`（腕 roll）正反向转，`S1/S2` 不动 |
| 10 | 逐个按 8 个预设键 | 每个键对应一条姿态；未标定的槽位按串口提示无动作 |
| 11 | 按住 `R3` + ↑↓ | 选关节；推左摇杆只动选中那一路；松开 R3 回笛卡尔，无跳变 |
| 12 | `L3`+`R3` 同按 200 ms | 急停 latch，串口 `STOP=1`；再按一次解除 |
| 13 | 急停锁定中按 `Start`/`Select` | 不切模式，`op=` 不变 |
| 14 | 拔掉接收器 | 电机停、机械臂保持姿态、`online=0`；插回自动恢复，模式不变 |

---

## 7. 风险

| 风险 | 后果 | 应对 |
|---|---|---|
| **`×` 不再是急停** | 肌肉记忆误按 ╳ 想停车，实际触发 P7 预设 | 只能靠训练 + 板载 Key；README 与操作手册必须把这条写成第一行 |
| 扳机是**开关量**（解码只取按键位，无压力值） | 夹爪/腕速度恒定，行程靠按住时长，手感不如行程比例控制 | 接受；要模拟量得扩 `ps2_protocol` 解析 0x79 的压力字节（不在 v0.1） |
| 急停组合要两键 | 单手操作时停不下来 | 板载 Key 兜底；`L3+R3` 是 PS2 通用复位组合，玩家本能 |
| `L3` 在两种模式语义不同 | 底盘模式当单电机测试修饰键，机械臂模式是急停组合的一半 | 靠 `mode_control` 先判组合再判模式分发，顺序不能反 |
| 模式二里左摇杆一推就动 | 切进模式二时鼠标/摇杆有偏移，臂立即开始走 | 进模式瞬间清速度 + 用 `kArmDeadzone=8`；实测漂移大就加到 12 |
| 按住 R3 标定时手部姿态别扭 | 标定效率低 | 接受；真要改就换 latch + 串口可见 `cal=1` 警示（v0.2 做法） |
| 底盘模式两键自转 80% 太猛 | 原地打转冲出去 | `kRotateDutyPercent` 单独可调，默认先给 60~80 由你试 |
| 8 条预设里 4 条是占位 | 按下没反应，以为死机 | 未标定槽位串口打 `pose=N empty` |

---

## 8. 需要你拍板的 5 个决策

| # | 决策 | 我给的默认 | 影响 |
|---|---|---|---|
| **D-1** | 急停改 `L3+R3`（200 ms） | 采纳 | 唯一的替代方案是把某个预设键让回急停，那 8 条预设就只剩 7 条 |
| **D-2** | 标定改「按住 R3」而非 latch | 采纳 | 换成 latch 就要额外的防遗忘手段 |
| **D-3** | 单电机测试改 `L3+方向`，反转改 `R1` | 采纳 | 不做就等于丢掉单轮点动能力 |
| **D-4** | 自转占空比 `kRotateDutyPercent` 默认 80 | 采纳，实测再调 | 与现状 L1/R1 旋转的手感一致 |
| **D-5** | 8 槽位与按键的对应（第 2.3 节表） | 采纳 | 改起来只是 `kPresetBinds` 一行 |

---

## 9. 与 arm_pose_v0.3 的边界

本文**不定义**：逆解、限位数值、工作区、平方律系数、预设姿态数值、`ArmFeedback` 字段。

本文**只定义**：哪个键产生什么「速度 / 姿态请求 / 模式状态」。

对接点只有 4 个函数（v0.3 必须提供）：

```cpp
void arm_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s);  // 每帧一次，松手传 0
void arm_joint_jog(unsigned axis, int percent_per_second);          // 四路统一，夹爪/腕也是 axis
void arm_pose_request(unsigned pose_index);                         // 0..7，替代 ArmPose 枚举分支
ArmFeedback arm_feedback();                                         // 串口反馈
```

`arm_pose_request(unsigned)` 取代现在的 `arm_pose_request(ArmPose)`（`inc/arm_pose.hpp:13`）——枚举只能表达「有几个程序员写了几个分支」，表索引能表达 8 条数据。这是 v0.1 唯一要求 v0.3 改的现有签名。
