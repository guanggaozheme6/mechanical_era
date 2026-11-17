# ps2_control v0.2 —— 砍掉四条旧通道，左摇杆全量交给笛卡尔

> 本文是 `ps2_control_v0.1.md` 的**增量修订**。v0.1 的模式状态机（Start 进模式一 / Select 进模式二、L3+R3 急停）、模式一的平移、模式二的夹爪/腕 roll/8 条预设**继续有效**，本文只写删除项与左摇杆的新语义。
> 删完之后的定稿按键表在第 3 节，**以那张表为准**，v0.1 的第 2.2 / 2.3 节作废。

---

## 0. 变更总览

| # | 删除项 | 原位置 | 删后 |
|---|---|---|---|
| 1 | `L3` 按住 → 右摇杆 X 参与旋转 | `src/drive_control.cpp:81-82` | 右摇杆彻底退出控制；`L3` 只服务 `L3+R3` 急停 |
| 2 | `START` + 单方向键 → 单电机测试 35%，`R2` 反转 | `src/drive_control.cpp:67-78`、`inc/car_config.hpp:16,33-34` | **整块删除，无替代**；单轮点动能力消失 |
| 3 | `×` 或 `Select` 按住 → 全零停车 | `src/drive_control.cpp:59-61` | 只保留 `!connected` 一条；停车统一由 `mode_control` 的 `stopped` 管 |
| 4 | 左摇杆 Y → 选中关节 jog 150 %/s | `src/mode_control.cpp:80-82` | **左摇杆 → 末端笛卡尔速度**（第 2 节） |
| 5 | `↑`/`↓` → 选关节 1→4 | `src/mode_control.cpp:61-66` | `↑↓` 归预设 P1/P2 |
| 6 | 连带：`g_joint`、`selected_joint`、`g_calibrating`、`cal=`、单关节标定子态 | 见第 1.6 / 1.7 | 全部删除，串口 `ARM` 行变短 |

删完之后**没有留下任何「单关节 jog」入口**：`arm_pose_set_jog` 这个函数还在（夹爪 servo[2] 与腕 roll servo[3] 靠它），但关节 0/1 只能由笛卡尔解算驱动。这是有意的收窄——见第 4.1 的后果说明。

---

## 1. 删除清单（逐行，可照着删）

### 1.1 右摇杆旋转（`src/drive_control.cpp:81-82`）

```cpp
// 删掉这两行
    int rotation =
        ps2_pressed(frame, Ps2Button::L3) && frame.analog ? axis(frame.right_x, false) : 0;
```

换成（v0.1 已定的 L2/R2 自转）：

```cpp
    const int rotation =
        (static_cast<int>(ps2_pressed(frame, Ps2Button::R2)) -
         static_cast<int>(ps2_pressed(frame, Ps2Button::L2))) *
        car_config::kRotateDutyPercent;
```

`src/drive_control.cpp:89-93` 的 L1/R1 旋转块同时删除（v0.1 已列）。

**连带清干净（低冗余）**：`right_x` / `right_y` 从此没有任何读者，字段与解码一起删：

| 文件 | 删什么 |
|---|---|
| `inc/ps2_protocol.hpp:30-31` | `uint8_t right_x = 128; uint8_t right_y = 128;` |
| `src/ps2_protocol.cpp:30-31` | `frame.right_x = bytes[5]; frame.right_y = bytes[6];` |
| `src/diagnostics.cpp:13-20` | 串口格式串里的 `RX=%lu` 及其实参 `frame.right_x` |
| `src/drive_control.cpp:20-33` | `axis()` 保留（左摇杆还在用），但 `invert` 参数从此只被 `true` 调用一次——保留签名，别为一行代码重构 |

> 保留 `Core/Src` 不动：解码层的字节 5/6 只是不再存进 `Ps2Frame`，报文字节照样收。

### 1.2 单电机测试（`src/drive_control.cpp:67-78`）

```cpp
// 整块删除（从 if (ps2_pressed(frame, Ps2Button::Start)) { 到对应的 }）
```

连带删除：

| 文件 | 删什么 |
|---|---|
| `inc/car_config.hpp:16` | `constexpr int kCalibrationDutyPercent = 35;` |
| `inc/car_config.hpp:33-34` | 对应的 `static_assert(...)` 两行 |

**没有替代键**。`R1` 在模式一变成空闲键，`R2` 归自转。

### 1.3 `×` / `Select` 按住停车（`src/drive_control.cpp:59-61`）

```cpp
// 删掉
    if (!frame.connected || ps2_pressed(frame, Ps2Button::Cross) ||
        ps2_pressed(frame, Ps2Button::Select)) {
        return {};
    }
// 换成
    if (!frame.connected) {
        return {};
    }
```

理由：`Cross` 已经是模式二的预设键 P7，`Select` 是模式键；这两条判定在按住的那一刻会**把底盘清零再恢复**，表现为「按 Select 进机械臂模式时底盘抽搐一下」。停车判定的唯一真值留在 `src/mode_control.cpp:120-124`。

### 1.4 关节选择（`src/mode_control.cpp:61-66`）

```cpp
// 删掉 up_edge / down_edge 两个变量（:46-47）与这一块
    if (up_edge) {
        g_joint = (g_joint + 1U) % 4U;
    }
    if (down_edge) {
        g_joint = (g_joint + 3U) % 4U;
    }
```

`↑↓` 归预设 P1/P2（v0.1 的 `kPresetBinds`）。

### 1.5 单关节 jog（`src/mode_control.cpp:80-82`）

```cpp
// 删掉
    const int jog = arm_axis_percent(frame.left_y, true);
    arm_pose_set_jog(g_joint, jog * car_config::kJogPercentPerSecond / 100);
```

换成笛卡尔（第 2 节）。`car_config::kJogPercentPerSecond`（`inc/car_config.hpp:23`）随之**没有读者**，删除；`static_assert`（`:36`）同删。

### 1.6 标定子态与 `g_joint` 的残留

| 文件:行 | 删什么 |
|---|---|
| `src/mode_control.cpp:9` | `bool g_calibrating = false;` |
| `src/mode_control.cpp:10` | `unsigned g_joint = 0;` |
| `src/mode_control.cpp:32-40` | 整个 `resync_arm_edges()`（v0.1 已改成位图 `g_prev_buttons`，切模式时一行搞定） |
| `src/mode_control.cpp:15-22` | 9 个 `g_xxx_prev`（同上，被位图取代） |
| `src/mode_control.cpp:58-60` | `START` → `g_calibrating` 取反（`START` 现在是模式键） |
| `src/mode_control.cpp:102,130` | `g_calibrating = false;` 两处 |
| `src/mode_control.cpp:147` | `output.calibrating = ...` |
| `src/mode_control.cpp:148` | `output.selected_joint = g_joint;` |
| `inc/mode_control.hpp:16-17` | `bool calibrating` / `unsigned selected_joint` 两个字段 |
| `src/main.cpp:46` | `const uint32_t report_ms = output.calibrating ? 200U : kReportMs;` → 直接 `kReportMs` |
| `src/main.cpp:49` | `report_arm(output.arm, output.selected_joint, output.calibrating)` → `report_arm(output.arm, output.arm_fb)` |
| `inc/diagnostics.hpp:10` | 签名同步 |
| `src/diagnostics.cpp:63-68` | `report_arm` 去掉 `selected_joint` / `calibrating` 两个参数与 `joint=%u cal=%u` 两段格式 |

**串口格式变化（必须提前通知任何在抄数的人）**：

```
旧: ARM S1=-45 S2=0 S3=0 S4=0 joint=0 cal=0
新: ARM S1=-45 S2=0 S3=0 S4=0 xy=(145.3,0.8) tgt=(145.3,0.8) th=(-40.9,90.3) ok=1 lim=0 wall=0
```

`S1..S4` 前缀与顺序**不动**（标定手法依赖它），只把 `joint=` `cal=` 换成 `arm_pose_v0.3.md` 定义的反馈字段。

### 1.7 `R3` 的标定子态

v0.1 里 `按住 R3 = 单关节 jog + ↑↓ 选关节`。**jog 和选关节都没了，这个子态失去全部内容，一并删除**。
`R3` 在模式二变成**空闲键**（`L3+R3` 急停仍要它）。不要偷偷给它加功能——空闲键是留给以后塞新东西的口子，加之前先改文档。

---

## 2. 左摇杆 → 笛卡尔：完整映射链

### 2.1 链条

```
frame.left_x / frame.left_y  (0..255, 128 中位)
   │  arm_axis_percent(axis, invert)     src/arm_control.cpp:4-19，死区 kArmDeadzone=8
   ▼
percent_x / percent_y (-100..100, 0 = 中位)
   │  arm_pose_stick_velocity(px, py, vx, vy)     ← v0.3 提供（arm_pose.cpp:216-235）
   │    f = percent / 100 → v = v_max·f·|f| 保号平方律（v_max = kMotion.cart_speed = 50 mm/s）
   │    合速度归一化：|v| > v_max 时等比缩到 v_max（双轴满偏 = √2·v_max）
   ▼
arm_pose_set_cart_velocity(vx, vy)    每帧一次；松手 / 断线 / 切模式传 0
```

> **平方律不写在 `mode_control` 里**。v0.3 把它和归一化一起封成 `arm_pose_stick_velocity()`，`mode_control` 只把摇杆换成 `-100..100` 就交出去 —— 否则同一套平方律会有两份实现，改一处漏一处。

```cpp
// src/mode_control.cpp —— update_arm_mode() 里唯一一段摇杆代码。
// 平方律与归一化在 arm_pose_stick_velocity() 内（arm_pose_v0.3.md §6.3），
// mode_control 不认识任何速度常数。
if (!frame.analog) {                       // 绿灯数字模式没有模拟量，必须挡
    arm_pose_set_cart_velocity(0.0F, 0.0F);
} else {
    const int px = arm_axis_percent(frame.left_x, false);
    const int py = arm_axis_percent(frame.left_y, true);
    float vx = 0.0F;
    float vy = 0.0F;
    arm_pose_stick_velocity(px, py, vx, vy);
    arm_pose_set_cart_velocity(vx, vy);
}
```

### 2.2 方向与符号（定稿）

| 摇杆 | 方向 | 末端 | 符号 |
|---|---|---|---|
| `left_y` 推上 | y 增大 | 抬升 | `invert=true`（沿用现有摇杆极性） |
| `left_y` 拉下 | — | 下压 | 同上 |
| `left_x` 推右 | x 增大 | 前伸 | `invert=false` |
| `left_x` 推左 | — | 回收 | 同上 |

死区用机械臂的 8（`kArmDeadzone`），不是底盘的 16——精调需要的死区更小，这一条 v0.1 已定，不变。

### 2.3 三条必须写进代码注释的性质

1. **这是速度，不是位置**。`arm_pose_update` 每帧积分 `v·dt`，1 ms 循环里同一个速度重复给 20 次也没关系（dt 会补上）。松手那一帧必须给 0，否则臂会一直走。
2. **合速度归一化**。双轴同时满偏时 `|v| = √2 × 50 = 70 mm/s`，比设计值快 41%。归一化后斜推也是 50 mm/s，方向不变。v0.2 定为**做**，v0.3 的实测若发现 `slew` 跟不上再议。
3. **数字模式必须挡**。绿灯模式下 `left_x/left_y` 停在解码默认值 128（`ps2_protocol.hpp:31-32`）→ percent 恒 0 → 速度 0，本来是安全的；但 `!frame.analog` 显式挡一道，避免以后有人把默认值改掉。

### 2.4 一帧里摇杆和预设同时发生

优先级：**预设请求 > 摇杆速度**。命中预设键的那一帧丢弃摇杆速度（`arm_pose_set_cart_velocity(0,0)`），否则下一帧闭环积分会把目标从预设点又拉回摇杆方向，表现为「按了预设却先动一下再回去」。

```cpp
bool requested = false;
for (const PresetBind &bind : kPresetBinds) {
    if (edge(frame, bind.button)) {
        arm_pose_request(bind.pose);
        requested = true;
    }
}
if (requested) {
    arm_pose_set_cart_velocity(0.0F, 0.0F);
} else {
    /* 第 2.1 节的摇杆链 */
}
```

### 2.5 三个输入可以同时用

笛卡尔走的是关节 0/1，夹爪/腕走的是关节 2/3 的 jog，两者互不影响。所以**左手摇杆摆位的同时，右手可以按扳机开合夹爪、按 L1/R1 转腕**——这是有意保留的并行性，v0.3 的关节层必须让 `arm_pose_set_cart_velocity` 与 `arm_pose_joint_jog(2/3, ...)` 在同一帧内共存。

---

## 3. 定稿按键表

### 3.1 模式一：底盘驱动（`Start` 进入，上电默认）

| 输入 | 行为 |
|---|---|
| 左摇杆 Y / X | 前进后退 / 左右平移（死区 16，满偏 100%） |
| `L2` / `R2` | 逆时针 / 顺时针自转（`kRotateDutyPercent`，默认 80；同按 = 相抵） |
| 方向键 ↑↓←→ | 平移按钮备份（80%） |
| `Start` ↑ | 留在模式一（幂等） |
| `Select` ↑ | 进模式二 |
| `L3`+`R3` | 急停 latch（200 ms） |
| 右摇杆 / `L1` / `R1` / △○╳□ | **无功能**（右摇杆整条通道已删） |

### 3.2 模式二：机械臂精调（`Select` 进入）

| 输入 | 行为 |
|---|---|
| **左摇杆 X / Y** | **末端笛卡尔速度**：X 前伸/回收，Y 抬升/下压；平方律，满杆 50 mm/s，死区 8，松手即停 |
| `L2` / `R2` | 夹爪 松 / 紧，±200 %/s（servo[2]） |
| `L1` / `R1` | 腕 roll 逆 / 顺，±150 %/s（servo[3]） |
| ↑ / ↓ / ← / → | 预设 P1 / P2 / P3 / P4 |
| △ / ○ / ╳ / □ | 预设 P5 / P6 / P7 / P8 |
| `Start` ↑ | 回模式一 |
| `L3`+`R3` | 急停 latch（200 ms） |
| `R3` 单按 | **无功能**（原标定子态已删） |

> 与 v0.1 的差异只有三处：左摇杆语义（joint jog → 笛卡尔）、`↑↓`（选关节 → 预设）、`R3`（标定 → 空闲）。其余原样。

---

## 4. 删除带来的后果（必须认下的部分）

### 4.1 标定手法变了：不再能「推关节到撞结构」

原来的标定靠单关节 jog 推到机械限位、抄 `S1..S4`。jog 删掉后：

| 标定项 | v0.1 的手法 | v0.2 的手法 |
|---|---|---|
| TBD-1 连杆长度 | 卡尺 | 卡尺（不变） |
| TBD-2 `zero_deg` | jog 到 percent=0，量角度 | 笛卡尔把末端摆到已知坐标，抄 `S1..S4` 反推百分比 → 反推角度（`arm_pose_v0.3` 的 `xy` 反馈直接给解算结果，看一眼就够） |
| TBD-3 `deg_per_percent` | jog 到 ±50%，量角度差 | 同上：两个已知坐标点 → 两组 `S1` → 斜率 |
| TBD-4 结构限位 | jog 推到刚碰结构，回退 2~3° | **做不到**：笛卡尔出口被 `range_*_deg` 钳着，等于「先有限位才能测限位」 |

**TBD-4 的唯一可行路径**：在固件里加一个**编译期开关**（`#if ARM_BRINGUP`）把笛卡尔绕开、直接写某一路百分比，只在标定期间打开、标定完必须关掉并重新烧录。这一类调试口子不进默认构建，也不接任何手柄键——否则它迟早会在赛场上被误触。

这是删掉 jog 的真实代价，写在这里而不是埋进风险表里。

### 4.2 单轮点动没了

换电机、查轮序、确认某个轮子接线，原来的手法是 `START+方向` 单独点动一个轮。删掉之后只剩整体平移，**判断单个轮子是否反向要靠在 4 个轮子上各贴一个箭头、整车慢速前进看哪个轮反了**。

如果这个能力你还想要，唯一干净的方案是把它做成「模式一里按住某个键」而不是删掉——但那又会回到 v0.1 的 `L3+方向`。**v0.2 按你的要求删了，没有替代**；要恢复说一声，改动量是 `drive_control.cpp` 里 12 行。

### 4.3 `ControlOutput` 瘦身

| 字段 | 变化 |
|---|---|
| `bool calibrating` | 删 |
| `unsigned selected_joint` | 删 |
| `ArmFeedback arm_fb` | 增（v0.3 定义） |

`src/main.cpp:46` 的「标定时 200 ms 刷新」分支随之消失，串口固定 500 ms 一次。抄 `S1..S4` 时臂是静止的（松手即停），500 ms 够用。

### 4.4 README 要重写的两段

- `README.md:45-58`（模式与两种模式的按键说明）——整段换成本文第 3 节。
- `README.md:78-87`（机械臂标定）——改成第 4.1 节的新手法 + 明说「没有单关节 jog、TBD-4 需 `ARM_BRINGUP` 编译开关」。

---

## 5. 文件改动清单（相对 v0.1 的增量）

| 文件 | v0.2 追加的动作 | 量级 |
|---|---|---|
| `inc/ps2_protocol.hpp` | 删 `right_x/right_y` | −2 行 |
| `src/ps2_protocol.cpp` | 删两行赋值 | −2 行 |
| `inc/car_config.hpp` | 删 `kCalibrationDutyPercent`、`kJogPercentPerSecond` 及两条 `static_assert` | −4 行 |
| `src/drive_control.cpp` | 除 v0.1 的改动外，再整块删单电机测试（`:67-78`）、删右摇杆旋转 | −24 行 |
| `src/mode_control.cpp` | 除 v0.1 的改动外，删关节选择、删 jog、删 `g_joint`/`g_calibrating`/`resync_arm_edges`；加笛卡尔映射 + 归一化 | 净 −35 行 |
| `inc/mode_control.hpp` | 删 `calibrating` / `selected_joint` | −2 行 |
| `src/main.cpp` | 除 v0.1 外，去掉 `report_ms` 分支 | −3 行 |
| `inc/diagnostics.hpp` / `src/diagnostics.cpp` | `report_arm` 瘦身 + 删 `RX=` | 局部 |
| `README.md` | 两段重写 | 整节 |

**净效果：这个固件比现在少一个单电机测试、少一条右摇杆通道、少一套关节选择/标定子态，多一条笛卡尔精调。代码总量基本持平，状态量少 3 个全局变量。**

---

## 6. 验证清单（替换 v0.1 的第 6 节）

| # | 动作 | 期望 |
|---|---|---|
| 1 | 上电 | `op=DRV`，四轮不动，舵机中位 |
| 2 | 左摇杆各方向 | 平移四轮方向正确 |
| 3 | 按住 `L2` / `R2` | 原地自转，方向正确；同按 = 停 |
| 4 | **按住 `L3` 再推右摇杆** | 底盘**完全不动**（右摇杆通道已删） |
| 5 | **按 `Select` 的那一刻** | 底盘**不抽搐**（`:59-61` 已删） |
| 6 | `Select` 进模式二 | `op=ARM`，四轮停，`ARM` 行带 `xy=` `th=` |
| 7 | 轻推左摇杆 | 末端立刻动、无跳变；松手立即停 |
| 8 | 双轴同时推到底 | 合速度仍 ≈50 mm/s（归一化生效），斜向不加速 |
| 9 | 按住 `L2` 同时推摇杆 | 夹爪开合与末端运动同时进行，互不打断 |
| 10 | 按住 `L1`/`R1` | `S4` 正反转，`S1/S2` 不动 |
| 11 | 推着摇杆按预设键 | 预设立即接管，本轮摇杆速度被丢弃，臂不再被拉回 |
| 12 | 按 8 个预设键 | 未被覆盖的槽位（P5~P8 未标定）无动作 |
| 13 | **按 `↑` / `↓`** | **触发预设 P1/P2，不再选关节**（串口无 `joint=`） |
| 14 | **单按 `R3`** | 无任何变化 |
| 15 | `L3`+`R3` 200 ms | 急停 latch；再按解除 |
| 16 | 拔接收器 | 电机停、臂保持、`online=0`；插回恢复，模式不变 |

---

## 7. 风险（v0.2 新增部分）

| 风险 | 后果 | 应对 |
|---|---|---|
| 关节 0/1 无手动入口 | IK 参数填错时，臂会「自信地」走向错误姿态，没有手动纠正手段 | 标定必须在 `ARM_BRINGUP` 开关下完成并目测；参数错时唯一手段是改 `arm_config` 重烧（30 s 一次） |
| TBD-4 结构限位无法实测 | 限位只能靠保守估，臂可能顶到挡块 | 先给保守值（比目测再收 10°），上电空载试跑观察堵转/发热 |
| 只剩一种摆位方式 | 摇杆推到底也只有 50 mm/s，大范围转移慢 | 接受（1 米行程 20 s）；要快就调 `jog_speed_mm_per_second` |
| 单轮点动消失 | 换电机/查轮序变麻烦 | 见 4.2，需要就恢复 `L3+方向` |

---

## 8. 决策

| # | 决策 | 值 | 备注 |
|---|---|---|---|
| **D-6** | 单电机测试：删 or 保留 | **删**（按你的要求） | 恢复成本：12 行 + `kCalibrationDutyPercent` |
| **D-7** | `R3` 单按 | **空闲** | 原标定子态随 jog 一起删 |
| **D-8** | 合速度归一化 | **做** | 斜推不加速，与 v0.2 的设计值一致 |
| **D-9** | TBD-4 结构限位怎么测 | **`ARM_BRINGUP` 编译期开关**，不进默认构建 | 不接任何手柄键 |
| **D-10** | 右摇杆字段 | **从 `Ps2Frame` 删掉** | 若以后要接（比如第二个操作手）再加回来 |

---

## 9. 与 `arm_pose_v0.3` 的接口（**已按 v0.3 实际符号定稿**）

```cpp
void arm_pose_stick_velocity(int forward_percent, int lift_percent,
                             float &vx_mm_per_s, float &vy_mm_per_s);  // 平方律 + 归一化
void arm_pose_set_cart_velocity(float vx_mm_per_s, float vy_mm_per_s); // 每帧，0 = 停
bool arm_pose_joint_jog(unsigned axis, int stick_percent);             // 只认 axis 2/3，0/1 返回 false
void arm_pose_request(unsigned pose_index);                            // 0..7，进门先清速度
ArmFeedback arm_pose_feedback();                                       // 定义在 arm_pose.hpp
```

三处与 v0.1 写法的差异，按 v0.3 为准：

| 项 | v0.1 的写法 | v0.3 实际 | 为什么改 |
|---|---|---|---|
| 函数前缀 | `arm_set_cart_velocity` / `arm_joint_jog` | `arm_pose_set_cart_velocity` / `arm_pose_joint_jog` | 模块名是 `arm_pose`，`arm_joint` 是内部层，不对外 |
| jog 单位 | `%/s` | **摇杆百分比 −100..100** | `mode_control` 不该知道 `kMotion.jog_percent_per_second[]`；换算在 arm_pose 里一次做完 |
| 平方律 | 写在 `mode_control` | **`arm_pose_stick_velocity()`** | 否则平方律两份实现（第 2.1 节） |

**不再需要**：`arm_pose_set_jog_mode(bool cartesian)`——没有关节模式了。
**必须支持**：同一帧内 `arm_pose_set_cart_velocity` 与 `arm_pose_joint_jog(2/3, …)` 共存（第 2.5 节）。
**必须保证**：`arm_pose_request()` 进门先清内部笛卡尔速度（v0.3 `arm_pose.cpp:166-169` 已实现），否则第 2.4 节的「按预设却先动一下」会在 arm_pose 内部复现。

**返回值**：`arm_pose_joint_jog` 返回 `bool`，`axis` 传 0/1 时返回 `false` —— 这条白名单是「关节 0/1 没有手动入口」的代码级保证，`mode_control` 不必自己判。
