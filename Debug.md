# Debug —— 待办、坑与验收记录

> 仓库根目录的**唯一**待办清单（原来 `plan/debug*.md` 的内容已并入这里和代码注释）。
> 分四级：**P0 阻塞**（不动实物或不定方案就干不下去）、**P1 标定参数**（按依赖顺序）、
> **P2 代码待办**（每条带 `file:line`）、**P3 文档**。末尾单列「风险与已知限制」——
> 风险不是待办，混进清单会被当成 bug 逐条去查。
>
> 每条写三样：**现状 / 建议 / 怎么验**。

版本：**v0.4.1**（单模式 + 只留中立位 + 右摇杆控臂；修回 v0.4 提交里被覆盖掉的
结构限位派生与中立位精简，见末尾「本次修复」）。上一版 v0.3 的操作方式已废，见 README §9。

---

## P0 阻塞项（上电前必须销）

### D-1 摇杆与按键的**方向/极性**全是约定，没在实物上验过

| | |
|---|---|
| 现状 | `mode_control.cpp` 的约定：左摇杆 Y 推上=前进（`arm_axis_percent(..., true)` 取反）、X 推右=右移；右摇杆 X 推右=前伸、Y 推上=抬升（同样 invert）；`L1`=左转 / `R1`=右转；`←`=腕正（左转）/ `→`=腕负。**一条都没实测**，PS2 摇杆个体差异还可能反相。 |
| 建议 | 先只看不动：烧一版正常固件，上电看串口第一行 `LX/LY/RX/RY`，把四个摇杆推到底各记一次（应为 0 和 255）；再逐键按下，看 `keys=` 的位。方向错了只改 `mode_control.cpp` 里那四个 invert 布尔/两个符号常量，不动其它文件。 |
| 怎么验 | `LX/LY/RX/RY` 推上/推下变成 0/255 且中位 128±8；按 `L1` 底盘向左自转、`R1` 向右；按 `←` 时 `ARM` 行的 `S4` 增大（正 = 左转，与模式化之前相反，这是 v0.4 故意的）。 |

### D-2 结构限位（θ1 7.10° / θ2 −42~132°）是**算出来的**，不是量出来的

| | |
|---|---|
| 现状 | `kGeom` 的限位来自 `kShoulderContactDeg = 4.10°`（大臂贴底盘，`asin(1/14)`）+ `kStopBackoffDeg = 3°`，小臂是舵机 180° 行程两端各退 3°。全部是几何推导，没有一件实物顶过。 |
| 建议 | 用 `-DARM_BRINGUP` 标定版逐 10% 加到顶住，回退 3°，记真实值；同时确认 **θ1 的 0°/180° 确实是「大臂躺在底盘上」**（理论值 4.10°，实物只会更靠内，因为有臂厚）。 |
| 怎么验 | 标定版串口打出的 `S1/S2` 与 `kGeom` 派生值之差 ≤ 1 格（0.9°）；顶住时不堵转、不发热。 |

### D-3 夹爪与腕的正负方向未实测

| | |
|---|---|
| 现状 | README/`mode_control.cpp` 约定：夹爪**正 = 张开**（`L2` = +100）、腕**正 = 左转**（`←` = +100）。没有任何实物依据。 |
| 建议 | 上电第一件事：轻按 `L2`/`R2` 看夹爪是松还是紧；按 `←`/`→` 看腕往哪边转。反了就改 `mode_control.cpp` 的 `gripper += 100` 符号或 `kWristLeftPercent`（各一行）。 |
| 怎么验 | 串口 `S3`/`S4` 增大时实物动作与约定一致。 |

---

## P1 标定参数（严格按这个顺序，前一步不确定别做下一步）

1. **量连杆**：肩→肘 = `upper_mm`，肘→腕点 = `fore_mm`（卡尺取轴心距）。**末端是腕点，没有工具延伸**。
2. **定零点**：`zero_deg` 必须等于行程中点（`static_assert(zero_at_mid())` 会卡住）。推 `percent = 0` 量实际角。
3. **定比例**：推到 ±50 量角度差 / 50，带符号 → `deg_per_percent`。
4. **定限位**：见 D-2。
5. **上电点与中立位一起定**：`arm_pose.cpp` 的 `kZeroPercent[4]` 与 `arm_config.hpp` 的 `kNeutralPose` 必须是同一姿态
   （`kNeutralPose == FK(kZeroPercent)`）。改一个忘另一个 → 按 Start 第一帧就跳，串口 `neutral_ok` 会变 0。
6. **夹爪/腕方向**：见 D-3。
7. **`sag_mm_per_100mm`**：水平位实测下垂 `d`，`coef = d / (x/100)`。默认 0（关闭）。
8. **`cart_speed_mm_per_second`**：右摇杆满偏手感；改大之前先确认 `slew_ok()` 编得过。

验收：把臂推到 (105.2, 38.3)，串口 `xy=` 与实物目标点目测偏差 < 5 mm；上电不碰手柄 `neutral_ok=1`。

---

## P2 代码待办

### D-4 固件没有干涉判据：末端可能撞底盘 / 钻地面

- **现状**：`Algorithm/src/arm_limit.cpp` 的 `feasible_at()` 只判「工作区矩形 + 关节限位 + 容差」。
  仿真里 `sim/collide.js` 把臂当线段、底盘当实体采样，固件没有这套。位置法只保证"解得出、在框内"，
  **不保证大臂/小臂不压到车体上表面**。
- **建议**：把 `sim/collide.js` 的判据搬进 `arm_limit.cpp`（大臂 6 点 / 小臂 8 点采样，1 mm 余隙，
  底座 14 mm 豁免），作为 `feasible_at()` 的第二个条件。也可以在 `arm_pose.cpp` 拿到 `Step` 之后 gate 一次 —— 
  但位置法的缩步在算法层里，放在那里会退化成"整帧拒绝"。
- **怎么验**：`node sim/verify/struct_limit.js` 的 A5 目前只覆盖中立位一条；8 条规划坐标的
  撞底盘判据要单独补（`sim/index.html` 的 `poseHits` 已有，缺的是主机侧 C++ 断言）。

### D-5 仿真页的预设表还是 8 条、还带 gripper/wrist 两列 —— **已修（v0.4.1）**

- **现状**：`sim/index.html` 的「预设姿态」仍是 8 条 × 4 列，但当**规划工具**保留（铺弧带、看余量）。
  导出已改成：`constexpr Point kNeutralPose = {x, y};` 一条 + 其余 7 条以注释附在后面；
  `kMotion` 也补齐了 `stick_damping` / `stick_box_tol` 两个字段（原来只打 5+1 个，粘进固件过不了 `static_assert`）。
- **怎么验**：把导出片段按 arm_config.hpp 的写法放进独立命名空间 + `static_assert` 比对，
  `g++ -std=c++14 -Werror` 直接编过（已跑过一次，见「本次修复」）。

### D-6 每帧开销没在 M3 上实测

- **现状**：位置法每帧 1~7 次 `solve_xy`（多数帧 1 次，边界才缩步），关节兜底只在卡死时走。
  主机侧 Debug 下 `arm_pose_update` 单帧微秒级，但 72 MHz 无 FPU 的 M3 没量过。
- **建议**：用 DWT CYCCNT 在 `mode_control_update` 前后打点，串口加一列最坏耗时；只在调试版开。
- **怎么验**：最坏帧（顶边界 + 关节兜底）< 1 ms 的一半，否则摇杆会掉帧。

### D-7 `arm_pose_request_xy()` 没有按键调用者 —— **已加用例（v0.4.1）**

- **现状**：`Device/inc/arm_pose.hpp` 声明、`arm_pose.cpp` 实现，v0.4 删掉预设后只有上位机 /
  动作序列会用（`arm_pose_at_target()` 是它的配套「到了没」查询）。`-Werror` 不会报未使用的外部函数。
- **怎么验**：`sim/verify/keymap_test.cpp` 的 K9：`arm_pose_request_xy(114, 0)` 后 `tgt ≈ (114, 0)` 且返回命中。

### D-8 急停/断线时夹爪与腕"保持"这件事没验过实物

- **现状**：`arm_pose_hold()` 只清 jog、保持 target（`arm_joint.cpp` 不变），所以急停时夹爪不会松开 ——
  这是设计意图，但夹着东西按急停的实测没做过。
- **建议**：夹 10 g 方块按 `L3+R3`，确认不松手、不回中；再拔掉手柄线确认同样行为。
- **怎么验**：`ARM` 行 `S1..S4` 在 STOP=1 期间不变。

### D-9 串口 `poses=` 字段名名不副实 —— **已修（v0.4.1）**

- **现状**：`Task/src/diagnostics.cpp` 现在打 `neutral_ok=%u`（1 = 中立位可达），`ArmFeedback.bad_poses`
  位图也换成了 `bool neutral_ok`。README §6 的样例行已同步。
- **怎么验**：上电 `neutral_ok=1`；把 `kNeutralPose` 故意改成不可达点，重烧应看到 `neutral_ok=0`
  （`static_assert(neutral_in_area())` 会先拦住「出框」那种改法，要验就改成框内但 θ1 越限的点）。

---

## P3 文档

- **D-10** `sim/README.md` §7「已知限制」里还写着"128.35 读法的两种"——固件末端是腕点（40.08），
  仿真页才用 168.43 的指尖口径。两者别混，README 已写明，sim 页面顶部也该加一句。
- **D-11** `plan/` 目录已清空（旧设计稿删除），引用它的注释要扫一遍。
  `grep -rn "plan/" Include Algorithm Device Task README.md sim`。

---

## 风险与已知限制（不是待办，但必须知道）

| # | 风险 | 后果 | 现状 |
|---|---|---|---|
| R-1 | **单模式下开车同时摆臂** | 转弯离心力 + 臂伸出 → 翻车/结构受力 | v0.4 的取舍：按键不再互斥，操作者自己负责。软上可以给"底盘有速度时限制臂速度"，但没做 |
| R-2 | **开环舵机无位置反馈** | `S1..S4`、`xy=` 全是命令值；实际堵转/掉步无从得知 | 只能靠目测 + 堵转电流（没有采样） |
| R-3 | **中立位与上电点强耦合** | 改 `kZeroPercent` 不同步 `kNeutralPose` → 按 Start 跳变 | 上电自检：`neutral_ok=0` 时按 Start 直接不动，算兜底 |
| R-4 | **腕 `S4` 若是 pitch 而非 roll** | 2R 平面模型失效，末端位置算错 | 未确认（D-5 同源） |
| R-5 | **未建模干涉** | 大臂压车体、末端钻地面 | D-4，仅靠 `kWorkspace` + 限位兜底 |
| R-6 | 框内死区 10% | `kWorkspace` 框里有约 500/4982 个点整帧拒发（θ1 越限） | `node sim/verify/struct_limit.js` 实测，属几何固有 |
| R-7 | 扳机/方向键是开关量 | 夹爪与腕只有恒速，行程靠按住时长，慢慢对位做不到 | 报文限制，无解 |

---

## 验收记录（v0.4.1，本机实测，可复跑）

| 项 | 命令 | 结果 |
|---|---|---|
| 固件编译 | `cmake --build build/Debug` / `build/Release` | 0 error / 0 warning；Debug FLASH 46356 B(17.7%)、Release 32648 B(12.5%)，RAM 3104 B(6.3%) |
| 按键映射 | `./sim/verify/keymap_test.exe` | **9/9 PASS**（右摇杆字节进帧、左摇杆只动底盘、右摇杆只动臂、L1R1 自转、扳机只动 S3、←→ 只动 S4、Start 回中立位不动 S3/S4、旧预设键失效、`request_xy` 坐标入口） |
| 摇杆限位（贴边滑） | `node sim/verify/stick_port_test.js` | **6/6 PASS**；与 `sim/stick_limit.js` 逐帧对拍 200 步，最大关节偏差 6.7e−4 %，标记 0 处不一致 |
| 仿真侧限位 | `node sim/verify/stick_test.js` | **7/7 PASS** |
| IK 对拍 | `node sim/verify/compare.js` | **PASS**（18881 点；奇异带 0.36 mm / 0.12°） |
| 结构限位派生 | `node sim/verify/struct_limit.js` | **5/5 PASS**（内半径右 109.13 / 左 57.50，弧带 6.95 mm，θ1 余量 3.02°） |

编译三条主机侧自检的一行命令见 `README.md` §8「主机侧自检」。

---

## 本次修复（v0.4.1，从 v0.4 的覆盖事故恢复）

`b8ff037`（v0.4）声称做了「kPoses[8] → kPoses[1]、删 Pose 枚举」，但实际提交里
`Include/arm_config.hpp` 被换成了 81ebb15 之前的旧副本：**结构限位派生整段丢失**，
`kGeom` 退回 0°/180° 与 −45°/+135°，`kPoses[8]` 反而原样留着。README、Debug.md、
`sim/arm_core.js`、`sim/README.md` 都已经按新限位写好，于是：

| 症状 | 证据 |
|---|---|
| 固件限位与文档/仿真不一致 | `struct_limit.js` A1、A3 FAIL（θ1[0,180] vs sim 7.1/172.9） |
| v0.4 的预设精简没落地 | `kPoses[8]` + `Pose` 枚举 + 8 条上电自检全在，而 `mode_control` 已无按键调它 |
| 串口字段名对不上 | 仍打 `poses=0x%02X` |

修回来的内容：

1. `kShoulderContactDeg` / `kStopBackoffDeg` 与派生限位（7.10 / 172.90、−42 / +132）、
   `zero_at_mid()` / `span_within_travel()` 及对应 4 条 `static_assert`；
2. 预设表按 v0.4 的原意精简成 `constexpr Point kNeutralPose{105.2, 38.3}`；
   `Preset` / `Pose` / `kPoses[]` / `kPoseCount` / `poses_in_travel()` 全删；
3. 上电自检从 8 位掩码改成 `bool neutral_ok`，串口改打 `neutral_ok=0/1`；
4. 顺手清掉三个没人调用的接口（`arm_pose_value` / `arm_pose_gripper_servo` /
   `arm_pose_bad_pose_mask`）、`drive_control.hpp` 的旧按键注释、`ps2.cpp` 漏统计的右摇杆变化；
5. **板载 Key 也冻结机械臂**（原来只停底盘，臂会继续走向原目标）；
6. 仿真页导出改成 `kNeutralPose` 一条 + 规划坐标注释，`kMotion` 补齐两个字段。

防复发：`arm_config.hpp` 的 `static_assert` 一整套都在编译期卡着限位/中点/跨度/工作区；
`struct_limit.js` 的 A1 专门对拍「sim 默认几何 == 固件真值」，两边任何一边漂了都会红。
