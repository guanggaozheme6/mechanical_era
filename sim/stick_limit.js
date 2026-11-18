/**
 ******************************************************************************
 * @file           : stick_limit.js
 * @brief          : 摇杆软件限位：末端域（固件现状）与关节域贴边滑两套策略
 ******************************************************************************
 *
 * 与固件对应关系：
 *   stepEndDomain() ⇔ arm_pose.cpp 的 cartesian_step() + apply_xy()
 *                     FK → 加一步 → 矩形钳位 → solve_xy 圆环钳位 → limited 才不动
 *   stepTangent()   ⇔ 新增策略（尚未进固件）：雅可比 → 阻尼最小二乘 → 关节限位裁剪
 *
 * 两者的差别只有一句话：
 *   末端域把「走不通」解释成「沿半径缩到圆环上」（越界照发，末端会自己滑走），
 *   关节域把「走不通」解释成「受限的那个关节停住、其余关节继续走」——
 *   于是末端沿可达域边界切向滑动，方向仍然是摇杆推的那一侧。
 *
 * 约束（与 arm_core.js 同源）：
 *   - 纯函数，不碰 DOM、不读时钟；时间步长 dt 由调用方给；
 *   - 几何/限位全部从传入的 g 派生，不出现硬编码数值；
 *   - 「干涉」这种与几何无关的判据由调用方以 feasible(p1, p2) 回调注入。
 *
 * 单位：mm / 度 / 行程百分比（-100..100）。
 */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (root) {
    root.StickLimit = api;
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const RAD = Math.PI / 180;

  /** 一步的判定结果 */
  const FLAG = {
    IDLE: "idle", // 摇杆在中位，没有位移
    OK: "ok", // 整步走通，精确命中
    WALL: "wall", // 末端域：被工作区矩形夹到边上（照发）
    LIMITED: "limited", // 末端域：θ1 越限，这一步整帧不下发
    SLIDE: "slide", // 关节域：某轴顶到限位/干涉，沿边界切向滑
    STUCK: "stuck" // 关节域：所有候选都走不通（正对墙或奇异点）
  };

  /** 回退用的步长序列：1 → 1/2 → … → 1/128，取第一个可行的 */
  const SCALES = [1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.0078125];

  function clampf(v, lo, hi) {
    return v < lo ? lo : v > hi ? hi : v;
  }

  function clampPercent(p) {
    return clampf(p, -100, 100);
  }

  function finite(v) {
    return typeof v === "number" && isFinite(v);
  }

  /** 行程百分比 → 连杆角（度） */
  function percentToDeg(calib, percent) {
    return calib.zeroDeg + percent * calib.degPerPercent;
  }

  /**
   * @brief 两路关节的行程限位（百分比），由角度限位派生
   * @details 与固件 arm_config::kJointLimits 同一算法：限位只存角度，百分比在这里派生，
   *          避免仿真与固件各维护一套。
   * @param g 几何模型
   * @return [{min,max},{min,max}]，顺序 = 大臂、小臂
   */
  function jointLimits(g) {
    const one = function (calib) {
      const dpp = calib.degPerPercent === 0 ? 1e-9 : calib.degPerPercent;
      const a = (calib.minDeg - calib.zeroDeg) / dpp;
      const b = (calib.maxDeg - calib.zeroDeg) / dpp;
      return { min: Math.min(a, b), max: Math.max(a, b) };
    };
    return [one(g.shoulder), one(g.elbow)];
  }

  /**
   * @brief 2R 末端雅可比（末端 = 指尖，肘→指尖 = foreMm + toolMm）
   * @details
   *   x = L1·cosθ1 + L2Σ·cos(θ1+θ2)
   *   y = L1·sinθ1 + L2Σ·sin(θ1+θ2)
   *   对「行程百分比」求导，所以每个 θ 都乘 deg_per_percent·π/180（rad / %）。
   *   用解析式而不是差分：固件移植时不必为 2×2 雅可比多调两次 FK。
   * @param g 几何模型
   * @param p1 大臂行程百分比
   * @param p2 小臂行程百分比
   * @return {j11,j12,j21,j22}：∂(x,y)/∂(p1,p2)，单位 mm/%
   */
  function jacobian(g, p1, p2) {
    const t1 = percentToDeg(g.shoulder, p1) * RAD;
    const t2 = percentToDeg(g.elbow, p2) * RAD;
    const s = t1 + t2;
    const l1 = g.upperMm;
    const l2 = g.foreMm + g.toolMm;
    const k1 = g.shoulder.degPerPercent * RAD;
    const k2 = g.elbow.degPerPercent * RAD;
    return {
      j11: (-l1 * Math.sin(t1) - l2 * Math.sin(s)) * k1,
      j12: (-l2 * Math.sin(s)) * k2,
      j21: (l1 * Math.cos(t1) + l2 * Math.cos(s)) * k1,
      j22: (l2 * Math.cos(s)) * k2
    };
  }

  /**
   * @brief 2×2 雅可比的奇异性指标
   * @details 用奇异值比值 σmax/σmin（对 2×2 有闭式解）。伸直（θ2=0）时两列平行，
   *          条件数 → ∞，这就是「奇异区抖动」的数学来源。
   * @param J 雅可比
   * @return 条件数；完全奇异时返回 Infinity
   */
  function condition(J) {
    const a = J.j11;
    const b = J.j12;
    const c = J.j21;
    const d = J.j22;
    const sum = a * a + b * b + c * c + d * d;
    const cross = a * c + b * d;
    const dif = a * a + b * b - c * c - d * d;
    const disc = Math.sqrt(Math.max(0, dif * dif + 4 * cross * cross));
    const s1 = Math.sqrt(Math.max(0, (sum + disc) / 2));
    const s2 = Math.sqrt(Math.max(0, (sum - disc) / 2));
    if (!(s2 > 1e-12)) {
      return Infinity;
    }
    return s1 / s2;
  }

  /**
   * @brief 阻尼最小二乘：末端位移 → 关节增量
   * @details dq = Jᵀ·(J·Jᵀ + λ²I)⁻¹·w。
   *          加 λ 的原因：θ2→0（臂伸直）或末端顶到外弧时 J 退化，无阻尼解会给出巨大的
   *          dq，被限位裁剪后正负来回 → 肉眼可见的抖动。λ 把这一步变软，代价是末端
   *          略慢于摇杆，换来的是不抖。
   * @param J 雅可比（mm/%）
   * @param wx 期望末端位移 x（mm）
   * @param wy 期望末端位移 y（mm）
   * @param damping 阻尼 λ，单位 mm/%
   * @return {dp1,dp2}：关节增量（%）
   */
  function dls(J, wx, wy, damping) {
    const lam2 = damping * damping;
    const a11 = J.j11 * J.j11 + J.j12 * J.j12 + lam2;
    const a12 = J.j11 * J.j21 + J.j12 * J.j22;
    const a22 = J.j21 * J.j21 + J.j22 * J.j22 + lam2;
    const det = a11 * a22 - a12 * a12;
    if (!(Math.abs(det) > 1e-18)) {
      return { dp1: 0, dp2: 0 };
    }
    const i11 = a22 / det;
    const i12 = -a12 / det;
    const i22 = a11 / det;
    const u1 = i11 * wx + i12 * wy;
    const u2 = i12 * wx + i22 * wy;
    return {
      dp1: J.j11 * u1 + J.j21 * u2,
      dp2: J.j12 * u1 + J.j22 * u2
    };
  }

  /**
   * @brief 带饱和重解的关节步：顶到限位就把该轴固定，用剩余自由度重算
   * @details 为什么不能只做 clamp：双自由度 DLS 解里两个关节是「合伙」凑出这段位移的。
   *          把其中一个直接夹到边界、另一个照原值走，等于让剩下的那个承担整段位移，
   *          末端速度会突然放大（实测顶到 θ2=−45° 后每帧 4.2 mm，是期望 1 mm 的 4 倍）。
   *          正确做法是固定住饱和轴，把「它已经贡献掉的那部分位移」从期望里减掉，
   *          再拿剩下的关节去解剩余位移 —— 也就是切向分量。
   * @param J 雅可比（mm/%）
   * @param wx 期望末端位移 x（mm）
   * @param wy 期望末端位移 y（mm）
   * @param p1 当前大臂行程百分比
   * @param p2 当前小臂行程百分比
   * @param lim jointLimits() 的结果
   * @param damping 阻尼 λ（mm/%）
   * @return {dp1,dp2,fixed[2]}：关节增量与哪两轴被饱和固定
   */
  function solveJointStep(J, wx, wy, p1, p2, lim, damping) {
    const lam2 = damping * damping;
    const dq = dls(J, wx, wy, damping);
    const raw1 = p1 + dq.dp1;
    const raw2 = p2 + dq.dp2;
    const over1 = raw1 < lim[0].min - 1e-9 || raw1 > lim[0].max + 1e-9;
    const over2 = raw2 < lim[1].min - 1e-9 || raw2 > lim[1].max + 1e-9;
    if (!over1 && !over2) {
      return { dp1: dq.dp1, dp2: dq.dp2, fixed: [false, false] };
    }

    const d2 = clampf(raw2, lim[1].min, lim[1].max) - p2;
    const d1 = clampf(raw1, lim[0].min, lim[0].max) - p1;

    if (over2 && !over1) {
      // 轴 2 固定：扣掉它的贡献，剩下的位移交给轴 1（单自由度 DLS）
      const rx = wx - J.j12 * d2;
      const ry = wy - J.j22 * d2;
      const den = J.j11 * J.j11 + J.j21 * J.j21 + lam2;
      const k = den > 1e-18 ? (J.j11 * rx + J.j21 * ry) / den : 0;
      const n1 = clampf(p1 + k, lim[0].min, lim[0].max);
      return { dp1: n1 - p1, dp2: d2, fixed: [false, true] };
    }
    if (over1 && !over2) {
      // 轴 1 固定：对称处理
      const rx = wx - J.j11 * d1;
      const ry = wy - J.j21 * d1;
      const den = J.j12 * J.j12 + J.j22 * J.j22 + lam2;
      const k = den > 1e-18 ? (J.j12 * rx + J.j22 * ry) / den : 0;
      const n2 = clampf(p2 + k, lim[1].min, lim[1].max);
      return { dp1: d1, dp2: n2 - p2, fixed: [true, false] };
    }
    // 两轴都饱和：这一步只剩被裁掉的那点位移，交给上层判 STUCK / 滑停
    return { dp1: d1, dp2: d2, fixed: [true, true] };
  }

  /**
   * @brief 末端（指尖）坐标
   * @param g 几何模型
   * @param p1 大臂行程百分比
   * @param p2 小臂行程百分比
   * @return {x,y}
   */
  function tip(g, p1, p2) {
    const t1 = percentToDeg(g.shoulder, p1) * RAD;
    const sum = t1 + percentToDeg(g.elbow, p2) * RAD;
    const l1 = g.upperMm;
    const l2 = g.foreMm + g.toolMm;
    return {
      x: l1 * Math.cos(t1) + l2 * Math.cos(sum),
      y: l1 * Math.sin(t1) + l2 * Math.sin(sum)
    };
  }

  /**
   * @brief 末端越出工作区矩形多少（mm，0 = 在框内）
   * @param g 几何模型
   * @param p1 大臂行程百分比
   * @param p2 小臂行程百分比
   * @param ws 工作区矩形，缺省返回 0
   * @return 越界量
   */
  function overflow(g, p1, p2, ws) {
    if (!ws) {
      return 0;
    }
    const t = tip(g, p1, p2);
    return Math.max(0, ws.xMin - t.x, t.x - ws.xMax, ws.yMin - t.y, t.y - ws.yMax);
  }

  /**
   * @brief 候选姿态是否可用（工作区矩形 + 调用方注入的干涉判据）
   * @details 矩形当「墙」而不是「投影面」：越界就判不可行，让 stepTangent 去试别的方向，
   *          而不是把末端拽到边上（拽到边上正是固件末端域的毛病）。
   *          唯一的例外是「不越界得更多」：顶在边界上时，任何关节运动都会让末端在法向上
   *          多出零点零几毫米，若严格按「必须进框」判，沿边切向滑会全被判死
   *          （实测 60 帧只走 15 mm）。所以判据是「越界量 ≤ 起始越界量 + tol」。
   * @param g 几何模型
   * @param p1 大臂行程百分比
   * @param p2 小臂行程百分比
   * @param opt {ws, feasible, tol, baseOver}
   * @return true 表示可下发
   */
  function feasibleAt(g, p1, p2, opt) {
    const o = opt || {};
    if (o.feasible && !o.feasible(p1, p2)) {
      return false;
    }
    if (!o.ws) {
      return true;
    }
    // 0.02 mm：flattenToRect 是线性化投影，实际会残留零点零几毫米的法向位移；
    // 容差比它大一点才不会被自己的残差判死（末端一个舵机步进是 2.6 mm，取它的 1/100）。
    const tol = finite(o.tol) ? o.tol : 0.02;
    const base = finite(o.baseOver) ? o.baseOver : 0;
    return overflow(g, p1, p2, o.ws) <= Math.max(base, tol) + 1e-9;
  }

  /**
   * @brief 末端域一步（= 固件 cartesian_step + apply_xy 的现行行为）
   * @details 顺序与固件一致：FK(current) → 加一步 → 矩形钳位 → solve_xy（圆环钳位 + 角度限位）
   *          → θ1 越限才整帧不下发，其余越界一律「钳到边界照发」。
   *          「照发」在 B 方案下的副作用：钳位是把目标沿方位角缩到弧带上，
   *          所以摇杆推着走、末端却在横向滑——这一版就是要让它在仿真里显形。
   * @param core ArmCore（复用 solveForward / solveXY，仿真与固件共用一套解算）
   * @param g 几何模型
   * @param st {p1,p2} 当前关节行程百分比
   * @param vx 末端 x 速度（mm/s）
   * @param vy 末端 y 速度（mm/s）
   * @param dt 步长（s）
   * @param opt {ws, sag}
   * @return 与 stepTangent 同构的结果对象
   */
  function stepEndDomain(core, g, st, vx, vy, dt, opt) {
    const o = opt || {};
    const p1 = clampPercent(st.p1);
    const p2 = clampPercent(st.p2);
    const from = core.solveForward(g, p1, p2);
    const want = { x: vx * dt, y: vy * dt };
    if (!finite(want.x) || !finite(want.y) || Math.hypot(want.x, want.y) < 1e-12) {
      return result(p1, p2, from, from, want, FLAG.IDLE, false, [false, false], NaN);
    }

    let tx = from.x + want.x;
    let ty = from.y + want.y + (o.sag || 0) * tx / 100;
    let wall = false;
    const ws = o.ws;
    if (ws) {
      if (tx < ws.xMin) {
        tx = ws.xMin;
        wall = true;
      } else if (tx > ws.xMax) {
        tx = ws.xMax;
        wall = true;
      }
      if (ty < ws.yMin) {
        ty = ws.yMin;
        wall = true;
      } else if (ty > ws.yMax) {
        ty = ws.yMax;
        wall = true;
      }
    }

    const sol = core.solveXY(g, tx, ty);
    if (sol.limited) {
      return result(p1, p2, from, from, want, FLAG.LIMITED, false, [false, false], NaN);
    }
    const n1 = clampPercent(sol.shoulderPercent);
    const n2 = clampPercent(sol.elbowPercent);
    const to = core.solveForward(g, n1, n2);
    const flag = wall || !sol.reachable ? FLAG.WALL : FLAG.OK;
    return result(n1, n2, from, to, want, flag, false, [false, false], NaN);
  }

  /**
   * @brief 把一步里的「法向分量」剔掉：贴在工作区矩形边界上时的切向滑
   * @details 关节限位裁剪只能处理「关节顶住」，处理不了「末端顶住矩形边」——
   *          因为末端越界是两轴联动的结果，哪个关节都不该被裁。
   *          做法是标准的约束零空间投影：n 是越界那一面的法向，
   *          a = nᵀJ 是法向位移对两个关节的敏感度，
   *          dq ← dq − aᵀ(a·dq)/(|a|²+λ²)，于是 n·J·dq ≈ 0，末端只能沿边滑。
   *          正对墙推（切向分量为 0）时 dq 被投影成 0，上层会判 STUCK —— 对的。
   * @param J 雅可比（mm/%）
   * @param step solveJointStep() 的结果
   * @param from 步进前末端坐标
   * @param ws 工作区矩形（缺省不投影）
   * @param damping 阻尼 λ（mm/%）
   * @return 投影后的 {dp1,dp2,fixed}
   */
  function flattenToRect(J, step, from, ws, damping) {
    if (!ws) {
      return step;
    }
    const predX = from.x + J.j11 * step.dp1 + J.j12 * step.dp2;
    const predY = from.y + J.j21 * step.dp1 + J.j22 * step.dp2;
    const over0 = Math.max(0, ws.xMin - from.x, from.x - ws.xMax, ws.yMin - from.y, from.y - ws.yMax);
    const over1 = Math.max(0, ws.xMin - predX, predX - ws.xMax, ws.yMin - predY, predY - ws.yMax);
    if (over1 <= over0 + 1e-9) {
      return step; // 没有越界得更多，不管
    }
    let nx = 0;
    let ny = 0;
    if (predX > ws.xMax) { nx = 1; } else if (predX < ws.xMin) { nx = -1; }
    if (predY > ws.yMax) { ny = 1; } else if (predY < ws.yMin) { ny = -1; }
    if (nx === 0 && ny === 0) {
      return step;
    }
    const a1 = nx * J.j11 + ny * J.j21;
    const a2 = nx * J.j12 + ny * J.j22;
    const den = a1 * a1 + a2 * a2 + damping * damping;
    if (!(den > 1e-18)) {
      return step;
    }
    const k = (a1 * step.dp1 + a2 * step.dp2) / den;
    return { dp1: step.dp1 - a1 * k, dp2: step.dp2 - a2 * k, fixed: step.fixed };
  }

  /**
   * @brief 关节域一步：雅可比 → 阻尼最小二乘 → 关节限位裁剪 → 沿边界切向滑
   * @details 分两段，顺序不能反：
   *
   *          **① 位置法（主路）**：目标 = 当前末端 + v·dt，直接 `solve_xy` 解。
   *          解不出来（圆环钳位 / θ1 越限 / 干涉 / 出矩形）就把目标点沿摇杆方向对半缩，
   *          取第一个真正可达且合法的点。这一段保证「手到哪，末端就沿那条直线走」。
   *
   *          **② 关节兜底（只在①全失败时）**：摇杆正对着一堵墙推，位置法一步都走不了，
   *          这时才把速度交给雅可比，沿「关节能走的方向」滑出去 ——
   *          候选按优先级：DLS 原方向 → x 单轴 → y 单轴 → 只动 θ1 → 只动 θ2，
   *          每个候选再从整步对半回退。顶到关节限位时用饱和重解（solveJointStep），
   *          越出矩形时剔掉法向分量（flattenToRect），剩下能走的那个分量就是切向滑。
   *
   *          为什么不能只用②：这个几何（L1=76、小臂 168.43）像一根带短把的长杆，
   *          末端竖直向下走时，DLS 的最小范数解会把大臂越转越平（θ1→2°），
   *          大臂先压到车体上表面、整帧 STUCK —— 实测「向下推」只能到 y=−62.9，
   *          而地面在 −77。位置法沿直线走时 θ1 从 30° 落到 8°，全程合法，
   *          能一路伸到地面（y=−76.9）。两条路必须都在，缺一个就有死角。
   * @param core ArmCore（位置法要用 solveXY；关节兜底不用）
   * @param g 几何模型
   * @param st {p1,p2} 当前关节行程百分比
   * @param vx 末端 x 速度（mm/s）
   * @param vy 末端 y 速度（mm/s）
   * @param dt 步长（s）
   * @param opt {damping, ws, feasible}
   * @return 结果对象
   */
  function stepTangent(core, g, st, vx, vy, dt, opt) {
    const o = opt || {};
    const lim = jointLimits(g);
    const p1 = clampPercent(st.p1);
    const p2 = clampPercent(st.p2);
    const from = tip(g, p1, p2);
    const want = { x: vx * dt, y: vy * dt };
    const mag = Math.hypot(want.x, want.y);
    if (!finite(mag) || mag < 1e-12) {
      return result(p1, p2, from, from, want, FLAG.IDLE, false, [false, false], NaN);
    }

    // baseOver：起点本来就越界多少。判据是「不许越界得更多」，所以预设被钳出框时
    // 也不会整帧卡死（它会先往回走，越界量单调下降）。
    const check = {
      ws: o.ws,
      feasible: o.feasible,
      tol: o.tol,
      baseOver: overflow(g, p1, p2, o.ws)
    };

    // ---- ① 位置法：沿摇杆方向直线走，解不出就缩步 ----
    for (let k = 0; k < SCALES.length; k++) {
      const s = SCALES[k];
      const tx = from.x + want.x * s;
      const ty = from.y + want.y * s;
      const sol = core.solveXY(g, tx, ty);
      // 圆环钳位（reachable=false）和 θ1 越限（limited=true）都不接受：
      // 前者就是「钳位照发」，后者是固件的整帧拒发，两个都不要。
      if (!sol.reachable || sol.limited) {
        continue;
      }
      const n1 = clampPercent(sol.shoulderPercent);
      const n2 = clampPercent(sol.elbowPercent);
      if (!feasibleAt(g, n1, n2, check)) {
        continue;
      }
      const to = tip(g, n1, n2);
      return result(n1, n2, from, to, want, s === 1 ? FLAG.OK : FLAG.SLIDE, s < 1, [false, false], NaN);
    }

    // ---- ② 关节兜底：位置法一步都走不了，说明正对着一堵墙 ----
    const J = jacobian(g, p1, p2);
    const cond = condition(J);
    const damping = finite(o.damping) && o.damping > 0 ? o.damping : 1.0;

    // 候选关节步，按优先级排：
    //   ① 摇杆原方向 ② x 单轴 ③ y 单轴（轴对齐的矩形边）
    //   ④⑤ 单关节兜底（只动 θ1 / 只动 θ2）
    //
    // ④⑤ 不是可选项：干涉墙的法向不在 J 的像空间里 —— 末端要往车前方地面伸时，
    // 「大臂几乎放平」这条路会压到车体上表面，而唯一走得通的「只折叠小臂」方向
    // 在 DLS 的最小范数解里根本不会出现（DLS 总是两个关节一起动）。没有这两条，
    // 摇杆推到那个位置就整帧 STUCK，看着像卡死。
    const cand = [];
    [
      { x: want.x, y: want.y },
      { x: want.x, y: 0 },
      { x: 0, y: want.y }
    ].forEach(function (w) {
      if (Math.hypot(w.x, w.y) < 1e-12) {
        return;
      }
      cand.push(flattenToRect(J, solveJointStep(J, w.x, w.y, p1, p2, lim, damping),
        from, o.ws, damping));
    });
    // 单关节：投影到 w 方向（k = Jⱼ·w / |Jⱼ|²，保证位移与摇杆方向同向且不超过 |w|）
    [0, 1].forEach(function (j) {
      const cx = j === 0 ? J.j11 : J.j12;
      const cy = j === 0 ? J.j21 : J.j22;
      const n2 = cx * cx + cy * cy;
      if (!(n2 > 1e-12)) {
        return;
      }
      const k = (cx * want.x + cy * want.y) / n2;
      if (Math.abs(k) < 1e-12) {
        return;
      }
      const raw = j === 0 ? { dp1: k, dp2: 0, fixed: [false, false] }
                          : { dp1: 0, dp2: k, fixed: [false, false] };
      cand.push(flattenToRect(J, raw, from, o.ws, damping));
    });

    for (let d = 0; d < cand.length; d++) {
      const step = cand[d];
      if (Math.abs(step.dp1) < 1e-12 && Math.abs(step.dp2) < 1e-12) {
        continue;
      }
      if (step.fixed[0] && step.fixed[1]) {
        continue; // 两轴都在边界上，这个方向上彻底走不动
      }
      for (let k = 0; k < SCALES.length; k++) {
        const s = SCALES[k];
        const raw1 = p1 + step.dp1 * s;
        const raw2 = p2 + step.dp2 * s;
        const n1 = clampf(raw1, lim[0].min, lim[0].max);
        const n2 = clampf(raw2, lim[1].min, lim[1].max);
        const cut = [
          step.fixed[0] || Math.abs(n1 - raw1) > 1e-9,
          step.fixed[1] || Math.abs(n2 - raw2) > 1e-9
        ];
        if (Math.abs(n1 - p1) < 1e-9 && Math.abs(n2 - p2) < 1e-9) {
          continue; // 这一步什么也没动，再小的步长也没用
        }
        if (!feasibleAt(g, n1, n2, check)) {
          continue;
        }
        const to = tip(g, n1, n2);
        const slid = cut[0] || cut[1] || d > 0 || s < 1;
        return result(n1, n2, from, to, want, slid ? FLAG.SLIDE : FLAG.OK, slid, cut, cond);
      }
    }

    return result(p1, p2, from, from, want, FLAG.STUCK, false, [false, false], cond);
  }

  /**
   * @brief 组装结果对象（两套策略共用同一个形状，方便上层无差别读数/绘图）
   * @param p1 新大臂行程百分比
   * @param p2 新小臂行程百分比
   * @param from 步进前末端
   * @param to 步进后末端
   * @param want 摇杆期望的位移（mm）
   * @param flag FLAG 之一
   * @param slide 是否发生了切向滑
   * @param cut 两轴是否被限位裁剪
   * @param cond 雅可比条件数（末端域不适用，给 NaN）
   */
  function result(p1, p2, from, to, want, flag, slide, cut, cond) {
    return {
      p1: p1,
      p2: p2,
      from: { x: from.x, y: from.y },
      to: { x: to.x, y: to.y },
      want: { x: want.x, y: want.y },
      got: { x: to.x - from.x, y: to.y - from.y },
      flag: flag,
      slide: slide,
      cut: [!!cut[0], !!cut[1]],
      cond: cond
    };
  }

  return {
    FLAG: FLAG,
    SCALES: SCALES,
    clampf: clampf,
    clampPercent: clampPercent,
    percentToDeg: percentToDeg,
    jointLimits: jointLimits,
    jacobian: jacobian,
    condition: condition,
    dls: dls,
    tip: tip,
    feasibleAt: feasibleAt,
    overflow: overflow,
    flattenToRect: flattenToRect,
    solveJointStep: solveJointStep,
    stepEndDomain: stepEndDomain,
    stepTangent: stepTangent
  };
});
