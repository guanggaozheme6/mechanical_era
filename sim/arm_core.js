/**
 ******************************************************************************
 * @file           : arm_core.js
 * @brief          : 平面 2R 正逆解 + 关节换算，与固件 Algorithm/src/arm_kinematics.cpp 逐行对应
 ******************************************************************************
 *
 * 约束（与固件同源，改这里必须回看 .cpp）：
 *   - 纯函数，没有任何全局可变状态，不碰 DOM、不读时钟；
 *   - 几何模型由调用方以对象传入（g），参数与算法分离；
 *   - 选支、圆环钳位、角度限位、FK 回算的顺序和容差与 C++ 完全一致，
 *     verify/compare.js 会拿真实的 arm_kinematics.cpp 编出来的 probe.exe 逐点对拍。
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
    root.ArmCore = api;
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const RAD = Math.PI / 180;
  const DEG = 180 / Math.PI;

  /** 限位比较容差（度）：边界点（θ 正好等于 range_*）不误报 limited */
  const LIMIT_EPS_DEG = 1e-3;
  /** 认为目标落在肩轴上的半径阈值（mm），小于此值方向未定义 */
  const MIN_RADIUS_MM = 1e-3;

  function clampf(v, lo, hi) {
    return v < lo ? lo : v > hi ? hi : v;
  }

  function finite(v) {
    return typeof v === "number" && isFinite(v);
  }

  /**
   * @brief 默认几何（与 Include/arm_config.hpp 的 kGeom 一致）
   * @note foreMm 是腕点连杆，toolMm 是腕点到夹爪末端的延伸；真实小臂 = 两者之和。
   */
  function defaultGeom() {
    return {
      upperMm: 76.0, // L1 肩轴 → 肘轴
      foreMm: 40.08, // L2 肘轴 → 腕点（固件现值）
      toolMm: 128.35, // 腕点 → 夹爪末端
      elbowSign: 1, // +1 = 高肘（上吊）构型
      shoulder: { zeroDeg: 90.0, degPerPercent: 0.9, minDeg: 0.0, maxDeg: 180.0 },
      elbow: { zeroDeg: 45.0, degPerPercent: 0.9, minDeg: -45.0, maxDeg: 135.0 }
    };
  }

  /** 真实小臂长度（肘轴 → 夹爪末端） */
  function foreLength(g) {
    return g.foreMm + g.toolMm;
  }

  /** 最大伸展半径 L1 + L2 */
  function maxReach(g) {
    return g.upperMm + foreLength(g);
  }

  /** 行程百分比限幅到 -100..100 */
  function clampPercent(p) {
    return clampf(p, -100.0, 100.0);
  }

  /** 行程百分比 → 连杆角（非有限输入返回 zero_deg） */
  function percentToDeg(calib, percent) {
    if (!finite(percent) || !finite(calib.zeroDeg) || !finite(calib.degPerPercent)) {
      return calib.zeroDeg;
    }
    const deg = calib.zeroDeg + percent * calib.degPerPercent;
    return finite(deg) ? deg : calib.zeroDeg;
  }

  /** 连杆角 → 行程百分比（非有限或 deg_per_percent == 0 返回 0） */
  function degToPercent(calib, deg) {
    if (!finite(deg) || !finite(calib.zeroDeg) || !finite(calib.degPerPercent) ||
        calib.degPerPercent === 0) {
      return 0.0;
    }
    const percent = (deg - calib.zeroDeg) / calib.degPerPercent;
    return finite(percent) ? percent : 0.0;
  }

  /**
   * @brief 某一侧的最小可达半径（由小臂角度限位反推，可指定小臂长度）
   * @details 右半平面吃 range_min 的绝对值（θ2 取负支），左半平面吃 range_max。
   *          超过 180° 按镜像折算（转过死点等价于 360−θ2）。
   * @param g 几何模型
   * @param l2 该次解算用的肘→末端长度（夹取点在夹爪上时比 foreLength 短）
   * @param wristRight true = 目标在右半平面
   */
  function minReachableRadiusAt(g, l2, wristRight) {
    let t = wristRight ? Math.abs(g.elbow.minDeg) : g.elbow.maxDeg;
    if (t < 0) {
      t = -t;
    }
    if (t > 180) {
      t = 360 - t;
    }
    const l1 = g.upperMm;
    let r2 = l1 * l1 + l2 * l2 + 2.0 * l1 * l2 * Math.cos(t * RAD);
    if (!finite(r2) || r2 < 0) {
      r2 = 0;
    }
    const r = Math.sqrt(r2);
    const geo = Math.abs(l1 - l2);
    return r > geo ? r : geo;
  }

  function minReachableRadius(g, wristRight) {
    return minReachableRadiusAt(g, foreLength(g), wristRight);
  }

  /**
   * @brief 正解：两路行程百分比 → 末端坐标（含中间点，给绘图用）
   * @param g 几何模型
   * @param p1 大臂行程百分比
   * @param p2 小臂行程百分比
   * @param l2 肘→末端的长度；夹取点落在夹爪上时比 foreLength(g) 短
   * @return {x,y,tipX,tipY,wristX,wristY,elbowX,elbowY,t1,t2}
   */
  function solveForwardAt(g, p1, p2, l2) {
    const t1 = percentToDeg(g.shoulder, p1) * RAD;
    const t2 = percentToDeg(g.elbow, p2) * RAD;
    const sum = t1 + t2;
    const l1 = g.upperMm;
    const ex = l1 * Math.cos(t1);
    const ey = l1 * Math.sin(t1);
    const wx = ex + g.foreMm * Math.cos(sum);
    const wy = ey + g.foreMm * Math.sin(sum);
    const tx = ex + l2 * Math.cos(sum);
    const ty = ey + l2 * Math.sin(sum);
    return {
      x: tx,
      y: ty,
      tipX: tx,
      tipY: ty,
      wristX: wx,
      wristY: wy,
      elbowX: ex,
      elbowY: ey,
      t1: t1 * DEG,
      t2: t2 * DEG
    };
  }

  function solveForward(g, p1, p2) {
    return solveForwardAt(g, p1, p2, foreLength(g));
  }

  /**
   * @brief 平面 2R 逆解（含五道数值防护，与 C++ solve_xy 同序）
   * @details 非法输入兜底 → 圆环钳位 → 余弦定理求肘/肩角 → 角度限位钳位 → FK 自检。
   *          高肘选支：θ2 的符号与目标 x 的符号相反（等价于「取肘点更高的一支」）。
   * @param g 几何模型
   * @param xMm 目标 x（mm，原点在肩轴，+x 向前）
   * @param yMm 目标 y（mm，+y 向上）
   * @return 逆解结果，永不返回 NaN
   */
  function solveXY(g, xMm, yMm) {
    return solveXYAt(g, xMm, yMm, foreLength(g));
  }

  /**
   * @brief 平面 2R 逆解，可指定小臂长度（夹取点落在夹爪上时用）
   * @details 与 solveXY 同一套防护，只是肘→末端的长度由调用方给：
   *          夹爪占末端 46 mm，抓取点可以在 [L2Σ−46, L2Σ] 之间任意位置，扫一遍就得到夹取范围。
   * @param g 几何模型
   * @param xMm 目标 x（mm）
   * @param yMm 目标 y（mm）
   * @param l2 这次解算的肘→目标长度（mm）
   */
  function solveXYAt(g, xMm, yMm, l2) {
    const l1 = g.upperMm;

    let reachable = true;
    let limited = false;

    // ---- 1. 非法输入兜底 ----
    let x = xMm;
    let y = yMm;
    if (!finite(x) || !finite(y)) {
      x = 0;
      y = 0;
      reachable = false;
    }

    // ---- 2. 圆环钳位：内边界取本侧角度限位允许的最小半径 ----
    const r = Math.sqrt(x * x + y * y);
    const rMax = l1 + l2;
    let rMin = minReachableRadiusAt(g, l2, x >= 0);
    if (!(rMin > MIN_RADIUS_MM)) {
      rMin = MIN_RADIUS_MM;
    }

    let tx = x;
    let ty = y;
    if (r > rMax) {
      const scale = rMax / r;
      tx = x * scale;
      ty = y * scale;
      reachable = false;
    } else if (r < rMin) {
      if (r < MIN_RADIUS_MM) {
        tx = rMin;
        ty = 0;
      } else {
        const scale = rMin / r;
        tx = x * scale;
        ty = y * scale;
      }
      reachable = false;
    }

    // ---- 3. 余弦定理求肘角，再求肩角（两支承共用 |θ2|=a、修正角 α、方位角 β）----
    const rr = Math.sqrt(tx * tx + ty * ty);
    let cosElbow = (rr * rr - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (!finite(cosElbow)) {
      cosElbow = -1;
    }
    cosElbow = clampf(cosElbow, -1, 1);

    const a = Math.acos(cosElbow);
    const alpha = Math.atan2(l2 * Math.sin(a), l1 + l2 * Math.cos(a));
    const beta = Math.atan2(ty, tx);

    // 取肘点更高的一支：θ2 的符号与目标 x 的符号相反
    const side = tx >= 0 ? -1 : 1;
    const t2 = g.elbowSign * side * a;
    const t1 = beta + (t2 >= 0 ? -alpha : alpha);

    let d1 = t1 * DEG;
    let d2 = t2 * DEG;
    if (!finite(d1)) {
      d1 = g.shoulder.zeroDeg;
    }
    if (!finite(d2)) {
      d2 = g.elbow.zeroDeg;
    }
    // θ1 归一化到 (-180, 180]
    if (d1 <= -180) {
      d1 += 360;
    } else if (d1 > 180) {
      d1 -= 360;
    }

    // ---- 4. 角度限位（带容差，边界点不误报）----
    const d1c = clampf(d1, g.shoulder.minDeg, g.shoulder.maxDeg);
    const d2c = clampf(d2, g.elbow.minDeg, g.elbow.maxDeg);
    limited = Math.abs(d1c - d1) > LIMIT_EPS_DEG || Math.abs(d2c - d2) > LIMIT_EPS_DEG;

    const p1 = clampPercent(degToPercent(g.shoulder, d1c));
    const p2 = clampPercent(degToPercent(g.elbow, d2c));

    // ---- 5. FK 回算（开环系统里唯一能发现「标定填错」的手段）----
    const fk = solveForwardAt(g, p1, p2, l2);
    const fx = finite(fk.x) ? fk.x : 0;
    const fy = finite(fk.y) ? fk.y : 0;

    return {
      reachable: reachable,
      limited: limited,
      clamped: !reachable,
      x: fx,
      y: fy,
      targetX: xMm,
      targetY: yMm,
      tx: tx,
      ty: ty,
      r: Math.sqrt(fx * fx + fy * fy),
      betaDeg: Math.atan2(ty, tx) * DEG,
      shoulderDeg: d1c,
      elbowDeg: d2c,
      shoulderPercent: p1,
      elbowPercent: p2,
      wristX: fk.wristX,
      wristY: fk.wristY,
      elbowX: fk.elbowX,
      elbowY: fk.elbowY
    };
  }

  /** 该点是否被固件判为「精确命中」（未被径向钳位、未被角度裁剪） */
  function exact(g, x, y) {
    const s = solveXY(g, x, y);
    return s.reachable && !s.limited;
  }

  /**
   * @brief 物理可达：任意一支（θ2 = ±a）能摆到该点，两关节都在限位内
   * @details 固件在 x>0 时只取 θ2<0（高肘），而 θ2<0 的行程只有 45°，
   *          所以「臂能到但固件解不出」的区域是真实存在的，画布上要分开显示。
   * @param g 几何模型
   * @param xMm 目标 x（mm）
   * @param yMm 目标 y（mm）
   * @return true 表示存在合法的 (θ1, θ2) 组合命中该点
   */
  function reachableAny(g, xMm, yMm) {
    return reachableAnyAt(g, foreLength(g), xMm, yMm);
  }

  /** 同上，可指定肘→目标长度（夹取点落在夹爪上时用） */
  function reachableAnyAt(g, l2, xMm, yMm) {
    if (!finite(xMm) || !finite(yMm)) {
      return false;
    }
    const l1 = g.upperMm;
    const r = Math.sqrt(xMm * xMm + yMm * yMm);
    const c = (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (!finite(c) || c < -1.0 || c > 1.0) {
      return false;
    }
    const a = Math.acos(clampf(c, -1, 1)) * DEG;
    const alpha = Math.atan2(l2 * Math.sin(a * RAD), l1 + l2 * Math.cos(a * RAD)) * DEG;
    const beta = Math.atan2(yMm, xMm) * DEG;
    const inShoulder = function (deg) {
      let v = deg;
      if (v <= -180) {
        v += 360;
      } else if (v > 180) {
        v -= 360;
      }
      return v >= g.shoulder.minDeg - LIMIT_EPS_DEG && v <= g.shoulder.maxDeg + LIMIT_EPS_DEG;
    };
    // θ2 = +a（折叠侧）
    if (a <= g.elbow.maxDeg + LIMIT_EPS_DEG && inShoulder(beta - alpha)) {
      return true;
    }
    // θ2 = -a（超伸侧）
    if (-a >= g.elbow.minDeg - LIMIT_EPS_DEG && inShoulder(beta + alpha)) {
      return true;
    }
    return false;
  }

  /**
   * @brief 扫描夹爪上的夹取点：沿末端 jawMm 长的夹爪线逐点解算
   * @details 夹爪占末端 jawMm，抓取点可以落在 [L2Σ−jawMm, L2Σ] 里任意位置。
   *          逐点试 solveXYAt，返回所有「精确命中」的夹取位置（s = 距末端多深）。
   * @param g 几何模型
   * @param xMm 目标 x（mm）
   * @param yMm 目标 y（mm）
   * @param stepMm 扫描步长（mm）
   * @return [{s, sol}, ...]，按 s 升序；空数组表示这条夹爪线咬不到该点
   */
  function scanGrasp(g, xMm, yMm, stepMm) {
    const fore = foreLength(g);
    const jaw = Math.min(Math.max(g.jawMm || 0, 0), Math.max(fore - 1, 0));
    const step = stepMm > 0 ? stepMm : 1;
    const out = [];
    for (let s = 0; s <= jaw + 1e-9; s += step) {
      const sol = solveXYAt(g, xMm, yMm, Math.max(MIN_RADIUS_MM, fore - s));
      if (sol.reachable && !sol.limited) {
        out.push({ s: s, sol: sol });
      }
    }
    return out;
  }

  return {
    RAD: RAD,
    DEG: DEG,
    LIMIT_EPS_DEG: LIMIT_EPS_DEG,
    MIN_RADIUS_MM: MIN_RADIUS_MM,
    clampf: clampf,
    clampPercent: clampPercent,
    percentToDeg: percentToDeg,
    degToPercent: degToPercent,
    foreLength: foreLength,
    maxReach: maxReach,
    minReachableRadius: minReachableRadius,
    minReachableRadiusAt: minReachableRadiusAt,
    solveForward: solveForward,
    solveForwardAt: solveForwardAt,
    solveXY: solveXY,
    solveXYAt: solveXYAt,
    scanGrasp: scanGrasp,
    exact: exact,
    reachableAny: reachableAny,
    reachableAnyAt: reachableAnyAt,
    defaultGeom: defaultGeom
  };
});
