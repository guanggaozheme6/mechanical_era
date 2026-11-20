/**
 ******************************************************************************
 * @file           : cube3d.js
 * @brief          : 立方体位姿与正交投影（纯数学，浏览器 / Node 通用）
 ******************************************************************************
 *
 * 坐标系（与画布一致）：
 *   X = 车头前方，Y = 上，Z = 指向观察者（右手系：X×Y = Z）。
 *
 * 立方体自身坐标系 (u, v, w)：
 *   u = 前臂方向（在 XY 平面内，yaw 就是 θ1+θ2）
 *   v = u 绕自身转 roll 之后的「夹爪开合」方向
 *   w = u×v
 * 六个面按 ±u / ±v / ±w 编号 1..6，界面上用同一套编号。
 */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (root) {
    root.Cube3D = api;
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const RAD = Math.PI / 180;

  /** 8 个顶点：(sx, sy, sz) ∈ {−1,+1}，索引 = sx?1:0 | sy?2:0 | sz?4:0 */
  const VERTS = [
    [-1, -1, -1], [1, -1, -1], [-1, 1, -1], [1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [-1, 1, 1], [1, 1, 1]
  ];

  /** 六个面：法向在局部坐标系里，顶点按环序给出 */
  const FACES = [
    { id: 1, label: "+u", axis: 0, sign: 1, idx: [1, 3, 7, 5], color: "#f97316" },
    { id: 2, label: "−u", axis: 0, sign: -1, idx: [0, 4, 6, 2], color: "#fdba74" },
    { id: 3, label: "+v", axis: 1, sign: 1, idx: [2, 3, 7, 6], color: "#22d3ee" },
    { id: 4, label: "−v", axis: 1, sign: -1, idx: [0, 1, 5, 4], color: "#0ea5e9" },
    { id: 5, label: "+w", axis: 2, sign: 1, idx: [4, 5, 7, 6], color: "#a78bfa" },
    { id: 6, label: "−w", axis: 2, sign: -1, idx: [0, 1, 3, 2], color: "#c4b5fd" }
  ];

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function mul(a, k) { return [a[0] * k, a[1] * k, a[2] * k]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function norm(a) {
    const n = Math.sqrt(dot(a, a)) || 1;
    return [a[0] / n, a[1] / n, a[2] / n];
  }

  /**
   * @brief 由 yaw / roll 求立方体自身的三个轴（世界坐标）
   * @param yawDeg 前臂方向角（θ1+θ2，度）
   * @param rollDeg 绕前臂轴的 roll（度）
   * @return [u, v, w]，各为 [x,y,z]
   */
  function basis(yawDeg, rollDeg) {
    const psi = yawDeg * RAD;
    const rho = rollDeg * RAD;
    const u = [Math.cos(psi), Math.sin(psi), 0];
    const n0 = [-Math.sin(psi), Math.cos(psi), 0];
    const z = [0, 0, 1];
    const v = add(mul(n0, Math.cos(rho)), mul(z, Math.sin(rho)));
    const w = add(mul(z, Math.cos(rho)), mul(n0, -Math.sin(rho)));
    return [u, v, w];
  }

  /**
   * @brief 8 个顶点（世界坐标）
   * @param center 立方体中心 [x,y,z]（mm，z 一般取 0）
   * @param sizeMm 边长
   * @param axes basis() 的结果
   */
  function worldVerts(center, sizeMm, axes) {
    const h = sizeMm / 2;
    return VERTS.map(function (s) {
      return add(center, add(add(mul(axes[0], s[0] * h), mul(axes[1], s[1] * h)), mul(axes[2], s[2] * h)));
    });
  }

  /**
   * @brief 相机基（从物体指向相机）
   * @param azDeg 方位角（度，绕 Y）
   * @param elDeg 俯仰角（度）
   */
  function camera(azDeg, elDeg) {
    const az = azDeg * RAD;
    const el = elDeg * RAD;
    const dir = norm([Math.cos(el) * Math.cos(az), Math.sin(el), Math.cos(el) * Math.sin(az)]);
    const right = norm(cross([0, 1, 0], dir));
    const up = norm(cross(dir, right));
    return { dir: dir, right: right, up: up };
  }

  /**
   * @brief 正交投影 + 背面剔除 + 画家排序 + 简易明暗
   * @return [{id,label,color,poly:[[sx,sy]...],depth,shade}...]，远的先画
   */
  function project(verts, axes, cam) {
    const light = norm([0.35, 0.75, 0.55]);
    const out = [];
    FACES.forEach(function (f) {
      const nw = mul(axes[f.axis], f.sign);
      if (dot(nw, cam.dir) <= 0.0001) { return; }
      const poly = f.idx.map(function (i) {
        return [dot(verts[i], cam.right), -dot(verts[i], cam.up)];
      });
      let depth = 0;
      f.idx.forEach(function (i) { depth += dot(verts[i], cam.dir); });
      depth /= f.idx.length;
      out.push({
        id: f.id,
        label: f.label,
        color: f.color,
        poly: poly,
        depth: depth,
        shade: 0.34 + 0.66 * Math.max(0, dot(nw, light))
      });
    });
    out.sort(function (a, b) { return a.depth - b.depth; });
    return out;
  }

  /** 有符号角差，归一化到 (−180, 180] */
  function angDiff(a, b) {
    let d = (a - b) % 360;
    if (d > 180) { d -= 360; }
    if (d <= -180) { d += 360; }
    return d;
  }

  /**
   * @brief 立方体最低顶点相对中心的高度（≤ 0）
   * @details 落地时中心高度 = 平面高度 − 这个值，斜着放才不会插进平面。
   */
  function minYOffset(axes, sizeMm) {
    const vs = worldVerts([0, 0, 0], sizeMm, axes);
    let m = Infinity;
    for (let i = 0; i < vs.length; i++) {
      if (vs[i][1] < m) { m = vs[i][1]; }
    }
    return m;
  }

  /**
   * @brief 落到最近的一个「面朝下」姿态
   * @details 这套参数化里 u 永远在 XY 平面内，所以"某个面法向竖直"只可能出现在
   *          ψ ∈ {0, ±90, 180}；ψ = ±90 时 roll 任意，否则 roll 也要落到 90° 的整数倍。
   *          返回最近的合法 (yaw, roll)，用 angDiff 取最短旋转。
   */
  function flatSnap(yawDeg, rollDeg) {
    const psiCands = [0, 90, 180, -90];
    let psi = 0;
    let bd = Infinity;
    psiCands.forEach(function (c) {
      const d = Math.abs(angDiff(yawDeg, c));
      if (d < bd) { bd = d; psi = c; }
    });
    let rho = rollDeg;
    if (Math.abs(psi) !== 90) {
      const rhoCands = [0, 90, 180, -90];
      let bd2 = Infinity;
      let pick = 0;
      rhoCands.forEach(function (c) {
        const d = Math.abs(angDiff(rollDeg, c));
        if (d < bd2) { bd2 = d; pick = c; }
      });
      rho = pick;
    }
    return { yawDeg: psi, rollDeg: rho };
  }

  /** 世界 XY 平面上的轮廓（凸包），给侧视图画剪影用 */
  function hullXY(points) {
    const pts = points.map(function (p) { return [p[0], p[1]]; })
      .sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    const cross2 = function (o, a, b) {
      return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    const lower = [];
    pts.forEach(function (p) {
      while (lower.length >= 2 && cross2(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
        lower.pop();
      }
      lower.push(p);
    });
    const upper = [];
    for (let i = pts.length - 1; i >= 0; i--) {
      const p = pts[i];
      while (upper.length >= 2 && cross2(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
        upper.pop();
      }
      upper.push(p);
    }
    lower.pop();
    upper.pop();
    return lower.concat(upper);
  }

  return {
    VERTS: VERTS,
    FACES: FACES,
    basis: basis,
    worldVerts: worldVerts,
    camera: camera,
    project: project,
    hullXY: hullXY,
    angDiff: angDiff,
    minYOffset: minYOffset,
    flatSnap: flatSnap,
    dot: dot
  };
});
