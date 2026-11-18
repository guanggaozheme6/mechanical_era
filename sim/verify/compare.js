/**
 * @file  compare.js
 * @brief 把 sim/arm_core.js 和真实固件 arm_kinematics.cpp（经 probe.exe）逐点对拍
 *
 * 用法（在仓库根目录）：
 *   g++ -std=c++14 -O2 -I Algorithm/inc Algorithm/src/arm_kinematics.cpp \
 *       sim/verify/probe.cpp -o sim/verify/probe.exe
 *   node sim/verify/compare.js
 */
"use strict";

const path = require("path");
const { execFileSync } = require("child_process");
const Core = require("../arm_core.js");

const PROBE = path.join(__dirname, "probe.exe");
const L1 = 76.0;
const L2TOTAL = 168.43;
const SIGN = 1;

// 与 probe.cpp 里写死的标定一致
const geom = {
  upperMm: L1,
  foreMm: L2TOTAL,
  toolMm: 0,
  elbowSign: SIGN,
  shoulder: { zeroDeg: 90, degPerPercent: 0.9, minDeg: 0, maxDeg: 180 },
  elbow: { zeroDeg: 45, degPerPercent: 0.9, minDeg: -45, maxDeg: 135 }
};

const TOL_MM = 0.01;
const TOL_DEG = 0.01;
const TOL_PCT = 0.01;

function runProbe(mode, lines) {
  const out = execFileSync(PROBE, [mode, String(L1), String(L2TOTAL), String(SIGN)], {
    input: lines.join("\n") + "\n",
    encoding: "utf8",
    maxBuffer: 64 * 1024 * 1024
  });
  return out.trim().split(/\r?\n/).map((l) => l.trim().split(/\s+/).map(Number));
}

/* ---------- 1. 逆解：网格 + 随机（跨越内半径 / 外半径 / 跨 x=0 / 限位边界） ---------- */
const ikPoints = [];
for (let x = -260; x <= 280; x += 6) {
  for (let y = -110; y <= 280; y += 6) {
    ikPoints.push([x, y]);
  }
}
// 固定种子随机点，覆盖整圈
let seed = 20261118;
function rnd() {
  seed = (seed * 1103515245 + 12345) & 0x7fffffff;
  return seed / 0x7fffffff;
}
for (let i = 0; i < 5000; i++) {
  const a = rnd() * 2 * Math.PI;
  const r = rnd() * 300;
  ikPoints.push([Math.cos(a) * r, Math.sin(a) * r]);
}
// 边界点：内半径 / 外半径 / 正好在 x=0 竖线上
const rMax = L1 + L2TOTAL;
const rR = Core.minReachableRadius(geom, true);
const rL = Core.minReachableRadius(geom, false);
for (let beta = -60; beta <= 140; beta += 2) {
  const ca = Math.cos((beta * Math.PI) / 180);
  const sa = Math.sin((beta * Math.PI) / 180);
  ikPoints.push([ca * rR, sa * rR], [ca * rMax, sa * rMax], [(ca * rR + ca * rMax) / 2, (sa * rR + sa * rMax) / 2]);
}
// 带内密采样（精确命中区，离边界 ≥ 0.5 mm，是真正考验公式的部分）
for (let beta = -20; beta <= 100; beta += 0.5) {
  const ca = Math.cos((beta * Math.PI) / 180);
  const sa = Math.sin((beta * Math.PI) / 180);
  for (let r = rR + 0.5; r <= rMax - 0.5; r += 0.5) {
    ikPoints.push([ca * r, sa * r]);
  }
}
for (let y = -100; y <= 240; y += 3) {
  ikPoints.push([0, y], [0.001, y], [-0.001, y]);
}

const ikLines = ikPoints.map((p) => p[0].toFixed(4) + " " + p[1].toFixed(4));
const ikRef = runProbe("ik", ikLines);
if (ikRef.length !== ikPoints.length) {
  console.error("probe 输出行数不对：", ikRef.length, "vs", ikPoints.length);
  process.exit(1);
}

/**
 * 奇异带判定：目标贴着外弧 / 本侧内弧，或解出来正好顶在关节限位上。
 * 这种点上 cos_elbow → ±1，θ = acos(x) 的敏感度按 sqrt(eps) 放大：
 * float32 的 1e-7 舍入就能让角度差 0.02°，连锁把 limited 标记翻掉。
 * 不是逻辑差异，单独统计。
 */
const R_BOUNDARY_MM = 0.2;
const A_BOUNDARY_DEG = 0.2;
function nearBoundary(x, y, mine, ref) {
  const r = Math.hypot(x, y);
  const rMax = L1 + L2TOTAL;
  const rMin = Core.minReachableRadius(geom, x >= 0);
  if (Math.abs(r - rMax) < R_BOUNDARY_MM || Math.abs(r - rMin) < R_BOUNDARY_MM) { return true; }
  const rs = Math.hypot(mine.x, mine.y);
  if (Math.abs(rs - rMax) < R_BOUNDARY_MM || Math.abs(rs - rMin) < R_BOUNDARY_MM) { return true; }
  const lim = [
    [geom.shoulder.minDeg, geom.shoulder.maxDeg],
    [geom.elbow.minDeg, geom.elbow.maxDeg]
  ];
  const dd = [mine.shoulderDeg, mine.elbowDeg, ref[2], ref[3]];
  for (let i = 0; i < 2; i++) {
    for (let j = 0; j < 2; j++) {
      if (Math.abs(dd[j * 2 + i] - lim[i][0]) < A_BOUNDARY_DEG ||
          Math.abs(dd[j * 2 + i] - lim[i][1]) < A_BOUNDARY_DEG) { return true; }
    }
  }
  return false;
}

let worst = { mm: 0, deg: 0, pct: 0, at: null };
let bWorst = { mm: 0, deg: 0 };
let flagDiff = 0;
let bCount = 0;
for (let i = 0; i < ikPoints.length; i++) {
  const [x, y] = ikPoints[i];
  const mine = Core.solveXY(geom, x, y);
  const ref = ikRef[i];
  const dx = Math.abs(mine.x - ref[0]);
  const dy = Math.abs(mine.y - ref[1]);
  const d1 = Math.abs(mine.shoulderDeg - ref[2]);
  const d2 = Math.abs(mine.elbowDeg - ref[3]);
  const dp1 = Math.abs(mine.shoulderPercent - ref[4]);
  const dp2 = Math.abs(mine.elbowPercent - ref[5]);
  const flagMine = (mine.reachable ? 1 : 0) * 2 + (mine.limited ? 1 : 0);
  const flagRef = ref[6] * 2 + ref[7];

  if (nearBoundary(x, y, mine, ref)) {
    bCount++;
    bWorst.mm = Math.max(bWorst.mm, dx, dy);
    bWorst.deg = Math.max(bWorst.deg, d1, d2);
    continue;
  }
  if (dx > worst.mm) { worst.mm = dx; worst.at = ["x", x, y]; }
  if (dy > worst.mm) { worst.mm = dy; worst.at = ["y", x, y]; }
  if (Math.max(d1, d2) > worst.deg) { worst.deg = Math.max(d1, d2); }
  if (Math.max(dp1, dp2) > worst.pct) { worst.pct = Math.max(dp1, dp2); }
  if (flagMine !== flagRef) {
    flagDiff++;
    if (flagDiff <= 5) {
      console.log("  标记不一致 @(" + x.toFixed(2) + ", " + y.toFixed(2) + ") mine=" + flagMine + " ref=" + flagRef);
    }
  }
}

/* ---------- 2. 正解：行程百分比网格 ---------- */
const fkLines = [];
const fkPairs = [];
for (let p1 = -100; p1 <= 100; p1 += 2.5) {
  for (let p2 = -100; p2 <= 100; p2 += 2.5) {
    fkLines.push(p1.toFixed(3) + " " + p2.toFixed(3));
    fkPairs.push([p1, p2]);
  }
}
const fkRef = runProbe("fk", fkLines);
let fkWorst = 0;
for (let i = 0; i < fkPairs.length; i++) {
  const mine = Core.solveForward(geom, fkPairs[i][0], fkPairs[i][1]);
  fkWorst = Math.max(fkWorst, Math.abs(mine.x - fkRef[i][0]), Math.abs(mine.y - fkRef[i][1]));
}

/* ---------- 3. 汇总 ---------- */
console.log("对拍点数：逆解 " + ikPoints.length + "（奇异带 " + bCount + "），正解 " + fkPairs.length);
console.log("  IK 位置最大偏差  " + worst.mm.toExponential(2) + " mm");
console.log("  IK 角度最大偏差  " + worst.deg.toExponential(2) + " °");
console.log("  IK 行程最大偏差  " + worst.pct.toExponential(2) + " %");
console.log("  IK 标记不一致    " + flagDiff + " / " + (ikPoints.length - bCount));
console.log("  奇异带（r≈内/外弧、顶限位）位置 " + bWorst.mm.toFixed(4) + " mm / 角度 " +
  bWorst.deg.toFixed(4) + " °（float32 vs double）");
console.log("  FK 位置最大偏差  " + fkWorst.toExponential(2) + " mm");
console.log("基准：r_max=" + rMax.toFixed(2) + " r_min 右=" + rR.toFixed(2) + " 左=" + rL.toFixed(2));

const pass =
  worst.mm <= TOL_MM &&
  worst.deg <= TOL_DEG &&
  worst.pct <= TOL_PCT &&
  fkWorst <= TOL_MM &&
  flagDiff === 0;

console.log(pass ? "PASS：arm_core.js 与 arm_kinematics.cpp 一致" : "FAIL：超出容差（见上）");
process.exit(pass ? 0 : 1);
