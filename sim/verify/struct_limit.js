/**
 ******************************************************************************
 * @file           : struct_limit.js
 * @brief          : D-2「结构限位」取证：底盘接触角 → 两端余量 → 弧带/预设的代价
 ******************************************************************************
 * 用法（在仓库根目录，两行）：
 *   g++ -std=c++14 -O2 -I Include -I Algorithm/inc -I Device/inc \
 *       Algorithm/src/arm_kinematics.cpp sim/verify/struct_probe.cpp -o sim/verify/struct_probe.exe
 *   node sim/verify/struct_limit.js
 *
 * 回答的问题：θ1/θ2 的端点到底该定在哪，凭什么。分三步：
 *   ① 结构接触角：底盘是实体、臂是线段（sim/collide.js），大臂放平时最早在 asin(CLA/R0) 蹭到，
 *      所以 θ1 的 0°/180° 是「躺在底盘上」，压根到不了 —— 挡住它的是底盘，不是舵机。
 *   ② 余量：顶住结构回退 3°（v0.2 起的约定），两端对称退 ⇒ zero 仍在中点，percent=0 不挪。
 *   ③ 代价：内半径从 108.12 涨到多少、弧带剩多厚、中立位还剩多少限位余量、框内有多少点变死区。
 *
 * 断言（任何一条不过退出码 1）：
 *   A1 sim/collide.js + arm_core.js 的默认几何 == arm_config.hpp 的真值（防两边各写一份）
 *   A2 底盘接触角与 kShoulderContactDeg 一致（±0.10°）
 *   A3 两端余量对称、θ1 余量 == kShoulderContactDeg 推出的值、two 端都还在 ±100% 内
 *   A4 中立位在固件 solve_xy 里精确命中（reachable && !limited），且在 kWorkspace 内
 *   A5 中立位的姿态（含 168.43 真实小臂）不撞底盘/地面
 *
 * v0.4 起固件只有中立位一条目标姿态；那 8 条规划坐标在 sim/index.html 的 FW_POSES，
 * 页面自己的自检（selfChecks）负责「都在工作区内」，本脚本不再重复。
 */
"use strict";

const path = require("path");
const { spawnSync } = require("child_process");
const Core = require(path.join(__dirname, "..", "arm_core.js"));
const Collide = require(path.join(__dirname, "..", "collide.js"));

const PROBE = path.join(__dirname, "struct_probe.exe");
/** 底盘四尺寸：与 sim/index.html 的 DEFAULT_CFG.chassis 一致 */
const CHASSIS = { lenMm: 265.08, frontMm: 72.46, clearanceMm: 37, bodyMm: 40 };
const DEG = 180 / Math.PI;

/* ---------------- 读数（真值来自 firmware） ---------------- */

function runProbe() {
  const res = spawnSync(PROBE, { encoding: "utf8" });
  if (res.error) {
    throw new Error("探针跑不起来（先按文件头编译 struct_probe.exe）：" + res.error.message);
  }
  if (res.status !== 0) {
    throw new Error("探针退出码 " + res.status + "：" + res.stderr);
  }
  const out = {};
  res.stdout.trim().split(/\r?\n/).forEach(function (line) {
    const f = line.trim().split(/\s+/);
    const num = f.slice(1).map(Number);
    if (f[0] === "geom") {
      out.geom = {
        upperMm: num[0], foreMm: num[1],
        shoulder: { zeroDeg: num[2], degPerPercent: num[3], minDeg: num[4], maxDeg: num[5] },
        elbow: { zeroDeg: num[6], degPerPercent: num[7], minDeg: num[8], maxDeg: num[9] },
        elbowSign: num[10]
      };
    } else if (f[0] === "limits") {
      out.limits = { s1Min: num[0], s1Max: num[1], s2Min: num[2], s2Max: num[3] };
    } else if (f[0] === "ws") {
      out.ws = { xMin: num[0], xMax: num[1], yMin: num[2], yMax: num[3] };
    } else if (f[0] === "neutral") {
      out.neutral = { x: num[0], y: num[1], reachable: num[2] === 1, limited: num[3] === 1,
                      s1: num[4], s2: num[5], th1: num[6], th2: num[7] };
    } else if (f[0] === "selftest") {
      out.neutralOk = num[0] === 1;
    }
  });
  return out;
}

/* ---------------- 结构接触角：大臂最早在哪一度蹭到/离开底盘 ---------------- */

/** 大臂（肩轴 → 肘轴）在给定 θ1 下是否撞底盘 */
function upperHits(th1Deg, upperMm) {
  const t = th1Deg * Core.RAD;
  return Collide.hitSegment(CHASSIS, 0, 0, upperMm * Math.cos(t), upperMm * Math.sin(t), 400);
}

/** 二分出「第一个不撞底盘」的 θ1 */
function contactAngle(upperMm) {
  let lo = 0; // 撞
  let hi = 20; // 不撞
  if (upperHits(hi, upperMm)) {
    return NaN;
  }
  for (let k = 0; k < 60; k++) {
    const mid = 0.5 * (lo + hi);
    if (upperHits(mid, upperMm)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return hi;
}

/* ---------------- 姿态 → 三段端点（给 collide 用） ---------------- */

function parts(g, l2, s1pct, s2pct) {
  const t1 = (g.shoulder.zeroDeg + s1pct * g.shoulder.degPerPercent) * Core.RAD;
  const t2 = (g.elbow.zeroDeg + s2pct * g.elbow.degPerPercent) * Core.RAD;
  const ux = Math.cos(t1 + t2);
  const uy = Math.sin(t1 + t2);
  const ex = g.upperMm * Math.cos(t1);
  const ey = g.upperMm * Math.sin(t1);
  return { ex: ex, ey: ey,
           wristX: ex + g.foreMm * ux, wristY: ey + g.foreMm * uy,
           tipX: ex + l2 * ux, tipY: ey + l2 * uy };
}

/* ---------------- 跑 ---------------- */

const P = runProbe();
const g = P.geom;
/** 固件口径：末端 = 腕点，没有工具延伸（toolMm = 0） */
const G = Object.assign(Core.defaultGeom(), { toolMm: 0 });
G.shoulder = Object.assign({}, g.shoulder);
G.elbow = Object.assign({}, g.elbow);
G.upperMm = g.upperMm;
G.foreMm = g.foreMm;

const checks = [];
function check(ok, name, detail) {
  checks.push({ ok: ok, name: name, detail: detail });
}

/* A1 仿真默认几何 == 固件真值 */
const dSim = Core.defaultGeom();
const a1 = Math.abs(dSim.shoulder.minDeg - g.shoulder.minDeg) < 1e-3 &&
           Math.abs(dSim.shoulder.maxDeg - g.shoulder.maxDeg) < 1e-3 &&
           Math.abs(dSim.elbow.minDeg - g.elbow.minDeg) < 1e-3 &&
           Math.abs(dSim.elbow.maxDeg - g.elbow.maxDeg) < 1e-3 &&
           Math.abs(dSim.upperMm - g.upperMm) < 1e-3 && Math.abs(dSim.foreMm - g.foreMm) < 1e-3;
check(a1, "A1 arm_core.js 默认几何 == arm_config.hpp",
  "sim θ1[" + dSim.shoulder.minDeg + "," + dSim.shoulder.maxDeg + "] θ2[" +
  dSim.elbow.minDeg + "," + dSim.elbow.maxDeg + "]，固件 θ1[" + g.shoulder.minDeg.toFixed(2) + "," +
  g.shoulder.maxDeg.toFixed(2) + "] θ2[" + g.elbow.minDeg.toFixed(2) + "," + g.elbow.maxDeg.toFixed(2) + "]");

/* A2 底盘接触角 */
const contact = contactAngle(g.upperMm);
const analytic = Math.asin(Collide.CLA_MM / Collide.R0_MM) * DEG;
const backoff = g.shoulder.minDeg - contact;
check(Math.abs(contact - analytic) < 0.10 && Math.abs(backoff - (g.shoulder.minDeg - contact)) < 1e-6,
  "A2 底盘接触角 = asin(CLA/R0) = " + analytic.toFixed(3) + "°",
  "实测二分 " + contact.toFixed(3) + "°，固件取 θ1 下限 " + g.shoulder.minDeg.toFixed(2) +
  "°（余量 " + backoff.toFixed(2) + "°）");

/* A3 两端对称 + 跨度在行程内 */
const mid1 = 0.5 * (g.shoulder.minDeg + g.shoulder.maxDeg);
const mid2 = 0.5 * (g.elbow.minDeg + g.elbow.maxDeg);
const span1 = (g.shoulder.maxDeg - g.shoulder.minDeg) / g.shoulder.degPerPercent;
const span2 = (g.elbow.maxDeg - g.elbow.minDeg) / g.elbow.degPerPercent;
check(Math.abs(mid1 - g.shoulder.zeroDeg) < 1e-3 && Math.abs(mid2 - g.elbow.zeroDeg) < 1e-3 &&
      span1 <= 200 && span2 <= 200 && backoff > 0,
  "A3 余量两端对称、zero 仍在中点、跨度 ≤ 200%",
  "θ1 跨度 " + span1.toFixed(1) + "%  θ2 跨度 " + span2.toFixed(1) + "%  余量 " + backoff.toFixed(2) + "°");

/* 派生的几何代价 */
const rMinR = Core.minReachableRadiusAt(G, g.foreMm, true);
const rMinL = Core.minReachableRadiusAt(G, g.foreMm, false);
const rMax = Core.maxReach(G);

/* A4 中立位精确命中 + 在框内（固件探针 + sim 各判一遍，两边必须一致） */
const n = P.neutral;
const simSol = Core.solveXY(G, n.x, n.y);
const simOk = simSol.reachable && !simSol.limited;
const fwOk = n.reachable && !n.limited;
const row = {
  x: n.x, y: n.y, r: Math.hypot(n.x, n.y), th1: n.th1, th2: n.th2, s1: n.s1, s2: n.s2,
  m1: Math.min(n.th1 - g.shoulder.minDeg, g.shoulder.maxDeg - n.th1),
  m2: Math.min(n.th2 - g.elbow.minDeg, g.elbow.maxDeg - n.th2),
  inBox: n.x >= P.ws.xMin && n.x <= P.ws.xMax && n.y >= P.ws.yMin && n.y <= P.ws.yMax
};
check(simOk && fwOk && row.inBox && P.neutralOk,
  "A4 中立位精确命中、在 kWorkspace 内、上电自检过",
  "固件 " + (fwOk ? "命中" : "不命中") + "，仿真 " + (simOk ? "命中" : "不命中") +
  "，出框 " + (row.inBox ? "否" : "是") + "，neutral_ok=" + (P.neutralOk ? 1 : 0) +
  "，限位余量 θ1 " + row.m1.toFixed(2) + "° / θ2 " + row.m2.toFixed(2) + "°");

/* A5 中立位姿态（含真实小臂 168.43）不撞底盘/地面 */
const realL2 = g.foreMm + Core.defaultGeom().toolMm;
const hits = Collide.poseHits(CHASSIS, parts(Object.assign({}, G, { foreMm: g.foreMm }), realL2, row.s1, row.s2));
check(hits.length === 0, "A5 中立位姿态（小臂 168.43 含工具）不撞底盘/地面",
  hits.length === 0 ? "干净" : "撞：" + hits.join("+"));

/* 框内死区量（limited = 整帧拒发，D-14/D-15 的暴露面） */
let lim = 0;
let tot = 0;
for (let x = P.ws.xMin; x <= P.ws.xMax; x += 1) {
  for (let y = P.ws.yMin; y <= P.ws.yMax; y += 1) {
    tot++;
    if (Core.solveXY(G, x, y).limited) {
      lim++;
    }
  }
}

/* ---------------- 报告 ---------------- */

console.log("固件几何 L1=" + g.upperMm.toFixed(2) + " L2=" + g.foreMm.toFixed(2) +
  "（末端=腕点）  θ1[" + g.shoulder.minDeg.toFixed(2) + "," + g.shoulder.maxDeg.toFixed(2) +
  "] θ2[" + g.elbow.minDeg.toFixed(2) + "," + g.elbow.maxDeg.toFixed(2) + "]");
console.log("结构接触角 " + contact.toFixed(3) + "°（= asin(" + Collide.CLA_MM + "/" + Collide.R0_MM +
  ") = " + analytic.toFixed(3) + "°，臂按线段、无厚度 ⇒ 这是下界）");
console.log("派生：r_min 右 " + rMinR.toFixed(2) + " / 左 " + rMinL.toFixed(2) + "  r_max " +
  rMax.toFixed(2) + "  弧带厚 " + (rMax - rMinR).toFixed(2) + " mm");
console.log("框内死区 " + lim + "/" + tot + " = " + (100 * lim / tot).toFixed(1) + "%" +
  "（limited=1 的整帧拒发点，见 D-14/D-15）");
console.log("");
console.log("  姿态      目标              r     θ1     θ2     S1     S2    θ1余量 θ2余量");
const nTxt = "(" + row.x.toFixed(1) + "," + row.y.toFixed(1) + ")";
console.log("  neutral " + nTxt + Array(Math.max(0, 16 - nTxt.length)).join(" ") +
  row.r.toFixed(1).padStart(5) + row.th1.toFixed(2).padStart(7) + row.th2.toFixed(2).padStart(7) +
  row.s1.toFixed(1).padStart(7) + row.s2.toFixed(1).padStart(7) +
  row.m1.toFixed(2).padStart(8) + row.m2.toFixed(2).padStart(7));
console.log("");
let bad = 0;
checks.forEach(function (c) {
  if (!c.ok) {
    bad++;
  }
  console.log((c.ok ? "  PASS  " : "  FAIL  ") + c.name + " —— " + c.detail);
});
console.log("");
console.log(bad === 0 ? "PASS（" + checks.length + " 项）" : "FAIL（" + bad + "/" + checks.length + " 项不过）");
process.exit(bad === 0 ? 0 : 1);
