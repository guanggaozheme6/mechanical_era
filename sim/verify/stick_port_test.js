/**
 ******************************************************************************
 * @file           : stick_port_test.js
 * @brief          : 固件贴边滑（Algorithm/src/arm_limit.cpp）与仿真（stick_limit.js）对拍
 ****************************************************************************** *
 * 用法：node sim/verify/stick_port_test.js
 *   先编探针（在仓库根目录，一行）：
 *     g++ -std=c++14 -O2 -I Algorithm/inc Algorithm/src/arm_limit.cpp
 *         Algorithm/src/arm_kinematics.cpp sim/verify/stick_probe.cpp
 *         -o sim/verify/stick_probe.exe
 *
 * 为什么要有这个文件：贴边滑的正确性肉眼看不出来。顶到限位后「没被裁的那一轴该走多快」
 * 必须用手算的数字卡住，否则会出现「末端突然 4 倍速冲出去」这种看着在动、其实已经错了
 * 的行为（初版就踩过）。同一套断言在 sim/verify/stick_test.js 里已经跑在仿真几何上；
 * 这里换成**固件几何**（末端 = 腕点，L2 = fore_mm = 40.08，没有工具延伸）跑真代码，
 * 外加逐帧与 JS 对拍 —— 移植抄错一处（顺序、容差、取整）都会在这一栏露出来。
 *
 * 断言（不满足就退出码 1）：
 *   P1 与 JS 逐帧一致：起点在框内的 200 步随机推杆，p1/p2 偏差 < 2e-3 %
 *      （框外起点另列「只报数不判」—— 那是 float vs double 在 130 mm 越界量上差
 *      0.1 µm 就能翻掉一个候选的锐边，两边都是合法分支，不属于移植错）
 *   P2 向内推：S2 顶到限位后停在 −100%，半径锁在右半平面内半径（±0.6 mm），无卡死
 *   P3 不冲：任何一帧的末端位移不超过期望步长的 1.15 倍
 *   P4 向外推到伸展极限：相邻帧位移方向逆转 0 次（不来回抖）
 *   P5 矩形当墙：斜推撞 x 上界后沿边滑（Δy > 15 mm），一步都不越界
 *   P6 放开矩形往下推 600 帧：卡死只允许发生在真实几何边界（θ1 顶 0° 且 θ2 顶 −45°），
 *      此前一帧都不许停，姿态不越行程
 */
"use strict";

const path = require("path");
const { spawnSync } = require("child_process");
const Core = require(path.join(__dirname, "..", "arm_core.js"));
const Limit = require(path.join(__dirname, "..", "stick_limit.js"));

const PROBE = path.join(__dirname, "stick_probe.exe");
const DT = 0.02;
const DAMP = 0.2; // = kMotion.stick_damping_mm_per_percent
const TOL = 0.02; // = kMotion.stick_box_tol_mm
const HOME = { p1: -66, p2: -86 };
/** 固件几何：末端 = 腕点（没有 128.35 的工具延伸），其余与 kGeom 一致 */
const G = Object.assign(Core.defaultGeom(), { toolMm: 0 });
/** kWorkspace */
const WS = { xMin: 70, xMax: 116, yMin: -20, yMax: 85 };
const P = { damping: DAMP, tol: TOL };

/* ---------------- 两条通道：真 C++ 与 JS 仿真 ---------------- */

function probeConfig(ws) {
  const g = G;
  return [
    g.upperMm, g.foreMm,
    g.shoulder.degPerPercent, g.shoulder.zeroDeg, g.shoulder.minDeg, g.shoulder.maxDeg,
    g.elbow.degPerPercent, g.elbow.zeroDeg, g.elbow.minDeg, g.elbow.maxDeg,
    g.elbowSign, DAMP, TOL,
    ws.xMin, ws.xMax, ws.yMin, ws.yMax
  ].join(" ");
}

/** 把一串步喂给真 C++ 探针，返回逐帧结果 */
function cppSteps(steps, ws) {
  const input = probeConfig(ws) + "\n" +
    steps.map(function (s) {
      return [s.p1, s.p2, s.vx, s.vy, s.dt === undefined ? DT : s.dt].join(" ");
    }).join("\n") + "\n";
  const res = spawnSync(PROBE, { input: input, encoding: "utf8" });
  if (res.error) {
    throw new Error("探针跑不起来（先按文件头编译）：" + res.error.message);
  }
  if (res.status !== 0) {
    throw new Error("探针退出码 " + res.status + "：" + res.stderr);
  }
  return res.stdout.trim().split(/\r?\n/).map(function (line) {
    const f = line.trim().split(/\s+/).map(Number);
    return {
      p1: f[0], p2: f[1], slide: f[2] === 1, cut1: f[3] === 1,
      cut2: f[4] === 1, stuck: f[5] === 1, idle: f[6] === 1
    };
  });
}

/** 同一串步喂给 JS 仿真限位 */
function jsSteps(steps, ws) {
  return steps.map(function (s) {
    const dt = s.dt === undefined ? DT : s.dt;
    const r = Limit.stepTangent(Core, G, { p1: s.p1, p2: s.p2 }, s.vx, s.vy, dt,
      { damping: DAMP, ws: ws, tol: TOL });
    return {
      p1: r.p1, p2: r.p2,
      slide: r.flag === Limit.FLAG.SLIDE,
      stuck: r.flag === Limit.FLAG.STUCK,
      idle: r.flag === Limit.FLAG.IDLE
    };
  });
}

/** 同一批 steps 一次喂完（探针支持批量，快 200 倍），逐步更新状态 */
function runBatch(engine, start, vx, vy, frames, ws) {
  const states = [];
  const results = [];
  let p1 = start.p1;
  let p2 = start.p2;
  if (engine === "js") {
    for (let i = 0; i < frames; i++) {
      const r = jsSteps([{ p1: p1, p2: p2, vx: vx, vy: vy }], ws)[0];
      states.push({ p1: p1, p2: p2 });
      results.push(r);
      p1 = r.p1;
      p2 = r.p2;
    }
    return { states: states, results: results, end: { p1: p1, p2: p2 } };
  }
  // cpp：一次一批会「每行都从同一初值算」，所以必须一帧一进程；用 JS 兜住的批量对拍在 P1
  for (let i = 0; i < frames; i++) {
    const r = cppSteps([{ p1: p1, p2: p2, vx: vx, vy: vy }], ws)[0];
    states.push({ p1: p1, p2: p2 });
    results.push(r);
    p1 = r.p1;
    p2 = r.p2;
  }
  return { states: states, results: results, end: { p1: p1, p2: p2 } };
}

function fk(p) {
  const f = Core.solveForward(G, p.p1, p.p2);
  return { x: f.wristX, y: f.wristY };
}

function overshoot(runRes, vx, vy) {
  const want = Math.hypot(vx * DT, vy * DT);
  let worst = 0;
  for (let i = 0; i < runRes.results.length; i++) {
    const a = fk(runRes.states[i]);
    const b = fk({ p1: runRes.results[i].p1, p2: runRes.results[i].p2 });
    worst = Math.max(worst, Math.hypot(b.x - a.x, b.y - a.y) / want);
  }
  return worst;
}

function reversals(runRes) {
  let n = 0;
  for (let i = 1; i < runRes.results.length; i++) {
    const a = fk(runRes.states[i - 1]);
    const b = fk(runRes.states[i]);
    const c = fk(runRes.results[i]);
    const d1 = { x: b.x - a.x, y: b.y - a.y };
    const d2 = { x: c.x - b.x, y: c.y - b.y };
    const m1 = Math.hypot(d1.x, d1.y);
    const m2 = Math.hypot(d2.x, d2.y);
    if (m1 > 0.05 && m2 > 0.05 && (d1.x * d2.x + d1.y * d2.y) / (m1 * m2) < -0.5) {
      n++;
    }
  }
  return n;
}

const checks = [];
function check(ok, name, detail) {
  checks.push({ ok: ok, name: name, detail: detail });
}

/* ---------------- P1 与 JS 逐帧对拍 ---------------- */

const rnd = [];
let seed = 12345;
function rand() { // 固定种子，可复现
  seed = (seed * 1103515245 + 12345) & 0x7fffffff;
  return seed / 0x7fffffff;
}
// 起点必须落在工作区矩形内：真实摇杆路径永远从框内出发。框外起点会让
// `overflow ≤ max(起点越界量, tol)` 变成「差 0.1 µm 就翻候选」的锐边，
// float 与 double 各选一个合法分支，这种不一致不是移植错，所以单独报数不判 PASS。
while (rnd.length < 200) {
  const p1 = -100 + 200 * rand();
  const p2 = -100 + 200 * rand();
  const f = Core.solveForward(G, p1, p2);
  if (f.wristX < WS.xMin + 0.5 || f.wristX > WS.xMax - 0.5) { continue; }
  if (f.wristY < WS.yMin + 0.5 || f.wristY > WS.yMax - 0.5) { continue; }
  rnd.push({ p1: p1, p2: p2, vx: -60 + 120 * rand(), vy: -60 + 120 * rand(),
             dt: 0.005 + 0.03 * rand() });
}
const pc = cppSteps(rnd, WS);
const pj = jsSteps(rnd, WS);
let worstP = 0;
let worstFlag = 0;
pc.forEach(function (c, i) {
  const j = pj[i];
  worstP = Math.max(worstP, Math.abs(c.p1 - j.p1), Math.abs(c.p2 - j.p2));
  if (c.slide !== j.slide || c.stuck !== j.stuck || c.idle !== j.idle) {
    worstFlag++;
  }
});
check(worstP < 2e-3 && worstFlag === 0, "P1 与 JS 逐帧一致（200 步随机，起点在框内）",
  "最大关节偏差 " + worstP.toExponential(2) + " %，标记不一致 " + worstFlag + " 处");

/* ---------------- P1b 框外起点：只报数，不判 ---------------- */

const rndOut = [];
seed = 99991;
for (let i = 0; i < 200; i++) {
  rndOut.push({ p1: -100 + 200 * rand(), p2: -100 + 200 * rand(),
                vx: -60 + 120 * rand(), vy: -60 + 120 * rand(), dt: 0.005 + 0.03 * rand() });
}
const oc = cppSteps(rndOut, WS);
const oj = jsSteps(rndOut, WS);
let worstO = 0;
let flagO = 0;
oc.forEach(function (c, i) {
  worstO = Math.max(worstO, Math.abs(c.p1 - oj[i].p1), Math.abs(c.p2 - oj[i].p2));
  if (c.slide !== oj[i].slide || c.stuck !== oj[i].stuck || c.idle !== oj[i].idle) {
    flagO++;
  }
});

/* ---------------- P2 向内推 ---------------- */

const inward = runBatch("cpp", HOME, -50, 0, 60, WS);
const lastIn = inward.end;
const rIn = Math.hypot(fk(lastIn).x, fk(lastIn).y);
const rMin = Core.minReachableRadius(G, true);
check(Math.abs(lastIn.p2 + 100) < 0.5 && Math.abs(rIn - rMin) < 0.6 &&
      inward.results.filter(function (r) { return r.stuck; }).length === 0,
  "P2 向内推锁在内半径上（无卡死）",
  "S2=" + lastIn.p2.toFixed(2) + "%  r=" + rIn.toFixed(2) +
  "（内半径 " + rMin.toFixed(2) + "）");

/* ---------------- P3 不冲 ---------------- */

const outward = runBatch("cpp", HOME, 50, 0, 60, WS);
const ov = Math.max(overshoot(inward, -50, 0), overshoot(outward, 50, 0),
                    overshoot(runBatch("cpp", HOME, 0, -50, 60, WS), 0, -50));
check(ov <= 1.15, "P3 不冲（单帧位移 ≤ 1.15 倍期望）", "最坏 " + ov.toFixed(2) + "×");

/* ---------------- P4 向外推到伸展极限 ---------------- */

const revOut = reversals(outward);
check(revOut === 0, "P4 向外推到伸展极限：不来回抖", "方向反转 " + revOut + " 次，终点 y=" +
  fk(outward.end).y.toFixed(1) + " r=" + Math.hypot(fk(outward.end).x, fk(outward.end).y).toFixed(1));

/* ---------------- P5 矩形当墙 ---------------- */

const h0 = fk(HOME);
const wsTight = { xMin: h0.x - 600, xMax: h0.x + 3, yMin: h0.y - 400, yMax: h0.y + 400 };
const wall = runBatch("cpp", HOME, 35, 35, 60, wsTight);
let maxX = -Infinity;
wall.results.forEach(function (r, i) {
  maxX = Math.max(maxX, fk({ p1: wall.states[i].p1, p2: wall.states[i].p2 }).x, fk(r).x);
});
const dyWall = fk(wall.end).y - h0.y;
check(maxX <= wsTight.xMax + 0.3 && dyWall > 15.0, "P5 矩形当墙：不越界 + 沿边切向滑",
  "最大 x=" + maxX.toFixed(1) + "（上界 " + wsTight.xMax.toFixed(1) + "），沿边走了 Δy=" +
  dyWall.toFixed(1) + " mm");

/* ---------------- P6 放开矩形往下推 ---------------- */

const wsLoose = { xMin: 0, xMax: 116, yMin: -77, yMax: 85 };
const down = runBatch("cpp", HOME, 0, -50, 600, wsLoose);
const stuckDown = down.results.filter(function (r) { return r.stuck; }).length;
const firstStuck = down.results.findIndex(function (r) { return r.stuck; });
let badPose = 0;
let lowY = Infinity;
down.results.forEach(function (r, i) {
  if (r.p1 < -100.001 || r.p1 > 100.001 || r.p2 < -100.001 || r.p2 > 100.001) {
    badPose++;
  }
  lowY = Math.min(lowY, fk({ p1: r.p1, p2: r.p2 }).y, fk(down.states[i]).y);
});
// 固件几何的底部就是 (θ1=0°, θ2=−45°) = S1/S2 双双顶 −100%，约 y=−28.3：
// 再往下需要 θ1<0，那是结构限位之外。所以「卡死」只要发生在两个关节都到边界之后就是对的。
const stopState = firstStuck >= 0 ? down.states[firstStuck] : HOME;
const atCorner = Math.abs(stopState.p1 + 100) < 1.0 && Math.abs(stopState.p2 + 100) < 1.0;
check(badPose === 0 && stuckDown > 0 && atCorner && lowY <= -28.0,
  "P6 向下推 600 帧：只停在真实边界（θ1=0°/θ2=−45°）",
  "第一次卡死在第 " + firstStuck + " 帧（S1=" + stopState.p1.toFixed(1) + "% S2=" +
  stopState.p2.toFixed(1) + "%），卡死共 " + stuckDown + " 帧，越行程 " + badPose +
  " 帧，最低 y=" + lowY.toFixed(1) + " mm");

/* ---------------- 报告 ---------------- */

console.log("固件几何 L1=" + G.upperMm.toFixed(2) + "  L2=" + G.foreMm.toFixed(2) +
  "（末端=腕点，无工具延伸）  右半平面内半径=" + rMin.toFixed(2) +
  "  最大伸展=" + Core.maxReach(G).toFixed(2));
console.log("");
console.log("对拍：框内起点 200 步 → 最大偏差 " + worstP.toExponential(2) +
  " %，标记不一致 " + worstFlag + " 处");
console.log("      框外起点 200 步 → 最大偏差 " + worstO.toExponential(2) +
  " %，标记不一致 " + flagO + " 处（锐边分支，只报数不判）");
console.log("");
[
  ["向内(−x)", inward, -50, 0],
  ["向外(+x)", outward, 50, 0],
  ["向下(−y)", runBatch("cpp", HOME, 0, -50, 60, WS), 0, -50],
  ["斜推撞墙", wall, 35, 35],
  ["向下 600 帧", down, 0, -50]
].forEach(function (row) {
  const t = row[1];
  console.log(row[0].padEnd(12) + " 终点(" + fk(t.end).x.toFixed(1) + "," + fk(t.end).y.toFixed(1) +
    ") S1=" + t.end.p1.toFixed(1) + "% S2=" + t.end.p2.toFixed(1) + "%" +
    " 最大位移=" + overshoot(t, row[2], row[3]).toFixed(2) + "×" +
    " 卡死=" + t.results.filter(function (r) { return r.stuck; }).length +
    " 滑=" + t.results.filter(function (r) { return r.slide; }).length +
    " 裁=" + t.results.filter(function (r) { return r.cut1 || r.cut2; }).length);
});
console.log("");
let bad = 0;
checks.forEach(function (c) {
  if (!c.ok) { bad++; }
  console.log((c.ok ? "  PASS  " : "  FAIL  ") + c.name + " —— " + c.detail);
});
console.log("");
console.log(bad === 0 ? "PASS（" + checks.length + " 项）" : "FAIL（" + bad + "/" + checks.length + " 项不过）");
process.exit(bad === 0 ? 0 : 1);
