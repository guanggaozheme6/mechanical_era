/**
 ******************************************************************************
 * @file           : stick_test.js
 * @brief          : 摇杆软件限位的手算对拍（node，不需要浏览器）
 ******************************************************************************
 * 用法：node sim/verify/stick_test.js
 *
 * 为什么要有这个文件：贴边滑的正确性肉眼看不出来 —— 顶到限位后「没被裁的那一轴
 * 该走多快」必须用手算的数字卡住，否则会出现「末端突然 4 倍速冲出去」这种
 * 看起来在动、其实已经错了的行为（初版就踩过）。
 *
 * 断言（不满足就退出码 1）：
 *   A1 向内推：θ2 顶到 −45° 后 S2 停在 −100%，半径锁在内半径上（±0.6 mm）
 *   A2 任何一帧的末端位移都不超过期望步长的 1.15 倍（不许冲）
 *   A3 向外推（末端接近伸直）抖动 ≤ 0.5 mm/帧²
 *   A4 向上推：关节域位移 ≥ 末端域的 2 倍（末端域被圆环钳位咬住）
 *   A5 矩形当墙：不越界 + 沿边切向滑
 *   A6 全场景不撞底盘/地面
 *   A7 向下推能伸到车前方地面（y ≤ 地面 + 3 mm）—— 末端域会停在半空
 */
"use strict";

const path = require("path");
const Core = require(path.join(__dirname, "..", "arm_core.js"));
const Limit = require(path.join(__dirname, "..", "stick_limit.js"));
const Collide = require(path.join(__dirname, "..", "collide.js"));

const g = Core.defaultGeom();
const CH = { lenMm: 265.08, frontMm: 72.46, clearanceMm: 37, bodyMm: 40 };
const DT = 0.02;

/* ---------------- 干涉判据（判据本体在 sim/collide.js，与页面同一套） ---------------- */
function feasible(p1, p2) {
  // 真实口径：solveForward 的末端 = 肘 + (fore + tool)·u（168.43），三段一起判
  return Collide.poseHits(CH, Core.solveForward(g, p1, p2)).length === 0;
}

/* ---------------- 跑一段 ---------------- */
function run(mode, start, vx, vy, frames, ws) {
  let st = { p1: start.p1, p2: start.p2 };
  const out = [];
  for (let i = 0; i < frames; i++) {
    const res = mode === "slide"
      ? Limit.stepTangent(Core, g, st, vx, vy, DT, { damping: 0.2, ws: ws, feasible: feasible })
      : Limit.stepEndDomain(Core, g, st, vx, vy, DT, { ws: ws });
    st = { p1: res.p1, p2: res.p2 };
    out.push(res);
  }
  return out;
}

function maxJitter(trace) {
  let j = 0;
  for (let i = 2; i < trace.length; i++) {
    const a = trace[i - 2].to;
    const b = trace[i - 1].to;
    const c = trace[i].to;
    j = Math.max(j, Math.hypot(c.x - 2 * b.x + a.x, c.y - 2 * b.y + a.y));
  }
  return j;
}

function travel(trace) {
  const a = trace[0].from;
  const b = trace[trace.length - 1].to;
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function flags(trace) {
  const f = {};
  trace.forEach((r) => { f[r.flag] = (f[r.flag] || 0) + 1; });
  return JSON.stringify(f);
}

/**
 * 相邻两帧位移方向反转的次数（夹角 > 120°）。
 * 这才是「抖动」的真实含义：二阶差把「顶墙后改沿切向滑」这种设计内的方向突变
 * 也算成抖，指标就失真了。
 */
function reversals(trace) {
  let n = 0;
  for (let i = 1; i < trace.length; i++) {
    const a = trace[i - 1].got;
    const b = trace[i].got;
    const ma = Math.hypot(a.x, a.y);
    const mb = Math.hypot(b.x, b.y);
    if (ma > 0.05 && mb > 0.05 && (a.x * b.x + a.y * b.y) / (ma * mb) < -0.5) {
      n++;
    }
  }
  return n;
}

function overshoot(trace) {
  let worst = 0;
  trace.forEach((r) => {
    const want = Math.hypot(r.want.x, r.want.y);
    if (want > 1e-9) {
      worst = Math.max(worst, Math.hypot(r.got.x, r.got.y) / want);
    }
  });
  return worst;
}

function anyHit(trace) {
  return trace.some((r) => !feasible(r.p1, r.p2));
}

/* ---------------- 场景 ---------------- */
const HOME = { p1: -66, p2: -86 };
const N = 60;
const checks = [];
function check(ok, name, detail) {
  checks.push({ ok: ok, name: name, detail: detail });
}

// A1 / A2 / A6 向内推
const inward = run("slide", HOME, -50, 0, N);
const lastIn = inward[inward.length - 1];
const rIn = Math.hypot(lastIn.to.x, lastIn.to.y);
const rMin = Core.minReachableRadius(g, true);
/** 小臂下端限位（百分比）：由限位真值反推，别写死 −100 */
const S2_MIN = Core.degToPercent(g.elbow, g.elbow.minDeg);
check(
  Math.abs(lastIn.p2 - S2_MIN) < 0.5 && Math.abs(rIn - rMin) < 0.6,
  "A1 向内推锁在内半径上",
  "S2=" + lastIn.p2.toFixed(2) + "%（限位 " + S2_MIN.toFixed(2) + "%）  r=" + rIn.toFixed(2) +
    "（内半径 " + rMin.toFixed(2) + "）"
);
const ov = overshoot(inward);
check(ov <= 1.15, "A2 不冲（单帧位移 ≤ 1.15 倍期望）", "最坏 " + ov.toFixed(2) + "×");
check(!anyHit(inward), "A6 不撞底盘/地面（向内）", anyHit(inward) ? "有帧不可行" : "全部可行");

// A3 向外推：接近最大伸展（奇异区）时不许来回抖
const outward = run("slide", HOME, 50, 0, N);
const revOut = reversals(outward);
check(
  revOut === 0 && overshoot(outward) <= 1.15,
  "A3 向外推到伸展极限：不来回抖、不冲",
  "方向反转 " + revOut + " 次，最大位移 " + overshoot(outward).toFixed(2) + "× 期望" +
    "（二阶差 " + maxJitter(outward).toFixed(3) + " mm/帧²，含顶墙后的方向突变）"
);

// A4 向下推：进入 β<−15.2° 的死角后，末端域「θ1 越限 → 整帧不下发」会卡住，
//    关节域让 θ1 顶在限位上、由 θ2 继续走，末端沿 θ1=0 这条边界滑下去。
const downSlide = run("slide", HOME, 0, -50, 120);
const downEnd = run("end", HOME, 0, -50, 120);
const limEnd = downEnd.filter(function (r) { return r.flag === "limited"; }).length;
const limSlide = downSlide.filter(function (r) { return r.flag === "limited"; }).length;
const yS = downSlide[downSlide.length - 1].to.y;
const yE = downEnd[downEnd.length - 1].to.y;
check(
  limEnd > 0 && limSlide === 0 && yS < yE - 2.0,
  "A4 向下进死角：末端域会 limited 卡住，关节域沿 θ1 边界滑",
  "limited 帧 末端域 " + limEnd + " / 关节域 " + limSlide +
    "；终点 y 关节域 " + yS.toFixed(1) + " vs 末端域 " + yE.toFixed(1)
);

// A5 矩形当墙：斜向推（+x +y），x 撞到上界后不该停住，而是沿边继续往 +y 滑，
//    而且一步都不许越界（末端域是「钳到边照发」，越界无从谈起，所以这条只测关节域）。
const t0 = Core.solveForward(g, HOME.p1, HOME.p2);
const wsTight = { xMin: t0.x - 600, xMax: t0.x + 3, yMin: t0.y - 400, yMax: t0.y + 400 };
const wallSlide = run("slide", HOME, 35, 35, N, wsTight);
const lastS = wallSlide[wallSlide.length - 1].to;
const maxX = Math.max.apply(null, wallSlide.map(function (r) { return r.to.x; }));
const dyWall = lastS.y - wallSlide[0].from.y;
check(
  maxX <= wsTight.xMax + 0.3 && dyWall > 15.0,
  "A5 矩形当墙：不越界 + 沿边切向滑",
  "最大 x=" + maxX.toFixed(1) + "（上界 " + wsTight.xMax.toFixed(1) + "），沿边走了 Δy=" +
    dyWall.toFixed(1) + " mm"
);

// A7 伸到地面：车前方那片地面（y=−77 上 x 217~223）是够得到的，
//    但「大臂压到车体上表面」会挡住直路，只有让 θ2 单独折叠才走得下去 ——
//    没有单关节兜底方向时这里会整帧 STUCK（实测卡在 y=−62.9）。
const wsFull = { xMin: -243, xMax: 244, yMin: -77, yMax: 243 };
const downFullSlide = run("slide", HOME, 0, -50, 600, wsFull);
const downFullEnd = run("end", HOME, 0, -50, 600, wsFull);
const lowS = downFullSlide.reduce(function (a, r) { return Math.min(a, r.to.y); }, Infinity);
const lowE = downFullEnd.reduce(function (a, r) { return Math.min(a, r.to.y); }, Infinity);
const groundY = -(CH.clearanceMm + CH.bodyMm);
check(
  lowS <= groundY + 3,
  "A7 向下推能伸到车前方地面",
  "关节域最低 y=" + lowS.toFixed(1) + " / 末端域 y=" + lowE.toFixed(1) +
    "（地面 " + groundY + "，差 " + (lowS - groundY).toFixed(1) + " mm）"
);

/* ---------------- 报告 ---------------- */
console.log("L1=" + g.upperMm.toFixed(2) + "  L2Σ=" + Core.foreLength(g).toFixed(2) +
  "  内半径右=" + rMin.toFixed(2) + "  最大伸展=" + Core.maxReach(g).toFixed(2));
console.log("");
const rows = [
  ["关节域 向内(-x)", inward],
  ["末端域 向内(-x)", run("end", HOME, -50, 0, N)],
  ["关节域 向外(+x)", outward],
  ["末端域 向外(+x)", run("end", HOME, 50, 0, N)],
  ["关节域 向下(-y)", downSlide],
  ["末端域 向下(-y)", downEnd],
  ["关节域 向下 到底", downFullSlide],
  ["末端域 向下 到底", downFullEnd]
];
rows.forEach(function (r) {
  const t = r[1];
  const last = t[t.length - 1];
  console.log(
    r[0].padEnd(18) +
    " 终(" + last.to.x.toFixed(1) + "," + last.to.y.toFixed(1) + ")" +
    " r=" + Math.hypot(last.to.x, last.to.y).toFixed(1) +
    " 位移=" + travel(t).toFixed(1) + " mm" +
    " 抖动=" + maxJitter(t).toFixed(3) +
    " 标记=" + flags(t)
  );
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
