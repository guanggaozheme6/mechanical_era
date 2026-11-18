/**
 ******************************************************************************
 * @file           : collide.js
 * @brief          : 臂体与底盘/地面的干涉判据（页面与 Node 探针共用一套）
 ******************************************************************************
 *
 * 为什么单独一个文件：`sim/index.html` 里原来内联着这 20 行，`sim/verify/struct_limit.js`
 * 又要用它算结构限位。抄第二份必然漂移，所以抽成 UMD 模块（写法和 arm_core.js 一致）：
 * 页面用 <script> 直接引，Node 用 require()。
 *
 * 模型口径（与页面画的那张图一致，改这里等于改整车的碰撞假设）：
 *   * 底盘是实体矩形：x ∈ [frontMm−lenMm, frontMm]、y ∈ [ground, 0]，上表面留 CLA_MM 余隙；
 *   * 肩轴 14 mm 半径内是底座/轴承座，本来就坐在底盘上，不算干涉；
 *   * 臂的三段（大臂 / 小臂 / 夹爪）按直线段采样判点，**没有厚度** —— 所以这里给出的
 *     接触角是下界，实物只会更靠内。
 */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (root) {
    root.SimCollide = api;
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  /** 臂体与底盘上表面之间留的最小间隙（mm）：小于它就是蹭/穿 */
  const CLA_MM = 1.0;
  /** 底座半径（mm）：这圈里的点天生贴在底盘上，不算干涉 */
  const R0_MM = 14.0;

  /** 地面高度（mm，原点在肩轴） */
  function groundY(chassis) {
    return -(chassis.clearanceMm + chassis.bodyMm);
  }

  /**
   * @brief 点是否撞进底盘或钻到地面以下
   * @param chassis 底盘四尺寸 {lenMm, frontMm, clearanceMm, bodyMm}
   * @param x 点 x（mm）
   * @param y 点 y（mm）
   * @return true 表示这个点不可用
   */
  function hitPoint(chassis, x, y) {
    if (x * x + y * y < R0_MM * R0_MM) {
      return false;
    }
    if (y < groundY(chassis)) {
      return true;
    }
    const rear = chassis.frontMm - chassis.lenMm;
    return x >= rear && x <= chassis.frontMm && y < CLA_MM;
  }

  /**
   * @brief 线段是否有一段落在底盘/地面里（等距采样）
   * @param chassis 底盘四尺寸
   * @param ax 起点 x（mm）
   * @param ay 起点 y（mm）
   * @param bx 终点 x（mm）
   * @param by 终点 y（mm）
   * @param n 采样段数（含两端共 n+1 个点）
   * @return true 表示撞上
   */
  function hitSegment(chassis, ax, ay, bx, by, n) {
    for (let t = 0; t <= n; t++) {
      const x = ax + (bx - ax) * t / n;
      const y = ay + (by - ay) * t / n;
      if (hitPoint(chassis, x, y)) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 某个姿态（大臂/小臂/夹爪三段）与底盘/地面干涉的段名
   * @param chassis 底盘四尺寸
   * @param pose 带 ex/ey/wristX/wristY/tipX/tipY 的姿态（页面 poseParts 的输出，
   *             也可以按同一形状自己拼：wrist = 肘 + (fore−jaw)·u，tip = 肘 + fore·u）
   * @return 段名数组，空数组 = 没干涉
   */
  function poseHits(chassis, pose) {
    const hits = [];
    if (hitSegment(chassis, 0, 0, pose.ex, pose.ey, 6)) {
      hits.push("大臂");
    }
    if (hitSegment(chassis, pose.ex, pose.ey, pose.wristX, pose.wristY, 8)) {
      hits.push("小臂");
    }
    if (hitSegment(chassis, pose.wristX, pose.wristY, pose.tipX, pose.tipY, 4)) {
      hits.push("夹爪");
    }
    return hits;
  }

  return {
    CLA_MM: CLA_MM,
    R0_MM: R0_MM,
    groundY: groundY,
    hitPoint: hitPoint,
    hitSegment: hitSegment,
    poseHits: poseHits
  };
});
