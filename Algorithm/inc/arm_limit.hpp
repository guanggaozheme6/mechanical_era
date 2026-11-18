/**
 ******************************************************************************
 * @file           : arm_limit.hpp
 * @brief          : 摇杆软件限位：两段式（位置法 + 关节域贴边滑），纯计算（Algorithm 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

// 与 sim/stick_limit.js 的 stepTangent() 逐段对应：
//   ① 位置法：目标点沿摇杆方向直线走，解不出就对半缩步，只接受「精确命中」
//   ② 关节兜底：雅可比 → 阻尼最小二乘 → 饱和重解 → 法向剔除（贴边切向滑）
//
// 为什么必须两段（单用任何一段都有硬伤，见 sim/README.md §8.1）：
//   * 只有关节域：末端竖直向下时 DLS 的最小范数解把大臂越转越平，先压到车体上表面；
//   * 只有位置法：摇杆正对一堵墙推，一步都走不了 → 顶到内弧/矩形就整帧停死。
//
// 约束（与 arm_kinematics 同源，改本文件时必须保持）：
//   * 只包含 arm_kinematics.hpp（Model）+ <cmath>，不认识 arm_config / arm_joint；
//   * 工作区矩形与关节限位由调用方以本文件自己的 POD（Box / Span）传入 ——
//     所以这一层不依赖任何工程配置，PC 单测可以喂假模型、假限位；
//   * 没有任何全局可变状态，全部函数是纯函数，不读 tick、不认识 ArmCommand；
//   * 关节状态全程 float，取整只发生在写 ArmCommand 那一层（贴边滑每帧可能只增
//     0.004%，任何中间取整都会把增量抹成 0，现象是「推不动」而不是报错）。
//
// 单位：mm / 度 / 行程百分比（-100..100）/ mm·s⁻¹。

/* Includes ------------------------------------------------------------------*/
#include "arm_kinematics.hpp"

namespace arm_limit {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单关节行程限位（百分比）
 * @details 与 arm_joint::Limits 同形但本层不认识关节层；调用方用 percent 派生值填进来。
 */
struct Span {
    float min_percent; // 行程下限
    float max_percent; // 行程上限
};

/**
 * @brief 笛卡尔工作区矩形（mm，原点在肩轴）
 * @details 与 arm_config::Workspace 同形；框内不等于逐点可达，它只是「命令允许范围」。
 */
struct Box {
    float x_min_mm; // 前方最小
    float x_max_mm; // 前方最大
    float y_min_mm; // 高度最小（可以是负的）
    float y_max_mm; // 高度最大
};

/**
 * @brief 一步的调参
 * @note 字段顺序 = 位置初始化顺序，只在末尾追加（C++14 没有 designated initializer）。
 */
struct Params {
    float damping;          // 关节兜底阻尼 λ（mm/%）：只影响卡死后的那种步，0 = 用内置缺省 1.0
    float sag_mm_per_100mm; // 水平位下垂预补偿，0 = 关闭；只作用于位置法
    float tol_mm;           // 矩形的线性化残差容差（mm），实际项目取 0.02
};

/**
 * @brief 2R 末端雅可比（对行程百分比求导，单位 mm/%）
 * @details x = L1·cosθ1 + L2·cos(θ1+θ2)，y 同理；∂/∂percent 要多乘
 *          deg_per_percent·π/180，所以四个元素都是「mm / 1% 行程」。
 */
struct Jacobian {
    float j11; // ∂x/∂p1
    float j12; // ∂x/∂p2
    float j21; // ∂y/∂p1
    float j22; // ∂y/∂p2
};

/**
 * @brief 一次关节增量（可带「哪一轴被限位固定」）
 */
struct JointStep {
    float dp1;   // 大臂增量（%）
    float dp2;   // 小臂增量（%）
    bool fixed1; // 大臂已顶在限位上，本步不许再动
    bool fixed2; // 小臂同上
};

/**
 * @brief 一步的判定结果
 * @details 上层只用这五个布尔决定「下不下发、反馈打什么标记」，不需要知道走了哪条路。
 */
struct Step {
    float p1_percent; // 新大臂行程 %（float，禁止取整）
    float p2_percent; // 新小臂行程 %
    bool slide;       // 走了贴边滑（缩步 / 饱和裁剪 / 换方向 / 切向分量之一）
    bool cut1;        // 大臂被关节限位裁剪
    bool cut2;        // 小臂被关节限位裁剪
    bool stuck;       // 一步都走不动（正对墙 / 墙角 / 奇异点）—— 此时 p1/p2 原样返回
    bool idle;        // 摇杆没有给位移 —— 此时 p1/p2 原样返回
};

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 回退用的步长序列：1 → 1/2 → … → 1/128
 * @note 位置法与关节兜底共用同一串；取第一个真正可行的。
 */
constexpr float kScales[] = {1.0F, 0.5F, 0.25F, 0.125F, 0.0625F, 0.03125F, 0.0078125F};

/** @brief kScales 的元素个数 */
constexpr unsigned kScaleCount = sizeof(kScales) / sizeof(kScales[0]);

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 2R 末端雅可比（解析式）
 * @details 用解析式而不是差分：固件不必为 2×2 多调两次 FK。末端 = 腕点，L2 = fore_mm。
 * @param model 几何模型
 * @param p1 大臂行程百分比
 * @param p2 小臂行程百分比
 * @return 雅可比（mm/%）
 */
Jacobian jacobian(const arm_kinematics::Model &model, float p1, float p2);

/**
 * @brief 2×2 雅可比的奇异性指标（σmax/σmin）
 * @details 伸直（θ2=0）时两列平行，条件数 → ∞，这就是「奇异区抖」的数学来源。
 * @param J 雅可比
 * @return 条件数；完全奇异时返回 HUGE_VALF
 */
float condition(const Jacobian &J);

/**
 * @brief 阻尼最小二乘：末端位移 → 关节增量
 * @details dq = Jᵀ(J·Jᵀ + λ²I)⁻¹·w。加 λ 的原因：θ2→0 或末端顶到外弧时 J 退化，
 *          无阻尼解会给出巨大的 dq，被限位裁剪后正负来回 → 肉眼可见的抖动。
 *          代价是末端略慢于摇杆，换来的是不抖（λ=0.2 时抖动 0.061 mm/帧²）。
 * @param J 雅可比（mm/%）
 * @param wx 期望末端位移 x（mm）
 * @param wy 期望末端位移 y（mm）
 * @param damping 阻尼 λ（mm/%）
 * @return 关节增量（%）
 */
JointStep dls(const Jacobian &J, float wx, float wy, float damping);

/**
 * @brief 带饱和重解的关节步：顶到限位就把该轴固定，用剩余自由度重算
 * @details 为什么不能只做 clamp：双自由度 DLS 解里两个关节是「合伙」凑出这段位移的。
 *          把其中一个直接夹到边界、另一个照原值走，等于让剩下的那个承担整段位移 ——
 *          实测顶到 θ2=−45° 后每帧 4.2 mm，是期望 1 mm 的 4 倍（末端像弹射一样窜出去）。
 *          正确做法是固定饱和轴、把它已经贡献掉的那部分从期望里减掉，再用单列解剩余位移。
 * @param J 雅可比（mm/%）
 * @param wx 期望末端位移 x（mm）
 * @param wy 期望末端位移 y（mm）
 * @param p1 当前大臂行程百分比
 * @param p2 当前小臂行程百分比
 * @param limits 两路行程限位（顺序 = 大臂、小臂）
 * @param damping 阻尼 λ（mm/%）
 * @return 关节增量 + 哪两轴被固定
 */
JointStep solve_joint_step(const Jacobian &J, float wx, float wy, float p1, float p2,
                           const Span (&limits)[2], float damping);

/**
 * @brief 把一步里的「法向分量」剔掉：贴在工作区矩形边界上时的切向滑
 * @details 关节限位裁剪只能处理「关节顶住」，处理不了「末端顶住矩形边」—— 因为末端越界
 *          是两轴联动的结果，哪个关节都不该被裁。做法是标准的约束零空间投影：
 *          n 是越界那一面的法向，a = nᵀJ 是法向位移对两个关节的敏感度，
 *          dq ← dq − aᵀ(a·dq)/(|a|²+λ²)，于是 n·J·dq ≈ 0，末端只能沿边滑。
 *          正对墙推（切向分量为 0）时 dq 被投影成 0，上层会判 STUCK —— 对的。
 * @param J 雅可比（mm/%）
 * @param step 待投影的关节步
 * @param from_x 步进前末端 x（mm）
 * @param from_y 步进前末端 y（mm）
 * @param box 工作区矩形
 * @param damping 阻尼 λ（mm/%）
 * @return 投影后的关节步
 */
JointStep flatten_to_rect(const Jacobian &J, JointStep step, float from_x, float from_y,
                          const Box &box, float damping);

/**
 * @brief 末端越出工作区矩形多少（mm，0 = 在框内）
 * @param model 几何模型
 * @param p1 大臂行程百分比
 * @param p2 小臂行程百分比
 * @param box 工作区矩形
 * @return 越界量
 */
float overflow(const arm_kinematics::Model &model, float p1, float p2, const Box &box);

/**
 * @brief 一步：位置法优先，关节域兜底
 * @details 分两段，顺序不能反：
 *
 *          **① 位置法（主路）**：目标 = 当前末端 + v·dt，直接 `solve_xy`。解不出来
 *          （圆环钳位 / θ1 越限 / 出矩形）就把目标点沿摇杆方向对半缩（1/2 … 1/128），
 *          取第一个真正可达且合法（精确命中）的点。这一段保证「手到哪，末端就沿那条直线走」。
 *          **圆环钳位后的点绝不能当结果**——那就是末端域的「钳位照发」，抖动和跨支跳变全回来。
 *
 *          **② 关节兜底（只在①全失败时）**：摇杆正对着一堵墙推，位置法一步都走不了，
 *          这时才把速度交给雅可比，沿「关节能走的方向」滑出去。候选按优先级：
 *          DLS 原方向 → x 单轴 → y 单轴 → 只动大臂 → 只动小臂，
 *          每个候选先从整步对半回退。顶到关节限位用饱和重解，越出矩形剔掉法向分量。
 *
 * @param model 几何模型
 * @param limits 两路行程限位（顺序 = 大臂、小臂）
 * @param box 工作区矩形
 * @param params 调参（阻尼 / 下垂补偿 / 矩形容差）
 * @param p1 当前大臂行程百分比
 * @param p2 当前小臂行程百分比
 * @param vx_mm_per_s 摇杆给的末端 x 速度
 * @param vy_mm_per_s 摇杆给的末端 y 速度
 * @param dt_s 本帧时长（秒）；非正 / 非有限当 0 处理
 * @return 本帧结果；无论走哪条路，返回值都可以直接写进关节层
 */
Step tangent_step(const arm_kinematics::Model &model, const Span (&limits)[2], const Box &box,
                  const Params &params, float p1, float p2, float vx_mm_per_s, float vy_mm_per_s,
                  float dt_s);

} // namespace arm_limit
