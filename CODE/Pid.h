#ifndef PID_MODULE_H
#define PID_MODULE_H
/*
 * ============================================================================
 * 文件名称：Pid.h
 * ============================================================================
 * 模块功能：双轮串级 PID 的控制器实现，两个层次放在同一个文件里：
 *
 *   第一层 位置式 PID（Car_Pid_*）—— 外环位置环
 *     Kp/Ki/Kd 作用于“累计位置误差 -> 目标转速”，带积分抗饱和、
 *     微分测量滤波与目标权重，输出限幅在 ±转速上限。
 *
 *   第二层 增量式 PID（Car_PidInc_*）—— 内环速度环
 *     Δu = Kp·Δe + Ki·e + Kd·Δ²e，带低通微分、抗饱和、低速增益调度、
 *     近设定点软化、大误差积分冻结、可标定静摩擦/速度前馈、
 *     数值合法性检查、跳变重锚定。
 *
 *   两层之间的约定
 *     5 ms 离散增益；内环输出直接是带符号 PWM 数值（±BOARD_PWM_LIMIT），
 *     外环输出是内环转速给定（±BOARD_RPM_LIMIT）。
 *     默认 PID_CASCADE_OUTER_ENABLE = 0，只跑内环。
 *
 *   运行视角
 *     控制中断每拍按 wheel 依次调用：
 *       （外环开启时）Car_Pid_update_dt -> 得到 reference；
 *       Car_PidInc_update_dt(reference, speed, 0, dt) -> 得到 output。
 *     任一环检测到数值异常会置 fault，任务层立即锁存 CAR_CONTROLLER_FAULT。
 * ============================================================================
 */

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 内环（增量式速度环）默认增益
 * 功能：5 ms 离散约定下的起步值，配合 8 拍测速窗口：单计数噪声约 3.4 rpm，
 *       Kp=14 对应约 48 PWM（0.48% 占空比）的残余纹波。
 * 取值：Kp=14.0、Ki=1.0、Kd=0.0；转速—增益对照见 Docs/Speed_PID_Tuning_Guide.md。
 * 注意：菜单和蓝牙命令 4 只写这三个值，并会掉电保存。
 * ------------------------------------------------------------------------- */
#define PID_DEFAULT_KP           14.0f    /* 内环比例增益，单位 PWM/rpm */
#define PID_DEFAULT_KI           1.0f     /* 内环积分增益，单位 PWM/(rpm·拍) */
#define PID_DEFAULT_KD           0.0f     /* 内环微分增益，单位 PWM/(rpm/拍) */

/* ---------------------------------------------------------------------------
 * 微分项处理
 * 功能：D 项测量值一阶低通时间常数与目标权重。
 * 取值：0.020 s（20 ms）；setpoint weight = 1.0 表示微分用完整目标。
 * ------------------------------------------------------------------------- */
#define PID_D_TAU_S              0.020f   /* 微分测量低通时间常数，单位秒 */
#define PID_SETPOINT_WEIGHT      1.0f     /* 微分目标权重：1=完整目标，0=只对反馈微分 */

/* ---------------------------------------------------------------------------
 * 低速增益调度（当前关闭）
 * 功能：在 LOW..HIGH 转速区间内按比例缩放 Kp/Ki。
 * 取值：LOW=HIGH=0 表示完全关闭；缩放系数 1.0 表示不缩放。
 * 注意：重新启用前必须做 160 rpm 以上的高速稳定性验证。
 * ------------------------------------------------------------------------- */
#define PID_LOW_SPEED_RPM        0.0f     /* 调度起点转速，0=关闭，单位 rpm */
#define PID_HIGH_SPEED_RPM       0.0f     /* 调度终点转速，0=关闭，单位 rpm */
#define PID_LOW_KP_SCALE         1.0f     /* 低速段 Kp 缩放系数 */
#define PID_LOW_KI_SCALE         1.0f     /* 低速段 Ki 缩放系数 */

/* ---------------------------------------------------------------------------
 * 近设定点软化（原积分分离）
 * 功能：|误差| 从 0 增大到 BAND_RPM 时，Ki 从 BAND_SCALE 倍平滑升到 1 倍，
 *       没有硬边界；带外保持完整积分，负载残差仍能修正。
 * 取值：带宽 10 rpm（约 3 个 40 ms 窗口计数）、最低 0.30 倍、不限目标速度。
 * ------------------------------------------------------------------------- */
#define PID_I_BAND_RPM           10.0f    /* 软化带，单位 rpm */
#define PID_I_BAND_SCALE         0.30f    /* 带内最低 Ki 倍数，0~1，不到 0 */
#define PID_I_BAND_SPEED_RPM     0.0f     /* 0=所有目标速度都参与软化 */

/* ---------------------------------------------------------------------------
 * 大误差积分关闭（滞回）
 * 功能：|误差| 超过 CUT_IN 时冻结积分（Ki=0），避免起步/阶跃前积累过多；
 *       回到 CUT_OUT 以内自动恢复。只冻结积分，P/D/FF 照常响应。
 * 取值：进 60 rpm、出 40 rpm（约 6 个窗口计数的滞回），0 表示禁用。
 * 用法：由 Car_PidInc_set_integral_cut 在初始化/改增益时写入对象，
 *       默认 0（禁用），因此不改变 PID 模块的默认行为和等价性测试。
 * ------------------------------------------------------------------------- */
#define PID_I_CUT_IN_RPM         60.0f    /* |误差| 大于此值：冻结积分 */
#define PID_I_CUT_OUT_RPM        40.0f    /* |误差| 小于此值：恢复积分 */

/* ---------------------------------------------------------------------------
 * 外环（位置式）开关与占位增益
 * 功能：外环把累计位置误差换成内环转速给定。默认关闭：关闭时内环直接
 *       跟随目标转速，行为与纯速度闭环一致。
 * 取值：启用=0（关闭）；位置环增益默认全 0，误开也不会突然驱动车轮。
 * 注意：外环没有蓝牙协议来源，启用前必须先接入任务层位置指令并重新整定。
 * ------------------------------------------------------------------------- */
#ifndef PID_CASCADE_OUTER_ENABLE
#define PID_CASCADE_OUTER_ENABLE 0        /* 1=启用位置外环，0=旁路 */
#endif
#if PID_CASCADE_OUTER_ENABLE != 0 && PID_CASCADE_OUTER_ENABLE != 1
#error PID_CASCADE_OUTER_ENABLE must be 0 or 1
#endif
#define PID_POS_DEFAULT_KP       0.0f     /* 外环比例增益，单位 rpm/圈 */
#define PID_POS_DEFAULT_KI       0.0f     /* 外环积分增益，单位 rpm/(圈·拍) */
#define PID_POS_DEFAULT_KD       0.0f     /* 外环微分增益，单位 rpm/(圈/拍) */
#define PID_POS_I_FRACTION       0.2f     /* 外环积分上限比例：0.2 × BOARD_RPM_LIMIT */

/* ---------------------------------------------------------------------------
 * 前馈模型
 * 功能：给内环叠加“静摩擦 + 速度”前馈，减小积分长期偏置。
 * 取值：MODEL_ENABLE=0 时用旧模型（恒定 Ks）；1 时用死区模型（D_run + G×rpm，
 *       起步补偿 D_start 随转速衰减）。当前启用死区模型（2026-11-23）。
 * 注意：这些都是浮点数，预处理器不能比较，由 Car_PidInc_set_friction_ff
 *       在初始化时逐项校验，非法直接进控制器故障。
 * ------------------------------------------------------------------------- */
#define PID_FF_MODEL_ENABLE      1        /* 0=旧 Ks/Kv 模型；1=死区前馈模型 */
#define PID_FF_DEAD_RUN_LEFT     200.0f   /* 新模型左轮运行死区，单位 PWM */
#define PID_FF_DEAD_RUN_RIGHT    247.0f   /* 新模型右轮运行死区，单位 PWM */
#define PID_FF_DEAD_START_LEFT   500.0f   /* 新模型左轮起步断裂 PWM */
#define PID_FF_DEAD_START_RIGHT  800.0f   /* 新模型右轮起步断裂 PWM */
#define PID_FF_GAIN_LEFT         10.98f   /* 新模型左轮速度前馈，单位 PWM/rpm */
#define PID_FF_GAIN_RIGHT        10.93f   /* 新模型右轮速度前馈，单位 PWM/rpm */
#define PID_FF_DECAY_RPM         10.0f    /* 新模型起步补偿衰减尺度，单位 rpm */
#define PID_FF_KS_LEFT           900.0f   /* 旧模型左轮恒定静摩擦前馈，单位 PWM */
#define PID_FF_KS_RIGHT          1200.0f  /* 旧模型右轮恒定静摩擦前馈，单位 PWM */
#define PID_FF_KV                0.0f     /* 旧模型速度前馈，单位 PWM/rpm */
#define PID_FF_KA                0.0f     /* 旧模型加速度前馈，单位 PWM/(rpm/s) */
#if PID_FF_MODEL_ENABLE != 0 && PID_FF_MODEL_ENABLE != 1
#error PID_FF_MODEL_ENABLE must be 0 or 1
#endif

/* ---------------------------------------------------------------------------
 * 增量式内环的重新锚定
 * 功能：连续这么多拍没有新目标时，把增量式内部状态重新锚定到当前输出，
 *       避免长期停机后第一拍产生大跳变。
 * 取值：100 拍（500 ms）。
 * ------------------------------------------------------------------------- */
#define PID_REANCHOR_TICKS       100U     /* 重新锚定拍数 */

#include <stdint.h>

/*
 * ============================================================================
 * 文件名称：Pid.h
 * ============================================================================
 * 模块功能：声明串级闭环使用的两种 PID 控制器。
 *
 *   1. 位置式 PID（Car_Pid_*）：输出 = P + I + D + 前馈，积分项显式累加。
 *      当前 Car_Core.c 把它用作串级控制的外环位置环，默认关闭。
 *   2. 增量式 PID（Car_PidInc_*）：输出按增量累加，u[k] = u[k-1] + Δu[k]，
 *      增益、滤波、前馈、限幅和抗饱和约定与位置式保持一致。
 *      当前 Car_Core.c 把它用作内环速度环，蓝牙调参只写这一组增益。
 *
 * 两种控制器的公共约定
 * ----------------------------------------------------------------------------
 *   - 内环速度环：目标/反馈为 rpm，加速度为 rpm/s，输出和前馈项为 PWM 数值；
 *     外环位置环：目标/反馈为输出轴圈数 rev，输出为内环转速给定 rpm；
 *   - Ki/Kd 沿用历史 5 ms 离散增益：
 *         I 增量 = Ki × error × dt / 0.005
 *         D 项   = -Kd × 0.005 × d(measure) / dt
 *   - 目标权重 beta 只作用于比例项，积分仍使用完整速度误差；
 *   - 本模块是纯算法，不访问外设；运行状态由上层（Car_Core.c）管理；
 *   - 所有接口要求 p 非空且独占访问：先 init，按需 configure，
 *     再周期调用 update；仅允许在停车时修改配置。
 * ============================================================================
 */

/* ========================================================================== */
/*                              位置式 PID                                    */
/* ========================================================================== */

/*
 * 位置式 PID 对象。
 *
 * 控制律：
 *     u = Kp × (beta × target - measure)
 *       + I
 *       + (-Kd × 0.005 × d(measure)/dt)
 *       + Kv × target + Ka × acceleration + Ks
 */
typedef struct
{
    /* ---------- 增益与限幅（来源：Car_Pid_init 参数或蓝牙命令 4） ---------- */
    float kp;             /* 比例增益，单位 PWM/rpm */
    float ki;             /* 积分增益，沿用 5 ms 离散约定 */
    float kd;             /* 微分增益，沿用 5 ms 离散约定 */
    float output_limit;   /* 输出绝对值上限，单位 PWM */

    /* ---------- 动态状态（由 update 维护） ---------- */
    float integral;       /* 已累积的积分输出，单位 PWM，受 integral_limit 限制 */
    float last_measure;   /* 上一周期实测转速，单位 rpm，用于计算测量变化率 */

    /* ---------- 配置（来源：Car_Pid_init 默认值或 configure 参数） ---------- */
    float beta;             /* 比例项目标权重，范围 [0,1] */
    float derivative_tau_s; /* 微分滤波时间常数，单位秒 */
    float integral_limit;   /* 积分项绝对值上限，单位 PWM */
    float ks;               /* 静摩擦前馈，单位 PWM */
    float kv;               /* 速度前馈系数，单位 PWM/rpm */
    float ka;               /* 加速度前馈系数，单位 PWM/(rpm/s) */

    /* ---------- 调试/遥测字段（由 update 写入） ---------- */
    float derivative;   /* 滤波后的测量变化率，单位 rpm/s */
    float p_term;       /* 本周期比例项输出，单位 PWM */
    float d_term;       /* 本周期微分项输出，单位 PWM */
    float ff_term;      /* 本周期前馈项输出，单位 PWM */
    float last_output;  /* 最近一次限幅后的输出，单位 PWM */

    /* ---------- 状态标志 ---------- */
    uint8_t started;    /* 1 表示已有上一周期测量值，可以计算微分 */
    uint8_t fault;      /* 1 表示数值异常已锁存故障，update 将返回 0 */
    uint8_t saturated;  /* 1 表示最近一次输出被限幅 */
} Car_Pid_Struct;

/**
 * @brief 清空对象并初始化增益和输出上限。
 * @param p     待初始化的位置式 PID 对象。
 * @param kp    比例增益，有限数，范围 [0, 655.35]。
 * @param ki    积分增益，有限数，范围 [0, 655.35]。
 * @param kd    微分增益，有限数，范围 [0, 655.35]。
 * @param limit 输出上限，有限数，范围 (0, 10000]。
 * @return 1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * @note 默认 beta = 1、微分滤波 0.02 s、积分上限等于输出上限、前馈为零。
 */
uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd, float limit);

/**
 * @brief 在已初始化的对象上配置目标权重、微分滤波和前馈。
 * @param p        已初始化的位置式 PID 对象。
 * @param beta     比例项目标权重，范围 [0, 1]。
 * @param tau      微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波。
 * @param fraction 积分上限比例，范围 (0, 1]，
 *                 积分上限 = 该比例 × output_limit。
 * @param ks       静摩擦前馈，范围 [0, output_limit]。
 * @param kv       速度前馈系数，范围 [0, 10000]。
 * @param ka       加速度前馈系数，范围 [0, 10000]。
 * @return 1 成功并清除动态历史及故障；0 拒绝且保留原配置与历史。
 */
uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float derivative_tau_s, float integral_fraction, float ks, float kv, float ka);

/**
 * @brief 保留配置，清空积分、微分历史、最近输出和状态标志。
 * @param p 位置式 PID 对象。
 * @note reset 会清除 fault；如果配置本身异常，须先修正参数再恢复控制。
 */
void Car_Pid_reset(Car_Pid_Struct *p);

/**
 * @brief 计算一次位置式速度控制输出，正负号表示驱动方向。
 * @param p            位置式 PID 对象。
 * @param target       参考转速（rpm），须为有限数。
 * @param measure      实测转速（rpm），须为有限数。
 * @param acceleration 参考加速度（rpm/s），用于 Ka 前馈。
 * @param dt           实际周期（秒），允许 [0.0001, 0.1]。
 * @return [-output_limit, output_limit] 内的 PWM 数值；故障时返回 0。
 * @note 零返回值也可能是正常输出，须结合 p->fault 判断；故障会锁存。
 */
float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure, float target_acceleration, float dt_s);

/**
 * @brief 固定 5 ms、参考加速度为 0 的兼容入口，其余语义同 update_dt。
 * @param p       位置式 PID 对象。
 * @param target  参考转速（rpm）。
 * @param measure 实测转速（rpm）。
 * @return 限幅后的 PWM 数值；故障时返回 0。
 */
float Car_Pid_update(Car_Pid_Struct *p, float target, float measure);

/* ========================================================================== */
/*                              增量式 PID                                    */
/* ========================================================================== */

/*
 * 增量式 PID 对象。
 *
 * 控制律：
 *     Δu[k] = ΔP + ΔI + ΔD + ΔFF
 *     u[k]  = u[k-1] + Δu[k]
 *     out   = clip(u[k], output_limit)
 *
 *   其中 P、D、FF 的绝对项定义与位置式完全相同，增量由相邻两周期作差得到；
 *   ΔI 是最终接纳的积分变化量。核心状态是未限幅递推值 unclamped_output，
 *   它保留 P 项的完整历史，因此 P 项顶到限幅再退出时不会产生反向冲击；
 *   last_output 只保存实际写到执行器的限幅输出，不能作为递推基准。
 *
 *   抗饱和用条件积分：本拍增量会把输出进一步推入同一方向饱和时，只撤掉
 *   积分增量，P/D/FF 仍照常响应。递推每 8 拍用绝对项重锚一次，纯为消除
 *   单精度累加漂移，不改变控制律，也不影响限幅判定。
 */
typedef struct
{
    /* ---------- 增益与限幅（来源：Car_PidInc_init 参数或本地默认值） ---------- */
    float kp;             /* 比例增益，单位 PWM/rpm */
    float ki;             /* 积分增益，沿用 5 ms 离散约定 */
    float kd;             /* 微分增益，沿用 5 ms 离散约定 */
    float output_limit;   /* 输出绝对值上限，单位 PWM */

    /* ---------- 动态状态（由 update 维护） ---------- */
    float integral;         /* 已累积的积分输出，单位 PWM，受 integral_limit 限制 */
    float last_measure;     /* 上一周期实测转速，单位 rpm，用于计算测量变化率 */
    float last_output;      /* 上一周期限幅后的执行器输出，单位 PWM */
    float unclamped_output; /* 上一周期未限幅的累加值 u[k-1]，单位 PWM */
    float last_p_term;      /* 上一周期比例项值，单位 PWM，用于求 ΔP */
    float last_d_term;      /* 上一周期微分项值，单位 PWM，用于求 ΔD */
    float last_ff_term;     /* 上一周期前馈项值，单位 PWM，用于求 ΔFF */

    /* ---------- 配置（来源：Car_PidInc_init 默认值或 configure 参数） ---------- */
    float beta;             /* 比例项目标权重，范围 [0,1] */
    float derivative_tau_s; /* 微分滤波时间常数，单位秒 */
    float integral_limit;   /* 积分项绝对值上限，单位 PWM */
    float ks;               /* 旧模型静摩擦前馈，单位 PWM */
    float kv;               /* 旧模型速度前馈系数，单位 PWM/rpm */
    float ka;               /* 加速度前馈系数，单位 PWM/(rpm/s) */

    /* ---------- 低速增益调度 / 近设定点软化 / 大误差冻结 / 死区前馈 ---------- */
    float low_speed_rpm;       /* 低速调度起点，0 表示禁用；单位 rpm */
    float high_speed_rpm;      /* 低速调度终点；单位 rpm */
    float kp_scale_low;        /* 低速 Kp 缩放系数，0~1 */
    float ki_scale_low;        /* 低速 Ki 缩放系数，0~1 */
    float integral_band_rpm;   /* 近设定点软化带，0 表示禁用；单位 rpm */
    float integral_scale;      /* 软化带内最低 Ki 倍数，0~1 */
    float integral_speed_rpm;  /* 目标低于此值才启用软化，0=不限；单位 rpm */
    float integral_cut_in_rpm; /* 大误差冻结进入阈值，0 表示禁用；单位 rpm */
    float integral_cut_out_rpm;/* 大误差冻结退出阈值；单位 rpm */
    float ff_dead_run;         /* 死区模型运行死区，单位 PWM */
    float ff_dead_start;       /* 死区模型起步断裂值，单位 PWM */
    float ff_gain;             /* 死区模型速度前馈，单位 PWM/rpm */
    float ff_decay_rpm;        /* 起步补偿衰减尺度，单位 rpm */
    uint8_t ff_model;          /* 1=死区前馈模型；0=旧 Ks/Kv 模型 */

    /* ---------- 调试/遥测字段（由 update 写入） ---------- */
    float derivative;   /* 滤波后的测量变化率，单位 rpm/s */
    float p_term;       /* 本周期比例项值，单位 PWM */
    float d_term;       /* 本周期微分项值，单位 PWM */
    float ff_term;      /* 本周期前馈项值，单位 PWM */
    float delta;        /* 本周期输出增量 Δu，单位 PWM */
    float kp_effective; /* 本拍实际 Kp = 基础 Kp × 低速调度系数 */
    float ki_effective; /* 本拍实际 Ki = 基础 Ki × 调度 × 软化系数（冻结时为 0） */
    float gain_blend;   /* 低速调度混合：0=低速系数，1=高速系数 */

    /* ---------- 状态标志 ---------- */
    uint8_t started;    /* 1 表示已有上一周期测量值，可以计算微分 */
    uint8_t fault;      /* 1 表示数值异常已锁存故障，update 将返回 0 */
    uint8_t saturated;  /* 1 表示最近一次输出被限幅 */
    uint8_t reanchor;   /* 递推重锚计数，每 100 拍用绝对项修正一次浮点漂移 */
    uint8_t integral_hold; /* 1 = 大误差期间冻结积分（60/40 rpm 滞回） */
} Car_PidInc_Struct;

/**
 * @brief 清空增量式 PID 对象并初始化增益和输出上限。
 * @param p     待初始化的增量式 PID 对象。
 * @param kp    比例增益，有限数，范围 [0, 655.35]。
 * @param ki    积分增益，有限数，范围 [0, 655.35]。
 * @param kd    微分增益，有限数，范围 [0, 655.35]。
 * @param limit 输出上限，有限数，范围 (0, 10000]。
 * @return 1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * @note 默认值与位置式一致：beta = 1、微分滤波 0.02 s、积分上限等于输出上限。
 */
uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki, float kd, float limit);

/**
 * @brief 在已初始化的增量式 PID 上配置目标权重、微分滤波和前馈。
 * @param p        已初始化的增量式 PID 对象。
 * @param beta     比例项目标权重，范围 [0, 1]。
 * @param tau      微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波。
 * @param fraction 积分上限比例，范围 (0, 1]。
 * @param ks       静摩擦前馈，范围 [0, output_limit]。
 * @param kv       速度前馈系数，范围 [0, 10000]。
 * @param ka       加速度前馈系数，范围 [0, 10000]。
 * @return 1 成功并清除动态历史及故障；0 拒绝且保留原配置与历史。
 */
uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float derivative_tau_s, float integral_fraction, float ks, float kv, float ka);

/**
 * @brief 配置低速增益调度和近设定点软化；不清除积分/P/FF 历史。
 * @param p              增量式 PID 对象。
 * @param low_speed_rpm  低速调度起点（rpm），0 表示禁用调度。
 * @param high_speed_rpm 高速调度终点（rpm），必须大于 low_speed_rpm。
 * @param kp_scale_low   低速 Kp 缩放系数，(0, 1]。
 * @param ki_scale_low   低速 Ki 缩放系数，(0, 1]。
 * @param band_rpm       近设定点软化带（rpm），0 表示禁用。
 * @param band_scale     软化带内最低 Ki 倍数，[0, 1]。
 * @param band_speed_rpm 仅在目标低于此值时启用软化（rpm），0=不限。
 * @return 1 成功；0 参数无效，保留原配置。
 * @note 调度的系数乘在 kp/ki 基础增益上；基础增益仍由 init/蓝牙命令 4 决定。
 */
uint8_t Car_PidInc_set_low_speed(Car_PidInc_Struct *p, float low_speed_rpm, float high_speed_rpm, float kp_scale_low, float ki_scale_low, float band_rpm, float band_scale, float band_speed_rpm);

/**
 * @brief 配置大误差积分冻结（滞回）：|误差| > in 冻结，< out 恢复。
 * @param p        增量式 PID 对象。
 * @param in_rpm   进入阈值（rpm），必须 > out_rpm；0 表示禁用。
 * @param out_rpm  退出阈值（rpm），>= 0。
 * @return 1 成功并清除冻结状态；0 参数无效，保留原配置。
 * @note 只冻结积分，P/D/FF 照常响应；与条件抗饱和、近设定点软化独立。
 */
uint8_t Car_PidInc_set_integral_cut(Car_PidInc_Struct *p, float in_rpm, float out_rpm);

/**
 * @brief 配置死区前馈模型：PWM = dead_run + gain×|rpm|，起步时 dead_start 衰减。
 * @param p          增量式 PID 对象。
 * @param dead_run   运行死区，单位 PWM，>= 0。
 * @param dead_start 起步断裂值，单位 PWM，>= dead_run。
 * @param gain       速度前馈，单位 PWM/rpm，>= 0。
 * @param decay_rpm  起步补偿衰减尺度，单位 rpm，> 0。
 * @return 1 成功并启用死区模型；0 参数无效，保留旧 Ks/Kv 模型。
 * @note 该配置不清除积分/P/FF 历史；适合在停车状态调用一次。
 */
uint8_t Car_PidInc_set_friction_ff(Car_PidInc_Struct *p, float dead_run, float dead_start, float gain, float decay_rpm);

/**
 * @brief 保留配置，清空积分、微分历史、上一拍输出和状态标志。
 * @param p 增量式 PID 对象。
 * @note 限幅输出和未限幅累加值同时清零，下一次输出从 0 开始累加。
 */
void Car_PidInc_reset(Car_PidInc_Struct *p);

/**
 * @brief 计算一次增量式速度控制输出。
 * @param p            增量式 PID 对象。
 * @param target       参考转速（rpm），须为有限数。
 * @param measure      实测转速（rpm），须为有限数。
 * @param acceleration 参考加速度（rpm/s），用于 Ka 前馈。
 * @param dt           实际周期（秒），允许 [0.0001, 0.1]。
 * @return [-output_limit, output_limit] 内的 PWM 数值；故障时返回 0。
 * @note 与位置式一样，零返回值也可能是正常输出，须结合 p->fault 判断。
 */
float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target, float measure, float target_acceleration, float dt_s);

/**
 * @brief 固定 5 ms、参考加速度为 0 的增量式兼容入口。
 * @param p       增量式 PID 对象。
 * @param target  参考转速（rpm）。
 * @param measure 实测转速（rpm）。
 * @return 限幅后的 PWM 数值；故障时返回 0。
 */
float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure);

#endif /* PID_MODULE_H */
