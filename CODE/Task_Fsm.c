/*
 * ============================================================================
 * 文件名称：Task_Fsm.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 整车控制核心，三层结构：
 *   第一层 状态机   Car_Init / Car_Fault / Car_LocalStop / Car_StartRace
 *   第二层 目标入口 Car_SetPid / Car_SetOpenLoop / Car_RaceWheels
 *   第三层 控制节拍 Car_Tick / Car_TickDt
 *
 * 单拍执行顺序（Car_TickDt）
 *   1. 校验实测周期，越界 -> CAR_TIMING_FAULT；
 *   2. 通信看门狗、堵转、方向、超速等保护；
 *   3. 压入测速窗口、算窗口转速、低通滤波、累计位置；
 *   4. reference = target（参考规划器已删除，不做加速度/加加速度限制）；
 *   5. 开环测试分支 / 外环（默认关闭）/ 内环增量 PID；
 *   6. 输出限幅、数值故障检查，结果写 c->output。
 *
 * 设计约定
 *   - motion 只决定“谁写 target”，执行通路完全一致；
 *   - 故障一旦锁存不覆盖，必须 Car_LocalStop 才能清除；
 *   - 本文件不碰 GPIO/PWM，硬件层读 c->output 后调用 Motor_Apply()。
 *
 * 函数列表（按文件内顺序）
 *   static float absolute / clamp
 *   static float wheel_ff_ks / wheel_ff_dead_run / wheel_ff_dead_start / wheel_ff_gain
 *   static void  speed_window_clear / speed_window_push / speed_window_rpm
 *   static void  open_loop_clear / reset_motion
 *   void         Car_Init / Car_Fault / Car_LocalStop / Car_StartRace
 *   uint8_t      Car_SetPid / Car_SetOpenLoop
 *   void         Car_RaceWheels
 *   void         Car_TickDt / Car_Tick
 * ============================================================================
 */
#include "Task_Fsm.h"

#include <string.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：absolute
 * 函数功能：求浮点数的绝对值。
 * 参数说明：x —— 任意有限浮点数。
 * 返回参数：x 的绝对值。
 * 使用示例：
 *     abs_speed = absolute(c->speed[i]);
 * 备注信息：仅用于速度、误差等量的数值大小判断，不改变控制方向。
 */
static float absolute(float x)
{
    return x < 0.0f ? -x : x;
}

/*
 * 函数名称：clamp
 * 函数功能：把浮点数限制到 [-lim, +lim]。
 * 参数说明：x   —— 待限幅的数值；
 *           lim —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     c->target[0] = clamp(rpm, BOARD_RPM_LIMIT);
 * 备注信息：调用者需保证 lim 为正有限数。
 */
static float clamp(float x, float lim)
{
    if (x > lim)     /* 正向超限压到上限 */
    {
        return lim;
    }

    if (x < -lim)    /* 负向超限压到下限 */
    {
        return -lim;
    }

    return x;        /* 未超限原样返回 */
}

/*
 * 函数名称：wheel_ff_ks
 * 函数功能：取指定车轮的静摩擦前馈系数。
 * 参数说明：wheel —— 轮号，0 左轮、1 右轮。
 * 返回参数：对应车轮的 Ks，单位 PWM 数值。
 * 使用示例：
 *     Car_PidInc_configure(&c->pid_speed[i], ..., wheel_ff_ks(i), ...);
 * 备注信息：两轮静摩擦门槛实测不同（左轮约 12%、右轮约 7%），共用一组
 *           Ks 会让右轮低速过冲、左轮起不来。默认值见 Board_Config.h（及各模块头）。
 */
static float wheel_ff_ks(unsigned wheel)
{
    return wheel ? PID_FF_KS_RIGHT : PID_FF_KS_LEFT;   /* 旧模型：静摩擦前馈 PWM */
}

/* 死区前馈模型参数：运行死区、起步断裂值、速度增益，按轮号选择。
 * 模型关闭时不编译，避免产生未使用函数的构建警告。 */
#if PID_FF_MODEL_ENABLE
/*
 * 函数名称：wheel_ff_dead_run
 * 函数功能：按轮号取“死区前馈模型”的运行死区（PWM）。
 * 参数说明：wheel —— 0 左轮、1 右轮。
 * 返回参数：对应轮的运行死区，单位 PWM 数值。
 * 备注信息：两轮机械差异明显（左轮摩擦大、右轮小），必须按轮分别取值；
 *           只在 PID_FF_MODEL_ENABLE=1 时编译。
 */
static float wheel_ff_dead_run(unsigned wheel)
{
    return wheel ? PID_FF_DEAD_RUN_RIGHT : PID_FF_DEAD_RUN_LEFT;
}

/*
 * 函数名称：wheel_ff_dead_start
 * 函数功能：按轮号取“从静止可靠起步”的断裂 PWM。
 * 参数说明：wheel —— 0 左轮、1 右轮。
 * 返回参数：起步断裂值，单位 PWM 数值。
 * 备注信息：取实测“刚好能转起来”的值，比运行死区大；它随转速平滑衰减到
 *           wheel_ff_dead_run()，所以只影响起步段，不影响稳态。
 */
static float wheel_ff_dead_start(unsigned wheel)
{
    return wheel ? PID_FF_DEAD_START_RIGHT : PID_FF_DEAD_START_LEFT;
}

/*
 * 函数名称：wheel_ff_gain
 * 函数功能：按轮号取速度前馈增益（PWM/rpm）。
 * 参数说明：wheel —— 0 左轮、1 右轮。
 * 返回参数：速度前馈增益，单位 PWM/rpm。
 * 备注信息：来自开环拟合的斜率，反映“每多 1 rpm 需要多少 PWM”；
 *           两轮分开取，避免共用一个增益导致左右轮加速不一致。
 */
static float wheel_ff_gain(unsigned wheel)
{
    return wheel ? PID_FF_GAIN_RIGHT : PID_FF_GAIN_LEFT;
}
#endif

/*
 * 函数名称：speed_window_clear
 * 函数功能：清空双轮测速滑动窗口，使下一次测速从空窗口重新开始。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 使用示例：
 *     speed_window_clear(c);      // 停车、故障或重新发车时
 * 备注信息：窗口清空后测速会在接下来 TASK_WINDOW_TICKS 拍内逐步
 *           恢复满窗；期间按实际已填拍数求平均，不会出现虚假零速。
 */
static void speed_window_clear(Car_Core *c)
{
    unsigned i;                                      /* 双轮循环下标，0 左轮 / 1 右轮 */

    for (i = 0; i < 2; i++)
    {
        c->speed_index[i] = 0;                       /* 写指针回到队首 */
        c->speed_fill[i] = 0;                        /* 有效拍数清零 */
        memset(c->speed_counts[i], 0, sizeof(c->speed_counts[i]));
        memset(c->speed_dt_us[i], 0, sizeof(c->speed_dt_us[i]));
        /* 窗口重开后，守卫也从空窗口重新播种；诊断计数保留累计值。 */
        c->speed_guard_raw[i] = 0.0f;
        c->speed_guard_hold[i] = 0;
        c->count_guard_hold[i] = 0;
    }
}

/*
 * 函数名称：speed_window_push
 * 函数功能：把本拍编码器增量和实测周期写入指定车轮的滑动窗口。
 * 参数说明：c      —— 控制对象；
 *           i      —— 轮号，0 左轮 / 1 右轮；
 *           counts —— 本拍有符号编码器增量；
 *           dt_us  —— 与增量对应的实测周期，单位微秒。
 * 返回参数：无。
 * 使用示例：
 *     speed_window_push(c, 0, left_counts, dt_us);
 * 备注信息：窗口满后自动覆盖最旧的一拍；写指针在 0..N-1 之间循环。
 */
static void speed_window_push(Car_Core *c, unsigned i, int32_t counts, uint32_t dt_us)
{
    unsigned index = c->speed_index[i];               /* 当前写指针 */

    c->speed_counts[i][index] = counts;
    c->speed_dt_us[i][index] = dt_us;

    index++;
    if (index >= TASK_WINDOW_TICKS)
    {
        index = 0;                                    /* 环形回绕 */
    }

    c->speed_index[i] = (uint8_t)index;

    if (c->speed_fill[i] < TASK_WINDOW_TICKS)
    {
        c->speed_fill[i]++;                           /* 上电初期逐步填满 */
    }
}

/*
 * 函数名称：speed_window_rpm
 * 函数功能：用固定窗口内的计数和与时间和计算平均转速。
 * 参数说明：c —— 控制对象；
 *           i —— 轮号，0 左轮 / 1 右轮。
 * 返回参数：窗口平均转速，单位 rpm；窗口时间为 0 时返回 0。
 * 使用示例：
 *     raw = speed_window_rpm(c, i);
 * 备注信息：固定 TASK_WINDOW_TICKS=8 拍（40 ms）；按实际已填拍数求和，
 *           不把量化误差再次放大。2026-11-22 已移除自适应窗口。
 */
static float speed_window_rpm(const Car_Core *c, unsigned i)
{
    int32_t counts = 0;                               /* 窗口计数和 */
    uint32_t dt_us = 0;                               /* 窗口时间和，微秒 */
    unsigned k;                                       /* 有效拍循环下标 */

    for (k = 0; k < c->speed_fill[i]; k++)
    {
        counts += c->speed_counts[i][k];
        dt_us += c->speed_dt_us[i][k];
    }

    if (dt_us == 0U)                                  /* 空窗口或异常周期 */
    {
        return 0.0f;
    }

    /* 转/秒 = counts / (CPR × 秒)，再乘 60 成 rpm；时间用微秒，补 1e6。 */
    return (float)counts * (60.0f * 1000000.0f) /
           (ENCODER_CPR * (float)dt_us);
}

/*
 * 函数名称：open_loop_clear
 * 函数功能：退出双轮开环 PWM 测试并清除命令和超时计数。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：STOP、故障、ARM、轮速命令和恢复闭环时都必须调用，
 *           保证不会把上一次测试的占空比留在执行器上。
 */
static void open_loop_clear(Car_Core *c)
{
    unsigned i;

    for (i = 0; i < 2; i++)
    {
        c->open_loop[i] = 0U;
        c->open_loop_pwm[i] = 0.0f;
        c->open_loop_us[i] = 0U;
    }
}

/*
 * 函数名称：reset_motion
 * 函数功能：停止运动并清除规划器、PID、测速窗口和保护计时。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 使用示例：
 *     reset_motion(c);            // STOP 命令、ARM 启动或故障时调用
 * 备注信息：保留实测转速 speed[]，供后续测速滤波继续使用；
 *           内环、外环积分和位置反馈全部清零，下一次运行从位置零点开始。
 *           测速窗口一并清空，避免把停车前的计数带入下一次发车。
 */
static void reset_motion(Car_Core *c)
{
    unsigned i;                                      /* 双轮循环下标，0 左轮 / 1 右轮 */

    speed_window_clear(c);                           /* 双轮窗口同步清空 */
    open_loop_clear(c);                              /* 退出开环测试 */
    c->outer_enable = 0U;                            /* 退出外环位置控制 */

    for (i = 0; i < 2; i++)
    {
        c->target[i] = 0.0f;                         /* 目标转速清零 */
        c->reference[i] = 0.0f;                      /* 内环转速给定清零 */
        c->position[i] = 0.0f;                       /* 累计位置回到零点 */
        c->position_target[i] = 0.0f;                /* 外环目标位置回到零点 */
        c->output[i] = 0.0f;                         /* 控制输出清零 */
        c->output_last[i] = 0.0f;                    /* 守卫保持值同步清零 */
        c->stall_us[i] = 0;                          /* 堵转计时清零 */
        c->overspeed_us[i] = 0;
        c->direction_us[i] = 0;                      /* 方向异常计时清零 */
        c->slow_speed[i] = absolute(c->speed[i]);    /* 慢速滤波重新播种 */
        Car_PidInc_reset(&c->pid_speed[i]);          /* 清除内环速度历史与故障 */
        Car_Pid_reset(&c->pid_position[i]);          /* 清除外环位置历史与故障 */
    }
}

/* ========================================================================== */
/*                              初始化与故障                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_ClearMotion
 * 函数功能：清空目标、参考、输出与保护计时，供协议层执行 STOP/ARM 时复用。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：不做状态切换；调用方自己决定进入 STOP 还是 RUN。
 */
void Car_ClearMotion(Car_Core *c)
{
    reset_motion(c);
}

/*
 * 函数名称：Car_OpenLoopClear
 * 函数功能：撤去两轮开环 PWM 测试状态（超时/状态切换时调用）。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 */
void Car_OpenLoopClear(Car_Core *c)
{
    open_loop_clear(c);
}

/*
 * 函数名称：Car_Init
 * 函数功能：上电初始化控制对象，创建双轮共享进度的参考规划器和左右轮
 *           两套独立的串级 PID（外环位置式位置环 + 内环增量式速度环），
 *           初始化后进入 CAR_STOP。
 * 参数说明：c —— 待初始化的控制对象。
 * 返回参数：无。
 * 使用示例：
 *     Car_Core car;
 *     Car_Init(&car);
 * 备注信息：规划器限制和 PID 默认参数均来自 Board_Config.h（及各模块头）；
 *           PID 配置失败时直接进入故障状态。外环即使默认关闭也会初始化，
 *           便于以后只改编译开关就能启用。
 */
void Car_Init(Car_Core *c)
{
    unsigned i;                                    /* 双轮循环下标，0 左轮 / 1 右轮 */

    memset(c, 0, sizeof(*c));                      /* 清空全部状态与历史 */

    for (i = 0; i < 2; i++)
    {
        /* 内环：增量式速度 PID，默认增益在 Pid.h（PID_DEFAULT_*），
         * 输出限幅在 Board_Config.h；停车时可由蓝牙命令 4 或板端菜单修改。 */
        Car_PidInc_init(&c->pid_speed[i],
                        PID_DEFAULT_KP,
                        PID_DEFAULT_KI,
                        PID_DEFAULT_KD,
                        BOARD_PWM_LIMIT);

        if (!Car_PidInc_configure(&c->pid_speed[i],
                                  PID_SETPOINT_WEIGHT,
                                  PID_D_TAU_S,
                                  1.0f,
                                  wheel_ff_ks(i),
                                  PID_FF_KV,
                                  PID_FF_KA))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 配置失败立即进入故障 */
        }

#if PID_FF_MODEL_ENABLE
        if (!Car_PidInc_set_friction_ff(&c->pid_speed[i],
                                        wheel_ff_dead_run(i),
                                        wheel_ff_dead_start(i),
                                        wheel_ff_gain(i),
                                        PID_FF_DECAY_RPM))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 死区前馈配置失败 */
        }
#endif
        if (!Car_PidInc_set_low_speed(&c->pid_speed[i],
                                      PID_LOW_SPEED_RPM,
                                      PID_HIGH_SPEED_RPM,
                                      PID_LOW_KP_SCALE,
                                      PID_LOW_KI_SCALE,
                                      PID_I_BAND_RPM,
                                      PID_I_BAND_SCALE,
                                      PID_I_BAND_SPEED_RPM))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 低速调度配置失败 */
        }

        if (!Car_PidInc_set_integral_cut(&c->pid_speed[i],
                                         PID_I_CUT_IN_RPM,
                                         PID_I_CUT_OUT_RPM))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 大误差积分冻结配置失败 */
        }

        /* 外环：位置式位置 PID，输出是内环转速给定，因此限幅用转速上限。
         * 默认增益全 0；由命令 7 的运行时开关 outer_enable 决定是否参与控制。
         * 积分上限取 0.2 × 转速限幅，避免积分单独打满内环给定。 */
        Car_Pid_init(&c->pid_position[i],
                     PID_POS_DEFAULT_KP,
                     PID_POS_DEFAULT_KI,
                     PID_POS_DEFAULT_KD,
                     BOARD_RPM_LIMIT);

        if (!Car_Pid_configure(&c->pid_position[i],
                               PID_SETPOINT_WEIGHT,
                               PID_D_TAU_S,
                               PID_POS_I_FRACTION,
                               0.0f,
                               0.0f,
                               0.0f))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 配置失败立即进入故障 */
        }
    }
}

/*
 * 函数名称：Car_Fault
 * 函数功能：记录一次故障；按 CAR_FAULT_STOP_ENABLE 决定是否停车。
 * 参数说明：c     —— 控制对象；
 *           fault —— 调用者传入的 CAR_*_FAULT 故障码。
 * 返回参数：无。
 * 使用示例：
 *     if (dt_us > TASK_MAX_CONTROL_US)
 *     {
 *         Car_Fault(&car, CAR_TIMING_FAULT);
 *     }
 * 备注信息：每个故障码有独立的 count/first_ms/last_ms；同一条件持续期间
 *           由 fault_hold 保持，不会每拍累加。记录模式下不清目标、不改状态、
 *           不清输出；传统模式保留原来的锁存停车行为。
 */
void Car_Fault(Car_Core *c, uint8_t fault)
{
    if (c == NULL)
    {
        return;
    }

    if (fault < CAR_FAULT_CODES)
    {
        CarFaultRecord *r = &c->fault_log[fault];

        if (c->fault_hold[fault] == 0U)
        {
            if (r->count == 0U)
            {
                r->first_ms = c->uptime_ms;
            }
            if (r->count < 0xFFFFU)
            {
                r->count++;
            }
        }
        c->fault_hold[fault] = (uint8_t)CAR_FAULT_HOLD_TICKS;
        r->last_ms = c->uptime_ms;
    }

    /* 记录模式下的故障降额：当前 PERCENT=0，不降速；改为 10 可恢复。 */
#if CAR_FAULT_DERATE_PERCENT > 0U
    c->fault_derate_us = CAR_FAULT_DERATE_HOLD_MS * 1000U;
#else
    c->fault_derate_us = 0U;
#endif

#if CAR_FAULT_STOP_ENABLE
    c->motion = CAR_MOTION_IDLE;
    if (c->state != CAR_FAULT)    /* 传统模式只记录第一次故障原因 */
    {
        c->fault = fault;
    }
    reset_motion(c);              /* 清空目标、参考和输出 */
    c->state = CAR_FAULT;         /* 进入故障状态 */
    c->parse_state = 0;           /* 丢弃未收完的蓝牙帧 */
#else
    c->fault = fault;             /* 记录模式：只更新最近故障码，继续运行 */
#endif
}

/*
 * 函数名称：Car_ClearFaultLog
 * 函数功能：清空故障记录与最近故障码，不停车、不改目标。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：只清诊断数据；电机目标、状态和输出保持不变。
 */
void Car_ClearFaultLog(Car_Core *c)
{
    if (c == NULL)
    {
        return;
    }
    memset(c->fault_log, 0, sizeof(c->fault_log));
    memset(c->fault_hold, 0, sizeof(c->fault_hold));
    c->fault_derate_us = 0U;
    c->fault = CAR_NO_FAULT;
}

/*
 * 函数名称：Car_LocalStop
 * 函数功能：本地停车：清目标/参考/输出、清最近故障码，回到 STOP。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：与 Car_Fault 的区别——本函数清最近故障码，是“允许重新发车”的入口；
 *           motion 一并回到 IDLE，之后蓝牙命令 3 或菜单发车才能重新取得目标来源。
 *           link_us 清零，避免下次启动时立刻被通信看门狗判超时。
 *           故障历史 fault_log 不清，由菜单“故障记录”页的 OK 单独清除。
 */
void Car_LocalStop(Car_Core *c)
{
    reset_motion(c);
    c->state = CAR_STOP;
    c->fault = c->parse_state = 0;
    memset(c->fault_hold, 0, sizeof(c->fault_hold));
    c->fault_derate_us = 0U;
    c->motion = CAR_MOTION_IDLE;
    c->link_us = 0;
}

/*
 * 函数名称：Car_StartRace
 * 函数功能：菜单/按键发车：STOP -> RUN，并把目标来源切到巡线。
 * 参数说明：c —— 控制对象。
 * 返回参数：1 发车成功；0 当前不在 STOP（例如故障未清或已在运行）。
 * 备注信息：先 Car_LocalStop 再置 RACE，保证上一场的故障码、目标、测速窗口
 *           全部清干净；发车后目标由 Track_Step 每拍写入 Car_RaceWheels。
 *           本函数不启动电机，输出仍要等第一拍 Car_TickDt 才产生。
 */
int Car_StartRace(Car_Core *c)
{
    if (c->state != CAR_STOP) return 0;
    Car_LocalStop(c);
    c->motion = CAR_MOTION_RACE;
    c->state = CAR_RUN;
    return 1;
}

/*
 * 函数名称：Car_SetPid
 * 函数功能：改写指定轮的内环速度环增益（菜单与蓝牙命令 4 共用）。
 * 参数说明：c     —— 控制对象；
 *           wheel —— 轮号，0 左轮、1 右轮；
 *           kp/ki/kd —— 新增益，单位沿用 5 ms 离散约定。
 * 返回参数：0 成功；1 不是 STOP 状态；2 轮号或增益非法。
 * 备注信息：参数传递上做了“先校验、后提交”：
 *           1) 增益先写进临时对象 next，逐项过 Car_PidInc_init/configure/
 *              set_low_speed 的数值检查（NaN、负数、上下界）；
 *           2) 只要有一项不合法就返回 2，c->pid_speed[wheel] 保持原值，
 *              不会出现“改了一半”的状态；
 *           3) 全部通过才整体赋值，所以菜单里调坏参数最多是“这次不生效”。
 *           外环位置环不开放蓝牙/菜单修改，这里只写 pid_speed。
 */
uint8_t Car_SetPid(Car_Core *c, unsigned wheel, float kp, float ki, float kd)
{
    Car_PidInc_Struct next;
    if (c->state != CAR_STOP) return 1;
    if (wheel > 1 || !Car_PidInc_init(&next, kp, ki, kd, BOARD_PWM_LIMIT) ||
        !Car_PidInc_configure(&next, PID_SETPOINT_WEIGHT, PID_D_TAU_S,
                              1.0f, wheel_ff_ks(wheel), PID_FF_KV, PID_FF_KA))
    {
        return 2;
    }
#if PID_FF_MODEL_ENABLE
    if (!Car_PidInc_set_friction_ff(&next,
                                    wheel_ff_dead_run(wheel),
                                    wheel_ff_dead_start(wheel),
                                    wheel_ff_gain(wheel),
                                    PID_FF_DECAY_RPM))
    {
        return 2;
    }
#endif
    if (!Car_PidInc_set_low_speed(&next,
                                  PID_LOW_SPEED_RPM,
                                  PID_HIGH_SPEED_RPM,
                                  PID_LOW_KP_SCALE,
                                  PID_LOW_KI_SCALE,
                                  PID_I_BAND_RPM,
                                  PID_I_BAND_SCALE,
                                  PID_I_BAND_SPEED_RPM))
    {
        return 2;
    }
    if (!Car_PidInc_set_integral_cut(&next,
                                     PID_I_CUT_IN_RPM,
                                     PID_I_CUT_OUT_RPM))
    {
        return 2;
    }
    c->pid_speed[wheel] = next;      /* 蓝牙命令 4 只写内环速度环 */
    return 0;
}

/*
 * 函数名称：Car_SetOuterPid
 * 函数功能：改写指定轮的外环位置环增益（仅在 STOP 状态接受）。
 * 参数说明：c     —— 控制对象；
 *           wheel —— 轮号，0 左轮、1 右轮；
 *           kp/ki/kd —— 位置环增益，单位 rpm/rev 与 5 ms 离散约定。
 * 返回参数：0 成功；1 不是 STOP；2 轮号或增益非法。
 * 备注信息：外环输出是内环转速给定，限幅 BOARD_RPM_LIMIT；积分上限取
 *           PID_POS_I_FRACTION × 该限幅，避免积分单独打满给定。
 */
uint8_t Car_SetOuterPid(Car_Core *c, unsigned wheel, float kp, float ki, float kd)
{
    Car_Pid_Struct next;

    if (c->state != CAR_STOP) return 1;
    if (wheel > 1U ||
        !Car_Pid_init(&next, kp, ki, kd, BOARD_RPM_LIMIT) ||
        !Car_Pid_configure(&next, PID_SETPOINT_WEIGHT, PID_D_TAU_S,
                           PID_POS_I_FRACTION, 0.0f, 0.0f, 0.0f))
    {
        return 2;
    }
    c->pid_position[wheel] = next;
    return 0;
}

/*
 * 函数名称：Car_SetOuter
 * 函数功能：运行时启用/关闭外环，并给左右轮设置相对位置阶跃。
 * 参数说明：c              —— 控制对象；
 *           enable         —— 1 启用，0 关闭；
 *           left_step_rev  —— 左轮相对当前位置的位移，单位 rev；
 *           right_step_rev —— 右轮相对位移。
 * 返回参数：0 成功；1 不是 RUN；2 参数非法（位移超 ±50 rev、首次启用时
 *           车轮未停住、或正在开环测试）。
 * 备注信息：只在 RUN 状态操作；首次启用要求双轮实测低于 10 rpm，避免
 *           行驶中硬切。关闭时位置目标回落到当前位置并复位外环积分。
 */
uint8_t Car_SetOuter(Car_Core *c, uint8_t enable, float left_step_rev, float right_step_rev)
{
    unsigned i;

    if (c->state != CAR_RUN) return 1;
    if (enable > 1U) return 2;
    if (left_step_rev != left_step_rev || right_step_rev != right_step_rev ||
        left_step_rev < -50.0f || left_step_rev > 50.0f ||
        right_step_rev < -50.0f || right_step_rev > 50.0f)
    {
        return 2;
    }
    if (c->open_loop[0] || c->open_loop[1]) return 2;

    if (enable)
    {
        if (!c->outer_enable &&
            (absolute(c->speed[0]) > 10.0f || absolute(c->speed[1]) > 10.0f))
        {
            return 2;
        }
        for (i = 0; i < 2; i++)
        {
            c->position_target[i] = c->position[i] +
                                    ((i == 0U) ? left_step_rev : right_step_rev);
            Car_Pid_reset(&c->pid_position[i]);
            Car_PidInc_reset(&c->pid_speed[i]);
        }
        c->target[0] = 0.0f;
        c->target[1] = 0.0f;
        c->reference[0] = 0.0f;
        c->reference[1] = 0.0f;
        c->outer_enable = 1U;
    }
    else
    {
        for (i = 0; i < 2; i++)
        {
            c->position_target[i] = c->position[i];
            Car_Pid_reset(&c->pid_position[i]);
        }
        c->outer_enable = 0U;
    }

    c->link_us = 0U;
    return 0;
}

/*
 * 函数名称：Car_SetOpenLoop
 * 函数功能：进入开环 PWM 测试：绕过两个 PID，直接把 PWM 给指定轮。
 * 参数说明：c     —— 控制对象；
 *           wheel —— 0 左轮、1 右轮；
 *           pwm   —— 带符号 PWM 数值，绝对值不超过 BT_OPENLOOP_PWM_LIMIT。
 * 返回参数：0 成功；1 不是 RUN；2 轮号/PWM 非法或当前不允许切换。
 * 备注信息：这是自动调参（命令 6）的专用通道，必须把车轮架空再用。
 *           进入条件写得很严，原因都在代码注释里：
 *           - 只允许 RUN；STOP 下应直接用命令 3 或菜单；
 *           - pwm != pwm 用来一次性拦掉 NaN（NaN 与自身比较为假）；
 *           - 首次进入要求双轮目标为 0 且实测 <10 rpm，防止行驶中硬切；
 *           - 每次进入都会清掉双轮开环状态，保证“一次只测一轮”；
 *           - 刷新命令会同步喂通信看门狗，超时则由 Car_TickDt 自动撤输出。
 */
uint8_t Car_SetOpenLoop(Car_Core *c, unsigned wheel, float pwm)
{
    uint8_t entering;

    if (c->state != CAR_RUN) return 1;
    if (wheel > 1 || pwm != pwm ||
        pwm < -BT_OPENLOOP_PWM_LIMIT || pwm > BT_OPENLOOP_PWM_LIMIT)
    {
        return 2;
    }

    /*
     * 首次进入开环时要求双轮目标为零且车轮基本停住，防止在正常行驶中
     * 直接切到开环。周期刷新时 open_loop 已置位，不再做这项检查，
     * 否则阶跃期间转速上升会让后续刷新被拒绝。
     */
    entering = (c->open_loop[0] == 0U && c->open_loop[1] == 0U) ? 1U : 0U;
    if (entering &&
        (c->target[0] != 0.0f || c->target[1] != 0.0f ||
         absolute(c->speed[0]) > 10.0f || absolute(c->speed[1]) > 10.0f))
    {
        return 2;
    }

    /*
     * 一次只测试一轮：先清掉双轮开环和规划器，避免另一轮带着旧目标运动。
     * target/reference 归零后，本拍不会再走正常 PID 分支。
     */
    open_loop_clear(c);
    c->target[0] = 0.0f;
    c->target[1] = 0.0f;
    c->reference[0] = 0.0f;
    c->reference[1] = 0.0f;

    c->open_loop[wheel] = 1U;
    c->open_loop_pwm[wheel] = pwm;
    c->open_loop_us[wheel] = 0U;
    Car_PidInc_reset(&c->pid_speed[wheel]);
    Car_Pid_reset(&c->pid_position[wheel]);
    c->link_us = 0U;                 /* 开环命令同样刷新通信看门狗 */
    return 0;
}

/*
 * 函数名称：Car_RaceWheels
 * 函数功能：巡线状态机每拍写入左右轮目标转速。
 * 参数说明：c   —— 控制对象；
 *           rpm —— 两个目标转速，单位 rpm，下标 0 左、1 右。
 * 返回参数：无。
 * 备注信息：只在“目标来源=RACE 且状态=RUN”时生效，其他时候直接返回：
 *           这样即使巡线模块误调用，也不会在手动模式或停车时驱动车轮。
 *           写入前按 BOARD_RPM_LIMIT 限幅，超限的视觉指令不会整条丢掉，
 *           而是等比压到上限；双轮都为 0 时顺手清空运动状态并复位 PID，
 *           避免停车瞬间积分残留。link_us 清零表示“这一拍有有效指令”。
 */
void Car_RaceWheels(Car_Core *c, const float rpm[2])
{
    if (c->motion != CAR_MOTION_RACE || c->state != CAR_RUN) return;
    c->target[0] = clamp(rpm[0], BOARD_RPM_LIMIT);
    c->target[1] = clamp(rpm[1], BOARD_RPM_LIMIT);
    if (rpm[0] == 0 && rpm[1] == 0) reset_motion(c);
    c->link_us = 0;
}

/*
 * 函数名称：Car_TickDt
 * 函数功能：使用实际控制周期更新测速、参考、PID 输出和运行保护。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量（有符号，前进为正）；
 *           right_count —— 右轮本周期编码器增量（有符号，前进为正）；
 *           dt_us       —— 与计数增量对应的实际时间，单位微秒。
 * 返回参数：无；双轮控制量写入 c->output[0] 和 c->output[1]。
 * 使用示例：
 *     // 在 PIT 控制中断中，取完编码器增量后调用
 *     Car_TickDt(&car, left_delta, right_delta, dt_us);
 *     output(0, car.output[0]);
 *     output(1, car.output[1]);
 * 备注信息：周期越界、通信超时、堵转或方向异常都会触发故障记录；
 *           默认记录模式下不停车，只把参考轮速最多下调
 *           CAR_FAULT_DERATE_PERCENT%，保持 CAR_FAULT_DERATE_HOLD_MS。
 *           本函数不操作 GPIO/PWM，硬件层负责把 output 应用到电机。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count, uint32_t dt_us)
{
    int32_t counts[2];                            /* 双轮编码器增量，0 左轮 / 1 右轮 */
    unsigned i;                                   /* 双轮循环下标 */
    float dt;                                     /* 实际控制周期，单位秒 */
    float fault_scale;                            /* 故障降额系数，最低 1-10% */

    dt = (float)dt_us * 0.000001f;                /* 微秒换算为秒 */

    /* 记录模式下的故障时间戳与“同一条件不重复计次”保持计数。 */
    c->uptime_ms += (uint32_t)(dt_us / 1000U);
    for (i = 0U; i < CAR_FAULT_CODES; i++)
    {
        if (c->fault_hold[i] != 0U)
        {
            c->fault_hold[i]--;
        }
    }

    /* 故障降额计时：多个故障只保持同一个 90% 上限，不叠加。 */
    if (c->fault_derate_us > dt_us)
    {
        c->fault_derate_us -= dt_us;
    }
    else
    {
        c->fault_derate_us = 0U;
    }
    fault_scale = (c->fault_derate_us > 0U) ?
        (1.0f - (float)CAR_FAULT_DERATE_PERCENT / 100.0f) : 1.0f;

    /* 周期越界时停止控制，避免用异常采样间隔计算速度和积分。 */
    if (dt_us < TASK_MIN_CONTROL_US || dt_us > TASK_MAX_CONTROL_US)
    {
        /* 只记录第一次故障 5 的周期，供菜单显示“故障 5 dt=...”。 */
        if (c->state != CAR_FAULT) c->timing_fault_us = dt_us;
        Car_Fault(c, CAR_TIMING_FAULT);
        return;
    }

    counts[0] = left_count;                       /* 左轮增量 */
    counts[1] = right_count;                      /* 右轮增量 */

    /* 蓝牙半帧超时：长时间收不到后续字节则丢弃当前半帧。 */
    if (c->parse_state &&
        (c->frame_us += dt_us) >= BT_FRAME_TIMEOUT_MS * 1000U)
    {
        c->parse_state = 0;                       /* 丢弃半帧，重新寻找帧头 */
    }

    /* 运行通信看门狗：运行中长时间没有有效命令则判定通信故障。 */
    if (c->state == CAR_RUN &&
        (c->link_us += dt_us) >= BT_LINK_TIMEOUT_MS * 1000U)
    {
        Car_Fault(c, CAR_LINK_FAULT);
    }

    /* 目标直通：不再走参考规划器，reference 直接取 target。 */

    for (i = 0; i < 2; i++)
    {
        /*
         * 先把本拍计数压入滑动窗口，再用窗口内的计数和/时间和求平均转速。
         * 单拍 5 ms 时 1 个编码器计数折算 27.3 rpm，直接送 PID 会形成
         * 27 rpm 的量化台阶；窗口平均后台阶缩小到 60/(CPR×窗口时间)。
         * 左右轮使用完全相同的窗口长度和复位时机，测速口径一致。
         */
        float raw;                                /* 窗口平均转速，rpm */
        float abs_speed;                          /* 实测转速绝对值，rpm */
#if TASK_SPEED_GUARD_ENABLE
        uint8_t guarded = 0U;                     /* 本拍测速是否被判为野值 */
#endif

#if TASK_SPEED_GUARD_ENABLE
        /*
         * 单拍计数物理上限：转速上限 × CPR × dt，再乘裕量。超过后用上一拍
         * 滤波转速推算的计数替换，避免野值进入窗口。窗口未填满时还没有
         * 可信的转速基准，先放行，让滤波器建立初值；STOP 状态不拦截，
         * 保证停车后的位置和速度仍按真实计数累计。
         */
        if (c->state == CAR_RUN&&!c->open_loop[i]&&
            c->speed_fill[i] >= TASK_WINDOW_TICKS)
        {
            float dt_s = (float)dt_us * 0.000001f;
            float max_counts = ENCODER_CPR * BOARD_RPM_FULL / 60.0f *
                               dt_s * TASK_COUNT_GUARD_FACTOR;

            if (absolute((float)counts[i]) > max_counts)
            {
                /*
                 * 连续超限达到保持拍数后放行，交给超速/堵转保护判断，
                 * 避免把真实的持续超速永久挡在窗口外；计数回到限内后
                 * 重新开始计数。
                 */
                if (c->count_guard_hold[i] <
                    TASK_SPEED_GUARD_HOLD_TICKS)
                {
                    c->count_guard_hold[i]++;
                    counts[i] = (int32_t)(c->speed[i] * ENCODER_CPR /
                                          60.0f * dt_s);
                    c->count_reject[i]++;
                    guarded = 1U;
                }
            }
            else
            {
                c->count_guard_hold[i] = 0;
            }
        }
#endif

        speed_window_push(c, i, counts[i], dt_us);
        raw = speed_window_rpm(c, i);

#if TASK_SPEED_GUARD_ENABLE
        /*
         * 窗口转速跳变守卫：正常参考加速度 200 rpm/s，5 ms 只变化 1 rpm；
         * 单拍 50 rpm 以上的跳变只可能来自坏计数或真实冲击。短于
         * TASK_SPEED_GUARD_HOLD_TICKS 的跳变保持上一次有效值；持续超过该
         * 拍数的变化视为真实变化并接受，避免把真实加减速永久挡住。
         */
        if (c->state == CAR_RUN &&!c->open_loop[i]&&
            c->speed_fill[i] >= TASK_WINDOW_TICKS)
        {
            if (absolute(raw - c->speed_guard_raw[i]) >
                TASK_SPEED_GUARD_STEP_RPM)
            {
                /*
                 * 连续跳变达到保持拍数后接受新电平，避免真实加减速被
                 * 永久挡住；回到 50 rpm 以内后重新开始计数。
                 */
                if (c->speed_guard_hold[i] <
                    TASK_SPEED_GUARD_HOLD_TICKS)
                {
                    c->speed_guard_hold[i]++;
                    raw = c->speed_guard_raw[i];
                    c->speed_reject[i]++;
                    guarded = 1U;
                }
            }
            else
            {
                c->speed_guard_hold[i] = 0;
            }
            c->speed_guard_raw[i] = raw;
        }
        else
        {
            /* STOP 或窗口未满：守卫跟随真实值，不拦截。 */
            c->speed_guard_raw[i] = raw;
            c->speed_guard_hold[i] = 0;
        }
#endif

        /*
         * 一阶低通滤波：残量化噪声由窗口压制，这里只做最后的平滑。
         * 系数随实际周期变化。停车时也继续更新，便于上位机观察是否停住。
         */
        c->speed[i] += (dt / (TASK_SPEED_FILTER_TAU_S + dt)) *
                       (raw - c->speed[i]);

        /* 累计输出轴位置，单位圈（rev），供外环位置环使用。 */
        c->position[i] += (float)counts[i] / ENCODER_CPR;

        if (c->state != CAR_RUN)
        {
            c->output[i] = 0.0f;                  /* 非运行状态撤去输出 */
            c->output_last[i] = 0.0f;             /* 守卫保持值同步清零 */
            continue;
        }

        /* 本拍参考 = 目标；故障降额期间统一乘 0.9（或配置值），不叠加。 */
        c->reference[i] = c->target[i] * fault_scale;

        /*
         * 开环 PWM 测试（蓝牙命令 6）：直接给执行器，绕过外环/内环。
         * 上位机必须周期刷新命令；超时后该轮立即撤去输出。开环期间
         * target/reference 已被清零，下面的超速、堵转和方向保护仍生效。
         */
        if (c->open_loop[i])
        {
            c->open_loop_us[i] += dt_us;

            if (c->open_loop_us[i] >= BT_OPENLOOP_TIMEOUT_MS * 1000U)
            {
                c->open_loop[i] = 0U;
                c->open_loop_pwm[i] = 0.0f;
                c->output[i] = 0.0f;
                c->output_last[i] = 0.0f;
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
            }
            else
            {
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
                c->output[i] = c->open_loop_pwm[i];
                c->output_last[i] = c->output[i];
            }
        }
        else
        {
            /*
             * 外环位置式位置环。运行时开关 outer_enable 为 1 时，位置误差
             * 换算为内环转速给定，并受 BOARD_RPM_LIMIT 限幅；为 0 时内环
             * 直接跟随 target。
             */
            if (c->outer_enable)
            {
                c->reference[i] = Car_Pid_update_dt(&c->pid_position[i],
                                                    c->position_target[i],
                                                    c->position[i],
                                                    0.0f,
                                                    dt);
            }

            /*
             * 单轮目标和参考均归零后撤去驱动、清除积分；
             * 不以 PID 主动“抱住”零速。外环启用时不能走这条分支，
             * 否则外环的输出一旦为 0 就会被每拍复位。
             */
            if (!c->outer_enable &&
                c->reference[i] == 0.0f && c->target[i] == 0.0f)
            {
                c->output[i] = 0.0f;
                c->output_last[i] = 0.0f;
                Car_PidInc_reset(&c->pid_speed[i]);
                Car_Pid_reset(&c->pid_position[i]);
            }
            else
            {
                /*
                 * 内环增量式速度 PID。参考规划器已删除，外环启用时内环
                 * 给定来自位置环，两者都没有规划加速度，因此前馈按 0。
                 */
                c->output[i] = Car_PidInc_update_dt(&c->pid_speed[i],
                                                    c->reference[i],
                                                    c->speed[i],
                                                    0.0f,
                                                    dt);
            }
        }

#if TASK_SPEED_GUARD_ENABLE
        /*
         * 本拍测速被判为野值时保持上一拍输出。PID 内部仍按保持后的测速
         * 更新，但执行器不会因为一个坏样本在 1~2 拍内掉到零，也就不会
         * 误触发反向或让 DIR 抖动。开环命令不经过这条保持逻辑。
         */
        if (guarded && !c->open_loop[i])
        {
            c->output[i] = c->output_last[i];
        }
#endif
        c->output_last[i] = c->output[i];

        /* 任一环内部检测到数值异常都会锁存 fault；记录模式下也要清掉
         * 该轮可能为 NaN/Inf 的输出，避免继续驱动，但不停另一轮。 */
        if (c->pid_speed[i].fault || c->pid_position[i].fault)
        {
            c->output[i] = 0.0f;
            c->output_last[i] = 0.0f;
            Car_Fault(c, CAR_CONTROLLER_FAULT);
            continue;
        }

        abs_speed = absolute(c->speed[i]);        /* 取实测转速绝对值 */

        /* 实测转速持续超过满量程的 90% 才停车，短暂超调由 100 ms 计时滤除。 */
        if (abs_speed > BOARD_OVERSPEED_RPM)
        {
            c->overspeed_us[i] += dt_us;
            if (c->overspeed_us[i] >= 100000U)
            {
                Car_Fault(c, CAR_OVERSPEED_FAULT);
                return;
            }
        }
        else
        {
            c->overspeed_us[i] = 0;
        }
        /* 转速慢速滤波：0.095 s 时间常数，用于识别换向时是否仍在减速。 */
        c->slow_speed[i] += (dt / (0.095f + dt)) *
                            (abs_speed - c->slow_speed[i]);

        /*
         * 堵转判断：
         * 参考转速和输出都比较大、实测转速却很低，且持续超过阈值时间，
         * 才判定为堵转，避免起步瞬间或短暂过载误触发。开环测试没有
         * 参考转速，因此只要输出超过门槛就按同样的条件计时。
         */
        if ((absolute(c->reference[i]) >= 40.0f || c->open_loop[i]) &&
            absolute(c->output[i]) >= TASK_STALL_PWM &&
            abs_speed < 10.0f)
        {
            c->stall_us[i] += dt_us;

            if (c->stall_us[i] >= TASK_STALL_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_STALL_FAULT);
            }
        }
        else
        {
            c->stall_us[i] = 0;                   /* 条件解除，堵转计时清零 */
        }

        /*
         * 方向异常判断：
         * 参考方向、输出方向都与实测转速方向相反，且转速较高，
         * 并持续超过阈值时间，说明车轮可能被卡住或接线方向错误。
         * slow_speed 用于排除明显减速过程中的正常换向；条件中断即清除计时。
         */
        if ((c->reference[i] * c->speed[i] < 0.0f || c->open_loop[i]) &&
            c->output[i] * c->speed[i] < 0.0f &&
            abs_speed > 100.0f &&
            abs_speed >= c->slow_speed[i] - 5.0f)
        {
            c->direction_us[i] += dt_us;

            if (c->direction_us[i] >= TASK_DIRECTION_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_DIRECTION_FAULT);
            }
        }
        else
        {
            c->direction_us[i] = 0;               /* 条件解除，方向计时清零 */
        }
    }
}

/*
 * 函数名称：Car_Tick
 * 函数功能：固定周期的兼容入口，供主机仿真或固定周期调用者使用。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量；
 *           right_count —— 右轮本周期编码器增量。
 * 返回参数：无。
 * 使用示例：
 *     Car_Tick(&car, left_delta, right_delta);
 * 备注信息：实际硬件使用 Car_TickDt，传入 TIM5 测得的真实周期。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
{
    Car_TickDt(c, left_count, right_count, BOARD_PERIOD_MS * 1000U);
}

