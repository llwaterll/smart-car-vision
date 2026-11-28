#ifndef TASK_FSM_H
#define TASK_FSM_H

/*
 * ============================================================================
 * 文件名称：Task_Fsm.h
 * ============================================================================
 * 模块功能：整车控制核心。文件内分三层，对外只暴露少量入口：
 *
 *   第一层 整车状态机  Car_Init / Car_LocalStop / Car_Fault / Car_StartRace
 *           STOP（可改 PID、可发车）-> RUN（接受目标）。
 *           默认 CAR_FAULT_STOP_ENABLE=0：故障只按码记录，不停车、不锁存；
 *           =1 时保留传统 FAULT（锁存第一个故障，必须先 STOP 清除再 ARM）。
 *           故障记录通过 Car_ClearFaultLog() 或菜单故障页清除，不影响电机目标。
 *
 *   第二层 运动目标    Car_SetPid / Car_SetOpenLoop / Car_RaceWheels
 *           所有目标来源（蓝牙命令 3/6、菜单、巡线状态机）都只写 target，
 *           之后共用同一条通路：限幅 -> 测速窗口/守卫 -> 内环 PID -> 输出。
 *
 *   第三层 控制节拍    Car_Tick / Car_TickDt
 *           每拍完成测速、保护判定（通信/堵转/方向/超速/时序）、PID 计算
 *           和输出，不直接操作 GPIO/PWM（硬件层负责应用 output）。
 *
 *   模式（motion）
 *     CAR_MOTION_IDLE  没有目标来源，输出为 0；
 *     CAR_MOTION_BT    蓝牙命令 3 手动调速；
 *     CAR_MOTION_RACE  巡线状态机自动发车。
 *     只有“谁写目标”不同，执行通路完全一致；RACE 期间蓝牙命令 3 被拒绝
 *     （result=1），协议语义与改造前保持一致。
 *
 *   运行视角
 *     控制中断：Car_TickDt(&car, 左计数, 右计数, 实测周期)
 *       -> 更新 speed/reference/output -> 硬件层 Motor_Apply()。
 * ============================================================================
 */

#include <stdint.h>

#include "Board_Config.h"
#include "Pid.h"
#include "Drv_Encoder.h"

/* ---------------------------------------------------------------------------
 * 测速滑动窗口
 * 功能：内环测速对最近 N 拍的计数与时间求和，把低分辨率编码器的量化台阶摊平。
 *       5 ms 单拍 1 个计数 = 27.3 rpm；8 拍（40 ms）后 1 个计数 ≈ 3.4 rpm。
 * 取值：8 拍固定，不做自适应。窗口越长分辨率越高、延迟越大（约窗口一半）。
 * ------------------------------------------------------------------------- */
#define TASK_WINDOW_TICKS        8U       /* 测速窗口拍数：8 × 5 ms = 40 ms */
#if TASK_WINDOW_TICKS < 1U || TASK_WINDOW_TICKS > 32U
#error TASK_WINDOW_TICKS must be 1..32

#endif

/* ---------------------------------------------------------------------------
 * 实测转速低通
 * 功能：窗口已经滤掉大部分量化噪声，这里只压残余；时间常数越大越平滑、
 *       反馈延迟越大。
 * 取值：0.015 s（15 ms）。
 * ------------------------------------------------------------------------- */
#define TASK_SPEED_FILTER_TAU_S  0.015f   /* 实测转速一阶低通时间常数，单位秒 */

/* ---------------------------------------------------------------------------
 * 测速合理性守卫
 * 功能：单拍计数超过物理上限的 1.5 倍即判野值，用上一拍推算值替换；
 *       窗口转速单拍跳变超过 150 rpm 且持续不足 3 拍时保持上一次有效值。
 * 取值：启用；1.5 倍；150 rpm；3 拍。事件计入 count_reject / speed_reject。
 * ------------------------------------------------------------------------- */
#define TASK_SPEED_GUARD_ENABLE     1     /* 1=启用测速守卫，0=关闭 */
#define TASK_COUNT_GUARD_FACTOR     1.5f  /* 单拍计数上限 = 物理上限 × 该系数 */
#define TASK_SPEED_GUARD_STEP_RPM   150.0f /* 允许的单拍转速跳变，单位 rpm */
#define TASK_SPEED_GUARD_HOLD_TICKS 3U    /* 跳变后保持上一有效值的拍数 */
#if TASK_SPEED_GUARD_ENABLE != 0 && TASK_SPEED_GUARD_ENABLE != 1
#error TASK_SPEED_GUARD_ENABLE must be 0 or 1
#endif
#if TASK_SPEED_GUARD_HOLD_TICKS < 1U
#error TASK_SPEED_GUARD_HOLD_TICKS must be >= 1
#endif

/* ---------------------------------------------------------------------------
 * 执行器保护阈值
 * 功能：堵转判定要求控制器确实在驱动（PWM 门槛），以及通信/帧/堵转/方向/时序
 *       各类超时。超时到点即锁存对应故障码。
 * 取值：PWM 门槛 1500（15% 满量程）；通信 500 ms、帧字节间隔 100 ms、
 *       堵转 1500 ms、方向异常 1000 ms；允许周期 1000..15000 µs。
 * ------------------------------------------------------------------------- */
#define TASK_STALL_PWM           1500.0f  /* 堵转判定中的 PWM 门槛，单位 PWM 数值 */
#define TASK_STALL_TIMEOUT_MS    1500U    /* 转速长期低于判定门限的允许时长，毫秒 */
#define TASK_DIRECTION_TIMEOUT_MS 1000U   /* 命令与实测方向相反的允许时长，毫秒 */
#define TASK_MIN_CONTROL_US      1000U    /* 允许的最短控制周期，微秒 */
#define TASK_MAX_CONTROL_US      15000U   /* 允许的最长控制周期，微秒 */

/* 整车状态，枚举值同时用于遥测，顺序不可改。 */
enum
{
    CAR_STOP,   /* 0：停车，允许修改 PID 参数 */
    CAR_RUN,    /* 1：运行，接受转速与开环命令 */
    CAR_FAULT   /* 2：故障，输出已清零，需 STOP 清除 */
};

/* 目标来源，决定“谁在写 target”，不改变执行通路。 */
enum
{
    CAR_MOTION_IDLE = 0,  /* 无目标来源：手动停车后 */
    CAR_MOTION_BT,        /* 蓝牙命令 3 手动调速 */
    CAR_MOTION_RACE       /* 巡线状态机自动发车 */
};

/* 故障码，枚举值同时用于遥测，顺序不可改。 */
enum
{
    CAR_NO_FAULT,         /* 0：无故障 */
    CAR_LINK_FAULT,       /* 1：运行通信超时 */
    CAR_STALL_FAULT,      /* 2：堵转 */
    CAR_DIRECTION_FAULT,  /* 3：转向异常 */
    CAR_RX_FAULT,         /* 4：蓝牙接收异常 */
    CAR_TIMING_FAULT,     /* 5：控制周期异常 */
    CAR_CONTROLLER_FAULT, /* 6：控制器配置或数值异常 */
    CAR_RACE_FAULT,       /* 7：比赛观测失效或动作越界 */
    CAR_OVERSPEED_FAULT   /* 8：测得转速持续过高 */
};

/* 故障记录：每个故障码独立累计；持续条件只在边沿计一次，清除后再次出现会再加一次。 */
#define CAR_FAULT_CODES       9U   /* 故障码 0..8 的记录槽数 */
#define CAR_FAULT_HOLD_TICKS  40U  /* 同一条件持续时不重复计次的保持拍数（40×5ms=200ms） */

typedef struct
{
    uint16_t count;      /* 分离的故障段数，饱和到 0xFFFF */
    uint32_t first_ms;   /* 第一次发生的控制运行时间，毫秒 */
    uint32_t last_ms;    /* 最近一次观察到该故障的时间，毫秒 */
} CarFaultRecord;

/* 整车控制对象：目标、实测、PID、保护计时与蓝牙解析上下文。 */
typedef struct
{
    /* ---------- 双轮串级控制状态 ---------- */
    Car_PidInc_Struct pid_speed[2];   /* 内环速度环：增量式 PID，下标 0 左、1 右 */
    Car_Pid_Struct pid_position[2];   /* 外环位置环：位置式 PID，下标 0 左、1 右 */
    float position[2];        /* 累计位置，单位输出轴圈数（rev） */
    float position_target[2]; /* 外环目标位置，单位 rev，由命令 7 设置 */
    uint8_t outer_enable;     /* 1 = 外环位置环参与控制；由命令 7 运行时切换 */
    float target[2];          /* 目标转速，单位 rpm，下标 0 左、1 右 */
    float reference[2];       /* 内环转速给定，单位 rpm，等于本拍 target */
    float speed[2];           /* 滤波后的实测转速，单位 rpm */
    float output[2];          /* 带符号 PWM 输出，正负号表示方向 */

    /* 开环 PWM 测试（蓝牙命令 6，自动调参用）：绕过两个 PID 直接给输出。 */
    uint8_t open_loop[2];     /* 1 表示该轮处于开环测试 */
    float open_loop_pwm[2];   /* 最近一次开环 PWM 命令，带符号 */
    uint32_t open_loop_us[2]; /* 距最近一次开环命令的时间，微秒 */

    /* 测速滑动窗口：每拍把计数增量与对应时间压入环形缓冲。 */
    int32_t speed_counts[2][TASK_WINDOW_TICKS]; /* 每拍有符号计数增量 */
    uint32_t speed_dt_us[2][TASK_WINDOW_TICKS]; /* 与计数对应的实测周期 */
    uint8_t speed_index[2];   /* 环形缓冲写指针，0..TASK_WINDOW_TICKS-1 */
    uint8_t speed_fill[2];    /* 已填入的有效拍数 */

    /* ---------- 计时与保护，单位均为微秒 ---------- */
    uint32_t link_us;         /* 运行通信看门狗计时 */
    uint32_t stall_us[2];     /* 左右轮堵转计时 */
    uint32_t direction_us[2]; /* 左右轮方向异常计时 */
    uint32_t overspeed_us[2]; /* 左右轮超速计时 */
    uint32_t frame_us;        /* 蓝牙帧字节间隔计时 */

    /* 转速绝对值的慢速滤波结果，用于识别换向时是否仍在减速。 */
    float slow_speed[2];

    /* 测速合理性守卫的中间量。 */
    float speed_guard_raw[2]; /* 上一次接受的窗口转速 */
    uint16_t speed_guard_hold[2]; /* 连续超限拍数 */
    uint16_t count_guard_hold[2]; /* 计数野值连续拍数 */
    float output_last[2];     /* 上一拍实际输出 */
    uint32_t count_reject[2]; /* 计数野值计数（诊断用） */
    uint32_t speed_reject[2]; /* 转速跳变计数（诊断用） */

    uint32_t frames;          /* 进入命令执行阶段的帧数 */

    /* ---------- 状态与协议字段 ---------- */
    uint8_t state;            /* 整车状态，取值见 CAR_STOP/CAR_RUN/CAR_FAULT */
    uint8_t motion;           /* 目标来源，取值见 CAR_MOTION_* */
    uint8_t fault;            /* 最近一次故障码，取值见 CAR_*_FAULT */
    uint32_t timing_fault_us; /* 首次触发故障 5 的实测控制周期 */
    uint8_t result;           /* 最近一条命令或帧的处理结果 */

    /* ---------- 故障记录（记录模式下不停车） ---------- */
    CarFaultRecord fault_log[CAR_FAULT_CODES]; /* 每个故障码独立计数与时间 */
    uint8_t fault_hold[CAR_FAULT_CODES];       /* 边沿保持计数，持续条件不重复计次 */
    uint32_t fault_derate_us;                  /* 故障降额剩余时间，>0 时参考轮速降到 (100-10)% */
    uint32_t uptime_ms;                        /* 控制运行时间，故障时间戳 */

    /* 蓝牙解析上下文（由 Proto_Bluetooth.c 维护）。 */
    uint8_t parse_state;      /* 解析状态机当前状态 0~5 */
    uint8_t cmd;              /* 当前帧的命令号 */
    uint8_t length;           /* 当前帧期望的负载长度 */
    uint8_t index;            /* 已接收的负载字节数 */
    uint8_t crc;              /* 当前帧累计 CRC-8 */
    uint8_t data[7];          /* 负载缓存，最大 7 字节，对应 PID 命令 */
} Car_Core;

/*
 * 函数名称：Car_Init
 * 函数功能：清空控制对象并装载默认参数。
 * 参数说明：c —— 待初始化对象。
 * 返回参数：无。
 * 备注信息：正常进入 STOP；控制器配置失败时直接进入 FAULT。
 */
void Car_Init(Car_Core *c);

/*
 * 函数名称：Car_StartRace
 * 函数功能：由菜单/按键发起自动发车：STOP -> RUN，并把目标来源设为巡线。
 * 参数说明：c —— 控制对象。
 * 返回参数：1 发车成功；0 当前不是 STOP。
 */
int Car_StartRace(Car_Core *c);

/*
 * 函数名称：Car_SetPid
 * 函数功能：仅在 STOP 下改写内环速度环增益（菜单与蓝牙命令 4 共用）。
 * 参数说明：c —— 控制对象；wheel —— 0 左、1 右；kp/ki/kd —— 新增益。
 * 返回参数：0 成功；1 状态不允许；2 参数无效。
 * 备注信息：新参数先在临时对象上校验，全部合法才整体生效；外环不开放。
 */
uint8_t Car_SetPid(Car_Core *c, unsigned wheel, float kp, float ki, float kd);

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
uint8_t Car_SetOuterPid(Car_Core *c, unsigned wheel, float kp, float ki, float kd);

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
uint8_t Car_SetOuter(Car_Core *c, uint8_t enable, float left_step_rev, float right_step_rev);

/*
 * 函数名称：Car_SetOpenLoop
 * 函数功能：进入一轮开环 PWM 测试，绕过两个 PID 直接给指定轮输出。
 * 参数说明：c —— 控制对象；wheel —— 0 左、1 右；pwm —— 带符号 PWM 数值。
 * 返回参数：0 成功；1 不是 RUN；2 轮号或 PWM 无效。
 * 备注信息：首次进入要求双轮目标为零且实测低于 10 rpm；上位机必须每
 *           BT_OPENLOOP_TIMEOUT_MS 刷新一次，否则自动撤去输出。必须架空。
 */
uint8_t Car_SetOpenLoop(Car_Core *c, unsigned wheel, float pwm);

/*
 * 函数名称：Car_LocalStop
 * 函数功能：本地停车：清目标、参考、输出与故障码，回到 STOP。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 */
void Car_LocalStop(Car_Core *c);

/*
 * 函数名称：Car_RaceWheels
 * 函数功能：巡线状态机每拍写入左右轮目标转速（单位 rpm）。
 * 参数说明：c —— 控制对象；rpm —— 两个目标转速，下标 0 左、1 右。
 * 返回参数：无。
 * 备注信息：只在“自动模式 + RUN”下生效；双轮目标为零时清空运动状态。
 */
void Car_RaceWheels(Car_Core *c, const float rpm[2]);

/*
 * 函数名称：Car_Tick
 * 函数功能：用固定 BOARD_PERIOD_MS 周期更新，供仿真或固定周期调用者使用。
 * 参数说明：c —— 控制对象；left_count/right_count —— 本拍有符号计数增量。
 * 返回参数：无。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count);

/*
 * 函数名称：Car_TickDt
 * 函数功能：用实测周期更新测速、控制输出与保护状态。
 * 参数说明：c —— 控制对象；left_count/right_count —— 本拍计数增量；
 *           dt_us —— 与计数对应的实际周期，单位微秒。
 * 返回参数：无。
 * 备注信息：周期超出 TASK_MIN/MAX_CONTROL_US 进入 CAR_TIMING_FAULT；
 *           结果写入 c->output，硬件层负责应用。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count, uint32_t dt_us);

/*
 * 函数名称：Car_Fault
 * 函数功能：记录一次故障。
 * 参数说明：c —— 控制对象；fault —— 非零的 CAR_*_FAULT 故障码。
 * 返回参数：无。
 * 备注信息：CAR_FAULT_STOP_ENABLE=0 时只记录、不改变目标/状态/输出；
 *           =1 时保留传统行为：锁存第一个故障、清输出并进入 CAR_FAULT。
 *           每个故障码独立记录 count/first_ms/last_ms；同一条件持续 200 ms
 *           内只算一次，条件消失后再次出现会再计一次。
 */
void Car_Fault(Car_Core *c, uint8_t fault);

/*
 * 函数名称：Car_ClearFaultLog
 * 函数功能：清空故障记录和最近故障码，不停车、不改目标。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：供菜单“故障记录”页的 OK 键调用；PE10 停车只清最近故障码，
 *           不清历史记录。
 */
void Car_ClearFaultLog(Car_Core *c);

/*
 * 函数名称：Car_ClearMotion
 * 函数功能：清空目标、参考、输出与保护计时（不做状态切换）。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 备注信息：供协议层执行 STOP/ARM 时复用，避免重复实现一份清零逻辑。
 */
void Car_ClearMotion(Car_Core *c);

/*
 * 函数名称：Car_OpenLoopClear
 * 函数功能：撤去两轮开环 PWM 测试状态（超时或状态切换时调用）。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 */
void Car_OpenLoopClear(Car_Core *c);

/* 蓝牙协议头在结构体定义之后引入：它需要 Car_Core 完整类型；
 * 反向包含本文件时由 include guard 直接跳过。 */
#include "Proto_Bluetooth.h"

#endif
