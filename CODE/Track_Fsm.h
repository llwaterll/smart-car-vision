#ifndef TRACK_FSM_H
#define TRACK_FSM_H

/*
 * ============================================================================
 * 文件名称：Track_Fsm.h
 * ============================================================================
 * 模块功能：巡线状态机（赛段切换 + 曲率跟随）的接口与赛场几何参数。
 *
 *   输入：视觉道路观测（曲率、元素、距离）、左右轮里程、陀螺角速度、时间；
 *   输出：左右轮目标转速（rpm），交给 Task_Fsm 的 Car_RaceWheels()。
 *
 *   运行视角
 *     菜单按 OK 发车 -> Track_StartWithRpm() 清零里程/航向并写入菜单转速
 *     -> 每 5 ms Track_Step() -> 跑满 TRACK_FINISH_M 后进入
 *     TRACK_SEGMENT_FINISHED，任务层停车并调用数独求解。
 * ============================================================================
 */

#include <stdint.h>

#include "Proto_Vision.h"

/* ---------------------------------------------------------------------------
 * 赛场几何（必须实车标定，单位：米）
 * 功能：轮径、轮距、单圈长度、绕障侧与偏移、终点里程。
 * 取值：轮径 0.0325（65 mm 直径，按载荷形变复核）；轮距 0.173
 *       （外 200 mm + 内 146 mm 取中）；单圈 55.061；绕障左 +1、偏移 0.18；
 *       终点 = 2 圈。
 * 注意：这些值直接决定速度换算与绕障轨迹，改之前必须实测。
 * ------------------------------------------------------------------------- */
#define TRACK_RADIUS_M           0.0325f  /* 驱动轮半径，米 */
#define TRACK_WIDTH_M            0.173f   /* 左右轮中心距（轮距），米 */
#define TRACK_LAP_M              55.061f  /* 单圈赛道长度，米 */
#define TRACK_GAP_ENTRY_M        0.0f     /* 断路入口里程；0=完全由视觉判定 */
#define TRACK_FINISH_M           (2.0f * TRACK_LAP_M) /* 终点里程：两圈 */
#define TRACK_BYPASS_SIDE        1.0f     /* 绕障方向：+1 左、-1 右 */
#define TRACK_BYPASS_OFFSET_M    0.18f    /* 绕障横向偏移，米 */
#define TRACK_YELLOW_CONFIRM_FRAMES 3U    /* 黄灯连续确认帧数，防单帧误检 */

/* ---------------------------------------------------------------------------
 * 比赛转速（与原作者菜单保持一致）
 * 功能：把“菜单里设定的巡航转速”换算成底盘线速度，并作为速度规划上限。
 * 取值：默认 300 rpm（约 1.02 m/s）；范围 20..900 rpm；单按 ±5、长按 ±20。
 * 注意：Track_Start() 内部仍按 0.55 m/s 初始化，便于单元测试复现旧行为；
 *       比赛发车走 Track_StartWithRpm()，使用菜单设定值。
 * ------------------------------------------------------------------------- */
#define TRACK_DEFAULT_RPM        300U     /* 默认巡航转速，rpm */
#define TRACK_MIN_RPM            20U      /* 菜单可设最低转速，rpm */
#define TRACK_MAX_RPM            900U     /* 菜单可设最高转速，rpm（与 90% 限幅一致） */
#define TRACK_RPM_STEP_FINE      5U       /* 单按 UP/DOWN 的步进，rpm */
#define TRACK_RPM_STEP_COARSE    20U      /* 长按连发的步进，rpm */
#define TRACK_RPM_TO_MPS(rpm)    ((float)(rpm) * (6.2831853f * TRACK_RADIUS_M / 60.0f))
#define TRACK_MPS_TO_RPM(mps)    ((float)(mps) * (60.0f / (6.2831853f * TRACK_RADIUS_M)))

/* ---------------------------------------------------------------------------
 * 视觉丢失策略
 * 功能：没有新帧或曲率无效时不再判故障，沿用最后一次接受的曲率继续行驶。
 * 取值：当前全速模式不再用 TRACK_HOLD_SPEED_MPS 降速，该宏保留作旧测试/
 *       以后恢复用；TRACK_VISION_LOST_STOP_MS=0 表示不因丢线停车，
 *       改成非 0 后在 TRACK 段连续丢失该毫秒数即判赛段故障（断路赛段豁免）。
 * ------------------------------------------------------------------------- */
#define TRACK_HOLD_SPEED_MPS     0.18f    /* 保留的旧“丢线保持速度”，当前不参与规划 */
#define TRACK_VISION_LOST_STOP_MS 0U      /* 0=不因丢线停车；非 0=丢失超时判故障 */

/* ---------------------------------------------------------------------------
 * 速度平滑（默认关闭）
 * 功能：自动模式下视觉每帧速度可能跳变，需要时给每拍速度指令加斜率限制。
 * 取值：0 表示关闭（直接跟随曲率规划）；>0 为每拍允许的最大变化，单位 rpm。
 * 注意：这是可选的补偿手段，主流程不依赖它。
 * ------------------------------------------------------------------------- */
#define TRACK_FRAME_SLEW_RPM     0.0f     /* 0=关闭速度斜率限制；>0=每拍最大变化 rpm */

/* 赛段枚举，顺序参与菜单显示与遥测，不可随意改。 */
typedef enum {
    TRACK_SEGMENT_TRACK,     /* 0：正常巡线 */
    TRACK_SEGMENT_GAP,       /* 1：断路盲走 */
    TRACK_SEGMENT_BYPASS,    /* 2：绕障 */
    TRACK_SEGMENT_RED,       /* 3：红灯停车 */
    TRACK_SEGMENT_FINISHED,  /* 4：完赛 */
    TRACK_SEGMENT_FAULT      /* 5：赛段故障 */
} TrackPhase;

/* 赛段故障原因，顺序与菜单“故障 7”显示一致。 */
typedef enum {
    TRACK_FAULT_NONE = 0,       /* 无赛段故障 */
    TRACK_FAULT_PERIOD,         /* 控制周期异常 */
    TRACK_FAULT_VISION,         /* 视觉丢失超时 */
    TRACK_FAULT_GAP,            /* 断路超限 */
    TRACK_FAULT_BYPASS,         /* 绕障超限 */
    TRACK_FAULT_RED             /* 红障超时 */
} TrackFaultReason;

/* 巡线状态：里程/航向积分、赛段入口快照与视觉保持量。 */
typedef struct {
    TrackPhase phase;           /* 当前赛段 */
    TrackFaultReason fault_reason; /* 进入 FAULT 的原因，供菜单显示 */
    float distance, yaw;        /* 累计里程（米）与航向（弧度） */
    float entry, entry_yaw;     /* 进入当前赛段时的里程与航向 */
    float offset, obstacle_m, goal_m; /* 绕障：横向偏移、障碍距离、剩余目标里程 */
    float last_gap, last_action;      /* 上次断路/绕障动作的里程，用于去重 */
    float straight_m;           /* 连续直线里程，用于确认断路入口 */
    float last_curve;           /* 最后一次接受的曲率，1/m（允许单边估计） */
    float last_measured_curve;  /* 最后一次实测曲率，1/m（不含单边估计） */
    float target_rpm;           /* 菜单设定的巡航转速，rpm；<=0 时回退到 0.55 m/s */
    uint32_t entered_ms, still_ms, last_road_ms; /* 赛段进入/静止/最近有效路况时间 */
    uint32_t last_yellow_ms;    /* 最近一次黄灯确认帧的时间，毫秒 */
    uint16_t last_sequence;     /* 视觉帧序号，用于识别“新帧” */
    uint8_t lap, clear_frames, held_element, have_sequence, timing_stop; /* 圈数/断路确认/元素去重/序号/停车计时 */
    uint8_t yellow_frames;      /* 连续黄灯确认帧数，达到阈值才进绕障 */
    uint8_t degraded, have_curve; /* 丢线降速标志 / 是否已有曲率 */
} TrackState;

/* 巡线输出：左右轮目标转速（rpm）。 */
typedef struct { float rpm[2]; } TrackDrive;

/*
 * 函数名称：Track_Start
 * 函数功能：发车时清空巡线状态：里程、航向、赛段和视觉保持全部复位。
 * 参数说明：r —— 巡线状态对象。
 * 返回参数：无。
 * 备注信息：默认巡航速度按 0.55 m/s 初始化，与改动前的速度规划一致；
 *           比赛发车使用 Track_StartWithRpm() 覆盖为菜单设定值。
 */
void Track_Start(TrackState *r);

/*
 * 函数名称：Track_StartWithRpm
 * 函数功能：按菜单设定转速发车。
 * 参数说明：r          —— 巡线状态对象；
 *           target_rpm —— 巡航转速，超出 TRACK_MIN_RPM..TRACK_MAX_RPM 会被夹住。
 * 返回参数：无。
 */
void Track_StartWithRpm(TrackState *r, float target_rpm);

/*
 * 函数名称：Track_ClampRpm
 * 函数功能：把转速限制到比赛允许范围，并把剩余 90% 限幅交给 drive()。
 * 参数说明：target_rpm —— 待限制的转速，允许 NaN（回退到默认值）。
 * 返回参数：夹紧后的转速，单位 rpm。
 */
float Track_ClampRpm(float target_rpm);

/*
 * 函数名称：Track_Step
 * 函数功能：推进一拍巡线：积分里程/航向、判定赛段、规划速度与转向。
 * 参数说明：r        —— 巡线状态；
 *           o        —— 最新道路观测，可为 NULL（无新帧）；
 *           left_m   —— 左轮本拍前进距离，米；
 *           right_m  —— 右轮本拍前进距离，米；
 *           dt       —— 本拍周期，秒；
 *           gyro     —— 陀螺 Z 轴角速度，rad/s；
 *           gyro_ok  —— 陀螺数据是否有效；
 *           stationary —— 整车是否静止（红灯停车判定用）；
 *           now_ms   —— 当前毫秒时间。
 * 返回参数：左右轮目标转速；赛段为 RED/FINISHED/FAULT 时返回 0/0。
 * 备注信息：dt 超出 1..15.1 ms 直接判赛段故障（控制节拍本身已经坏了）。
 */
TrackDrive Track_Step(TrackState *r, const CarVisionObservation *o, float left_m, float right_m, float dt, float gyro, uint8_t gyro_ok, uint8_t stationary, uint32_t now_ms);

#endif
