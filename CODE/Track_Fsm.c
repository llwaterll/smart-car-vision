/*
 * ============================================================================
 * 文件名称：Track_Fsm.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 巡线状态机：把“视觉曲率 + 里程 + 航向”变成左右轮目标转速，并负责赛段切换。
 *
 *   赛段（TrackPhase）
 *     TRACK_SEGMENT_TRACK    正常巡线：曲率决定角速度；丢线时沿用上次曲率并降速
 *     TRACK_SEGMENT_GAP      断路：盲走固定低速，靠航向保持直线，连续看到路再回 TRACK
 *     TRACK_SEGMENT_BYPASS   绕障：在进入赛段时的坐标系里跟踪横向偏移，越过后归位
 *     TRACK_SEGMENT_RED      红灯：静止停留 5 s 后放行，12 s 仍未完成判故障
 *     TRACK_SEGMENT_FINISHED 跑完两圈：停车并触发数独收尾
 *     TRACK_SEGMENT_FAULT    赛段异常：由任务层锁存 CAR_RACE_FAULT
 *
 *   数据流（每 5 ms 一拍）
 *     视觉快照 o（可为 NULL）+ 左/右轮本拍米数 dl/dr + 实测周期 dt
 *       -> 里程 ds、航向 dyaw（轮速差与陀螺按 15%/85% 融合）
 *       -> 赛段判定（TRACK/GAP/BYPASS/RED/FINISHED/FAULT）
 *       -> 速度规划（第一圈全速；第二圈保留原比例上限；曲率/丢线/元素/终点减速已关闭）
 *       -> drive(v, omega)：自行车模型算左右轮 rpm，并限制内轮不反转
 *
 * 参数传递约定（重要）
 *   - dl/dr 是“本拍增量”，单位米，不是累计里程；累计由本文件维护。
 *   - dt 是实测周期（秒），1..15.1 ms 之外直接判赛段故障：
 *     说明 5 ms 控制节拍本身已经坏了，继续算没有意义。
 *   - o 为 NULL 表示“这一拍没有可用视觉帧”，不是“视觉故障”：
 *     此时沿用 last_curve 并降速，只有 TRACK_VISION_LOST_STOP_MS 非 0 才停车。
 *   - stationary 由任务层给（停车且双轮转速近似为 0），只用于红灯计时。
 *
 * 函数列表
 *   static float      limit(float x, float bound)
 *   float             Track_ClampRpm(float target_rpm)
 *   void              Track_Start(TrackState *r)
 *   void              Track_StartWithRpm(TrackState *r, float target_rpm)
 *   static TrackDrive drive(float v, float omega)
 *   static void       enter(TrackState *r, TrackPhase phase, uint32_t now)
 *   TrackDrive        Track_Step(...)
 * ============================================================================
 */
#include "Track_Fsm.h"
#include "Board_Config.h"
#include <math.h>
#include <string.h>

/*
 * 函数名称：limit
 * 函数功能：把浮点数限制到 [-bound, +bound]。
 * 参数说明：x     —— 待限幅的值；
 *           bound —— 正的上限（调用方保证 > 0）。
 * 返回参数：限幅后的值。
 * 备注信息：只用于角速度和横向偏移这类对称量，不处理 NaN：
 *           上游的视觉与陀螺数据在进入本模块前已经做过有效性检查。
 */
static float limit(float x, float bound) {
    return fmaxf(-bound, fminf(bound, x));
}

/*
 * 函数名称：Track_Start
 * 函数功能：发车时清零巡线状态。
 * 参数说明：r —— 巡线状态对象。
 * 返回参数：无。
 * 备注信息：last_gap / last_action 置 -100 m 而不是 0：
 *           这两个量用来“记住上一次断路/绕障发生在多少米处”，
 *           初值必须足够负，否则刚发车就会被 0.75 圈的“去重距离”挡住。
 */
void Track_Start(TrackState *r) {
    memset(r, 0, sizeof(*r));
    r->lap = 1;
    r->target_rpm = TRACK_MPS_TO_RPM(0.55f);   /* 改动前的默认巡航速度 */
    r->last_gap = r->last_action = -100.0f;
}

/*
 * 函数名称：Track_ClampRpm
 * 函数功能：把菜单设定转速夹到比赛允许范围。
 * 参数说明：target_rpm —— 待限制的转速；NaN/Inf 回退到默认值。
 * 返回参数：夹紧后的转速，rpm。
 * 备注信息：上限同时受 TRACK_MAX_RPM 和板级 90% 转速限幅约束，
 *           保证“菜单允许给的目标”不会高于保护判据。
 */
float Track_ClampRpm(float target_rpm) {
    float hi = fminf((float)TRACK_MAX_RPM, BOARD_RPM_LIMIT);

    if (!isfinite(target_rpm)) return (float)TRACK_DEFAULT_RPM;
    if (target_rpm < (float)TRACK_MIN_RPM) return (float)TRACK_MIN_RPM;
    if (target_rpm > hi) return hi;
    return target_rpm;
}

/*
 * 函数名称：Track_StartWithRpm
 * 函数功能：按菜单设定转速发车。
 * 参数说明：r          —— 巡线状态对象；
 *           target_rpm —— 巡航转速，超出范围会被夹住。
 * 返回参数：无。
 * 备注信息：比赛发车统一走这里；Track_Start() 只保留旧默认值给单元测试。
 */
void Track_StartWithRpm(TrackState *r, float target_rpm) {
    Track_Start(r);
    r->target_rpm = Track_ClampRpm(target_rpm);
}

/*
 * 函数名称：drive
 * 函数功能：由底盘线速度 v 与角速度 omega 计算左右轮目标转速。
 * 参数说明：v     —— 底盘前进线速度，单位 m/s，负值表示后退；
 *           omega —— 底盘角速度，单位 rad/s，逆时针（左转）为正。
 * 返回参数：左右轮目标转速（rpm），下标 0 左、1 右。
 * 备注信息：1) factor 把 m/s 换算成 rpm：60 / (2πR)；
 *           2) half 是差速项，限幅到 |v| 以内，保证内轮最多降到 0，绝不反转：
 *              内轮反转会被地面拖着走，触发任务层的堵转/方向保护直接停车；
 *           3) 最后按 BOARD_RPM_LIMIT 等比缩放，保持左右比例不变，
 *              避免“只砍一侧”把车带偏。
 */
static TrackDrive drive(float v, float omega) {
    TrackDrive out;                                        /* 左右轮 rpm 结果 */
    float factor = 60.0f / (6.2831853f * TRACK_RADIUS_M);  /* m/s -> rpm 的换算系数 */
    float half = 0.5f * TRACK_WIDTH_M * omega;             /* 单轮相对底盘的速度增量，m/s */
    float peak;                                            /* 两轮中较大的转速绝对值 */

    /* 曲率尖峰可能要求内轮反转，而车体仍在前进：内轮会被地面拖住并触发保护，
     * 所以把差速项压到 |half| <= |v|，内轮最多到 0。 */
    half = limit(half, fabsf(v));
    out.rpm[0] = (v - half) * factor;
    out.rpm[1] = (v + half) * factor;

    /* 等比缩放到转速上限：只缩不放，避免弯道外侧轮把整条曲线拉出限幅。 */
    peak = fmaxf(fabsf(out.rpm[0]), fabsf(out.rpm[1]));
    if (peak > BOARD_RPM_LIMIT) {
        out.rpm[0] *= BOARD_RPM_LIMIT / peak;
        out.rpm[1] *= BOARD_RPM_LIMIT / peak;
    }
    return out;
}

/*
 * 函数名称：enter
 * 函数功能：记录进入某个赛段时的里程、航向和计时，作为该赛段的参考原点。
 * 参数说明：r     —— 巡线状态；
 *           phase —— 目标赛段；
 *           now   —— 当前毫秒时间。
 * 返回参数：无。
 * 备注信息：entry/entry_yaw 是“本赛段的局部坐标系原点”，GAP 与 BYPASS 的
 *           相对位移都相对它计算；last_road_ms 一起刷新，使每个盲走赛段
 *           重新开始计“丢线时间”。clear_frames 清零，防止把上一赛段的
 *           连续有效帧带到新赛段。
 */
static void enter(TrackState *r, TrackPhase phase, uint32_t now) {
    r->phase = phase;
    r->entry = r->distance;
    r->entry_yaw = r->yaw;
    r->offset = 0;
    r->entered_ms = now;
    r->last_road_ms = now;
    r->clear_frames = 0;
}

/*
 * 函数名称：Track_Step
 * 函数功能：推进一拍巡线，返回左右轮目标转速。
 * 参数说明：r         —— 巡线状态（里程/航向/赛段都在这里累积）；
 *           o         —— 最新道路观测快照，NULL 表示本拍没有可用视觉帧；
 *           dl/dr     —— 左/右轮本拍前进距离，单位米（增量，不是累计值）；
 *           dt        —— 本拍实测周期，单位秒；
 *           gyro      —— 陀螺 Z 轴角速度，单位 rad/s，逆时针为正；
 *           gyro_ok   —— 陀螺数据是否新鲜（0 时只用轮速差推航向）；
 *           stationary—— 整车是否静止，仅红灯赛段使用；
 *           now       —— 当前毫秒时间，用于各类超时。
 * 返回参数：左右轮目标转速（rpm）；RED/FINISHED/FAULT 赛段返回 0/0。
 * 备注信息：本函数不改电机、不读硬件，所有副作用都写进 r，便于单测复现。
 *           周期非法时直接把赛段置 FAULT：控制节拍坏了，巡线再算也没有意义。
 */
TrackDrive Track_Step(TrackState *r, const CarVisionObservation *o, float dl, float dr, float dt, float gyro, uint8_t gyro_ok, uint8_t stationary, uint32_t now) {
    TrackDrive stop = {{0,0}};                 /* 需要停车时直接返回的 0/0 */
    float ds = 0.5f * (dl + dr);               /* 本拍底盘前进距离：两轮平均，米 */
    float dyaw = (dr - dl) / TRACK_WIDTH_M;    /* 轮速差推算的本拍航向变化，rad */
    float s, curve, speed, heading, target, error, along; /* 赛段内里程/曲率/速度/航向/目标偏移/误差 */
    float v_target, lap_v;                     /* 菜单巡航速度/第二圈上限 */
    int road, measured_road, new_frame, in_window; /* 看到路/实测路/新帧/断路窗口 */
    int action_ready, yellow;                  /* 元素动作条件/黄灯候选 */

    /* 完赛或已判故障后不再推进，重复调用保持停车输出。 */
    if (r->phase == TRACK_SEGMENT_FINISHED || r->phase == TRACK_SEGMENT_FAULT) return stop;

    /* 控制周期越界（<1 ms 或 >15.1 ms）说明 5 ms 节拍已经失效，直接判赛段故障。 */
    if (!(dt >= 0.001f && dt <= 0.0151f)) {
        r->fault_reason = TRACK_FAULT_PERIOD;
        r->phase = TRACK_SEGMENT_FAULT;
        return stop;
    }

    /* 航向融合：陀螺短时更可信，给 85%；轮速差在丢陀螺时兜底，给 15%。
     * 陀螺失效（gyro_ok=0）或读数不是有限值时，整拍只用轮速差。 */
    if (gyro_ok && isfinite(gyro)) dyaw = 0.85f * gyro * dt + 0.15f * dyaw;
    r->yaw += dyaw;
    r->distance += ds;

    /* 圈数按单圈长度分段；到终点里程立即完赛（本拍就返回 0/0）。 */
    r->lap = r->distance >= TRACK_LAP_M ? 2 : 1;
    if (r->distance >= TRACK_FINISH_M) {
        r->phase = TRACK_SEGMENT_FINISHED;
        return stop;
    }

    /* ---- 视觉帧解析：区分“没有新帧”“有新帧但没看到路”两种情况 ---- */
    new_frame = 0;
    road = 0;
    measured_road = 0;
    if (o) {
        /* 序号变化才算新帧：同一帧被多次调用不能重复触发赛段判定。 */
        new_frame = !r->have_sequence || o->sequence != r->last_sequence;
        r->last_sequence = o->sequence;
        r->have_sequence = 1;

        /* 只有“曲率有效 + 置信度 ≥ 0.35”才算看到路；否则仍按丢线处理。
         * 单边补线的估计曲率只能用于普通巡线转向，不能用于发车、断路接回
         * 或刷新丢线停车计时，所以另设 measured_road。 */
        road = (o->valid_mask & VISION_CURVATURE_VALID) && o->confidence >= 0.35f;
        measured_road = road && !(o->valid_mask & VISION_CURVATURE_ESTIMATED);
        if (measured_road || (road && r->phase == TRACK_SEGMENT_TRACK)) {
            r->last_curve = limit(o->curvature_inv_m, 15.0f);  /* 曲率限幅 15 /m，防脏数据 */
            r->have_curve = 1;
        }
        if (measured_road) {
            r->last_measured_curve = r->last_curve;
            r->last_road_ms = now;
        }
        /* 元素去重：同一元素在新帧里反复出现时，只有离开 0.8 m 后才允许再次触发。 */
        if (new_frame && o->element != r->held_element &&
            r->distance - r->last_action > 0.8f) r->held_element = 0;
    }

    r->degraded = 0;   /* 只有正常巡线段才会置“丢线降速”提示 */

    /* ---- TRACK 段：根据视觉元素决定是否切到 GAP / BYPASS / RED ---- */
    if (r->phase == TRACK_SEGMENT_TRACK) {
        along = r->distance - (r->lap - 1) * TRACK_LAP_M;   /* 本圈内里程 */
        /* 断路入口窗口：TRACK_GAP_ENTRY_M=0 表示完全由视觉判定，不限位置。 */
        in_window = TRACK_GAP_ENTRY_M <= 0 || fabsf(along - TRACK_GAP_ENTRY_M) < 1.0f;

        /* 元素动作条件：必须是实测路 + 有距离 + 置信度足够 + 不是刚处理过的元素。
         * 单边估计曲率可以转向，但不能触发绕障/红灯这类实体动作。 */
        action_ready = measured_road && (o->valid_mask & VISION_DISTANCE_VALID) &&
                       o->element != r->held_element && o->confidence >= 0.50f;
        yellow = action_ready && o->element == VISION_ELEMENT_YELLOW &&
                 o->distance_m <= 0.80f;

        /* 黄灯连续确认：只有新帧才累加；条件中断、或两帧间隔超过视觉时效，
         * 立即清零。单帧误检不会让车提前切进绕障。 */
        if (!yellow) {
            r->yellow_frames = 0;
        } else if (new_frame) {
            if ((uint32_t)(now - r->last_yellow_ms) > VIS_MAX_AGE_MS) {
                r->yellow_frames = 0;
            }
            r->last_yellow_ms = now;
            if (r->yellow_frames < 0xFFU) r->yellow_frames++;
        }

        if (!road) {
            /* 看不到路时才可能是断路：要求视觉给出 GAP 元素、位置窗口正确、
             * 距上次断路超过 0.75 圈、且最近确有直线（≥0.10 m），
             * 三者一起排除“普通弯道丢线”被误判成断路。 */
            if (o && o->element == VISION_ELEMENT_GAP && in_window &&
                r->distance - r->last_gap > TRACK_LAP_M * 0.75f &&
                (TRACK_GAP_ENTRY_M > 0 || r->straight_m >= 0.10f)) {
                enter(r, TRACK_SEGMENT_GAP, now);
                r->last_gap = r->distance;
            }
        } else if (action_ready) {
            /* 绕障：黄灯连续 3 帧确认且距离 ≤0.80 m。obstacle_m 取测量值
             * （最小 0.08 m），goal_m 是“越过障碍再走 0.42 m”的总目标里程。 */
            if (yellow && r->yellow_frames >= TRACK_YELLOW_CONFIRM_FRAMES) {
                enter(r, TRACK_SEGMENT_BYPASS, now);
                r->obstacle_m = fmaxf(0.08f, o->distance_m);
                r->goal_m = r->obstacle_m + 0.42f;
            } else if (o->element == VISION_ELEMENT_RED && r->lap == 2 &&
                       o->distance_m <= 0.20f) {
                /* 红灯只在第二圈、距离 ≤0.20 m 时生效。 */
                enter(r, TRACK_SEGMENT_RED, now);
                r->timing_stop = 0;
            }
            /* 真的切走了才记“已处理该元素”，防止同一元素连续触发。 */
            if (r->phase != TRACK_SEGMENT_TRACK) {
                r->held_element = o->element;
                r->last_action = r->distance;
            }
        }
    }

    s = r->distance - r->entry;   /* 进入当前赛段后走过的距离，米 */

    /* ---- RED 段：静止 5 s 放行，12 s 未完成判故障 ---- */
    if (r->phase == TRACK_SEGMENT_RED) {
        if (!stationary) r->timing_stop = 0;                    /* 车还在动：重新计时 */
        else if (!r->timing_stop) { r->timing_stop = 1; r->still_ms = now; }
        else if (now - r->still_ms >= 5000U) {
            r->phase = TRACK_SEGMENT_TRACK;                     /* 停够 5 s：回到巡线 */
            r->last_road_ms = now;
        }
        if (now - r->entered_ms > 12000U) {
            r->fault_reason = TRACK_FAULT_RED;
            r->phase = TRACK_SEGMENT_FAULT;
        }
        return stop;                                            /* 红灯期间始终 0/0 */
    }

    heading = r->yaw - r->entry_yaw;   /* 相对赛段入口的航向偏差，rad */

    /* ---- GAP 段：盲走；1.30 m 后连续 3 个新帧看到路才算通过 ---- */
    if (r->phase == TRACK_SEGMENT_GAP) {
        /* 超过 1.95 m 或 12.5 s 还没出来：说明断路比预期长，判故障。 */
        if (s > 1.95f || now - r->entered_ms > 12500U) {
            r->fault_reason = TRACK_FAULT_GAP;
            r->phase = TRACK_SEGMENT_FAULT;
            return stop;
        }
        /* clear_frames 只在“新帧且 s≥1.30 m 且看到路”时累加，任何一帧不满足即清零，
         * 避免摄像头的单帧误检就提前结束盲走。 */
        if (new_frame) {
            if (s >= 1.30f && measured_road) ++r->clear_frames;
            else r->clear_frames = 0;
        }
        if (r->clear_frames < 3) return drive(0.22f, limit(-3.0f * heading, 1.2f));
        r->phase = TRACK_SEGMENT_TRACK;
        r->straight_m = 0;
        r->last_road_ms = now;
    }

    /* ---- BYPASS 段：在入口坐标系里跟踪横向偏移 ---- */
    if (r->phase == TRACK_SEGMENT_BYPASS) {
        /* 用实测前进距离积分横向/纵向位移：
         *   offset  += ds·sin(heading)：横向让开障碍
         *   goal_m  -= ds·cos(heading)：剩余纵向里程
         * 用实测值而不是规划值积分，推车/打滑时也能对上真实位置。 */
        r->offset += ds * sinf(heading);
        r->goal_m -= ds * cosf(heading);
        along = r->obstacle_m + 0.42f - r->goal_m;   /* 已经越过障碍多少米 */

        /* 12 s 未完成、航向偏差 >1.55 rad、或冲过头 0.25 m：都判故障。 */
        if (now - r->entered_ms > 12000U || fabsf(heading) > 1.55f || r->goal_m < -0.25f) {
            r->fault_reason = TRACK_FAULT_BYPASS;
            r->phase = TRACK_SEGMENT_FAULT;
            return stop;
        }

        /* 目标横向偏移；越障后按剩余里程线性收回，避免“贴着障碍回正”刮到。 */
        target = TRACK_BYPASS_SIDE * TRACK_BYPASS_OFFSET_M;
        if (along > r->obstacle_m + 0.10f) target *= fmaxf(0, r->goal_m / 0.32f);

        /* 回到原线：纵向到位、看到路、横向偏差 <0.075 m 三条同时满足。 */
        if (r->goal_m <= 0.02f && measured_road && fabsf(r->offset) < 0.075f) {
            r->phase = TRACK_SEGMENT_TRACK;
            r->last_road_ms = now;
        } else {
            /* 航向式转向：atan(5.5·横向误差) 产生期望航向，再乘 4.5 得到角速度，
             * 角速度限幅 2.5 rad/s，低速 0.18 m/s 保证有足够时间回正。 */
            error = atanf(5.5f * (target - r->offset));
            return drive(0.18f, limit(4.5f * (error - heading), 2.5f));
        }
    }

    /* ---- 正常巡线：丢线只降速不停车，除非显式配置了停车超时 ---- */
    r->degraded = r->phase == TRACK_SEGMENT_TRACK && !measured_road;
#if TRACK_VISION_LOST_STOP_MS > 0
    if (r->degraded && r->have_curve &&
        now - r->last_road_ms >= TRACK_VISION_LOST_STOP_MS) {
        r->fault_reason = TRACK_FAULT_VISION;
        r->phase = TRACK_SEGMENT_FAULT;
        return stop;
    }
#endif
    curve = r->last_curve;
    /* 直线累计只看实测曲率：单边估计不能拿来确认断路入口。
     * |曲率| < 1.75 /m 认为在直道，一进弯立刻清零。 */
    r->straight_m = fabsf(r->last_measured_curve) < 1.75f ? r->straight_m + fmaxf(0,ds) : 0;

    /* 菜单巡航转速换算成 v_target。
     * 按当前需求：第一圈直接全速；第二圈保留原来的比例上限。
     * 曲率限速、视觉丢失/单边估计 0.18 m/s、低置信度/环岛/十字 0.24 m/s、
     * 红障 0.20 m/s 和终点前包络都已移除，不再主动减速。 */
    v_target = (isfinite(r->target_rpm) && r->target_rpm > 0.0f)
             ? TRACK_RPM_TO_MPS(Track_ClampRpm(r->target_rpm)) : 0.55f;
    lap_v = (r->lap == 1) ? v_target
          : fminf(v_target, fmaxf(0.32f, v_target * (0.32f / 0.55f)));
    speed = lap_v;

    /* 角速度 = 速度 × 曲率（自行车模型），再交给 drive() 变成左右轮 rpm。
     * drive() 仍会把单轮 rpm 夹到 BOARD_RPM_LIMIT，内轮最多到 0。 */
    return drive(speed, speed * curve);
}
