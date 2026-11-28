/*
 * ============================================================================
 * 文件名称：Menu_Task.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 板端菜单的全部逻辑，界面只依赖 MenuView 快照，不直接读控制对象：
 *   1. Menu_Init()          ：建立组/项环形链表，菜单回到组页；
 *   2. Menu_StartCondition()：能否发车的 10 种条件 + 连续 3 帧有效赛道确认；
 *   3. Menu_Key()           ：沿链表导航、增益编辑、动作返回；
 *   4. Menu_ShowResult()    ：跳转到数独结果页；
 *   5. Menu_DrawRow()       ：按行渲染，每行只算自己需要的字段。
 *
 * 设计要点
 *   - 组和组内项都是环形双向链表（MenuNode 的 prev/next）。UP/DOWN 只沿
 *     prev/next 走一步，不按数组长度取模，也没有“走到头”的特殊分支；
 *   - 节点表同时保存显示文字，绘制和按键逻辑共用同一份数据，
 *     不会出现“显示的和实际执行的不是同一项”；
 *   - 控制中断只拿 Menu_Key() 的返回值，不在这里发车或写 Flash；
 *   - 增益编辑用“放大 100 倍的整数”做候选值，避免浮点累加误差；
 *   - 发车要求 START_READY：停车、静止、没有待处理数独、视觉路况有效、
 *     且视觉会话与上一场不同（防止一发车就吃上一场的旧帧）。
 *
 * 函数列表
 *   static void    menu_link(void)
 *   static void    menu_ensure_linked(void)
 *   static void    menu_sanitize(MenuState *m)
 *   static const char *node_name(uint8_t node)
 *   void           Menu_Init(MenuState *m)
 *   StartCondition Menu_StartCondition(...)
 *   uint16_t       Menu_GetRaceRpm(const MenuState *m)
 *   void           Menu_Disarm(MenuState *m)
 *   MenuAction     Menu_PollArmed(MenuState *m, StartCondition start)
 *   MenuAction     Menu_Key(...)
 *   void           Menu_ShowResult(MenuState *m)
 *   void           Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
 * ============================================================================
 */
#include "Menu_Task.h"
#include "Drv_Lcd.h"
#include <stdio.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * 菜单节点
 * 功能：一个节点既可以是“组”，也可以是“组内项”；文字、归属和链表指针
 *       放在同一张表里。
 * 布局：组节点固定为 0..MENU_GROUPS-1，且组节点号 == MenuGroup 值；
 *       组 g 的第 i 项节点号 = s_nodes[g].first + i。
 * ------------------------------------------------------------------------- */
typedef struct {
    const char *name;       /* 显示文字（菜单项英文名集中在这里） */
    uint8_t group;          /* 项所在组；组节点就是自己的组号 */
    uint8_t item;           /* 组内项号；组节点为 0 */
    uint8_t first;          /* 组节点：第一项节点号；项节点不用 */
    uint8_t count;          /* 组节点：项数；项节点不用 */
    uint8_t prev;           /* 环形链表前一个节点 */
    uint8_t next;           /* 环形链表后一个节点 */
} MenuNode;

enum {
    NODE_RACE_BASE    = MENU_GROUPS,             /* 4：Race 第一项 */
    NODE_SENSORS_BASE = NODE_RACE_BASE + 4,      /* 8：Sensors 第一项 */
    NODE_LEFT_BASE    = NODE_SENSORS_BASE + 4,   /* 12：左轮 PID 第一项 */
    NODE_RIGHT_BASE   = NODE_LEFT_BASE + 4,      /* 16：右轮 PID 第一项 */
    MENU_NODE_COUNT   = NODE_RIGHT_BASE + 4      /* 20：节点总数 */
};

static MenuNode s_nodes[MENU_NODE_COUNT] = {
    /* 组节点（0..3），first/count 指向该组第一项和项数 */
    {"比赛控制",       MENU_RACE,      0, NODE_RACE_BASE,    4, 0, 0},
    {"传感器监控",     MENU_SENSORS,   0, NODE_SENSORS_BASE, 4, 0, 0},
    {"左轮内环PID",    MENU_LEFT_PID,  0, NODE_LEFT_BASE,    4, 0, 0},
    {"右轮内环PID",    MENU_RIGHT_PID, 0, NODE_RIGHT_BASE,   4, 0, 0},
    /* Race 项（4..7） */
    {"发车启动",       MENU_RACE, 0, 0, 0, 0, 0},
    {"比赛状态",       MENU_RACE, 1, 0, 0, 0, 0},
    {"数独结果",       MENU_RACE, 2, 0, 0, 0, 0},
    {"比赛转速设置",   MENU_RACE, 3, 0, 0, 0, 0},
    /* Sensors 项（8..11） */
    {"左右轮转速",     MENU_SENSORS, 0, 0, 0, 0, 0},
    {"陀螺仪 MPU6050", MENU_SENSORS, 1, 0, 0, 0, 0},
    {"视觉数据",       MENU_SENSORS, 2, 0, 0, 0, 0},
    {"故障记录",       MENU_SENSORS, 3, 0, 0, 0, 0},
    /* 左轮内环 PID 项（12..15） */
    {"比例 Kp",           MENU_LEFT_PID, 0, 0, 0, 0, 0},
    {"积分 Ki",           MENU_LEFT_PID, 1, 0, 0, 0, 0},
    {"微分 Kd",           MENU_LEFT_PID, 2, 0, 0, 0, 0},
    {"保存参数到Flash",   MENU_LEFT_PID, 3, 0, 0, 0, 0},
    /* 右轮内环 PID 项（16..19） */
    {"比例 Kp",           MENU_RIGHT_PID, 0, 0, 0, 0, 0},
    {"积分 Ki",           MENU_RIGHT_PID, 1, 0, 0, 0, 0},
    {"微分 Kd",           MENU_RIGHT_PID, 2, 0, 0, 0, 0},
    {"保存参数到Flash",   MENU_RIGHT_PID, 3, 0, 0, 0, 0},
};

/* 非节点表数据：发车原因、赛段、保存状态只用于显示，不参与导航。
 * conditions[] 必须与 StartCondition 的枚举顺序一一对应。 */
static const char *const conditions[] = {
    "条件就绪,可发车",      /* START_READY */
    "请先停车或清故障",    /* START_STOP_FIRST */
    "车轮仍在转动",        /* START_MOVING */
    "正在求解数独结果",    /* START_RESULT */
    "视觉功能已关闭",      /* START_NO_VISION */
    "等待视觉握手",        /* START_NO_SESSION */
    "视觉无新帧",          /* START_NO_FRAME */
    "视觉协议错误",        /* START_VISION_ERROR */
    "等待有效赛道",        /* START_NO_ROAD */
    "等待视觉新会话"       /* START_NEW_SESSION */
};
static const char *const phases[] = {"巡线","断路","绕障","红障停","完赛","故障"};
/* 故障记录页的简称，下标与 CAR_*_FAULT 一致。 */
static const char *const fault_names[CAR_FAULT_CODES] = {
    "无故障", "通信超时", "堵转", "方向异常", "接收异常",
    "控制周期", "控制器", "比赛故障", "超速"
};
enum { FB_NONE, FB_APPLIED, FB_STOP, FB_INVALID, FB_VISION };

static uint8_t s_linked;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     建立组/项环形双向链表
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：menu_link
 * 函数功能：为所有组节点和项节点填写 prev/next。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：节点号固定，循环次数由表中的 count 决定，没有无界遍历。
 *           重复调用是安全的，Menu_Init() 可以用它修复被破坏的链表。
 */
static void menu_link(void)
{
    uint8_t g, i, base, count;

    /* 组这一层：4 个组首尾相接 */
    for (g = 0; g < MENU_GROUPS; g++) {
        s_nodes[g].prev = (uint8_t)((g + MENU_GROUPS - 1U) % MENU_GROUPS);
        s_nodes[g].next = (uint8_t)((g + 1U) % MENU_GROUPS);
    }

    /* 每个组内部：项节点首尾相接；节点范围异常时跳过，绝不越界写 */
    for (g = 0; g < MENU_GROUPS; g++) {
        base = s_nodes[g].first;
        count = s_nodes[g].count;
        if (count == 0U || (unsigned)base + count > MENU_NODE_COUNT) {
            continue;
        }
        for (i = 0; i < count; i++) {
            s_nodes[base + i].prev = (uint8_t)(base + ((i + count - 1U) % count));
            s_nodes[base + i].next = (uint8_t)(base + ((i + 1U) % count));
        }
    }
    s_linked = 1;
}

/* 第一次导航前按需建链；不依赖调用方是否先调 Menu_Init()。 */
static void menu_ensure_linked(void)
{
    if (!s_linked) {
        menu_link();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把菜单状态夹回合法范围
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：menu_sanitize
 * 函数功能：校验 depth/group/item，并据此重新算出当前节点号。
 * 参数说明：m —— 菜单状态；可为 NULL。
 * 返回参数：无。
 * 备注信息：防止外部误写 RAM、旧快照或未初始化状态把索引带到表外。
 *           item 越界时回到该组第一项；group 越界时回到 Race。
 */
static void menu_sanitize(MenuState *m)
{
    uint8_t g;

    if (m == NULL) {
        return;
    }
    if (m->depth > 2U) {
        m->depth = 0U;
    }
    if (m->group >= MENU_GROUPS) {
        m->group = MENU_RACE;
    }
    if (m->fault_page > 1U) {
        m->fault_page = 0U;
    }
    for (g = 0; g < MENU_GROUPS; g++) {
        if (m->item[g] >= s_nodes[g].count) {
            m->item[g] = 0U;
        }
    }
    m->node = (m->depth == 0U) ? m->group
                               : (uint8_t)(s_nodes[m->group].first + m->item[m->group]);
    if (m->node >= MENU_NODE_COUNT) {
        m->node = m->group;
    }
}

/* 按节点号取显示文字；越界时返回 "?"，绘制不会因为坏索引越界。 */
static const char *node_name(uint8_t node)
{
    return node < MENU_NODE_COUNT ? s_nodes[node].name : "?";
}

/*
 * 函数名称：Menu_Init
 * 函数功能：把菜单复位到组页，并建立组/项环形链表。
 * 参数说明：m —— 菜单状态；NULL 时直接返回。
 * 返回参数：无。
 * 备注信息：只复位导航与编辑状态，params_status 回到 PARAMS_DEFAULTS；
 *           平台层随后按 Flash 加载结果覆盖它。
 */
void Menu_Init(MenuState *m)
{
    uint8_t i;

    if (m == NULL) {
        return;
    }
    menu_link();
    m->depth = 0U;
    m->group = MENU_RACE;
    m->node = MENU_RACE;
    for (i = 0; i < MENU_GROUPS; i++) {
        m->item[i] = 0U;
    }
    m->feedback = FB_NONE;
    m->candidate = 0U;
    m->race_armed = 0U;
    m->fault_page = 0U;
    m->race_rpm = TRACK_DEFAULT_RPM;
    m->params_status = PARAMS_DEFAULTS;
}

/* 视觉功能是否编译进固件：关闭视觉时不允许“预备”，也不能发车。 */
static int vision_compiled(void)
{
#if BOARD_VISION_ENABLE
    return 1;
#else
    return 0;
#endif
}

/* 发车转速夹紧与步进；口径与 Track_Fsm 的 TRACK_* 宏保持一致。 */
static uint16_t clamp_rpm(unsigned rpm)
{
    unsigned hi = (unsigned)TRACK_MAX_RPM;

    if ((float)hi > BOARD_RPM_LIMIT) hi = (unsigned)BOARD_RPM_LIMIT;
    if (rpm < TRACK_MIN_RPM) return (uint16_t)TRACK_MIN_RPM;
    if (rpm > hi) return (uint16_t)hi;
    return (uint16_t)rpm;
}

static uint16_t step_rpm(unsigned current, uint8_t key_id, uint8_t key_type)
{
    unsigned hi = (unsigned)TRACK_MAX_RPM;
    unsigned step = key_type == KEY_EVENT_REPEAT ? TRACK_RPM_STEP_COARSE : TRACK_RPM_STEP_FINE;

    if ((float)hi > BOARD_RPM_LIMIT) hi = (unsigned)BOARD_RPM_LIMIT;
    current = clamp_rpm(current);
    if (key_id == KEY_UP) {
        return (uint16_t)(current + step > hi ? hi : current + step);
    }
    return (uint16_t)(current < TRACK_MIN_RPM + step ? TRACK_MIN_RPM : current - step);
}

/*
 * 函数名称：VisionHs_Phase
 * 函数功能：由视觉启用/链路/会话/新鲜帧/最近拒绝原因计算握手阶段。
 * 参数说明：enabled       —— BOARD_VISION_ENABLE；
 *           link_alive    —— 是否收到任意 CRC 正确帧；
 *           session_valid —— 是否已绑定 HELLO 会话；
 *           fresh         —— 200 ms 内是否有新鲜 TYPE=1；
 *           reject        —— 最近一次业务拒绝原因。
 * 返回参数：VisionHandshakePhase。
 * 备注信息：判定顺序固定，菜单和发车条件共用同一份结果，避免两处判断漂移。
 */
VisionHandshakePhase VisionHs_Phase(uint8_t enabled, uint8_t link_alive,
                                    uint8_t session_valid, uint8_t fresh, uint8_t reject)
{
    if (!enabled) return VISION_HS_OFF;
    if (!link_alive) return VISION_HS_OFFLINE;
    if (reject == VISION_REJECT_PAYLOAD || reject == VISION_REJECT_TYPE_LENGTH ||
        reject == VISION_REJECT_TILE)
        return VISION_HS_ERROR;
    if (!session_valid) return VISION_HS_WAIT_HELLO;
    if (!fresh) return VISION_HS_STALE;
    return VISION_HS_BOUND;
}

/*
 * 函数名称：start_condition
 * 函数功能：计算“这一帧如果有效，是否满足发车的静态条件”。
 * 参数说明：car/still/pending/hs/road/previous_session 同 Menu_StartCondition。
 * 返回参数：START_READY 表示可以进入连续帧确认；其余为具体原因。
 * 备注信息：判断顺序即优先级，不能随意交换：
 *           1) 非 STOP -> 先停车/清故障；2) 车还在动 -> 等停稳；
 *           3) 正在算数独 -> 等结果；4) 视觉未编译 -> 视觉功能已关闭；
 *           5) 未绑定会话 -> 等待握手；6) 已绑定但无新帧 -> 视觉无新帧；
 *           7) 协议错误 -> 视觉协议错误；
 *           8) 有帧但会话号与上一场相同 -> 视觉端没重新开始；
 *           9) 曲率无效/单边估计/置信度 <0.35/仍有实体元素 -> 等有效赛道。
 */
static StartCondition start_condition(const Car_Core *car, uint8_t still, uint8_t pending,
                                      const VisionHandshake *hs,
                                      const CarVisionObservation *road, uint32_t previous_session)
{
    if (car == NULL) return START_STOP_FIRST;
    if (car->state != CAR_STOP) return START_STOP_FIRST;
    if (!still) return START_MOVING;
    if (pending) return START_RESULT;
    if (!vision_compiled()) return START_NO_VISION;
    if (hs == NULL) return START_NO_SESSION;

    switch (hs->phase) {
    case VISION_HS_OFF:
        return START_NO_VISION;
    case VISION_HS_ERROR:
        return START_VISION_ERROR;
    case VISION_HS_OFFLINE:
        return hs->session_valid ? START_NO_FRAME : START_NO_SESSION;
    case VISION_HS_WAIT_HELLO:
        return START_NO_SESSION;
    case VISION_HS_STALE:
        return START_NO_FRAME;
    case VISION_HS_BOUND:
        break;
    default:
        return START_NO_SESSION;
    }

    if (!road || road->session == 0U) return START_NO_FRAME;
    if (road->session == previous_session) return START_NEW_SESSION;
    /* 单边估计曲率只能用于巡线转向，不能作为发车依据；有实体元素也不发车。 */
    if (!(road->valid_mask & VISION_CURVATURE_VALID) ||
        (road->valid_mask & VISION_CURVATURE_ESTIMATED) ||
        !(road->confidence >= 0.35f) || road->element != VISION_ELEMENT_NONE)
        return START_NO_ROAD;
    return START_READY;
}

/*
 * 函数名称：Menu_StartCondition
 * 函数功能：判断“现在能不能发车”，返回具体原因供菜单显示。
 * 参数说明：confirm          —— 连续有效帧确认状态；首次传全 0；
 *           car              —— 控制对象，只用 state 判断是否停车；
 *           still            —— 轮速是否近似为零（由平台层用 speed[] 判定）；
 *           pending          —— 是否正在准备数独结果（完赛收尾期间禁止新车）；
 *           hs               —— 本拍握手快照；
 *           road             —— 最新道路观测，NULL 表示本拍没有可用视觉帧；
 *           previous_session —— 上一场比赛绑定的视觉会话号。
 * 返回参数：发车条件枚举，只有 START_READY 才允许按 OK 发车。
 * 备注信息：静态条件通过后，还要连续 3 个同一会话、序号递增、未过期的
 *           有效赛道帧才算 READY，防止单帧误检就发车。条件一旦中断，
 *           确认计数立即清零。
 */
StartCondition Menu_StartCondition(MenuRoadConfirm *confirm, const Car_Core *car, uint8_t still, uint8_t pending, const VisionHandshake *hs, const CarVisionObservation *road, uint32_t previous_session)
{
    const unsigned required_frames = 3U;
    StartCondition condition;
    uint16_t delta;

    if (confirm == NULL) return START_NO_ROAD;
    condition = start_condition(car, still, pending, hs, road, previous_session);
    if (condition != START_READY) {
        confirm->count = 0U;
        return condition;
    }

    /* 会话变化或两帧间隔超过视觉时效：重新开始计数。 */
    if (confirm->session != road->session ||
        (uint32_t)(road->received_ms - confirm->received_ms) > VIS_MAX_AGE_MS) {
        confirm->count = 0U;
    }
    if (confirm->count) {
        delta = (uint16_t)(road->sequence - confirm->sequence);
        if (delta == 0U) return confirm->count >= required_frames ? START_READY : START_NO_ROAD;
        if (delta >= 0x8000U) {
            confirm->count = 0U;
            return START_NO_ROAD;
        }
    }
    confirm->session = road->session;
    confirm->sequence = road->sequence;
    confirm->received_ms = road->received_ms;
    if (confirm->count < required_frames) confirm->count++;
    return confirm->count >= required_frames ? START_READY : START_NO_ROAD;
}

/* 读取比赛转速；0 或未初始化时回退到默认值，并夹到允许范围。 */
uint16_t Menu_GetRaceRpm(const MenuState *m)
{
    if (m == NULL || m->race_rpm == 0U) return clamp_rpm(TRACK_DEFAULT_RPM);
    return clamp_rpm(m->race_rpm);
}

/* 取消“预备自动发车”；任何停车/故障/离开页面路径都可以调用。 */
void Menu_Disarm(MenuState *m)
{
    if (m != NULL) m->race_armed = 0U;
}

/*
 * 函数名称：Menu_PollArmed
 * 函数功能：检查“预备发车”状态；视觉就绪时返回 MENU_LAUNCH。
 * 参数说明：m     —— 菜单状态；
 *           start —— 当前发车条件。
 * 返回参数：MENU_IDLE / MENU_LAUNCH。
 * 备注信息：车动、故障、离开发车页或正在收尾时自动取消预备；
 *           真正发车仍由平台层调用 Car_StartRace() 执行。
 */
MenuAction Menu_PollArmed(MenuState *m, StartCondition start)
{
    if (m == NULL || !m->race_armed) return MENU_IDLE;
    if (!vision_compiled() || m->depth != 2U || m->group != MENU_RACE ||
        m->item[MENU_RACE] != 0U || start == START_STOP_FIRST ||
        start == START_MOVING || start == START_RESULT ||
        start == START_VISION_ERROR) {
        m->race_armed = 0U;
        return MENU_IDLE;
    }
    if (start == START_READY) {
        m->race_rpm = Menu_GetRaceRpm(m);
        m->race_armed = 0U;
        m->item[MENU_RACE] = 1U;
        return MENU_LAUNCH;
    }
    return MENU_IDLE;
}

/*
 * 函数名称：Menu_Key
 * 函数功能：处理一个按键事件，沿链表移动光标或修改/应用增益，返回要执行的动作。
 * 参数说明：m     —— 菜单状态（层级 depth、组 group、各组光标 item[]、当前节点 node）；
 *           key   —— 按键事件（键号 + 按下/连发）；
 *           car   —— 控制对象，应用增益与判断状态时读写；
 *           start —— 当前发车条件，只有 READY 才允许发车。
 * 返回参数：MENU_IDLE 什么也不做；MENU_LAUNCH 请求平台发车；
 *           MENU_SAVE_PARAMS 请求平台写 Flash 保存参数。
 * 备注信息：三级导航：
 *           第 0 级选组（沿组链表循环）；第 1 级选组内项（沿项链表循环）；
 *           第 2 级是“值/动作”。UP/DOWN 只走 prev/next 一步。
 *           增益编辑用“放大 100 倍的整数”存候选值（candidate）：
 *             - 单位与蓝牙命令 4 的编码一致（float×100 -> uint16）；
 *             - 连发一次 ±10（0.10），单按一次 ±1（0.01），避免浮点累加误差；
 *             - 只有按 OK 才真正调用 Car_SetPid，按 BACK 直接丢弃候选值。
 *           本函数不写 Flash、不启动电机，只返回“要做什么”，副作用集中在平台层。
 *           入口先把状态夹回合法范围，坏索引不会把 prev/next 带到表外。
 */
MenuAction Menu_Key(MenuState *m, Key_Event_Struct key, Car_Core *car, StartCondition start)
{
    uint8_t item, pid, next;
    int start_page, rpm_page;

    if (m == NULL || car == NULL || key.id >= KEY_NUM) {
        return MENU_IDLE;
    }
    menu_ensure_linked();
    menu_sanitize(m);

    if (m->params_status == PARAMS_BUSY) {
        return MENU_IDLE;
    }
    m->feedback = FB_NONE;
    item = m->item[m->group];
    pid = (uint8_t)(m->group >= MENU_LEFT_PID);
    start_page = (m->group == MENU_RACE && item == 0U);
    rpm_page = (m->group == MENU_RACE && item == 3U);

    /* 车动、故障或正在收尾时一律取消预备，避免误自动发车。 */
    if (car->state != CAR_STOP || start == START_STOP_FIRST ||
        start == START_MOVING || start == START_RESULT) {
        m->race_armed = 0U;
    }

    if (key.id == KEY_BACK && key.type == KEY_EVENT_PRESS) {
        if (m->depth == 2U && start_page && m->race_armed) {
            m->race_armed = 0U;                    /* 先取消等待，不退出页面 */
        } else if (m->depth) {
            m->race_armed = 0U;
            --m->depth;
            /* 退回组页时当前节点换成组节点；退回项页时仍是该项节点。 */
            m->node = (m->depth == 0U) ? m->group
                                       : (uint8_t)(s_nodes[m->group].first + m->item[m->group]);
        }
    } else if (key.id == KEY_UP || key.id == KEY_DOWN) {
        if (m->depth < 2U) {
            next = (key.id == KEY_UP) ? s_nodes[m->node].prev : s_nodes[m->node].next;
            if (next >= MENU_NODE_COUNT) {
                return MENU_IDLE;
            }
            m->node = next;
            if (m->depth == 0U) {
                m->group = next;                    /* 组节点号就是组号 */
            } else {
                m->item[m->group] = s_nodes[next].item;
            }
        } else if (start_page) {
            /* 发车页可直接调速；已预备时 UP 先取消等待，避免误改。 */
            if (m->race_armed) {
                if (key.id == KEY_UP) m->race_armed = 0U;
            } else if (car->state == CAR_STOP) {
                m->race_rpm = step_rpm(Menu_GetRaceRpm(m), key.id, key.type);
            }
        } else if (rpm_page) {
            unsigned base = m->candidate ? m->candidate : Menu_GetRaceRpm(m);
            m->candidate = step_rpm(base, key.id, key.type);
        } else if (m->group == MENU_SENSORS && item == 3U) {
            /* 故障记录页：UP 第 1 页（0..5），DOWN 第 2 页（6..8）。 */
            m->fault_page = (key.id == KEY_UP) ? 0U : 1U;
        } else if (pid && item < 3U) {
            /* Fine taps (Ki/Kd), coarse auto-repeat for larger changes. */
            unsigned step = key.type == KEY_EVENT_REPEAT ? 10U : 1U;
            if (key.id == KEY_UP)
                m->candidate = m->candidate > 65535U-step ? 65535U : m->candidate+step;
            else
                m->candidate = m->candidate < step ? 0U : m->candidate-step;
        }
    } else if (key.id == KEY_OK && key.type == KEY_EVENT_PRESS) {
        if (m->depth == 0U) {
            m->depth = 1U;
            m->node = (uint8_t)(s_nodes[m->group].first + m->item[m->group]);
        } else if (m->depth == 1U) {
            m->depth = 2U;
            if (rpm_page) {
                m->candidate = Menu_GetRaceRpm(m);
            } else if (pid && item < 3U) {
                Car_PidInc_Struct *p = &car->pid_speed[m->group - MENU_LEFT_PID];
                float gains[] = {p->kp, p->ki, p->kd};
                m->candidate = (unsigned)(gains[item]*100.0f+0.5f);
            }
        } else if (pid && item == 3U) {
            if (car->state != CAR_STOP || !(fabsf(car->speed[0]) < 2 && fabsf(car->speed[1]) < 2)
                || start == START_RESULT) {
                m->params_status = PARAMS_STOP_FIRST;
            } else {
                m->params_status = PARAMS_BUSY;
                return MENU_SAVE_PARAMS;
            }
        } else if (pid) {
            /* 第 2 级第 0..2 项：按 OK 把候选值写进内环。wheel 由“当前组”换算：
             * MENU_LEFT_PID -> 0、MENU_RIGHT_PID -> 1；外环不开放。 */
            unsigned wheel = m->group - MENU_LEFT_PID;
            Car_PidInc_Struct *p = &car->pid_speed[wheel];
            float gains[] = {p->kp,p->ki,p->kd};
            uint8_t result;
            gains[item] = m->candidate / 100.0f;
            /* 先组装完整增益再一次性提交；失败只影响本次，不破坏已生效参数。 */
            result = Car_SetPid(car,wheel,gains[0],gains[1],gains[2]);
            m->feedback = result == 0 ? FB_APPLIED : (result == 1 ? FB_STOP : FB_INVALID);
        } else if (rpm_page) {
            /* 转速页：停车且未预备才允许应用；BACK 直接丢弃候选值。 */
            if (car->state != CAR_STOP || m->race_armed) {
                m->feedback = FB_STOP;
            } else {
                m->race_rpm = clamp_rpm(m->candidate ? m->candidate : Menu_GetRaceRpm(m));
                m->candidate = m->race_rpm;
                m->feedback = FB_APPLIED;
            }
        } else if (start_page) {
            /* 发车页：就绪立即发车；视觉未就绪则切换“预备”，就绪后自动开跑。 */
            m->race_rpm = Menu_GetRaceRpm(m);
            if (!vision_compiled()) {
                m->race_armed = 0U;
                m->feedback = FB_INVALID;
            } else if (start == START_READY) {
                m->race_armed = 0U;
                m->item[MENU_RACE] = 1U;
                return MENU_LAUNCH;
            } else if (start == START_NO_VISION || start == START_NO_SESSION ||
                       start == START_NO_FRAME || start == START_NO_ROAD ||
                       start == START_NEW_SESSION) {
                m->race_armed = (uint8_t)!m->race_armed;
            } else if (start == START_VISION_ERROR) {
                /* 协议错误通常需要重启图像端或复位 F407 才会恢复，不允许预备。 */
                m->race_armed = 0U;
                m->feedback = FB_VISION;
            } else {
                m->race_armed = 0U;
                m->feedback = FB_STOP;
            }
        } else if (m->group == MENU_SENSORS && item == 3U) {
            /* 故障记录页：OK 清空历史，不停车、不改目标。 */
            Car_ClearFaultLog(car);
            m->feedback = FB_NONE;
        }
    }
    return MENU_IDLE;
}

/*
 * 函数名称：Menu_ShowResult
 * 函数功能：把菜单位置跳到 “Race -> Sudoku result” 页。
 * 参数说明：m —— 菜单状态；NULL 时直接返回。
 * 返回参数：无。
 * 备注信息：完赛或求解完成时调用；直接把 depth 设成 2、组设为 RACE、项设为 2，
 *           用户不用再手动翻页。feedback 清零，避免上一次的“已应用”提示残留。
 */
void Menu_ShowResult(MenuState *m)
{
    if (m == NULL) {
        return;
    }
    menu_ensure_linked();
    m->depth = 2U;
    m->group = MENU_RACE;
    m->item[MENU_RACE] = 2U;
    m->race_armed = 0U;
    m->feedback = FB_NONE;
    menu_sanitize(m);
}

/*
 * 函数名称：Menu_DrawRow
 * 函数功能：按当前快照画第 row 行（0..9）。
 * 参数说明：v      —— 一屏数据快照，平台层在临界区一次性填好；
 *           row    —— 行号 0..9；
 *           board  —— 数独答案（36 格），结果页使用；
 *           result —— 数独求解状态，决定结果页显示“进度”还是“答案”。
 * 返回参数：无。
 * 备注信息：行布局：
 *             0 标题；1 项目名；2..7 内容（不同页面含义不同）；
 *             8 状态栏（STOP/FAULT/RACE/MANUAL + 故障码 + 视觉丢失提示）；
 *             9 操作提示（PE10 急停 / UP-DN 移动 / OK 进入 / BACK 返回）。
 *           只读快照，不访问控制对象，所以主循环刷新菜单不会和控制中断抢数据。
 *           入口先夹住 row/group/item/start/phase/params_status，任何坏值都不会越界。
 *           最后一行统一交给 Lcd_Print，由驱动做“逐字符比较、只重画变化部分”。
 */
void Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result)
{
    const MenuState *m;
    UI_Style_Enum style = UI_STYLE_NORMAL;
    char s[LCD_LINE_BYTES] = "";
    unsigned group, item, count, depth, start, status, phase_count, rpm, hs_phase;

    if (v == NULL || row >= LCD_ROWS) {
        return;
    }
    m = &v->menu;
    group = m->group;
    if (group >= MENU_GROUPS) {
        group = MENU_RACE;
    }
    count = s_nodes[group].count;
    item = m->item[group];
    if (item >= count) {
        item = 0U;
    }
    depth = m->depth;
    if (depth > 2U) {
        depth = 0U;
    }
    start = (unsigned)v->start;
    if (start >= (unsigned)START_COUNT) {
        start = (unsigned)START_NO_ROAD;
    }
    hs_phase = (unsigned)v->vision_phase;
    if (hs_phase >= (unsigned)VISION_HS_COUNT) {
        hs_phase = (unsigned)VISION_HS_OFF;
    }
    rpm = v->race_rpm ? clamp_rpm(v->race_rpm) : Menu_GetRaceRpm(m);

    if (row == 0) {
        const char *title = depth == 0U ? "智能车菜单" : s_nodes[group].name;
        style = UI_STYLE_TITLE;
        snprintf(s,sizeof s,"%s  T%07lus",title,(unsigned long)v->uptime_s);
    } else if (row == 9) {
        style = UI_STYLE_HINT;
        if (v->state != CAR_STOP) {
            snprintf(s,sizeof s,"PE10: 急停/清故障");
        } else if (m->race_armed) {
            snprintf(s,sizeof s,"OK/返回/PE10: 取消等待");
        } else if (depth < 2U) {
            snprintf(s,sizeof s,"上下移动  OK进入  返回上级");
        } else if (group == MENU_SENSORS && item == 3U) {
            snprintf(s,sizeof s,"OK清空 上下翻页  返回");
        } else if (group == MENU_RACE && item == 0U) {
            snprintf(s,sizeof s,"上下调RPM  OK发车  返回");
        } else if ((group == MENU_RACE && item == 3U) ||
                   (group >= MENU_LEFT_PID && item < 3U)) {
            snprintf(s,sizeof s,"上下调节  OK应用  返回放弃");
        } else if (group >= MENU_LEFT_PID && item == 3U) {
            snprintf(s,sizeof s,"OK写入Flash  PE9返回");
        } else {
            snprintf(s,sizeof s,"PE9: 返回");
        }
    } else if (row == 8) {
        char lim[16] = "";
        if (v->fault_derate && CAR_FAULT_DERATE_PERCENT > 0U) {
            snprintf(lim,sizeof lim," 限速%u%%",(unsigned)(100U - CAR_FAULT_DERATE_PERCENT));
        }
        if (v->fault == CAR_TIMING_FAULT) {
            style = UI_STYLE_WARN;
            snprintf(s,sizeof s,"故障 5  dt=%luus%s",(unsigned long)v->timing_fault_us,lim);
        } else if (v->fault == CAR_RACE_FAULT) {
            static const char *const reasons[] = {"", "PIT dt", "视觉丢失", "断路超限", "绕障超限", "红障超时"};
            unsigned why = (unsigned)v->race_fault_reason;
            style = UI_STYLE_WARN;
            snprintf(s,sizeof s,"故障 7  %s%s",
                     why < sizeof reasons / sizeof reasons[0] ? reasons[why] : "",lim);
        } else if (v->state == CAR_STOP && m->race_armed) {
            style = UI_STYLE_WARN;
            snprintf(s,sizeof s,"待视觉握手 %3uRPM 故障%u",rpm,(unsigned)v->fault);
        } else {
            style = (v->state == CAR_FAULT || v->degraded) ? UI_STYLE_WARN :
                    (v->state == CAR_RUN && v->motion == CAR_MOTION_RACE) ? UI_STYLE_GOOD : UI_STYLE_HINT;
            snprintf(s,sizeof s,"%s%s  故障 %u%s",v->state == CAR_STOP ? "停车" :
                     (v->state == CAR_FAULT ? "故障" : (v->motion == CAR_MOTION_RACE ? "比赛" : "手动")),
                     v->degraded ? " 视觉丢失" : "", (unsigned)v->fault,lim);
        }
    } else if (depth < 2U) {
        unsigned n = row-1;
        unsigned ncount = depth ? count : (unsigned)MENU_GROUPS;
        if (n < ncount) {
            const char *name = depth ? node_name((uint8_t)(s_nodes[group].first + n))
                                     : s_nodes[n].name;
            if (n == (depth ? item : group)) style = UI_STYLE_SELECT;
            snprintf(s,sizeof s,"%s %s",style == UI_STYLE_SELECT ? ">" : " ",name);
        } else if (depth == 0U && row == 5U) {
            /* 设备状态页：摄像头由香橙派 TYPE=3 状态帧报告，不依赖比赛会话。
             * 没有状态帧但道路帧新鲜时，说明旧版香橙派仍在出图，按已连接兜底。 */
            if (!v->vision_enabled) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"摄像头: 已关闭");
            } else if (v->camera_status == VISION_CAMERA_NO_FRAME) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"摄像头: 无画面");
            } else if (v->camera_status == VISION_CAMERA_STREAMING ||
                       (v->camera_status == VISION_CAMERA_UNKNOWN && v->fresh)) {
                style = UI_STYLE_GOOD;
                snprintf(s,sizeof s,"摄像头: 已连接");
            } else if (v->have_obs) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"摄像头: 数据过期");
            } else if (v->pi_connected) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"摄像头: 等待数据");
            } else {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"摄像头: 无法确认");
            }
        } else if (depth == 0U && row == 6U) {
            /* 链路已连接不等于已绑定会话：未握手要单独显示，避免与发车页矛盾。 */
            if (!v->vision_enabled || hs_phase == (unsigned)VISION_HS_OFF) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"香橙派: 已关闭");
            } else if (hs_phase == (unsigned)VISION_HS_OFFLINE) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"香橙派: 未连接");
            } else if (hs_phase == (unsigned)VISION_HS_ERROR) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"香橙派: 协议错误");
            } else if (hs_phase == (unsigned)VISION_HS_WAIT_HELLO) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"香橙派: 未握手");
            } else if (v->pi_connected) {
                style = UI_STYLE_GOOD;
                snprintf(s,sizeof s,"香橙派: 已连接");
            } else {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"香橙派: 未连接");
            }
        } else if (depth == 0U && row == 7U) {
            /* 蓝牙状态右侧显示电池电压；每分钟由 Battery_Poll 更新一次，
             * 电压与蓝牙状态共用这一行，未采样时显示占位符。 */
            style = v->bt_connected ? UI_STYLE_GOOD : UI_STYLE_WARN;
            if (v->battery_valid) {
                snprintf(s,sizeof s,"蓝牙上位机: %s %4.1fV",
                         v->bt_connected ? "已连接" : "未连接",
                         (double)v->battery_mv / 1000.0);
            } else {
                snprintf(s,sizeof s,"蓝牙上位机: %s  --.-V",
                         v->bt_connected ? "已连接" : "未连接");
            }
        }
    } else if (row == 1) {
        if (group == MENU_SENSORS && item == 3U) {
            unsigned page = m->fault_page ? 1U : 0U;
            snprintf(s,sizeof s,"故障记录 P%u/2",page + 1U);
        } else {
            snprintf(s,sizeof s,"%s",node_name((uint8_t)(s_nodes[group].first + item)));
        }
    } else if (group == MENU_RACE && item == 0U) {
        if (row == 2) {
            style = (!m->race_armed && v->state == CAR_STOP) ? UI_STYLE_EDIT : UI_STYLE_NORMAL;
            snprintf(s,sizeof s,"目标 %3u rpm (%.2f m/s)",rpm,(double)TRACK_RPM_TO_MPS(rpm));
        } else if (row == 3) {
            if (!v->vision_enabled || hs_phase == (unsigned)VISION_HS_OFF) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 配置已关闭");
            } else if (hs_phase == (unsigned)VISION_HS_OFFLINE) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 链路离线");
            } else if (hs_phase == (unsigned)VISION_HS_ERROR) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 协议错误");
            } else if (hs_phase == (unsigned)VISION_HS_WAIT_HELLO) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 等待握手");
            } else if (hs_phase == (unsigned)VISION_HS_STALE) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 握手后无帧");
            } else if (start == (unsigned)START_NEW_SESSION) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉: 等待新会话");
            } else if (start == (unsigned)START_NO_ROAD) {
                style = UI_STYLE_WARN;
                if (!(v->obs.valid_mask & VISION_CURVATURE_VALID))
                    snprintf(s,sizeof s,"无有效曲率  元素 %u",(unsigned)v->obs.element);
                else if (v->obs.valid_mask & VISION_CURVATURE_ESTIMATED)
                    snprintf(s,sizeof s,"等待实测曲率");
                else if (!(v->obs.confidence >= 0.35f))
                    snprintf(s,sizeof s,"置信度 %.3f < 0.350",(double)v->obs.confidence);
                else if (v->obs.element != VISION_ELEMENT_NONE)
                    snprintf(s,sizeof s,"元素 %u  等待元素 0",(unsigned)v->obs.element);
                else
                    snprintf(s,sizeof s,"有效赛道帧 %u/3",(unsigned)v->launch_road_count);
            } else {
                float conf = isfinite(v->obs.confidence) ? v->obs.confidence : 0.0f;
                if (conf < 0.0f) conf = 0.0f;
                if (conf > 1.0f) conf = 1.0f;
                style = UI_STYLE_GOOD;
                snprintf(s,sizeof s,"视觉: 已开启 置信度 %.2f",(double)conf);
            }
        } else if (row == 4) {
            snprintf(s,sizeof s,"%s",conditions[start]);
        } else if (row == 5) {
            if (m->race_armed) {
                style = UI_STYLE_EDIT;
                snprintf(s,sizeof s,"OK/返回: 取消等待发车");
            } else if (v->vision_enabled && start == (unsigned)START_READY) {
                style = UI_STYLE_EDIT;
                snprintf(s,sizeof s,"OK: 立即发车 (%u rpm)",rpm);
            } else if (v->vision_enabled && hs_phase != (unsigned)VISION_HS_OFF &&
                       (start == (unsigned)START_NO_VISION ||
                        start == (unsigned)START_NO_SESSION ||
                        start == (unsigned)START_NO_FRAME ||
                        start == (unsigned)START_NO_ROAD ||
                        start == (unsigned)START_NEW_SESSION)) {
                style = UI_STYLE_EDIT;
                snprintf(s,sizeof s,"OK: 预备(就绪自动开跑)");
            }
        } else if (row == 6) {
            if (m->feedback == FB_STOP) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"请先停车并保持车轮静止");
            } else if (m->feedback == FB_INVALID) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"请开启 BOARD_VISION_ENABLE=1");
            } else if (m->feedback == FB_VISION) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"视觉协议错误,重启图像端");
            } else if (m->race_armed) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"电机已锁停,视觉就绪即跑");
            } else if (v->state == CAR_STOP) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"上下键: +-5 rpm(长按+-20)");
            }
        } else if (row == 7) {
            style = UI_STYLE_HINT;
            snprintf(s,sizeof s,"发车页也可直接上下调速");
        }
    } else if (group == MENU_RACE && item == 1U) {
        phase_count = (unsigned)(sizeof phases / sizeof phases[0]);
        if (row == 2) {
            unsigned phase = v->phase;
            if (phase >= phase_count) phase = phase_count - 1U;
            snprintf(s,sizeof s,"第%u圈  %s",(unsigned)v->lap,v->lap ? phases[phase] : "空闲");
        }
        if (row == 3) snprintf(s,sizeof s,"里程 %.2f m",v->distance);
        if (row == 4) snprintf(s,sizeof s,"航向 %.1f deg",v->yaw_deg);
        if (row == 5) snprintf(s,sizeof s,"数独子块 %u/4",
            (unsigned)((v->tile_mask&1)+((v->tile_mask>>1)&1)+((v->tile_mask>>2)&1)+((v->tile_mask>>3)&1)));
        if (row == 6 && v->degraded) {
            style = UI_STYLE_WARN;
            snprintf(s,sizeof s,"视觉丢失: 保持上次曲率");
        }
        if (row == 7) {
            float spd0 = isfinite(v->speed[0]) ? v->speed[0] : 0.0f;
            float spd1 = isfinite(v->speed[1]) ? v->speed[1] : 0.0f;
            spd0 = fmaxf(-999.0f, fminf(999.0f, spd0));
            spd1 = fmaxf(-999.0f, fminf(999.0f, spd1));
            snprintf(s,sizeof s,"设%3u L%4.0f R%4.0f rpm",rpm,(double)spd0,(double)spd1);
        }
    } else if (group == MENU_RACE && item == 3U) {
        unsigned edit_rpm = clamp_rpm(m->candidate ? m->candidate : rpm);
        if (row == 2) snprintf(s,sizeof s,"当前 %3u rpm (%.2f m/s)",rpm,(double)TRACK_RPM_TO_MPS(rpm));
        if (row == 3) {
            style = UI_STYLE_EDIT;
            snprintf(s,sizeof s,"编辑 %3u rpm (%.2f m/s)",edit_rpm,(double)TRACK_RPM_TO_MPS(edit_rpm));
        }
        if (row == 4) snprintf(s,sizeof s,"范围 %u..%u rpm",(unsigned)TRACK_MIN_RPM,(unsigned)TRACK_MAX_RPM);
        if (row == 5) snprintf(s,sizeof s,"上下+-5(连发+-20)  OK确认");
        if (row == 6) snprintf(s,sizeof s,"%s", m->feedback == FB_APPLIED ? "已生效" :
                               (v->state != CAR_STOP || m->feedback == FB_STOP ? "请先停车再应用" :
                                "按返回键放弃修改"));
        if (row == 7) snprintf(s,sizeof s,"发车页也可直接上下调速");
    } else if (group == MENU_RACE) {
        if (row == 2 && result != SUDOKU_SOLVED) {
            const char *status_text = "采集中: 完赛后自动求解";
            switch (result) {
            case SUDOKU_WAITING: status_text = "已停车: 等待末尾子块"; break;
            case SUDOKU_MISSING: status_text = "缺少子块,无法求解"; break;
            case SUDOKU_INVALID: status_text = "题面冲突或无解"; break;
            case SUDOKU_AMBIGUOUS: status_text = "存在多解,非唯一解"; break;
            case SUDOKU_LIMIT: status_text = "求解超出搜索上限"; break;
            default: break;
            }
            snprintf(s,sizeof s,"%s",status_text);
        }
        if (result != SUDOKU_SOLVED && row >= 3 && row <= 6) {
            const char *const names[] = {"左上(Tag0/1)","右上(Tag2/3)","左下(Tag4/5)","右下(Tag6/7)"};
            unsigned t = row-3;
            snprintf(s,sizeof s,"%s  %s",names[t],(v->tile_mask & (1U<<t)) ? "已收" : "缺失");
        }
        if (result == SUDOKU_SOLVED && row >= 2 && row <= 7) {
            const uint8_t *b = board+6*(row-2);
            snprintf(s,sizeof s,"  %u  %u  %u | %u  %u  %u", b[0],b[1],b[2],b[3],b[4],b[5]);
        }
    } else if (group >= MENU_LEFT_PID && item == 3U) {
        static const char *const messages[] = {
            "当前使用默认参数", "已从Flash加载参数", "已保存,重启自动恢复", "参数无变化,无需重写",
            "读写失败,请重试", "PID参数无效", "保存中...请勿断电", "请先停车并完成结果求解"
        };
        status = (unsigned)m->params_status;
        if (status >= (unsigned)(sizeof messages / sizeof messages[0])) {
            status = (unsigned)PARAMS_ERROR;
        }
        if (row == 2) snprintf(s,sizeof s,"同时保存左右轮共6项PID");
        if (row == 3) {
            style = (status == (unsigned)PARAMS_ERROR || status == (unsigned)PARAMS_INVALID)
                    ? UI_STYLE_WARN : UI_STYLE_NORMAL;
            snprintf(s,sizeof s,"%s",messages[status]);
        }
        if (row == 4) snprintf(s,sizeof s,"%s",!v->params_saved ? "Flash无已存参数" :
                              (v->params_dirty ? "有未保存的修改" : "内存与已存参数一致"));
        if (row == 5 && m->params_status != PARAMS_BUSY) {
            int ready = v->state == CAR_STOP && fabsf(v->speed[0]) < 2 && fabsf(v->speed[1]) < 2
                        && start != START_RESULT;
            style = ready ? UI_STYLE_EDIT : UI_STYLE_NORMAL;
            snprintf(s,sizeof s,"%s",ready ? "OK: 保存参数到Flash" :
                     (start == START_RESULT ? "请等待数独求解完成" : "请停车并保持轮静止"));
        }
        if (row == 6) snprintf(s,sizeof s,"仅保存已按OK生效的参数");
    } else if (group >= MENU_LEFT_PID) {
        if (row == 2) snprintf(s,sizeof s,"当前 %.2f",v->gains[item]);
        if (row == 3) { style = UI_STYLE_EDIT; snprintf(s,sizeof s,"编辑 %.2f",m->candidate/100.0); }
        if (row == 4) snprintf(s,sizeof s,"上下+-  OK应用到内存");
        if (row == 6) snprintf(s,sizeof s,"%s",m->feedback == FB_APPLIED ? "已生效" :
                               (v->state != CAR_STOP || m->feedback == FB_STOP ? "请先停车再应用" :
                               (m->feedback == FB_INVALID ? "参数无效" : "按返回键放弃修改")));
        if (row == 7) snprintf(s,sizeof s,"调好后请进保存页写Flash");
    } else if (group == MENU_SENSORS && item == 3U) {
        unsigned page = m->fault_page ? 1U : 0U;
        unsigned idx = page * 6U + (row - 2U);
        if (idx < CAR_FAULT_CODES) {
            const CarFaultRecord *r = &v->fault_log[idx];
            if (r->count == 0U) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"%u %s 未发生",idx,fault_names[idx]);
            } else {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"%u %s x%u %lus",idx,fault_names[idx],
                         (unsigned)r->count,(unsigned long)(r->last_ms / 1000U));
            }
        }
    } else if (item == 0U) {
        if (row == 2) snprintf(s,sizeof s,"     目标   实测   PWM%%");
        if (row == 3 || row == 4) {
            unsigned w = row-3;
            snprintf(s,sizeof s,"%s %6.1f %6.1f %6.1f", w ? "右" : "左",v->reference[w],v->speed[w],v->pwm[w]);
        }
    } else if (item == 1U) {
        if (row == 2) snprintf(s,sizeof s,"%s",!v->imu_present ? "MPU6050不可用" :
                               (v->samples < 100 ? "校零中: 请保持静止" : (v->gyro_ok ? "陀螺仪数据正常" : "陀螺仪过期/不可用")));
        if (row == 3) snprintf(s,sizeof s,"零偏采样 %u/100",(unsigned)v->samples);
        if (row == 4 && v->samples == 100) snprintf(s,sizeof s,"Z轴零偏 %+.2f deg/s",v->bias_dps);
        if (row == 5 && v->gyro_ok) snprintf(s,sizeof s,"Z轴速率 %+.2f deg/s",v->gyro_dps);
    } else if (!v->fresh) {
        if (row == 2) snprintf(s,sizeof s,"无新鲜视觉帧");
    } else {
        if (row == 2) snprintf(s,sizeof s,"会话 %lu",(unsigned long)v->obs.session);
        if (row == 3) snprintf(s,sizeof s,"帧号 %u  置信度 %.2f",(unsigned)v->obs.sequence,v->obs.confidence);
        if (row == 4 && (v->obs.valid_mask & VISION_CURVATURE_VALID))
            snprintf(s,sizeof s,"曲率 %+.2f /m",v->obs.curvature_inv_m);
        if (row == 5) snprintf(s,sizeof s,"元素 %u",(unsigned)v->obs.element);
    }
    /* 接收统计与是否新鲜无关：bad 表示 CRC/长度错，rejected 表示 CRC 对但
     * 会话、序号、年龄或字段越界被业务层拒绝。两者持续增长说明协议不匹配。 */
    if ((group == MENU_SENSORS && item == 2U) && depth >= 2U) {
        if (row == 6) snprintf(s,sizeof s,"正常 %lu  错帧 %lu",
            (unsigned long)v->vision_ok,(unsigned long)v->vision_bad);
        if (row == 7) snprintf(s,sizeof s,"拒绝 %lu",(unsigned long)v->vision_rejected);
    }
    Lcd_Print((uint8_t)row,style,"%s",s);
}
