/*
 * ============================================================================
 * 文件名称：Menu_Task.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 板端菜单的全部逻辑，界面只依赖 MenuView 快照，不直接读控制对象：
 *   1. Menu_Init()          ：建立组/项环形链表，菜单回到组页；
 *   2. Menu_StartCondition()：能否发车的 6 种条件；
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
    NODE_SENSORS_BASE = NODE_RACE_BASE + 3,      /* 7：Sensors 第一项 */
    NODE_LEFT_BASE    = NODE_SENSORS_BASE + 3,   /* 10：左轮 PID 第一项 */
    NODE_RIGHT_BASE   = NODE_LEFT_BASE + 4,      /* 14：右轮 PID 第一项 */
    MENU_NODE_COUNT   = NODE_RIGHT_BASE + 4      /* 18：节点总数 */
};

static MenuNode s_nodes[MENU_NODE_COUNT] = {
    /* 组节点（0..3），first/count 指向该组第一项和项数 */
    {"比赛控制",       MENU_RACE,      0, NODE_RACE_BASE,    3, 0, 0},
    {"传感器监控",     MENU_SENSORS,   0, NODE_SENSORS_BASE, 3, 0, 0},
    {"左轮内环PID",    MENU_LEFT_PID,  0, NODE_LEFT_BASE,    4, 0, 0},
    {"右轮内环PID",    MENU_RIGHT_PID, 0, NODE_RIGHT_BASE,   4, 0, 0},
    /* Race 项（4..6） */
    {"发车启动",    MENU_RACE, 0, 0, 0, 0, 0},
    {"比赛状态",    MENU_RACE, 1, 0, 0, 0, 0},
    {"数独结果",    MENU_RACE, 2, 0, 0, 0, 0},
    /* Sensors 项（7..9） */
    {"左右轮转速",     MENU_SENSORS, 0, 0, 0, 0, 0},
    {"陀螺仪 MPU6050", MENU_SENSORS, 1, 0, 0, 0, 0},
    {"视觉数据",       MENU_SENSORS, 2, 0, 0, 0, 0},
    /* 左轮内环 PID 项（10..13） */
    {"比例 Kp",           MENU_LEFT_PID, 0, 0, 0, 0, 0},
    {"积分 Ki",           MENU_LEFT_PID, 1, 0, 0, 0, 0},
    {"微分 Kd",           MENU_LEFT_PID, 2, 0, 0, 0, 0},
    {"保存参数到Flash",   MENU_LEFT_PID, 3, 0, 0, 0, 0},
    /* 右轮内环 PID 项（14..17） */
    {"比例 Kp",           MENU_RIGHT_PID, 0, 0, 0, 0, 0},
    {"积分 Ki",           MENU_RIGHT_PID, 1, 0, 0, 0, 0},
    {"微分 Kd",           MENU_RIGHT_PID, 2, 0, 0, 0, 0},
    {"保存参数到Flash",   MENU_RIGHT_PID, 3, 0, 0, 0, 0},
};

/* 非节点表数据：发车原因、赛段、保存状态只用于显示，不参与导航。 */
static const char *const conditions[] = {"条件就绪,可发车", "请先停车或清故障", "车轮仍在转动",
    "正在求解数独结果", "等待识别有效赛道", "等待视觉 HELLO"};
static const char *const phases[] = {"巡线","断路","绕障","红障停","完赛","故障"};
enum { FB_NONE, FB_APPLIED, FB_STOP, FB_INVALID };

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
    m->params_status = PARAMS_DEFAULTS;
}

/*
 * 函数名称：Menu_StartCondition
 * 函数功能：判断“现在能不能发车”，返回具体原因供菜单显示。
 * 参数说明：car              —— 控制对象，只用 state 判断是否停车；
 *           still            —— 轮速是否近似为零（由平台层用 speed[] 判定）；
 *           pending          —— 是否正在准备数独结果（完赛收尾期间禁止新车）；
 *           road             —— 最新道路观测，NULL 表示本拍没有可用视觉帧；
 *           previous_session —— 上一场比赛绑定的视觉会话号。
 * 返回参数：发车条件枚举，只有 START_READY 才允许按 OK 发车。
 * 备注信息：判断顺序即优先级，不能随意交换：
 *           1) 非 STOP -> 先停车/清故障；
 *           2) 车还在动 -> 等停稳；
 *           3) 正在算数独 -> 等结果；
 *           4) 没有视觉帧、曲率无效或置信度 <0.35 -> 等有效路况；
 *           5) 视觉会话号与上一场相同 -> 视觉端没重新开始，要求重启会话；
 *           全部通过才 READY。这样发车瞬间一定“停稳 + 有路 + 新会话”，
 *           避免一按 OK 就冲到上一场的旧曲率上。
 */
StartCondition Menu_StartCondition(const Car_Core *car, uint8_t still, uint8_t pending, const CarVisionObservation *road, uint32_t previous_session)
{
    if (car == NULL) return START_STOP_FIRST;
    if (car->state != CAR_STOP) return START_STOP_FIRST;
    if (!still) return START_MOVING;
    if (pending) return START_RESULT;
    /* 单边估计曲率只能用于巡线转向，不能作为发车依据。 */
    if (!road || (road->valid_mask & VISION_CURVATURE_ESTIMATED) ||
        !(road->valid_mask & VISION_CURVATURE_VALID) || !(road->confidence >= 0.35f))
        return START_NO_ROAD;
    if (road->session == previous_session) return START_NEW_SESSION;
    return START_READY;
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

    if (key.id == KEY_BACK && key.type == KEY_EVENT_PRESS) {
        if (m->depth) {
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
            if (pid && item < 3U) {
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
        } else if (m->group == MENU_RACE && item == 0U && start == START_READY) {
            m->item[MENU_RACE] = 1U;
            return MENU_LAUNCH;
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
    unsigned group, item, count, depth, start, status, phase_count;

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

    if (row == 0) {
        style = UI_STYLE_TITLE;
        snprintf(s,sizeof s,"%s", depth == 0U ? "智能车菜单" : s_nodes[group].name);
    } else if (row == 9) {
        style = UI_STYLE_HINT;
        if (v->state != CAR_STOP) {
            snprintf(s,sizeof s,"PE10: 急停/清故障");
        } else if (depth < 2U) {
            snprintf(s,sizeof s,"上下移动  OK进入  返回上级");
        } else if (group >= MENU_LEFT_PID && item < 3U) {
            snprintf(s,sizeof s,"上下调节  OK应用  返回放弃");
        } else if (group >= MENU_LEFT_PID && item == 3U) {
            snprintf(s,sizeof s,"OK写入Flash  返回");
        } else {
            snprintf(s,sizeof s,"返回: 上一级");
        }
    } else if (row == 8) {
        style = (v->state == CAR_FAULT || v->degraded) ? UI_STYLE_WARN : UI_STYLE_HINT;
        snprintf(s,sizeof s,"%s%s  故障 %u",v->state == CAR_STOP ? "停车" :
                 (v->state == CAR_FAULT ? "故障" : (v->motion == CAR_MOTION_RACE ? "比赛" : "手动")),
                 v->degraded ? " 视觉丢失" : "", (unsigned)v->fault);
    } else if (depth < 2U) {
        unsigned n = row-1;
        unsigned ncount = depth ? count : (unsigned)MENU_GROUPS;
        if (n < ncount) {
            const char *name = depth ? node_name((uint8_t)(s_nodes[group].first + n))
                                     : s_nodes[n].name;
            if (n == (depth ? item : group)) style = UI_STYLE_SELECT;
            snprintf(s,sizeof s,"%s %s",style == UI_STYLE_SELECT ? ">" : " ",name);
        } else if (depth == 0U && row == 5U) {
            /* 设备状态页：摄像头由香橙派 TYPE=3 状态帧报告，不依赖比赛会话。 */
            if (!v->vision_enabled) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"摄像头: 已关闭");
            } else if (v->camera_status == VISION_CAMERA_STREAMING) {
                style = UI_STYLE_GOOD;
                snprintf(s,sizeof s,"摄像头: 已连接");
            } else if (v->camera_status == VISION_CAMERA_NO_FRAME) {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"摄像头: 无画面");
            } else {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"摄像头: 无法确认");
            }
        } else if (depth == 0U && row == 6U) {
            if (!v->vision_enabled) {
                style = UI_STYLE_HINT;
                snprintf(s,sizeof s,"香橙派: 已关闭");
            } else if (v->pi_connected) {
                style = UI_STYLE_GOOD;
                snprintf(s,sizeof s,"香橙派: 已连接");
            } else {
                style = UI_STYLE_WARN;
                snprintf(s,sizeof s,"香橙派: 未连接");
            }
        }
    } else if (row == 1) {
        snprintf(s,sizeof s,"%s",node_name((uint8_t)(s_nodes[group].first + item)));
    } else if (group == MENU_RACE && item == 0U) {
        if (row == 3) snprintf(s,sizeof s,"%s",conditions[start]);
        if (row == 5 && start == START_READY) {
            style = UI_STYLE_EDIT;
            snprintf(s,sizeof s,"OK: 立即发车");
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
