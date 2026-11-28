#ifndef MENU_TASK_H
#define MENU_TASK_H

/*
 * ============================================================================
 * 文件名称：Menu_Task.h
 * ============================================================================
 * 模块功能：板端菜单：三级导航（组 -> 项 -> 值/动作）、发车条件判定、
 *           内环 PID 在线修改与掉电保存入口、数独结果显示。
 *
 *   运行视角
 *     控制中断每拍把按键事件交给 Menu_Key()，它只返回“要做什么”
 *     （MENU_LAUNCH / MENU_SAVE_PARAMS），真正的发车、写 Flash 由平台层执行；
 *     主循环每 MENU_REFRESH_MS 取一次快照并开始刷屏，10 行在随后几轮
 *     主循环里连续画完。
 *
 *   菜单结构（与实车一致）
 *     Race        ：Start race / Race status / Sudoku result / Race RPM
 *     Sensors     ：Wheel speeds / IMU / Vision / Fault log
 *     Left wheel inner PID  ：Kp / Ki / Kd / Save parameters
 *     Right wheel inner PID ：同上
 *
 *   导航结构
 *     组和组内项都组织成环形双向链表：节点表定义名字/所属组/项号，
 *     初始化时把 prev/next 连好。UP/DOWN 只沿 prev/next 走一步，
 *     不需要按数组长度取模，也不存在“走到头”的特殊分支。
 * ============================================================================
 */

#include <stdint.h>

#include "Drv_Key.h"
#include "Task_Fsm.h"
#include "Track_Fsm.h"
#include "Pid_Storage.h"
#include "Proto_Vision.h"
#include "Sudoku.h"

/* ---------------------------------------------------------------------------
 * 菜单刷新节拍
 * 功能：主循环每隔多少毫秒重画一行，10 行刷完整屏约 1.2 s。
 * 取值：120 ms。调小会让屏幕更“跟手”，但会挤占陀螺采样与遥测发送。
 * ------------------------------------------------------------------------- */
#define MENU_REFRESH_MS          120U   /* 单行刷新间隔，毫秒 */

typedef enum { MENU_RACE, MENU_SENSORS, MENU_LEFT_PID, MENU_RIGHT_PID, MENU_GROUPS } MenuGroup;

/* 视觉自动握手阶段：由平台层用 STATUS/会话/新鲜帧计算，菜单与发车条件共用。 */
typedef enum {
    VISION_HS_OFF = 0,       /* 视觉功能未编译/配置关闭 */
    VISION_HS_OFFLINE,       /* 视觉已启用，但链路没有合法帧 */
    VISION_HS_WAIT_HELLO,    /* 链路在线，尚未绑定 HELLO 会话 */
    VISION_HS_STALE,         /* 已绑定会话，但没有新鲜 TYPE=1 */
    VISION_HS_BOUND,         /* 已绑定会话，且有新鲜 TYPE=1 */
    VISION_HS_ERROR,         /* 最近拒绝原因是字段/类型/数独块错误 */
    VISION_HS_COUNT
} VisionHandshakePhase;

/* 握手快照：所有字段在同一拍内取值，避免菜单显示新旧混合状态。 */
typedef struct {
    uint8_t enabled;         /* BOARD_VISION_ENABLE */
    uint8_t link_alive;      /* !Uart_VisionFault() && CarVision_LinkAlive */
    uint8_t session_valid;   /* 已绑定视觉会话 */
    uint8_t fresh;           /* 200 ms 内有新鲜 TYPE=1 */
    uint8_t reject;          /* 最近一次业务拒绝原因，取值见 VISION_REJECT_* */
    VisionHandshakePhase phase;
} VisionHandshake;

/* 发车条件：只有 START_READY 才允许 OK 发车；START_COUNT 只用于边界检查。 */
typedef enum { START_READY, START_STOP_FIRST, START_MOVING, START_RESULT,
               START_NO_VISION, START_NO_SESSION, START_NO_FRAME,
               START_VISION_ERROR, START_NO_ROAD, START_NEW_SESSION,
               START_COUNT } StartCondition;

/* 菜单动作：返回给平台层执行。 */
typedef enum { MENU_IDLE, MENU_LAUNCH, MENU_SAVE_PARAMS } MenuAction;

/* 发车确认：连续收到 3 个同一会话、序号递增、未过期的有效赛道帧才 READY。 */
typedef struct {
    uint32_t session;      /* 确认中的视觉会话号 */
    uint32_t received_ms;  /* 上一确认帧的本机接收时间 */
    uint16_t sequence;     /* 上一确认帧序号 */
    uint8_t count;         /* 已连续确认的帧数 */
} MenuRoadConfirm;

typedef struct {
    uint8_t depth, group, item[MENU_GROUPS], feedback; /* 光标层级/组/项/反馈 */
    uint8_t node;                                      /* 当前链表节点（组节点或项节点） */
    uint8_t race_armed;                                /* 1=已预备，视觉就绪后自动发车 */
    uint8_t fault_page;                                /* 故障记录页：0=0..5，1=6..8 */
    unsigned candidate;                                /* 正在编辑的增益(×100)或转速 */
    uint16_t race_rpm;                                 /* 菜单设定的比赛转速，rpm */
    CarParamsStatus params_status;                     /* 最近一次保存结果 */
} MenuState;

/* 一屏数据快照：平台层在临界区一次性填好，主循环逐行渲染。 */
typedef struct {
    MenuState menu;
    StartCondition start;
    uint8_t state, fault, motion, lap, phase, fresh, degraded; /* 整车/赛段/视觉状态 */
    uint8_t fault_derate;                                      /* 1=当前处于故障降额（参考轮速 90%） */
    uint8_t imu_present, gyro_ok, samples;                     /* 陀螺状态 */
    uint8_t tile_mask;                                         /* 已收到的数独子块位图 */
    uint8_t params_dirty, params_saved;                        /* 参数是否与 Flash 一致 */
    uint8_t launch_road_count;                                 /* 发车已连续确认的有效赛道帧数 */
    uint32_t vision_ok, vision_bad, vision_rejected;           /* 视觉接收统计 */
    CarVisionCameraStatus camera_status;                       /* 摄像头状态（香橙派 TYPE=3） */
    uint8_t pi_connected, vision_enabled, bt_connected;        /* 香橙派/视觉启用/蓝牙上位机 */
    uint8_t have_obs;                                          /* 已有观测快照，用于区分数据过期/等待数据 */
    uint8_t battery_valid;                                     /* 1=电池电压已采样 */
    uint16_t battery_mv;                                       /* 电池供电电压，mV */
    uint8_t session_valid;                                     /* 是否已绑定视觉会话 */
    uint8_t vision_phase;                                      /* VisionHandshakePhase 快照 */
    uint8_t vision_reject;                                     /* 最近一次业务拒绝原因 */
    uint16_t race_rpm;                                         /* 当前比赛转速，rpm */
    uint32_t uptime_s;                                         /* 上电运行时间，秒 */
    uint32_t timing_fault_us;                                  /* 首次故障 5 的控制周期 */
    TrackFaultReason race_fault_reason;                        /* 故障 7 的赛段原因 */
    CarFaultRecord fault_log[CAR_FAULT_CODES];                 /* 每个故障码的计数/首末时间 */
    float distance, yaw_deg, gyro_dps, bias_dps;
    float speed[2], reference[2], pwm[2], gains[3];
    CarVisionObservation obs;
} MenuView;

/*
 * 函数名称：Menu_Init
 * 函数功能：把菜单复位到组页，并建立组/项环形链表。
 * 参数说明：m —— 菜单状态；NULL 时直接返回。
 * 返回参数：无。
 * 备注信息：只复位导航与编辑状态，params_status 回到 PARAMS_DEFAULTS；
 *           平台层随后按 Flash 加载结果覆盖它。
 */
void Menu_Init(MenuState *m);

/*
 * 函数名称：VisionHs_Phase
 * 函数功能：由视觉启用/链路/会话/新鲜帧/最近拒绝原因计算握手阶段。
 * 参数说明：enabled       —— BOARD_VISION_ENABLE；
 *           link_alive    —— 是否收到任意 CRC 正确帧；
 *           session_valid —— 是否已绑定 HELLO 会话；
 *           fresh         —— 200 ms 内是否有新鲜 TYPE=1；
 *           reject        —— 最近一次业务拒绝原因。
 * 返回参数：VisionHandshakePhase。
 * 备注信息：判定顺序固定：OFF -> OFFLINE -> ERROR -> WAIT_HELLO -> STALE -> BOUND。
 *           ERROR 只认 6/7/8（字段/类型长度/数独块），与图像端 PROTOCOL_ERROR 对齐。
 */
VisionHandshakePhase VisionHs_Phase(uint8_t enabled, uint8_t link_alive,
                                    uint8_t session_valid, uint8_t fresh, uint8_t reject);

/*
 * 函数名称：Menu_StartCondition
 * 函数功能：综合整车状态、轮速、数独收尾、握手阶段与视觉路况，给出能否发车。
 * 参数说明：confirm          —— 连续有效帧确认状态；首次传全 0；
 *           car              —— 控制对象；still —— 轮速是否近似为零；
 *           pending          —— 是否正在准备数独结果；
 *           hs               —— 本拍握手快照；
 *           road             —— 最新道路观测，NULL 表示本拍没有可用视觉帧；
 *           previous_session —— 上一场比赛的视觉会话号。
 * 返回参数：发车条件枚举；只有 START_READY 才允许发车。
 * 备注信息：要求连续 3 个同一会话、序号递增、未过期的有效赛道帧，
 *           避免单帧误检就发车；条件中断会清零确认计数。
 *           START_NO_VISION 只表示视觉未编译/配置关闭；未握手和无新帧分开报告。
 */
StartCondition Menu_StartCondition(MenuRoadConfirm *confirm, const Car_Core *car, uint8_t still, uint8_t pending, const VisionHandshake *hs, const CarVisionObservation *road, uint32_t previous_session);

/*
 * 函数名称：Menu_GetRaceRpm / Menu_Disarm
 * 函数功能：读取比赛转速（0 时回退默认值）/ 取消“预备自动发车”。
 * 参数说明：m —— 菜单状态；可为 NULL。
 * 返回参数：Menu_GetRaceRpm 返回夹紧后的转速，rpm。
 */
uint16_t Menu_GetRaceRpm(const MenuState *m);
void Menu_Disarm(MenuState *m);

/*
 * 函数名称：Menu_PollArmed
 * 函数功能：检查“预备发车”状态；视觉就绪时返回 MENU_LAUNCH。
 * 参数说明：m     —— 菜单状态；
 *           start —— 当前发车条件。
 * 返回参数：MENU_IDLE / MENU_LAUNCH。
 * 备注信息：车动、故障、离开发车页或正在收尾时会自动取消预备。
 */
MenuAction Menu_PollArmed(MenuState *m, StartCondition start);

/*
 * 函数名称：Menu_Key
 * 函数功能：处理一个按键事件，推进光标或修改/应用增益，返回要执行的动作。
 * 参数说明：m —— 菜单状态；key —— 按键事件；car —— 控制对象；
 *           start —— 当前发车条件。
 * 返回参数：MENU_IDLE / MENU_LAUNCH / MENU_SAVE_PARAMS。
 * 备注信息：增益修改只在 STOP 生效；保存请求由平台层确认静止后执行。
 */
MenuAction Menu_Key(MenuState *m, Key_Event_Struct key, Car_Core *car, StartCondition start);

/*
 * 函数名称：Menu_ShowResult
 * 函数功能：把菜单位置跳到 “Race -> Sudoku result” 页。
 * 参数说明：m —— 菜单状态。
 * 返回参数：无。
 */
void Menu_ShowResult(MenuState *m);

/*
 * 函数名称：Menu_DrawRow
 * 函数功能：按当前快照画第 row 行（0..9）。
 * 参数说明：v —— 快照；row —— 行号；board —— 数独答案；result —— 求解状态。
 * 返回参数：无。
 */
void Menu_DrawRow(const MenuView *v, unsigned row, const uint8_t board[36], uint8_t result);

#endif
