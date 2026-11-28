#include "Menu_Task.h"
#include "Sudoku.h"
#include "Proto_Bluetooth.h"
#include "Drv_Lcd.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char rows[LCD_ROWS][LCD_LINE_BYTES];

/* 与 Drv_Lcd 的逻辑列规则一致：ASCII 1 列，UTF-8 汉字 2 列。 */
static unsigned display_columns(const char *s)
{
    unsigned n = 0;
    while (*s) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x80U) { n++; s++; }
        else if ((c & 0xE0U) == 0xC0U) { n += 2; s += 2; }
        else if ((c & 0xF0U) == 0xE0U) { n += 2; s += 3; }
        else { n++; s++; }
    }
    return n;
}

void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    va_list args;
    int n;
    (void)style;
    assert(row < LCD_ROWS);
    va_start(args,fmt);
    n = vsnprintf(rows[row],sizeof rows[row],fmt,args);
    va_end(args);
    assert(n >= 0 && n < (int)sizeof rows[row]);
    assert(display_columns(rows[row]) <= LCD_COLS);
}

static MenuAction press(MenuState *m, Car_Core *c, unsigned key, StartCondition ready)
{
    Key_Event_Struct event = {(uint8_t)key,KEY_EVENT_PRESS};
    return Menu_Key(m,event,c,ready);
}

int main(void)
{
    Car_Core car;
    MenuState menu = {0};
    MenuRoadConfirm confirm = {0};
    CarVisionObservation road = {0};
    VisionHandshake hs = {0};
    MenuView view = {0};
    uint8_t board[36] = {0};
    unsigned row, group, item;
    float before;
    Car_Init(&car);
    /* PID submenu must offer a fourth entry for explicit persistent saving. */
    menu.depth = 1;
    menu.group = MENU_LEFT_PID;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.item[MENU_LEFT_PID] == 3);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(menu.depth == 2);
    {
        Key_Event_Struct repeat = {KEY_OK,KEY_EVENT_REPEAT};
        assert(Menu_Key(&menu,repeat,&car,START_READY)==MENU_IDLE);
    }
    car.speed[0]=3;
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    car.speed[0]=0;
    car.state=CAR_RUN;
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    car.state=CAR_STOP;
    assert(press(&menu,&car,KEY_OK,START_RESULT)==MENU_IDLE);
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_SAVE_PARAMS);
    assert(menu.params_status==PARAMS_BUSY);
    assert(press(&menu,&car,KEY_OK,START_READY)==MENU_IDLE);
    memset(&menu,0,sizeof menu);
    memset(&confirm,0,sizeof confirm);
    road.session = 12;
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
    hs.enabled = BOARD_VISION_ENABLE;
    hs.link_alive = 1U;
    hs.session_valid = 1U;
    hs.fresh = 1U;
    hs.reject = VISION_REJECT_NONE;
    hs.phase = VISION_HS_BOUND;
    /* 握手阶段判定顺序：关闭 > 离线 > 协议错误 > 未握手 > 无新帧 > 已绑定。 */
    assert(VisionHs_Phase(0,1,1,1,VISION_REJECT_NONE) == VISION_HS_OFF);
    assert(VisionHs_Phase(1,0,0,0,VISION_REJECT_NONE) == VISION_HS_OFFLINE);
    assert(VisionHs_Phase(1,1,1,1,VISION_REJECT_PAYLOAD) == VISION_HS_ERROR);
    assert(VisionHs_Phase(1,1,0,0,VISION_REJECT_NONE) == VISION_HS_WAIT_HELLO);
    assert(VisionHs_Phase(1,1,1,0,VISION_REJECT_NONE) == VISION_HS_STALE);
    assert(VisionHs_Phase(1,1,1,1,VISION_REJECT_NONE) == VISION_HS_BOUND);
#if BOARD_VISION_ENABLE
    road.sequence = 65534U;
    road.received_ms = 10U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    for (unsigned tick = 0; tick < 4; ++tick)
        assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.sequence = 0U;                 /* 序号回绕，但中间夹着非道路业务帧 */
    road.received_ms = 20U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.sequence = 1U;
    road.received_ms = 30U;
    road.element = VISION_ELEMENT_CROSS;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.element = VISION_ELEMENT_NONE;
    for (unsigned i = 1; i <= 3; ++i) { /* 连续 3 帧有效赛道才 READY */
        ++road.sequence;
        road.received_ms += 40U;
        assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) ==
               (i == 3 ? START_READY : START_NO_ROAD));
    }
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_READY);
    ++road.sequence;
    road.received_ms += VIS_MAX_AGE_MS + 1U; /* 帧间隔超时：重新计数 */
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += VIS_MAX_AGE_MS;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += 10U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_READY);
    road.session = 13U;                 /* 会话切换：重新确认 */
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.session = 12U;
    /* Single-side estimated curvature may steer, but must never launch. */
    road.valid_mask = VISION_CURVATURE_VALID | VISION_CURVATURE_ESTIMATED;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.valid_mask = VISION_CURVATURE_VALID;
    assert(Menu_StartCondition(&confirm,&car,0,0,&hs,&road,11) == START_MOVING);
    assert(Menu_StartCondition(&confirm,&car,1,1,&hs,&road,11) == START_RESULT);
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,NULL,11) == START_NO_FRAME);
    road.session = 0U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_FRAME);
    road.session = 12U;
    road.confidence = NAN;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_ROAD);
    road.confidence = 0.9f;
    road.valid_mask = 0;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,12) == START_NEW_SESSION);
    road.valid_mask = VISION_CURVATURE_VALID;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,12) == START_NEW_SESSION);

    /* 握手阶段拆分：未握手、无新帧、协议错误必须给出不同原因。 */
    hs.phase = VISION_HS_WAIT_HELLO; hs.session_valid = 0U; hs.fresh = 0U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,NULL,11) == START_NO_SESSION);
    hs.phase = VISION_HS_STALE; hs.session_valid = 1U; hs.fresh = 0U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,NULL,11) == START_NO_FRAME);
    hs.phase = VISION_HS_ERROR; hs.reject = VISION_REJECT_PAYLOAD;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,NULL,11) == START_VISION_ERROR);
    hs.phase = VISION_HS_OFF; hs.enabled = 0U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,NULL,11) == START_NO_VISION);
    /* 恢复 BOUND，后续发车/预备用例继续使用。 */
    hs.enabled = BOARD_VISION_ENABLE;
    hs.session_valid = 1U; hs.fresh = 1U; hs.reject = VISION_REJECT_NONE;
    hs.phase = VISION_HS_BOUND;
#else
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_NO_VISION);
    assert(Menu_StartCondition(&confirm,&car,0,0,&hs,&road,11) == START_MOVING);
    assert(Menu_StartCondition(&confirm,&car,1,1,&hs,&road,11) == START_RESULT);
#endif

    /* Entering either menu level must never start a motor. Long OK cannot launch. */
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    {
        Key_Event_Struct repeat = {KEY_OK,KEY_EVENT_REPEAT};
        assert(Menu_Key(&menu,repeat,&car,START_READY) == MENU_IDLE);
    }
    assert(press(&menu,&car,KEY_OK,START_NO_ROAD) == MENU_IDLE && car.state == CAR_STOP);
#if BOARD_VISION_ENABLE
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_LAUNCH);
    assert(menu.item[MENU_RACE] == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(Car_StartRace(&car));
    assert(Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11) == START_STOP_FIRST);
    Car_LocalStop(&car);
#else
    /* 视觉关闭时发车页只能给出 FB_INVALID，绝不能返回 MENU_LAUNCH。 */
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(menu.item[MENU_RACE] == 0U && menu.race_armed == 0U);
    assert(menu.feedback != 0U);
#endif

    /* Race RPM (4th item): fine/coarse edit, BACK discards, OK applies. */
    Menu_Init(&menu);
    press(&menu,&car,KEY_OK,START_READY);          /* 进入比赛控制组 */
    press(&menu,&car,KEY_UP,START_READY);          /* 环形链表上一项 = 比赛转速 */
    assert(menu.item[MENU_RACE] == 3);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    assert(menu.candidate == TRACK_DEFAULT_RPM);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.candidate == TRACK_DEFAULT_RPM + TRACK_RPM_STEP_FINE);
    {
        Key_Event_Struct repeat = {KEY_UP,KEY_EVENT_REPEAT};
        assert(Menu_Key(&menu,repeat,&car,START_READY) == MENU_IDLE);
    }
    assert(menu.candidate == TRACK_DEFAULT_RPM + TRACK_RPM_STEP_FINE + TRACK_RPM_STEP_COARSE);
    press(&menu,&car,KEY_BACK,START_READY);
    assert(menu.depth == 1 && Menu_GetRaceRpm(&menu) == TRACK_DEFAULT_RPM);
    press(&menu,&car,KEY_OK,START_READY);
    {
        Key_Event_Struct repeat = {KEY_DOWN,KEY_EVENT_REPEAT};
        assert(Menu_Key(&menu,repeat,&car,START_READY) == MENU_IDLE);
    }
    press(&menu,&car,KEY_OK,START_READY);
    assert(Menu_GetRaceRpm(&menu) == TRACK_DEFAULT_RPM - TRACK_RPM_STEP_COARSE);
    /* 运行中不能应用转速。 */
    assert(Car_StartRace(&car));
    press(&menu,&car,KEY_UP,START_STOP_FIRST);
    press(&menu,&car,KEY_OK,START_STOP_FIRST);
    assert(Menu_GetRaceRpm(&menu) == TRACK_DEFAULT_RPM - TRACK_RPM_STEP_COARSE);
    Car_LocalStop(&car);

    /* 发车页直接调速 + 预备自动发车。 */
    Menu_Init(&menu);
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_DOWN,START_NO_VISION);
    assert(Menu_GetRaceRpm(&menu) == TRACK_DEFAULT_RPM - TRACK_RPM_STEP_FINE);
    menu.race_rpm = TRACK_MIN_RPM;
    press(&menu,&car,KEY_DOWN,START_NO_VISION);
    assert(Menu_GetRaceRpm(&menu) == TRACK_MIN_RPM);
    menu.race_rpm = TRACK_MAX_RPM;
    press(&menu,&car,KEY_UP,START_NO_VISION);
    assert(Menu_GetRaceRpm(&menu) == TRACK_MAX_RPM);
    menu.race_rpm = 120U;
#if BOARD_VISION_ENABLE
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 1U);
    assert(Menu_PollArmed(&menu,START_NO_ROAD) == MENU_IDLE && menu.race_armed == 1U);
    /* 未握手和无新帧保持预备；协议错误取消预备且给出反馈。 */
    assert(Menu_PollArmed(&menu,START_NO_FRAME) == MENU_IDLE && menu.race_armed == 1U);
    assert(Menu_PollArmed(&menu,START_VISION_ERROR) == MENU_IDLE && menu.race_armed == 0U);
    assert(press(&menu,&car,KEY_OK,START_VISION_ERROR) == MENU_IDLE &&
           menu.race_armed == 0U && menu.feedback != 0U);
    assert(press(&menu,&car,KEY_OK,START_NO_SESSION) == MENU_IDLE && menu.race_armed == 1U);
    memset(&confirm,0,sizeof confirm);
    road.element = VISION_ELEMENT_NONE;
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
    road.session = 12U;
    for (unsigned i = 1; i <= 3; ++i) {
        StartCondition state;
        ++road.sequence;
        road.received_ms += 40U;
        state = Menu_StartCondition(&confirm,&car,1,0,&hs,&road,11);
        assert(Menu_PollArmed(&menu,state) == (i == 3 ? MENU_LAUNCH : MENU_IDLE));
    }
    assert(menu.race_armed == 0U && menu.item[MENU_RACE] == 1U && Menu_GetRaceRpm(&menu) == 120U);
    menu.item[MENU_RACE] = 0U;
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_LAUNCH);
    assert(menu.item[MENU_RACE] == 1U);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(Car_StartRace(&car));
    Car_LocalStop(&car);
#else
    assert(press(&menu,&car,KEY_OK,START_NO_VISION) == MENU_IDLE && menu.race_armed == 0U);
#endif

    memset(&menu,0,sizeof menu);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.group == MENU_RIGHT_PID);
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.group == MENU_LEFT_PID);
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    before = car.pid_speed[0].kp;
    press(&menu,&car,KEY_UP,START_READY);
    press(&menu,&car,KEY_BACK,START_READY);
    assert(car.pid_speed[0].kp == before); /* cancellation preserves gains */
    press(&menu,&car,KEY_OK,START_READY);
    press(&menu,&car,KEY_UP,START_READY);
    press(&menu,&car,KEY_OK,START_READY);
    assert(fabsf(car.pid_speed[0].kp-before-0.01f) < 0.0001f);
    before = car.pid_speed[0].kp;
    assert(Car_StartRace(&car));
    press(&menu,&car,KEY_UP,START_STOP_FIRST);
    press(&menu,&car,KEY_OK,START_STOP_FIRST);
    assert(car.pid_speed[0].kp == before);
    Car_LocalStop(&car);
    menu.candidate = 65535;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.candidate == 65535);
    menu.candidate = 0;
    press(&menu,&car,KEY_DOWN,START_READY);
    assert(menu.candidate == 0);
    assert(Car_SetPid(&car,2,1,1,1) == 2);
    assert(Car_SetPid(&car,0,NAN,1,1) == 2);
    assert(car.pid_speed[0].kp == before);
    Menu_ShowResult(&menu);
    assert(menu.depth == 2 && menu.group == MENU_RACE && menu.item[MENU_RACE] == 2);

    /* Menu_Init rebuilds the circular links and returns to the group page. */
    Menu_Init(&menu);
    assert(menu.depth == 0 && menu.group == MENU_RACE && menu.item[MENU_RACE] == 0);
    assert(menu.node == MENU_RACE);

    /* Corrupt indices are clamped, not followed out of the node table. */
    menu.depth = 9;
    menu.group = (uint8_t)(MENU_GROUPS + 5);
    press(&menu,&car,KEY_OK,START_READY);
    assert(menu.depth == 1 && menu.group == MENU_RACE);
    memset(&menu,0,sizeof menu);
    menu.depth = 1;
    menu.group = MENU_RACE;
    menu.item[MENU_RACE] = 99;
    press(&menu,&car,KEY_UP,START_READY);
    assert(menu.group == MENU_RACE && menu.item[MENU_RACE] < 4);

    /* Invalid key ids and NULL objects are ignored instead of being dereferenced. */
    {
        Key_Event_Struct bad = {(uint8_t)KEY_NUM, KEY_EVENT_PRESS};
        Key_Event_Struct ok = {KEY_OK, KEY_EVENT_PRESS};
        assert(Menu_Key(&menu,bad,&car,START_READY) == MENU_IDLE);
        assert(Menu_Key(NULL,ok,&car,START_READY) == MENU_IDLE);
    }
    assert(Menu_StartCondition(&confirm,NULL,1,0,&hs,&road,11) == START_STOP_FIRST);
    Menu_ShowResult(NULL);
    Menu_DrawRow(NULL,0,board,0);

    /* Render every detail page through the actual formatter and fixed row width. */
    view.menu.depth = 2;
    view.samples = 100;
    view.imu_present = 1;
    for (group=0; group<MENU_GROUPS; ++group) {
        view.menu.group = (uint8_t)group;
        for (item=0; item<4U; ++item) {
            view.menu.item[group] = (uint8_t)item;
            for (row=0; row<LCD_ROWS; ++row) Menu_DrawRow(&view,row,board,2);
        }
    }
    view.menu.group=MENU_RIGHT_PID;
    view.menu.item[MENU_RIGHT_PID]=3;
    for(item=PARAMS_DEFAULTS;item<=PARAMS_STOP_FIRST;item++) {
        view.menu.params_status=(CarParamsStatus)item;
        for(row=0;row<LCD_ROWS;row++) Menu_DrawRow(&view,row,board,0);
    }
    view.menu.params_status=PARAMS_SAVED;
    view.params_saved=1; view.params_dirty=1;
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"有未保存"));
    view.start=START_RESULT;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"请等待数独"));
    view.start=START_READY;
    view.menu.group = MENU_SENSORS;
    view.menu.item[MENU_SENSORS] = 1;
    view.gyro_ok = 0;
    Menu_DrawRow(&view,5,board,0);
    assert(!rows[5][0]); /* stale gyro values are hidden */
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 2;
    view.tile_mask = 5;
    Menu_DrawRow(&view,2,board,SUDOKU_AMBIGUOUS);
    assert(strstr(rows[2],"存在多解"));
    Menu_DrawRow(&view,3,board,SUDOKU_MISSING);
    assert(strstr(rows[3],"已收"));
    Menu_DrawRow(&view,4,board,SUDOKU_MISSING);
    assert(strstr(rows[4],"缺失"));
    Menu_DrawRow(&view,2,board,SUDOKU_INVALID);
    assert(strstr(rows[2],"题面冲突"));

    /* Camera / Orange Pi status rows come from the vision protocol snapshot. */
    memset(&view,0,sizeof view);
    view.menu.depth = 0;
    view.vision_enabled = 1;
    view.vision_phase = VISION_HS_OFFLINE;
    view.camera_status = VISION_CAMERA_NO_FRAME;
    view.pi_connected = 0;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"无画面"));
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"未连接"));
    view.camera_status = VISION_CAMERA_STREAMING;
    view.pi_connected = 1;
    view.vision_phase = VISION_HS_BOUND;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"已连接"));
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"已连接"));
    /* 链路在但未握手：首页不能显示“已连接”，必须与发车页一致。 */
    view.vision_phase = VISION_HS_WAIT_HELLO;
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"未握手"));
    view.vision_phase = VISION_HS_ERROR;
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"协议错误"));
    view.vision_enabled = 0;
    view.vision_phase = VISION_HS_OFF;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"已关闭"));
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"已关闭"));
    view.bt_connected = 1;
    Menu_DrawRow(&view,7,board,0);
    assert(strstr(rows[7],"已连接"));
    view.bt_connected = 0;
    Menu_DrawRow(&view,7,board,0);
    assert(strstr(rows[7],"未连接"));

    /* 电池电压显示在蓝牙状态右侧；未采样时用占位符，不能显示成 0 V。 */
    view.battery_valid = 0;
    Menu_DrawRow(&view,7,board,0);
    assert(strstr(rows[7],"未连接") && strstr(rows[7],"--.-V"));
    view.battery_valid = 1;
    view.battery_mv = 12600U;
    Menu_DrawRow(&view,7,board,0);
    assert(strstr(rows[7],"未连接") && strstr(rows[7],"12.6V"));
    view.bt_connected = 1;
    Menu_DrawRow(&view,7,board,0);
    assert(strstr(rows[7],"已连接") && strstr(rows[7],"12.6V"));

    /* 发车页第 3/4 行：握手阶段与条件文案必须一一对应。 */
    memset(&view,0,sizeof view);
    view.menu.depth = 2;
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 0;
    view.vision_enabled = 1;
    view.vision_phase = VISION_HS_WAIT_HELLO;
    view.start = START_NO_SESSION;
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"等待握手"));
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"等待视觉握手"));
    view.vision_phase = VISION_HS_ERROR;
    view.start = START_VISION_ERROR;
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"协议错误"));
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"视觉协议错误"));
    view.vision_phase = VISION_HS_STALE;
    view.start = START_NO_FRAME;
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"握手后无帧"));
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"视觉无新帧"));
    view.vision_phase = VISION_HS_OFFLINE;
    view.start = START_NO_SESSION;
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"链路离线"));

    /* 标题运行时间、故障 5/7 细分和预备发车状态栏与原版菜单一致。 */
    view.uptime_s = 1234567U;
    Menu_DrawRow(&view,0,board,0);
    assert(strstr(rows[0],"T1234567"));
    view.state = CAR_FAULT;
    view.fault = CAR_TIMING_FAULT;
    view.timing_fault_us = 20000U;
    Menu_DrawRow(&view,8,board,0);
    assert(strstr(rows[8],"故障 5") && strstr(rows[8],"20000us"));
    /* 记录模式下故障不停车：state=RUN 也要显示故障 5 细分。 */
    view.state = CAR_RUN;
    Menu_DrawRow(&view,8,board,0);
    assert(strstr(rows[8],"故障 5") && strstr(rows[8],"20000us"));
    view.state = CAR_FAULT;
    view.fault = CAR_RACE_FAULT;
    view.race_fault_reason = TRACK_FAULT_GAP;
    Menu_DrawRow(&view,8,board,0);
    assert(strstr(rows[8],"故障 7") && strstr(rows[8],"断路超限"));
    view.state = CAR_STOP;
    view.fault = 0;
    view.menu.race_armed = 1;
    view.race_rpm = 250U;
    Menu_DrawRow(&view,8,board,0);
    assert(strstr(rows[8],"待视觉握手") && strstr(rows[8],"250RPM"));
    view.menu.race_armed = 0;
    /* 故障降额提示：仅当降额开启时才显示“限速xx%”。 */
    view.fault_derate = 1;
    Menu_DrawRow(&view,8,board,0);
#if CAR_FAULT_DERATE_PERCENT > 0U
    assert(strstr(rows[8],"限速90%"));
#else
    assert(!strstr(rows[8],"限速"));
#endif
    view.fault_derate = 0;

    /* 故障记录页：每个故障码独立计数，上下翻页，OK 清空但不停车。 */
    memset(&view,0,sizeof view);
    view.menu.depth = 2;
    view.menu.group = MENU_SENSORS;
    view.menu.item[MENU_SENSORS] = 3;
    view.fault_log[CAR_LINK_FAULT].count = 2;
    view.fault_log[CAR_LINK_FAULT].first_ms = 1000;
    view.fault_log[CAR_LINK_FAULT].last_ms = 5000;
    view.fault_log[CAR_OVERSPEED_FAULT].count = 1;
    view.fault_log[CAR_OVERSPEED_FAULT].last_ms = 9000;
    view.menu.fault_page = 0;
    Menu_DrawRow(&view,1,board,0);
    assert(strstr(rows[1],"P1/2"));
    Menu_DrawRow(&view,2,board,0);
    assert(strstr(rows[2],"无故障"));
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"通信超时") && strstr(rows[3],"x2") && strstr(rows[3],"5s"));
    view.menu.fault_page = 1;
    Menu_DrawRow(&view,1,board,0);
    assert(strstr(rows[1],"P2/2"));
    Menu_DrawRow(&view,4,board,0);   /* 第 2 页：idx 8 = 超速 */
    assert(strstr(rows[4],"超速") && strstr(rows[4],"x1"));
    Menu_DrawRow(&view,9,board,0);
    assert(strstr(rows[9],"OK清空"));

    memset(&menu,0,sizeof menu);
    menu.depth = 2;
    menu.group = MENU_SENSORS;
    menu.item[MENU_SENSORS] = 3;
    menu.fault_page = 1;
    car.fault_log[CAR_STALL_FAULT].count = 1;
    car.fault = CAR_STALL_FAULT;
    {
        uint8_t state_before = car.state;
        assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
        assert(car.fault_log[CAR_STALL_FAULT].count == 0 && car.fault == CAR_NO_FAULT);
        assert(car.state == state_before);
        press(&menu,&car,KEY_UP,START_READY);
        assert(menu.fault_page == 0);
        press(&menu,&car,KEY_DOWN,START_READY);
        assert(menu.fault_page == 1);
    }

    /* Out-of-range row/start/phase/save-status must clamp, never index out of bounds. */
    Menu_DrawRow(&view,LCD_ROWS,board,0);
    memset(&view,0,sizeof view);
    view.menu.depth = 2;
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 0;
    view.start = (StartCondition)99;
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"等待有效"));
    view.menu.item[MENU_RACE] = 1;
    view.lap = 1;
    view.phase = 99;
    Menu_DrawRow(&view,2,board,0);
    assert(strstr(rows[2],"故障"));
    view.menu.group = MENU_LEFT_PID;
    view.menu.item[MENU_LEFT_PID] = 3;
    view.menu.params_status = (CarParamsStatus)99;
    Menu_DrawRow(&view,3,board,0);
    assert(strstr(rows[3],"读写失败"));
    puts("PASS three-level menu, launch gates, PID transaction, cancellation and rendering");
    return 0;
}
