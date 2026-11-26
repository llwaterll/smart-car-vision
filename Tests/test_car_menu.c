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
#if BOARD_VISION_ENABLE
    road.sequence = 65534U;
    road.received_ms = 10U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    for (unsigned tick = 0; tick < 4; ++tick)
        assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.sequence = 0U;                 /* 序号回绕，但中间夹着非道路业务帧 */
    road.received_ms = 20U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.sequence = 1U;
    road.received_ms = 30U;
    road.element = VISION_ELEMENT_CROSS;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.element = VISION_ELEMENT_NONE;
    for (unsigned i = 1; i <= 3; ++i) { /* 连续 3 帧有效赛道才 READY */
        ++road.sequence;
        road.received_ms += 40U;
        assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) ==
               (i == 3 ? START_READY : START_NO_ROAD));
    }
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_READY);
    ++road.sequence;
    road.received_ms += VIS_MAX_AGE_MS + 1U; /* 帧间隔超时：重新计数 */
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += VIS_MAX_AGE_MS;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    ++road.sequence;
    road.received_ms += 10U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_READY);
    road.session = 13U;                 /* 会话切换：重新确认 */
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.session = 12U;
    /* Single-side estimated curvature may steer, but must never launch. */
    road.valid_mask = VISION_CURVATURE_VALID | VISION_CURVATURE_ESTIMATED;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.valid_mask = VISION_CURVATURE_VALID;
    assert(Menu_StartCondition(&confirm,&car,0,0,&road,11) == START_MOVING);
    assert(Menu_StartCondition(&confirm,&car,1,1,&road,11) == START_RESULT);
    assert(Menu_StartCondition(&confirm,&car,1,0,NULL,11) == START_NO_VISION);
    road.session = 0U;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_VISION);
    road.session = 12U;
    road.confidence = NAN;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_ROAD);
    road.confidence = 0.9f;
    road.valid_mask = 0;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,12) == START_NEW_SESSION);
    road.valid_mask = VISION_CURVATURE_VALID;
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,12) == START_NEW_SESSION);
#else
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_NO_VISION);
    assert(Menu_StartCondition(&confirm,&car,0,0,&road,11) == START_MOVING);
    assert(Menu_StartCondition(&confirm,&car,1,1,&road,11) == START_RESULT);
#endif

    /* Entering either menu level must never start a motor. Long OK cannot launch. */
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE && menu.depth == 2);
    {
        Key_Event_Struct repeat = {KEY_OK,KEY_EVENT_REPEAT};
        assert(Menu_Key(&menu,repeat,&car,START_READY) == MENU_IDLE);
    }
    assert(press(&menu,&car,KEY_OK,START_NO_ROAD) == MENU_IDLE && car.state == CAR_STOP);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_LAUNCH);
    assert(menu.item[MENU_RACE] == 1);
    assert(press(&menu,&car,KEY_OK,START_READY) == MENU_IDLE);
    assert(Car_StartRace(&car));
    assert(Menu_StartCondition(&confirm,&car,1,0,&road,11) == START_STOP_FIRST);
    Car_LocalStop(&car);

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
    memset(&confirm,0,sizeof confirm);
    road.element = VISION_ELEMENT_NONE;
    road.valid_mask = VISION_CURVATURE_VALID;
    road.confidence = 0.9f;
    road.session = 12U;
    for (unsigned i = 1; i <= 3; ++i) {
        StartCondition state;
        ++road.sequence;
        road.received_ms += 40U;
        state = Menu_StartCondition(&confirm,&car,1,0,&road,11);
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
    assert(Menu_StartCondition(&confirm,NULL,1,0,&road,11) == START_STOP_FIRST);
    Menu_ShowResult(NULL);
    Menu_DrawRow(NULL,0,board,0);

    /* Render every detail page through the actual formatter and fixed row width. */
    view.menu.depth = 2;
    view.samples = 100;
    view.imu_present = 1;
    for (group=0; group<MENU_GROUPS; ++group) {
        view.menu.group = (uint8_t)group;
        for (item=0; item<(group==MENU_RACE?4U:(group>=MENU_LEFT_PID?4U:3U)); ++item) {
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
    view.camera_status = VISION_CAMERA_NO_FRAME;
    view.pi_connected = 0;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"无画面"));
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"未连接"));
    view.camera_status = VISION_CAMERA_STREAMING;
    view.pi_connected = 1;
    Menu_DrawRow(&view,5,board,0);
    assert(strstr(rows[5],"已连接"));
    Menu_DrawRow(&view,6,board,0);
    assert(strstr(rows[6],"已连接"));
    view.vision_enabled = 0;
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

    /* Out-of-range row/start/phase/save-status must clamp, never index out of bounds. */
    Menu_DrawRow(&view,LCD_ROWS,board,0);
    memset(&view,0,sizeof view);
    view.menu.depth = 2;
    view.menu.group = MENU_RACE;
    view.menu.item[MENU_RACE] = 0;
    view.start = (StartCondition)99;
    Menu_DrawRow(&view,4,board,0);
    assert(strstr(rows[4],"等待识别"));
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
