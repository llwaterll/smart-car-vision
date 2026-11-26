/*
 * ============================================================================
 * 文件名称：Platform_Main.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 整车装配与调度，本文件是唯一“知道所有模块”的地方：
 *
 *   共享对象：Car_Core（任务层）、TrackState（巡线）、CarVision（视觉缓存）、
 *             MenuState（菜单）、CarParams（掉电参数）、数独结果与收尾状态。
 *
 *   控制中断 Car_PIT_Handler()（5 ms，实测周期）
 *     1. Clock_PeriodUs()        取实际周期
 *     2. Encoder_Delta()         取左右轮计数增量
 *     3. 写参数期间：强制停车、输出 0，丢弃本拍
 *     4. 蓝牙故障 -> 停车并清队列；否则每拍最多解析 32 字节命令
 *     5. track_tick()            按键/发车 + Track_Step 写目标
 *     6. Car_TickDt() + Motor_Apply()  闭环并输出 PWM
 *     7. 视觉：故障则作废缓存，否则每拍最多解析 64 字节
 *     8. finish_tick() + 每 20 拍发一帧 16 通道遥测
 *
 *   主循环
 *     poll_params() 写 Flash（阻塞，必须停车后做）
 *     Vofa_Poll()   缓慢发送遥测
 *     Gyro_Poll()   每 10 ms 读一次陀螺
 *     poll_menu()   每 120 ms 画一行菜单
 *
 *   为什么这样分：
 *     中断里只做“必须准时”的事；刷屏、写 Flash、读陀螺都放主循环，
 *     避免把 5 ms 的控制节拍拖长。视觉解析也放在输出之后，保证电机
 *     输出不受解析耗时影响。
 *
 * 函数列表
 *   static uint8_t copy_tiles(uint8_t tiles[4][9], uint32_t now)
 *   static void    finish_tick(uint32_t now)
 *   static void    poll_menu(void)
 *   static void    track_tick(int32_t l, int32_t r, uint32_t dt_us)
 *   static void    poll_params(void)
 *   void           Car_PIT_Handler(void)
 *   void           Car_Main(void)
 *   void           Car_HoldMotorsOff(void)
 *   uint8_t        Car_GetVisionObservation(CarVisionObservation *out)
 *   uint8_t        Car_GetVisionTile(uint8_t index, CarVisionTile *out)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Platform_Main.h"
#include <math.h>
#include <string.h>

/* 整车控制对象：目标、实测、PID 和保护计时。 */
static Car_Core car;

/* 巡线状态：里程、航向、赛段。 */
static TrackState track;

/* 视觉协议解析器：只维护观测与数独缓存，不直接控制电机。 */
static CarVision vision;

/* 菜单状态与最近一次发车条件。 */
static MenuState menu;
static StartCondition menu_start;

/* 发车连续帧确认：必须连续 3 个有效赛道帧才 READY。 */
static MenuRoadConfirm launch_road;

/* 蓝牙上位机链路：最近一次解析出完整命令帧的时间；仅用于设备状态显示。
 * 与 Proto_Bluetooth 的运行看门狗（BT_LINK_TIMEOUT_MS=500 ms）相互独立：
 * 这里只回答“上位机还连着吗”，不参与停车保护。 */
static uint32_t bt_last_ms;
#define BT_STATUS_TIMEOUT_MS 2500U

/* 掉电参数：最近一次保存成功的内容，用于判断“RAM 与 Flash 是否一致”。 */
static CarParams saved_params;
static uint8_t params_saved;

/* 数独显示缓冲与求解状态。 */
static uint8_t result_board[36];
static uint8_t result_status;

/* 完赛收尾：等车停稳、给最后一帧数独最多 1 s，然后求解。 */
static uint8_t finish_tiles[4][9];
static volatile uint8_t finish_pending, finish_ready;
static uint32_t finish_started_ms, finish_still_ms, display_session;
static uint32_t race_session;    /* 本场比赛绑定的视觉会话；0 表示没有进行中的轮次 */
static uint8_t finish_timing, finish_tile_mask;

/* 整车是否静止（停车且双轮转速近似为零），陀螺零偏标定用。 */
static volatile uint8_t chassis_still;

/* 平台层请求标志：写参数、时基重挂、菜单整屏重画。 */
static volatile uint8_t params_pending, params_rebase, menu_refresh;

/* ---------------------------------------------------------------------------
 * 独立看门狗
 * 功能：主循环约 4 s 不喂狗就复位，把“卡在某个忙等/死循环”变成可恢复的重启。
 * 取值：LSI 约 32 kHz，PR=4（/64）得 500 Hz，RLR=2000 -> 4 s。
 * 注意：4 s 大于内部 Flash 擦除时间（约 1~2 s），保存参数不会误复位；
 *       调试 halt 时通过 DBGMCU 冻结计数，不会被看门狗打断。
 *       只允许在主循环末尾喂狗：卡在主循环调用的任何函数里就不会喂。
 * ------------------------------------------------------------------------- */
#define IWDG_RELOAD_COUNT   2000U

static void iwdg_start(void)
{
    __HAL_DBGMCU_FREEZE_IWDG();
    IWDG->KR = 0x5555U;             /* 解锁 PR/RLR */
    IWDG->PR = 4U;                  /* LSI/64 -> 500 Hz */
    IWDG->RLR = IWDG_RELOAD_COUNT;  /* 2000 / 500 = 4 s */
    IWDG->KR = 0xCCCCU;             /* 启动；启动后不能关闭 */
}

static inline void iwdg_feed(void)
{
    IWDG->KR = 0xAAAAU;
}

/* ---------------------------------------------------------------------------
 * 状态灯
 * 功能：心跳灯每 500 ms 翻转；按键灯在每次按下时闪 1~4 下。
 * 取值：UP/DOWN/OK/BACK 分别对应 1/2/3/4 下，每下约 150 ms。
 * 注意：只在 PIT 中断里调用，不和主循环抢 GPIO。
 * ------------------------------------------------------------------------- */
static unsigned key_led_elapsed_ms;
static unsigned key_led_toggles;

static void status_led_key(uint8_t id)
{
    if (id >= KEY_NUM) {
        return;
    }
    key_led_elapsed_ms = 0U;
    key_led_toggles = 2U * ((unsigned)id + 1U) - 1U;   /* 1/3/5/7 次翻转 = 1~4 个脉冲 */
    NEUQ_GPIO_write(BOARD_KEY_LED_PIN, GPIO_High);
}

static void status_led_tick(void)
{
    static unsigned heartbeat_elapsed_ms;

    heartbeat_elapsed_ms += BOARD_PERIOD_MS;
    if (heartbeat_elapsed_ms >= 500U) {
        heartbeat_elapsed_ms -= 500U;
        NEUQ_GPIO_toggle(BOARD_HEARTBEAT_LED_PIN);
    }

    if (key_led_toggles) {
        key_led_elapsed_ms += BOARD_PERIOD_MS;
        if (key_led_elapsed_ms >= 150U) {
            key_led_elapsed_ms -= 150U;
            NEUQ_GPIO_toggle(BOARD_KEY_LED_PIN);
            --key_led_toggles;
        }
    }
}

/*
 * 函数名称：copy_tiles
 * 函数功能：把四个数独子块复制成 4×9 数组，返回已收到子块的位图。
 * 参数说明：tiles —— 目标数组，可为 NULL（只取位图）；
 *           now   —— 当前毫秒时间。
 * 返回参数：位图，bit0..bit3 分别对应左上/右上/左下/右下。
 * 备注信息：必须在 PIT 中或关中断后调用，保证四个块来自同一会话。
 */
static uint8_t copy_tiles(uint8_t tiles[4][9], uint32_t now)
{
    unsigned i;
    uint8_t mask = 0;

    for (i = 0; i < 4; ++i)
    {
        CarVisionTile tile;

        if (CarVision_GetTile(&vision, (uint8_t)i, now, &tile))
        {
            mask |= (uint8_t)(1U << i);
            if (tiles)
            {
                memcpy(tiles[i], tile.cells, 9);
            }
        }
    }

    return mask;
}

/*
 * 函数名称：finish_tick
 * 函数功能：完赛后等车停稳，再收齐最后一帧数独块，最多等 1 s。
 * 参数说明：now —— 当前毫秒时间。
 * 返回参数：无。
 * 备注信息：置 finish_ready 后由主循环求解并显示，求解期间 PIT 阻止新车。
 */
static void finish_tick(uint32_t now)
{
    uint8_t still;

    if (!finish_pending || finish_ready)
    {
        return;
    }

    still = (car.state == CAR_STOP && fabsf(car.speed[0]) < 2 && fabsf(car.speed[1]) < 2);
    if (!still)
    {
        finish_timing = 0;
        return;
    }

    if (!finish_timing)
    {
        finish_still_ms = now;
        finish_timing = 1;
    }
    if (now - finish_still_ms < 100U)
    {
        return;
    }

    finish_tile_mask = copy_tiles(NULL, now);
    if (finish_tile_mask != 15U && now - finish_started_ms < 1000U)
    {
        return;
    }
    finish_tile_mask = copy_tiles(finish_tiles, now);
    finish_ready = 1;
}

/*
 * 函数名称：poll_menu
 * 函数功能：主循环里按行刷新菜单，每 MENU_REFRESH_MS 画一行。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：整屏快照在临界区内一次性取好，避免同一屏出现新旧混合状态；
 *           数独求解也在本函数里做（阻塞几毫秒，此时车已停）。
 */
static void poll_menu(void)
{
    static MenuView view;
    static unsigned row = LCD_ROWS;
    static uint32_t last;
    static uint8_t shown_result;
    uint32_t now = HAL_GetTick(), mask;

    if (menu_refresh)
    {
        menu_refresh = 0;
        row = LCD_ROWS;
        last = now - MENU_REFRESH_MS;
    }

    if (finish_ready && finish_pending)
    {
        /* PIT 在 finish_pending 期间禁止新车，求解时数独块是稳定的。 */
        SudokuStatus solved = (finish_tile_mask == 15U) ?
            Sudoku_SolveDetailed((const uint8_t (*)[9])finish_tiles, result_board) : SUDOKU_MISSING;

        mask = __get_PRIMASK();
        __disable_irq();
        result_status = (uint8_t)solved;
        Menu_ShowResult(&menu);
        finish_ready = finish_pending = 0;
        __set_PRIMASK(mask);
        row = LCD_ROWS;
        last = now - MENU_REFRESH_MS;
    }

    if (row == LCD_ROWS)
    {
        unsigned i;
        const Car_PidInc_Struct *pid;

        if (now - last < MENU_REFRESH_MS)
        {
            return;
        }
        last = now;

        mask = __get_PRIMASK();
        __disable_irq();
        view.menu = menu;
        {
            CarParams current;
            CarParams_Capture(&car, &current);
            view.params_saved = params_saved;
            view.params_dirty = !params_saved || !CarParams_Equal(&current, &saved_params);
        }
        view.start = menu_start;
        view.launch_road_count = launch_road.count;
        view.race_rpm = Menu_GetRaceRpm(&menu);
        view.state = car.state;
        view.motion = car.motion;
        view.fault = car.fault;
        view.phase = (uint8_t)track.phase;
        view.degraded = (car.motion == CAR_MOTION_RACE) ? track.degraded : 0;
        view.lap = track.lap;
        view.distance = track.distance;
        view.yaw_deg = track.yaw * 57.29578f;
        for (i = 0; i < 2; ++i)
        {
            view.speed[i] = car.speed[i];
            view.reference[i] = car.reference[i];
            view.pwm[i] = car.output[i] / 100.0f;
        }
        pid = &car.pid_speed[menu.group == MENU_RIGHT_PID];
        view.gains[0] = pid->kp;
        view.gains[1] = pid->ki;
        view.gains[2] = pid->kd;
        view.fresh = !Uart_VisionFault() && CarVision_GetObservation(&vision, now, &view.obs);
        view.vision_ok = vision.accepted_frames;
        view.vision_bad = vision.bad_frames;
        view.vision_rejected = vision.rejected_frames;
        view.vision_enabled = (uint8_t)BOARD_VISION_ENABLE;
        view.session_valid = vision.session_valid;
        view.camera_status = CarVision_GetCameraStatus(&vision, now);
        /* 香橙派链路只看是否收到合法帧，与相机是否出图独立。 */
        view.pi_connected = (uint8_t)(!Uart_VisionFault() &&
            CarVision_LinkAlive(&vision, now));
        view.bt_connected = (uint8_t)(bt_last_ms != 0U &&
            (uint32_t)(now - bt_last_ms) <= BT_STATUS_TIMEOUT_MS);
        view.tile_mask = (result_status == SUDOKU_COLLECTING || result_status == SUDOKU_WAITING) ?
            copy_tiles(NULL, now) : finish_tile_mask;
        shown_result = result_status;
        __set_PRIMASK(mask);

        view.imu_present = Gyro_Present();
        view.samples = (uint8_t)Gyro_Samples();
        view.gyro_ok = Gyro_Fresh();
        view.bias_dps = Gyro_BiasRad() * 57.29578f;
        view.gyro_dps = Gyro_RateRad() * 57.29578f;
        row = 0;
    }

    Menu_DrawRow(&view, row++, result_board, shown_result);
}

/*
 * 函数名称：track_tick
 * 函数功能：每拍处理按键/发车，并在自动模式下推进巡线状态机写目标转速。
 * 参数说明：l/r     —— 左右轮本拍计数增量；
 *           dt_us   —— 本拍实际周期，微秒。
 * 返回参数：无。
 * 备注信息：PE10 的停车优先级最高：先处理急停，再处理菜单事件；
 *           发车成功后才启动 Track_Step，避免“未发车就按视觉跑”。
 */
static void track_tick(int32_t l, int32_t r, uint32_t dt_us)
{
    static unsigned scan_ms, stop_release_ms;
    Key_Event_Struct key;
    CarVisionObservation o;
    uint32_t now = HAL_GetTick();
    uint8_t fresh = CarVision_GetObservation(&vision, now, &o);
    uint8_t still = (fabsf(car.speed[0]) < 2 && fabsf(car.speed[1]) < 2);
    uint8_t up, launch = 0;

    chassis_still = still && car.state == CAR_STOP;
    up = (NEUQ_GPIO_read(KEY_UP_PIN) == GPIO_Low) ? 1U : 0U;
    status_led_tick();                              /* 心跳/按键灯，每 5 ms 推进一步 */

    scan_ms += BOARD_PERIOD_MS;
    if (scan_ms >= KEY_SCAN_PERIOD_MS)
    {
        Key_Scan();
        scan_ms = 0;
    }

    /* 停车优先于排队的 OK，也优先于本拍收到的蓝牙 ARM；整段按住都算停车。 */
    if (up && (car.state != CAR_STOP || stop_release_ms))
    {
        Car_LocalStop(&car);
        Menu_Disarm(&menu);
        stop_release_ms = KEY_DEBOUNCE_COUNT * KEY_SCAN_PERIOD_MS;
    }
    else if (stop_release_ms)
    {
        Car_LocalStop(&car);
        stop_release_ms -= BOARD_PERIOD_MS;
    }

    if (finish_pending)
    {
        Car_LocalStop(&car);
        Menu_Disarm(&menu);
    }

    /* 停车且收尾完成后结束本轮视觉会话：发送端不必重启，下一次 HELLO
     * 即使会话号相同也能重新绑定，但必须等新的实测道路帧才能发车。 */
    if (car.state == CAR_STOP && !finish_pending && race_session != 0U)
    {
        if (vision.session_valid && vision.session == race_session)
        {
            CarVision_CloseRaceSession(&vision);
            fresh = 0;
        }
        race_session = 0U;
        Menu_Disarm(&menu);
        memset(&launch_road, 0, sizeof launch_road);
    }

    menu_start = Menu_StartCondition(&launch_road, &car, still, finish_pending,
                                     (fresh && !Uart_VisionFault()) ? &o : NULL,
                                     race_session);

    while (Key_GetEvent(&key))
    {
        MenuAction action;

        if (key.type == KEY_EVENT_PRESS)
        {
            status_led_key(key.id);                 /* 每个键按下闪 1~4 下 */
        }
        if (stop_release_ms || (up && key.id != KEY_UP))
        {
            continue;
        }

        action = Menu_Key(&menu, key, &car, menu_start);
        if (action == MENU_SAVE_PARAMS)
        {
            params_pending = 1;
            Car_LocalStop(&car);
            break;
        }
        if (action == MENU_LAUNCH)
        {
            launch = 1;
        }
    }

    /* 预备发车：视觉就绪后自动触发，和按 OK 走同一条发车通路。 */
    if (Menu_PollArmed(&menu, menu_start) == MENU_LAUNCH)
    {
        launch = 1;
    }
    if (launch && Car_StartRace(&car))
    {
        Track_StartWithRpm(&track, (float)Menu_GetRaceRpm(&menu));
        display_session = o.session;
        race_session = o.session;
        result_status = SUDOKU_COLLECTING;
        menu_start = START_STOP_FIRST;
    }

    if (car.motion == CAR_MOTION_RACE)
    {
        float metres = 6.2831853f * TRACK_RADIUS_M / ENCODER_CPR;
        TrackDrive cmd = Track_Step(&track,
                                    (fresh && !Uart_VisionFault()) ? &o : NULL,
                                    l * metres, r * metres, dt_us * 0.000001f,
                                    Gyro_RateRad(), Gyro_Fresh(), still, now);

        if (track.phase == TRACK_SEGMENT_FAULT)
        {
            Car_Fault(&car, CAR_RACE_FAULT);
        }
        else if (track.phase == TRACK_SEGMENT_FINISHED)
        {
            finish_pending = 1;
            finish_ready = finish_timing = 0;
            finish_started_ms = now;
            result_status = SUDOKU_WAITING;
            Menu_ShowResult(&menu);
            Car_LocalStop(&car);
        }
        else
        {
            Car_RaceWheels(&car, cmd.rpm);
        }
    }
}

/*
 * 函数名称：poll_params
 * 函数功能：把当前内环增益写入 Flash 掉电保存。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：只在停车且双轮静止时执行；写 Flash 会阻塞取指，写入前先关中断、
 *           撤输出，写完后重建 FIFO、清串口溢出并丢弃残留字节。
 */
static void poll_params(void)
{
    CarParams current;
    CarParamsStatus status;
    uint32_t mask;

    if (!params_pending)
    {
        return;
    }

    mask = __get_PRIMASK();
    __disable_irq();
    if (car.state != CAR_STOP || finish_pending ||
        !(fabsf(car.speed[0]) < 2 && fabsf(car.speed[1]) < 2))
    {
        menu.params_status = PARAMS_STOP_FIRST;
        params_pending = 0;
        menu_refresh = 1;
        __set_PRIMASK(mask);
        return;
    }
    CarParams_Capture(&car, &current);
    Motor_Apply(0, 0);
    Motor_Apply(1, 0);
    __set_PRIMASK(mask);

    /* 前台执行：保留中断以便 HAL 超时；PIT 已被上方条件挡住；
     * 串口入队暂停，避免把 Flash 阻塞期间堆积的字节当成命令。 */
    Uart_Gate(1);
    Lcd_Print(3, UI_STYLE_WARN, "保存中...请勿断电");
    status = CarParams_FlashSave(&current);
    Uart_Gate(0);

    mask = __get_PRIMASK();
    __disable_irq();
    if (status == PARAMS_SAVED || status == PARAMS_UNCHANGED)
    {
        saved_params = current;
        params_saved = 1;
    }
    menu.params_status = status;
    Car_LocalStop(&car);
    Uart_Reset();
    Uart_Gate(0);
    CarVision_Invalidate(&vision);
    Key_ClearEvents();
    params_rebase = 1;
    params_pending = 0;
    menu_refresh = 1;
    __set_PRIMASK(mask);
}

/*
 * 函数名称：telemetry_i16
 * 函数功能：把浮点量按比例缩放、四舍五入并饱和到 int16 范围。
 * 参数说明：value —— 待打包的浮点值；
 *           scale —— 缩放系数（rpm 用 1，位置 0.01 rev 用 100）。
 * 返回参数：int16 范围内的整数；NaN/过小返回 -32768，过大返回 32767。
 * 备注信息：不依赖 lroundf，避免为了一个遥测帧引入额外的库函数。
 */
static int16_t telemetry_i16(float value, float scale)
{
    float scaled = value * scale;

    if (!(scaled > -32768.0f))          /* NaN 或小于下限 */
    {
        return -32768;
    }
    if (scaled > 32767.0f)
    {
        return 32767;
    }

    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

/*
 * 函数名称：telemetry_put_i16
 * 函数功能：把有符号 16 位整数按大端顺序写入字节缓冲。
 * 参数说明：p     —— 目标字节位置；
 *           value —— 待写入的整数。
 * 返回参数：无。
 */
static void telemetry_put_i16(uint8_t *p, int16_t value)
{
    p[0] = (uint8_t)((uint16_t)value >> 8);
    p[1] = (uint8_t)value;
}

/*
 * 函数名称：telemetry_fast_frame
 * 函数功能：组装并发送快帧（左右实测 rpm + 状态），100 Hz。
 * 参数说明：c —— 整车控制对象。
 * 返回参数：无。
 * 备注信息：帧格式 A5 | 左 rpm int16 | 右 rpm int16 | 状态 u8 | XOR。
 */
static void telemetry_fast_frame(const Car_Core *c)
{
    uint8_t frame[7];                   /* 7 字节整帧 */

    frame[0] = 0xA5U;
    telemetry_put_i16(&frame[1], telemetry_i16(c->speed[0], 1.0f));
    telemetry_put_i16(&frame[3], telemetry_i16(c->speed[1], 1.0f));
    frame[5] = c->state;
    frame[6] = (uint8_t)(frame[0] ^ frame[1] ^ frame[2] ^
                         frame[3] ^ frame[4] ^ frame[5]);
    (void)Vofa_PushFrame(frame, sizeof(frame));
}

/*
 * 函数名称：telemetry_slow_frame
 * 函数功能：组装并发送慢帧（左右位置 + 状态），20 Hz。
 * 参数说明：c —— 整车控制对象。
 * 返回参数：无。
 * 备注信息：帧格式 A6 | 左位置 int16(0.01 rev) | 右位置 int16 | 状态 u8 | XOR。
 */
static void telemetry_slow_frame(const Car_Core *c)
{
    uint8_t frame[7];                   /* 7 字节整帧 */

    frame[0] = 0xA6U;
    telemetry_put_i16(&frame[1], telemetry_i16(c->position[0], 100.0f));
    telemetry_put_i16(&frame[3], telemetry_i16(c->position[1], 100.0f));
    frame[5] = c->state;
    frame[6] = (uint8_t)(frame[0] ^ frame[1] ^ frame[2] ^
                         frame[3] ^ frame[4] ^ frame[5]);
    (void)Vofa_PushFrame(frame, sizeof(frame));
}

/*
 * 函数名称：Car_PIT_Handler
 * 函数功能：5 ms 控制中断：采样、解析、控制、输出、视觉、遥测。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：执行顺序固定；电机输出在视觉解析之前完成，避免解析耗时
 *           影响 PWM 相位。遥测为打包二进制帧：A5 快帧 100 Hz、
 *           A6 慢帧 20 Hz，合计约 840 B/s。
 */
void Car_PIT_Handler(void)
{
    static unsigned telemetry_fast; /* 快帧分频计数 */
    static unsigned telemetry_slow; /* 慢帧分频计数 */
    unsigned n;                     /* 本拍已处理的字节数 */
    uint8_t byte;                   /* 从 FIFO 读出的临时字节 */
    uint32_t dt_us;                 /* 实际控制周期，微秒 */
    int32_t l;                      /* 左轮本拍计数增量 */
    int32_t r;                      /* 右轮本拍计数增量 */

    dt_us = Clock_PeriodUs();

    l = Encoder_Delta(0);
    r = Encoder_Delta(1);

    /* Flash 阻塞期间不做控制：保持停车、双轮输出 0，并丢弃本拍采样。 */
    if (params_pending || params_rebase)
    {
        params_rebase = 0;
        Car_LocalStop(&car);
        Motor_Apply(0, 0);
        Motor_Apply(1, 0);
        return;
    }

    /* 蓝牙字节可能丢失，不再信任残留队列：先停车，再清空。 */
    if (Uart_BtFault())
    {
        if (car.motion != CAR_MOTION_RACE)
        {
            Car_Fault(&car, CAR_RX_FAULT);
        }
        Uart_DrainBt();
        Uart_ClearBtFault();
    }
    else
    {
        uint32_t now = HAL_GetTick();
        uint32_t frames = car.frames;

        /* 每拍最多处理 32 字节，限制串口解析占用的中断时间。 */
        for (n = 0; n < 32U && Uart_TakeBt(&byte); n++)
        {
            Car_FeedByte(&car, byte);
        }
        /* 解析出完整命令帧才认为上位机在线；纯串口噪声不刷新。 */
        if (car.frames != frames) bt_last_ms = now;
    }

    /* 用实测周期更新控制器，并把双轮输出应用到电机。 */
    track_tick(l, r, dt_us);
    Car_TickDt(&car, l, r, dt_us);
    Motor_Apply(0, car.output[0]);
    Motor_Apply(1, car.output[1]);

    /*
     * 输出之后再解析视觉，每拍最多 64 字节；视觉只更新缓存，不直接控制。
     * 蓝牙故障停车，视觉故障只让缓存失效。
     */
    if (Uart_VisionFault())
    {
        CarVision_Invalidate(&vision);
        Uart_DrainVision();
        Uart_ClearVisionFault();
    }
    else
    {
        uint32_t now = HAL_GetTick();

        for (n = 0; n < 64U && Uart_TakeVision(&byte); n++)
        {
            uint32_t accepted = vision.accepted_frames;
            CarVisionObservation road;
            uint8_t road_fresh;

            /* 只有停车状态才允许接受新的 HELLO 会话。 */
            CarVision_Feed(&vision, byte, now,
                           (car.state == CAR_STOP && !finish_pending) ? 1U : 0U);
            if (vision.accepted_frames == accepted) continue;

            /* 每接受一帧就推进一次发车确认，避免一拍拍内多帧被漏计。 */
            road_fresh = CarVision_GetObservation(&vision, now, &road);
            menu_start = Menu_StartCondition(&launch_road, &car, chassis_still,
                                             finish_pending,
                                             (road_fresh && !Uart_VisionFault()) ? &road : NULL,
                                             race_session);
        }
    }

    /* 视觉会话切换：清空数独结果，避免把上一场的数据当本场。 */
    if (vision.session_valid && vision.session != display_session)
    {
        display_session = vision.session;
        result_status = SUDOKU_COLLECTING;
    }

    finish_tick(HAL_GetTick());

    /* 打包二进制遥测：快帧 100 Hz、慢帧 20 Hz。 */
    if (++telemetry_fast >= BOARD_TELEMETRY_FAST_TICKS)
    {
        telemetry_fast = 0;
        telemetry_fast_frame(&car);
    }

    if (++telemetry_slow >= BOARD_TELEMETRY_SLOW_TICKS)
    {
        telemetry_slow = 0;
        telemetry_slow_frame(&car);
    }
}

/*
 * 函数名称：Car_HoldMotorsOff
 * 函数功能：上电最早期把两路 EN 拉低。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：保留该名字供 HAL main() 调用，实际动作在驱动层。
 */
void Car_HoldMotorsOff(void)
{
    Motor_HoldOff();
}

/*
 * 函数名称：Car_GetVisionObservation
 * 函数功能：临界区内复制道路观测快照。
 * 参数说明：out —— 接收副本的对象。
 * 返回参数：1 成功；0 无数据/过期/参数无效。
 */
uint8_t Car_GetVisionObservation(CarVisionObservation *out)
{
    uint32_t saved = __get_PRIMASK();
    uint8_t result;

    __disable_irq();
    result = CarVision_GetObservation(&vision, HAL_GetTick(), out);
    __set_PRIMASK(saved);
    return result;
}

/*
 * 函数名称：Car_GetVisionTile
 * 函数功能：临界区内复制指定索引的数独子块。
 * 参数说明：index —— 0..3；out —— 接收副本的对象。
 * 返回参数：1 成功；0 参数无效或数据不可用。
 */
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out)
{
    uint32_t saved = __get_PRIMASK();
    uint8_t result;

    __disable_irq();
    result = CarVision_GetTile(&vision, index, HAL_GetTick(), out);
    __set_PRIMASK(saved);
    return result;
}

/*
 * 函数名称：Car_Main
 * 函数功能：初始化全部外设和共享对象后进入主循环。
 * 参数说明：无。
 * 返回参数：正常不会返回。
 * 备注信息：初始化顺序有依赖：
 *   1. 先建控制对象与参数；
 *   2. 先拉高 Flash 片选，再复用 PA5/PA6/PA7；
 *   3. 方向、PWM、编码器、串口按依赖顺序初始化；
 *   4. 建立编码器基准与控制时基；
 *   5. 最后启动 PIT，避免中断访问未初始化对象。
 */
void Car_Main(void)
{
    int loaded;

#if BOARD_LED_TEST
    /*
     * 最小自检：不依赖 SysTick/HAL/中断，只用忙等翻转 PA8。
     * 独立上电后 LED2 闪烁 = MCU 在执行本固件；不闪 = 启动/烧录问题。
     */
    {
        volatile uint32_t d;

        NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);
        for (;;)
        {
            NEUQ_GPIO_write(PA8, GPIO_Low);
            for (d = 0U; d < 2000000U; d++) { }
            NEUQ_GPIO_write(PA8, GPIO_High);
            for (d = 0U; d < 2000000U; d++) { }
        }
    }
#endif

    /* 独立上电也要在首个外设延时之前使能 DWT。 */
    NEUQ_Delay_init();
    /* 蓝牙独占 USART1，在初始化 LCD/I²C 之前关闭 printf 输出。 */
    NEUQ_Debug_set_output_enabled(0);

    Car_Init(&car);                                  /* 控制对象复位到 STOP */
    Menu_Init(&menu);                                /* 链表式菜单复位到组页 */
    loaded = CarParams_Load(&CarParams_FlashIO, &saved_params);
    params_saved = (uint8_t)(loaded == 1 && CarParams_Apply(&car, &saved_params));
    menu.params_status = params_saved ? PARAMS_LOADED :
                         (loaded == 0 ? PARAMS_DEFAULTS : PARAMS_ERROR);
    CarVision_Init(&vision);
    Key_Init();
    Uart_Init();
    Uart_Gate(0);

    /* 先取消板载 Flash 片选，再把共用的 PA5/PA6/PA7 用于方向和编码器。 */
    NEUQ_GPIO_init(BOARD_FLASH_CS, GPIO_Mode_Output_PushPull, GPIO_High);

    Motor_Init();                                    /* 方向脚 + PWM 通道 */
    Encoder_Init();                                  /* 编码器 + 输入滤波 + 基准 */

    /* 再次确认 W25Q 片选为高：只要 CS 为高，Flash 不会驱动 MISO。 */
    NEUQ_GPIO_write(BOARD_FLASH_CS, GPIO_High);

#if BOARD_VISION_ENABLE
    NEUQ_Uart_init((UART_DEV_ENUM)BOARD_UART_VIS_INDEX, BOARD_UART_VIS_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)BOARD_UART_VIS_INDEX, 4);
#endif

    NEUQ_GPIO_init(BOARD_HEARTBEAT_LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(BOARD_KEY_LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);

    Lcd_Init();
    Gyro_Init();

    /* 慢速初始化做完，才开始接收蓝牙命令。 */
    NEUQ_Uart_init((UART_DEV_ENUM)BOARD_UART_BT_INDEX, BOARD_UART_BT_BAUD);
    Vofa_Init((uint8_t)BOARD_UART_BT_INDEX);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)BOARD_UART_BT_INDEX, 3);

    /* 时基初始化失败时 dt 保持 0，Car_TickDt 会判时序故障。 */
    Clock_Init();

    NEUQ_PIT_init_ms(PIT_CH1, BOARD_PERIOD_MS);
    iwdg_start();                                    /* 初始化完成后再启动 4 s 看门狗 */

    /* 控制和按键由 PIT 驱动，慢速外设分次在主循环推进。 */
    for (;;)
    {
        poll_params();
        Vofa_Poll();
        Gyro_Poll(chassis_still);
        Lcd_Poll(HAL_GetTick());                     /* 周期整屏重画 / SPI 超时后重初始化 */
        poll_menu();
        iwdg_feed();                                 /* 只有主循环完整走完一轮才喂狗 */
    }
}
