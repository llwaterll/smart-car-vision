/*
 * ============================================================================
 * 文件名称：Drv_Key.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 四键扫描 + 事件队列：
 *   1. Key_Init()      ：按键引脚与消抖状态初始化；
 *   2. Key_Scan()      ：消抖、判按下、判长按连发，只往队列记事件；
 *   3. Key_GetEvent()  ：主循环取事件；
 *   4. Key_ClearEvents()：丢弃残留事件。
 *
 * 为什么扫描放中断、处理放主循环：
 *   10 ms 固定采样消抖时间才准确；中断里只入队，不会因为刷屏/写 Flash
 *   拖慢采样。队列是单写单读，head 只有中断写、tail 只有主循环写，
 *   两边各改各的，不需要关中断。
 *
 * 防卡死：
 *   - 队列满时丢“新事件”，不覆盖旧事件，也不等待；
 *   - 入口检查 id/type/下标，坏值直接丢弃；
 *   - 长按计时做饱和处理，不会回绕成“永远不到长按”。
 *
 * 函数列表
 *   static void key_push(uint8_t id, uint8_t type)
 *   void        Key_Init(void)
 *   void        Key_Scan(void)
 *   uint8_t     Key_GetEvent(Key_Event_Struct *event)
 *   void        Key_ClearEvents(void)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Drv_Key.h"

/* 一个键的静态配置 + 扫描状态，放在同一张表里，扫描循环只按下标取一次。 */
typedef struct {
    GPIO_Ports_ENUM pin;        /* 引脚 */
    GPIO_Level      active;     /* 按下时读到的电平 */
    uint8_t         stable;     /* 消抖后认定的状态：1 按下，0 松开 */
    uint8_t         count;      /* 连续读到“与认定不同”的次数 */
    uint16_t        hold_ms;    /* 已按住多久，用于长按连发 */
} KeyState;

static KeyState s_keys[KEY_NUM] = {
    {KEY_UP_PIN,   KEY_UP_ACTIVE_LEVEL,   0, 0, 0},
    {KEY_DOWN_PIN, KEY_DOWN_ACTIVE_LEVEL, 0, 0, 0},
    {KEY_OK_PIN,   KEY_OK_ACTIVE_LEVEL,   0, 0, 0},
    {KEY_BACK_PIN, KEY_BACK_ACTIVE_LEVEL, 0, 0, 0},
};

// 事件队列（环形缓冲）：只有中断写 s_head，只有主循环写 s_tail，两边各改各的，不需要关中断
static volatile Key_Event_Struct s_queue[KEY_QUEUE_SIZE];
static volatile uint8_t          s_head = 0;    // 下一个事件写到哪里
static volatile uint8_t          s_tail = 0;    // 下一个事件从哪里读

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     往队列里放一个事件；队列满了就丢掉这个新事件（主循环卡住时不会覆盖旧事件）
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：key_push
 * 函数功能：把一个按键事件放进环形队列。
 * 参数说明：id   —— 键号，取值见 Key_Id_Enum；
 *           type —— 事件类型，KEY_EVENT_PRESS 或 KEY_EVENT_REPEAT。
 * 返回参数：无。
 * 备注信息：队列满时丢弃“新事件”而不是覆盖旧事件：消费端按顺序处理，
 *           旧事件更接近用户当时的意图。先写数据再移动 head，
 *           主循环看到 head 变化时数据一定已写好；head 只在中断里写，
 *           tail 只在 Key_GetEvent 里写，单写单读，不需要关中断。
 *           异常的 id/type 或越界下标直接丢弃，不修改队列。
 */
static void key_push(uint8_t id, uint8_t type)
{
    uint8_t next;

    if (id >= KEY_NUM || (type != KEY_EVENT_PRESS && type != KEY_EVENT_REPEAT)) {
        return;
    }
    if (s_head >= KEY_QUEUE_SIZE) {
        s_head = 0;
    }
    if (s_tail >= KEY_QUEUE_SIZE) {
        s_tail = 0;
    }
    next = (uint8_t)((s_head + 1U) % KEY_QUEUE_SIZE);
    if (next == s_tail) {
        return;
    }
    s_queue[s_head].id   = id;
    s_queue[s_head].type = type;
    s_head = next;                      // 先写数据再移动 head，主循环看到 head 变化时数据一定已经写好
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化按键引脚
// 备注信息     按下是低电平的键配置成内部上拉（松开时被拉高），按下是高电平的键配置成内部下拉，
//              这样按键悬空（没接）时读到的都是"松开"
//-------------------------------------------------------------------------------------------------------------------
void Key_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++) {
        KeyState *k = &s_keys[i];
        GPIO_Mode_ENUM mode = (GPIO_Mode_ENUM)((k->active == GPIO_Low) ? GPIO_Mode_Input_PullUp : GPIO_Mode_Input_PullDown);
        NEUQ_GPIO_init(k->pin, mode, GPIO_High);
        k->stable  = 0U;
        k->count   = 0U;
        k->hold_ms = 0U;
    }
    s_head = 0U;
    s_tail = 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     扫描一次所有按键（在定时中断里每 KEY_SCAN_PERIOD_MS 调用一次）
// 备注信息     消抖原理：机械按键按下/松开的瞬间触点会抖动几毫秒，读到的电平忽高忽低。
//              只有"读到的状态"连续 KEY_DEBOUNCE_COUNT 次（默认 3 次 = 30 ms）都与"认定的状态"不同，
//              才承认按键真的变了；中间只要有一次读回原状态，计数就清零重来，抖动因此被滤掉。
//              连发原理：上/下键认定按下后开始计时，按住满 KEY_REPEAT_DELAY_MS 产生第一次连发，
//              之后每隔 KEY_REPEAT_PERIOD_MS 再产生一次。
//-------------------------------------------------------------------------------------------------------------------
void Key_Scan(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++) {
        KeyState *k = &s_keys[i];
        uint8_t pressed = (NEUQ_GPIO_read(k->pin) == k->active) ? 1U : 0U;   // 这一次读到的状态

        if (pressed != k->stable) {
            // 读到的和认定的不一样：可能真的按下/松开了，也可能只是抖动，先计数
            if (k->count < 0xFFU) {
                k->count++;
            }
            if (k->count >= KEY_DEBOUNCE_COUNT) {
                k->stable  = pressed;           // 连续多次都这样，认定状态真的变了
                k->count   = 0U;
                k->hold_ms = 0U;
                if (pressed) {
                    key_push(i, KEY_EVENT_PRESS);
                }
            }
        }
        else {
            // 读到的和认定的一样：之前的变化只是抖动，计数清零
            k->count = 0U;

            // 上/下键一直按着：计时，到时间就产生连发事件
            if (k->stable && (i == KEY_UP || i == KEY_DOWN)) {
                if (k->hold_ms > (uint16_t)(0xFFFFU - KEY_SCAN_PERIOD_MS)) {
                    k->hold_ms = 0xFFFFU;       // 饱和，避免回绕
                } else {
                    k->hold_ms = (uint16_t)(k->hold_ms + KEY_SCAN_PERIOD_MS);
                }
                if (k->hold_ms >= KEY_REPEAT_DELAY_MS) {
                    key_push(i, KEY_EVENT_REPEAT);
                    k->hold_ms = (uint16_t)(KEY_REPEAT_DELAY_MS - KEY_REPEAT_PERIOD_MS);  // 再过一个连发间隔触发下一次
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环取出一个按键事件
// 返回参数     1 取到事件  0 队列为空
//-------------------------------------------------------------------------------------------------------------------
uint8_t Key_GetEvent(Key_Event_Struct *event)
{
    if (event == NULL) {
        return 0U;
    }
    if (s_tail >= KEY_QUEUE_SIZE) {
        s_tail = 0U;
    }
    if (s_head >= KEY_QUEUE_SIZE) {
        s_head = 0U;                        // 只在下标被异常写坏时自愈
    }
    if (s_tail == s_head) {
        return 0U;
    }
    event->id   = s_queue[s_tail].id;
    event->type = s_queue[s_tail].type;
    s_tail = (uint8_t)((s_tail + 1U) % KEY_QUEUE_SIZE);
    return 1U;
}

void Key_ClearEvents(void)
{
    s_tail = s_head;
}
