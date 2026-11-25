/*
 * ============================================================================
 * 文件名称：Drv_Lcd.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 文本行缓存 + 增量刷新：
 *   1. Lcd_Init()       ：初始化屏幕并清空行缓存；
 *   2. Lcd_Print()      ：格式化一行 UTF-8 文字，逐列与缓存比较，只重画变化的字符；
 *   3. Lcd_Invalidate() ：丢掉缓存，下次整行重画（切页/复位后用）。
 *
 * 一行怎么变成“列”：
 *   ASCII 占 1 个逻辑列，汉字占 2 个逻辑列（首列存 Unicode 码点，次列存占位符）。
 *   补空格、截断和差分比较都按半角列数计算，与纯 ASCII 版本的排版完全一致。
 *
 * 行缓存的意义：
 *   屏幕刷新是整机最慢的操作（毫秒级），菜单每 120 ms 刷 10 行。
 *   缓存把“每行要显示什么”留在 RAM 里，没变的字符一个像素都不发。
 *
 * 函数列表
 *   void Lcd_Init(void)
 *   void Lcd_Invalidate(void)
 *   void Lcd_Poll(uint32_t now)
 *   void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
 *   static const uint8_t *lcd_find_zh_glyph(uint16_t unicode)
 *   static void lcd_draw_zh20(uint16_t x, uint16_t y, const uint8_t *bitmap)
 *   static void lcd_reinit(uint32_t now)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include <stdarg.h>
#include <stdio.h>
#include "Drv_Lcd.h"
#include "Drv_Lcd_FontZh.h"

// NEUQ_LCD200 把当前字色/底色和显存缓冲留在 .c 里，没有通过头文件导出；
// 汉字整块写入需要直接填 LCD200_Buff，所以按原定义在这里声明。
extern LCD200_Config_Struct LCD200_Config;
extern uint16_t LCD200_Buff[4096];

// 汉字占两个逻辑列：首列存 Unicode 码点，次列存这个占位符，表示“右半边属于前一个字”
#define LCD_CELL_CONT (0xFFFFU)

// SPI 故障后重初始化屏幕的最小间隔：SPI 仍坏时不要频繁清屏拖慢主循环
#define LCD_RECOVER_COOLDOWN_MS (2000U)

// 每种样式的字色和底色（RGB888，库会自动转成屏幕用的 RGB565），顺序与 UI_Style_Enum 一致
static const uint32_t s_pen_color[UI_STYLE_NUM]  = {LCD_BLACK, LCD_WHITE, LCD_WHITE, LCD_BLACK, DARK_GREEN, LCD_RED, DARK_GREY};
static const uint32_t s_back_color[UI_STYLE_NUM] = {LCD_WHITE, DARK_BLUE, LCD_BLUE,  LCD_YELLOW, LCD_WHITE, LCD_WHITE, LCD_WHITE};

// 屏幕内容缓存：每行 32 个逻辑半角列，汉字首列存码点、次列存 LCD_CELL_CONT
static uint16_t s_cells[LCD_ROWS][LCD_COLS];
static uint8_t  s_style[LCD_ROWS];
static uint8_t  s_valid[LCD_ROWS];       // 0 表示这一行的缓存不可信（刚上电 / 被清过），下次必须整行重画

// 自愈状态：上次整屏重画/重初始化的时刻（毫秒）
static uint32_t s_last_heal_ms;
static uint32_t s_last_recover_ms;
static uint8_t  s_recovered;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在汉字字库里查找一个字模
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：lcd_find_zh_glyph
 * 函数功能：按 Unicode 码点二分查找 20x20 汉字点阵。
 * 参数说明：unicode —— 汉字的 Unicode 码点。
 * 返回参数：指向 60 字节点阵的指针；字库里没有这个字时返回 NULL。
 * 备注信息：字模表按码点升序排列，所以可以二分。找不到时不在这里报错，
 *           交给 lcd_draw_zh20 画一个方框，方便一眼看出菜单里有未收录的字。
 */
static const uint8_t *lcd_find_zh_glyph(uint16_t unicode)
{
    unsigned lo = 0;
    unsigned hi = LCD_ZH_COUNT;

    while (lo < hi) {
        unsigned mid = lo + ((hi - lo) >> 1);
        uint16_t key = s_lcd_zh_font[mid].unicode;

        if (key == unicode) {
            return s_lcd_zh_font[mid].bitmap;
        }
        if (key < unicode) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return NULL;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画一个 20x20 汉字（字库缺字时画方框）
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：lcd_draw_zh20
 * 函数功能：在屏幕指定位置画一个 20x20 汉字或占位方框。
 * 参数说明：x, y   —— 左上角像素坐标；
 *           bitmap —— 60 字节点阵，NULL 表示字库缺字。
 * 返回参数：无。
 * 备注信息：点阵按行存储，每行 LCD_ZH_STRIDE 字节，最高位对应最左像素。
 *           先用当前样式的字色/底色填满 400 个像素，再一次 NEUQ_LCD200_WriteBuff
 *           整块发送；不要逐点调用 DrawPoint，否则一个汉字会拆成 400 次 SPI 事务。
 *           字色/底色由调用方 Lcd_Print 通过 SetPenColor/SetBackColor 提前设好。
 */
static void lcd_draw_zh20(uint16_t x, uint16_t y, const uint8_t *bitmap)
{
    uint16_t pen  = LCD200_Config.Pen_Color;
    uint16_t back = LCD200_Config.Background_Color;
    uint16_t idx  = 0;
    uint8_t row, col;

    if (bitmap != NULL) {
        for (row = 0; row < LCD_ZH_HEIGHT; row++) {
            const uint8_t *bits = bitmap + (unsigned)row * LCD_ZH_STRIDE;
            for (col = 0; col < LCD_ZH_WIDTH; col++) {
                uint8_t bit = (uint8_t)(bits[col >> 3] & (0x80U >> (col & 7U)));
                LCD200_Buff[idx++] = bit ? pen : back;
            }
        }
    } else {
        // 缺字占位：外面留 2 像素边距，画一个 16x16 的空心方框
        for (row = 0; row < LCD_ZH_HEIGHT; row++) {
            for (col = 0; col < LCD_ZH_WIDTH; col++) {
                uint8_t border = (uint8_t)((row >= 2U && row <= 17U && col >= 2U && col <= 17U) &&
                                           (row == 2U || row == 17U || col == 2U || col == 17U));
                LCD200_Buff[idx++] = border ? pen : back;
            }
        }
    }

    NEUQ_LCD200_SetAddress(x, y, (uint16_t)(x + LCD_ZH_WIDTH - 1U), (uint16_t)(y + LCD_ZH_HEIGHT - 1U));
    NEUQ_LCD200_WriteBuff(LCD200_Buff, (uint16_t)(LCD_ZH_WIDTH * LCD_ZH_HEIGHT));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化屏幕
// 备注信息     NEUQ_LCD200_init 内部会调用 printf 打印调试信息，所以之前必须先 NEUQ_Debug_init 初始化串口，
//              否则 printf 会一直等待一个没打开的串口
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：Lcd_Init
 * 函数功能：初始化屏幕并把行缓存清空。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：初始化由 NEUQ_LCD200 完成；行缓存清空后第一次 Lcd_Print 会整行重画，
 *           避免把上电前屏幕上的随机内容当成“已经显示过”。
 */
void Lcd_Init(void)
{
    NEUQ_LCD200_init();                         // 横屏、白底清屏、打开背光
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);    // 10x20 字模，一行 32 个半角列 / 16 个汉字
    Lcd_Invalidate();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     忘掉行缓存，下次显示时整行重画
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：Lcd_Invalidate
 * 函数功能：把每一行的缓存标记为不可信。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：切页、清屏或屏幕重新初始化后调用。只清“可信标志”不清内容，
 *           下一次 Lcd_Print 会把整行每个单元重新发一遍，保证屏幕与缓存重新一致。
 */
void Lcd_Invalidate(void)
{
    uint8_t row;
    for (row = 0; row < LCD_ROWS; row++) {
        s_valid[row] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重新初始化屏幕（SPI 超时后的恢复路径）
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：lcd_reinit
 * 函数功能：重新配置 ST7789，并让所有行缓存失效。
 * 参数说明：now —— 当前毫秒时间，用来重置整屏重画计时。
 * 返回参数：无。
 * 备注信息：Lcd_Init 内部会重新初始化 SPI/ST7789、清掉该设备的超时
 *           故障锁存、重新设置字体并让缓存失效；init 里的清屏循环在
 *           故障锁存期间是“立即返回”，不会卡住。
 */
static void lcd_reinit(uint32_t now)
{
    Lcd_Init();
    s_last_heal_ms = now;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕自愈轮询（主循环周期调用）
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：Lcd_Poll
 * 函数功能：周期整屏重画；SPI 等待超时后重新初始化屏幕。
 * 参数说明：now —— 当前毫秒时间（HAL_GetTick）。
 * 返回参数：无。
 * 备注信息：电池供电时屏幕可能失步，但 MCU 仍在正常跑。每 LCD_HEAL_PERIOD_MS
 *           让缓存失效一次，10 行会依次整行重画，把屏幕内容拉回正确状态；
 *           若底层 SPI 已经超时锁存，则直接重初始化屏幕。重初始化有冷却时间，
 *           避免 SPI 一直坏时反复清屏拖慢主循环。
 */
void Lcd_Poll(uint32_t now)
{
    if (NEUQ_Hard_SPI_Fault(LCD200_SPI_USE)) {
        if (!s_recovered || (uint32_t)(now - s_last_recover_ms) >= LCD_RECOVER_COOLDOWN_MS) {
            s_recovered = 1;
            s_last_recover_ms = now;
            lcd_reinit(now);
        }
        return;
    }

    if ((uint32_t)(now - s_last_heal_ms) >= LCD_HEAL_PERIOD_MS) {
        s_last_heal_ms = now;
        Lcd_Invalidate();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一整行（ASCII + UTF-8 汉字混排）
// 参数说明     row     行号 0 ~ LCD_ROWS-1
// 参数说明     style   样式，见 UI_Style_Enum
// 参数说明     fmt     和 printf 一样的 UTF-8 格式字符串
// 备注信息     1. 先把内容解码成 32 个逻辑半角列（汉字占 2 列，不足补空格），
//                 这样新内容一定能完全覆盖旧内容；
//              2. 样式变了或缓存不可信就整行重画；否则逐列比较，只画变了的单元。
//-------------------------------------------------------------------------------------------------------------------
void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    char     line[LCD_LINE_BYTES];
    uint16_t cells[LCD_COLS];
    va_list  args;
    int      len;
    uint8_t  col, full;
    uint16_t y = (uint16_t)(row * LCD_ROW_PITCH);
    const uint8_t *p;

    if (row >= LCD_ROWS || style >= UI_STYLE_NUM || fmt == NULL) {
        return;
    }
    // SPI 已锁存超时：不要继续写（写了也只是一串立即返回），等 Lcd_Poll 重初始化。
    if (NEUQ_Hard_SPI_Fault(LCD200_SPI_USE)) {
        return;
    }

    // 1. 格式化成 UTF-8 文本，再解码成逻辑列；坏字节按空格显示，保证一行始终 32 列
    line[0] = '\0';
    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (len < 0) {
        line[0] = '\0';
    }

    col = 0;
    p = (const uint8_t *)line;
    while (*p != '\0' && col < LCD_COLS) {
        if (*p < 0x80U) {
            // 可见 ASCII 原样显示，控制字符和其它字节换成空格
            cells[col++] = (uint16_t)((*p >= 32U && *p <= 126U) ? *p : (uint8_t)' ');
            p++;
        } else if ((p[0] & 0xE0U) == 0xC0U && (p[1] & 0xC0U) == 0x80U) {
            // 2 字节 UTF-8（Latin、希腊字母等）
            uint16_t cp = (uint16_t)(((uint16_t)(p[0] & 0x1FU) << 6) | (uint16_t)(p[1] & 0x3FU));
            if (col + 2U > LCD_COLS) {
                break;                          // 行尾放不下一个汉字，剩余位置补空格
            }
            cells[col++] = cp;
            cells[col++] = LCD_CELL_CONT;
            p += 2;
        } else if ((p[0] & 0xF0U) == 0xE0U && (p[1] & 0xC0U) == 0x80U && (p[2] & 0xC0U) == 0x80U) {
            // 3 字节 UTF-8（汉字）
            uint16_t cp = (uint16_t)(((uint16_t)(p[0] & 0x0FU) << 12) |
                                     ((uint16_t)(p[1] & 0x3FU) << 6) |
                                     (uint16_t)(p[2] & 0x3FU));
            if (col + 2U > LCD_COLS) {
                break;
            }
            cells[col++] = cp;
            cells[col++] = LCD_CELL_CONT;
            p += 3;
        } else {
            // 非法 UTF-8 或 4 字节字符（字库不支持）：占一列显示空格
            cells[col++] = (uint16_t)' ';
            p++;
        }
    }
    while (col < LCD_COLS) {
        cells[col++] = (uint16_t)' ';
    }

    NEUQ_LCD200_SetPenColor(s_pen_color[style]);
    NEUQ_LCD200_SetBackColor(s_back_color[style]);

    // 2. 样式变了或缓存不可信：整行重画；否则只重画内容不同的单元
    full = (uint8_t)((!s_valid[row] || s_style[row] != (uint8_t)style) ? 1 : 0);
    col = 0;
    while (col < LCD_COLS) {
        uint16_t c = cells[col];

        if (c > 0x7FU && c != LCD_CELL_CONT && (col + 1U) < LCD_COLS) {
            // 汉字：一次覆盖两列，所以两列缓存一起更新
            if (full || s_cells[row][col] != c || s_cells[row][col + 1U] != LCD_CELL_CONT) {
                lcd_draw_zh20((uint16_t)(col * ASCII_Font20.Width), y, lcd_find_zh_glyph(c));
                s_cells[row][col] = c;
                s_cells[row][col + 1U] = LCD_CELL_CONT;
            }
            col += 2;
        } else {
            // 半角字符（或放不下的宽字符占位）：只覆盖一列
            uint8_t ch = (c >= 32U && c <= 126U) ? (uint8_t)c : (uint8_t)' ';
            if (full || s_cells[row][col] != (uint16_t)ch) {
                NEUQ_LCD200_DisplayChar((uint16_t)(col * ASCII_Font20.Width), y, ch);
                s_cells[row][col] = (uint16_t)ch;
            }
            col++;
        }
    }

    s_style[row] = (uint8_t)style;
    s_valid[row] = 1;
}
