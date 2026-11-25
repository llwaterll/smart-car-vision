#ifndef DRV_LCD_H
#define DRV_LCD_H
/*
 * ============================================================================
 * 文件名称：Drv_Lcd.h
 * ============================================================================
 * 模块功能：基于 NEUQ_LCD200 的“按行显示文字”，横屏 320x240，10 行 × 32 列。
 *
 *   外设资源：ST7789V2 并口屏，初始化与像素写入由 NEUQ_LCD200 完成。
 *
 *   文字格式：ASCII 用库自带的 10x20 字模；UTF-8 汉字用 20x20 点阵，
 *   占两个半角列，所以一行最多 32 个半角字符或 16 个汉字。
 *
 *   运行视角
 *     菜单每 120 ms 只重画“变了的那一行里变了的那几个字符”：
 *     每行缓存上次显示的内容，逐列比较后才发送像素，所以刷新数值不闪。
 *
 *   为什么不用 LCD200_DisplayString 整行重画：
 *     一个 10×20 字符要传 200 像素，整页重画几十毫秒并伴随闪屏；
 *     按字符增量刷新通常只传几个字符，快且不闪。汉字整块 20×20
 *     一次写入，也不会退化成逐点调用。
 *
 *   失步自愈（防止电池供电时屏幕花屏/卡死）：
 *     1. 主循环调 Lcd_Poll()，每 LCD_HEAL_PERIOD_MS 让缓存失效一次，
 *        随后 10 行依次整行重画，屏幕内容能自己恢复；
 *     2. SPI 等待有超时锁存，一旦超时，Lcd_Poll() 重新初始化屏幕，
 *        不会停在某个像素的忙等里。
 * ============================================================================
 */

#include <stdint.h>

#define LCD_ROWS             (10)        // 行数
#define LCD_COLS             (32)        // 每行逻辑列数（320 像素 / 半角宽 10；1 个汉字占 2 列）
#define LCD_LINE_BYTES       (96)        // 单行 UTF-8 格式化缓冲区字节数（含结尾 '\0'）
#define LCD_ROW_PITCH        (24)        // 行距，单位像素（字高 20 + 行间隔 4）
#define LCD_HEAL_PERIOD_MS   (2000U)     // 周期整屏重画间隔：LCD 失步后靠它恢复内容

typedef enum {                          // 行的显示样式（字色 / 底色）
    UI_STYLE_NORMAL = 0,                // 黑字白底：普通内容
    UI_STYLE_TITLE,                     // 白字深蓝底：页面标题
    UI_STYLE_SELECT,                    // 白字蓝底：光标所在行
    UI_STYLE_EDIT,                      // 黑字黄底：正在编辑的参数
    UI_STYLE_GOOD,                      // 深绿字：正常 / 运行中
    UI_STYLE_WARN,                      // 红字：警告 / 请等待
    UI_STYLE_HINT,                      // 灰字：操作提示
    UI_STYLE_NUM
} UI_Style_Enum;

void Lcd_Init(void);                                                    // 初始化屏幕（白底）
void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...); // 显示一整行：不足补空格，超出截断
void Lcd_Invalidate(void);                                              // 忘掉缓存，下次显示时每行整行重画
void Lcd_Poll(uint32_t now);                                            // 主循环周期调用：整屏重画 + SPI 超时后重初始化

#endif
