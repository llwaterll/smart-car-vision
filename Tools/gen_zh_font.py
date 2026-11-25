"""Generate the 20x20 Chinese bitmap table (CODE/Drv_Lcd_FontZh.h) from UI strings.

Scans string literals in CODE/Menu_Task.c and CODE/Platform_Main.c (the files
that hand text to Lcd_Print), renders every unique non-ASCII BMP character into
a 20x20 1-bit bitmap with SimHei (uniform stroke width), and emits a
code-point-sorted lookup table for binary search on STM32F407.

Existing glyphs already present in the output header are reused by default, so
the table grows incrementally: translate a menu string, rerun this script, and
only the new characters are rasterised. Pass --prune to drop glyphs that no UI
string references any more.

Rendering new glyphs needs Pillow and a Chinese TrueType font; neither is
needed when every requested character is already in the table. The original
tool lived in the upstream repository; paths, symbol names and the generated
header banner here follow this project's conventions.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
UI_SOURCES = [
    ROOT / "CODE" / "Menu_Task.c",
    ROOT / "CODE" / "Platform_Main.c",
]
OUT_HDR = ROOT / "CODE" / "Drv_Lcd_FontZh.h"

FONT_CANDIDATES = [
    pathlib.Path(r"C:\Windows\Fonts\simhei.ttf"),
    pathlib.Path(r"C:\Windows\Fonts\msyh.ttc"),
    pathlib.Path(r"C:\Windows\Fonts\simsun.ttc"),
]

WIDTH = 20
HEIGHT = 20
STRIDE = (WIDTH + 7) // 8  # 3 bytes per row (24 bits, top 20 bits used)
FONT_SIZE = 19
THRESHOLD = 142

# C 源码里的注释与字符串：切掉注释，保留字符串/字符字面量
TOKEN_RE = re.compile(
    r"""//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'""",
    re.DOTALL,
)
# 已生成的字模项：{ 0x4E00U, { 0x00, ... } },
ENTRY_RE = re.compile(r"\{\s*0x([0-9A-Fa-f]+)U,\s*\{\s*([0-9A-Fa-fx,\s]+?)\s*\}\s*\}")

BANNER = """/*
 * ============================================================================
 * 文件名称：Drv_Lcd_FontZh.h
 * ============================================================================
 * 模块功能：20x20 汉字点阵表，供 Lcd_Print 显示 UTF-8 菜单文字。
 *
 *   点阵按行存储，每行 LCD_ZH_STRIDE 字节，最高位对应最左像素；
 *   数组按 Unicode 码点升序排列，查找用二分。
 *
 *   本文件由 Tools/gen_zh_font.py 扫描菜单源文件里的中文字符串生成。
 *   默认保留已有字模，菜单新增中文后重新运行即可增量补字；
 *   加 --prune 会删掉已不再被 UI 字符串引用的字。
 * ============================================================================
 */
/* 以下字模数据由生成器直接产出，请勿手工修改。 */"""


def strip_c_comments(text: str) -> str:
    """Replace comments with nothing while keeping string and char literals."""
    return TOKEN_RE.sub(
        lambda m: m.group(0) if m.group(0)[0] in ('"', "'") else "",
        text,
    )


def extract_string_literals(text: str) -> list[str]:
    clean = strip_c_comments(text)
    return re.findall(r'"((?:\\.|[^"\\])*)"', clean)


def collect_chars() -> list[str]:
    """Return every unique non-ASCII BMP character used in UI string literals."""
    chars: set[str] = set()
    for src in UI_SOURCES:
        if not src.exists():
            raise FileNotFoundError(f"UI source not found: {src}")
        for lit in extract_string_literals(src.read_text(encoding="utf-8")):
            for ch in lit:
                if ord(ch) > 127:
                    chars.add(ch)
    for ch in chars:
        if ord(ch) > 0xFFFF:
            raise ValueError(f"Non-BMP character U+{ord(ch):X} not supported: {ch!r}")
    return sorted(chars, key=ord)


def parse_existing(path: pathlib.Path) -> dict[int, bytes]:
    """Read the glyphs already in the output header so they can be reused."""
    if not path.exists():
        return {}
    glyphs: dict[int, bytes] = {}
    for match in ENTRY_RE.finditer(path.read_text(encoding="utf-8")):
        code = int(match.group(1), 16)
        values = [int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})", match.group(2))]
        if len(values) != HEIGHT * STRIDE:
            raise ValueError(
                f"U+{code:04X} has {len(values)} bitmap bytes, expected {HEIGHT * STRIDE}"
            )
        glyphs[code] = bytes(values)
    return glyphs


def load_pillow():
    """Import Pillow lazily so reusing an existing table needs no dependency."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise SystemExit(
            "渲染新字需要 Pillow：python -m pip install pillow\n"
            "（已有字模的增量/保留模式不需要它）"
        ) from exc
    return Image, ImageDraw, ImageFont


def load_font():
    _, _, ImageFont = load_pillow()
    for path in FONT_CANDIDATES:
        if path.exists():
            return ImageFont.truetype(str(path), FONT_SIZE)
    raise FileNotFoundError(
        "未找到中文字体，请确认 C:\\Windows\\Fonts 下有 simhei.ttf / msyh.ttc / simsun.ttc"
    )


def render_glyph(font, ch: str) -> bytes:
    Image, ImageDraw, _ = load_pillow()
    # Render onto an oversized canvas, crop the inked bbox, center inside 20x20.
    canvas = Image.new("L", (40, 40), 0)
    draw = ImageDraw.Draw(canvas)
    draw.text((10, 10), ch, fill=255, font=font)
    bbox = canvas.getbbox()
    out = Image.new("L", (WIDTH, HEIGHT), 0)
    if bbox is not None:
        cropped = canvas.crop(bbox)
        cw, ch_h = cropped.size
        if cw > WIDTH or ch_h > HEIGHT:
            scale = min(WIDTH / cw, HEIGHT / ch_h)
            cw = max(1, int(round(cw * scale)))
            ch_h = max(1, int(round(ch_h * scale)))
            resample = getattr(Image, "Resampling", Image).LANCZOS
            cropped = cropped.resize((cw, ch_h), resample)
        ox = (WIDTH - cw) // 2
        oy = (HEIGHT - ch_h) // 2
        out.paste(cropped, (ox, oy))

    px = out.load()
    rows = bytearray()
    for y in range(HEIGHT):
        row_bytes = [0] * STRIDE
        for x in range(WIDTH):
            if px[x, y] >= THRESHOLD:
                row_bytes[x >> 3] |= 0x80 >> (x & 7)
        rows.extend(row_bytes)
    assert len(rows) == HEIGHT * STRIDE
    return bytes(rows)


def preview_glyph(ch: str, data: bytes) -> str:
    lines = [f"=== {ch} (U+{ord(ch):04X}) ==="]
    for y in range(HEIGHT):
        row = data[y * STRIDE : (y + 1) * STRIDE]
        bits = "".join(
            "#" if (row[x >> 3] & (0x80 >> (x & 7))) else "." for x in range(WIDTH)
        )
        lines.append(bits)
    return "\n".join(lines)


def render_header(entries: list[tuple[int, bytes]]) -> str:
    """Emit the complete header in the same layout used by the checked-in file."""
    lines = BANNER.splitlines() + [
        "#ifndef DRV_LCD_FONT_ZH_H",
        "#define DRV_LCD_FONT_ZH_H",
        "",
        "#include <stdint.h>",
        "",
        f"#define LCD_ZH_WIDTH   {WIDTH}U",
        f"#define LCD_ZH_HEIGHT  {HEIGHT}U",
        f"#define LCD_ZH_STRIDE  {STRIDE}U",
        f"#define LCD_ZH_BYTES   {HEIGHT * STRIDE}U",
        "",
        "typedef struct {",
        "    uint16_t unicode;",
        "    uint8_t  bitmap[LCD_ZH_BYTES];",
        "} LcdZhGlyph;",
        "",
        "static const LcdZhGlyph s_lcd_zh_font[] = {",
    ]
    for code, bitmap in entries:
        hex_vals = ", ".join(f"0x{b:02X}" for b in bitmap)
        lines.append(f"    /* {chr(code)} U+{code:04X} */")
        lines.append(f"    {{ 0x{code:04X}U, {{ {hex_vals} }} }},")
    lines += [
        "};",
        "",
        "#define LCD_ZH_COUNT ((unsigned)(sizeof(s_lcd_zh_font) / sizeof(s_lcd_zh_font[0])))",
        "",
        "#endif",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="从菜单字符串生成 20x20 汉字点阵表（CODE/Drv_Lcd_FontZh.h）"
    )
    parser.add_argument(
        "--prune",
        action="store_true",
        help="只保留当前 UI 字符串引用的字模（会删掉文件里的旧字）",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="在终端用 #/. 打印几个字模，便于检查栅格化结果",
    )
    args = parser.parse_args(argv)

    chars = collect_chars()
    wanted = {ord(ch) for ch in chars}
    existing = parse_existing(OUT_HDR)

    if args.prune:
        merged = {code: bitmap for code, bitmap in existing.items() if code in wanted}
    else:
        merged = dict(existing)

    new_codes = sorted(wanted - set(merged))
    if new_codes:
        font = load_font()
        for code in new_codes:
            merged[code] = render_glyph(font, chr(code))

    if not merged:
        raise RuntimeError(
            "字库为空：请先在菜单源文件中加入中文字符串，或去掉 --prune"
        )

    entries = sorted(merged.items())
    OUT_HDR.write_text(render_header(entries), encoding="utf-8")
    total_bytes = len(entries) * (2 + HEIGHT * STRIDE)
    print(
        f"Generated {OUT_HDR.relative_to(ROOT)}: {len(entries)} glyphs "
        f"(~{total_bytes} bytes), {len(new_codes)} new, {len(entries) - len(new_codes)} kept"
    )

    if args.preview:
        sample = [
            (code, bitmap)
            for code, bitmap in entries
            if chr(code) in ("智", "赛", "视", "速")
        ][:4] or entries[:4]
        for code, bitmap in sample:
            print(preview_glyph(chr(code), bitmap))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
