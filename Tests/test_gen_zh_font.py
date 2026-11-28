import contextlib
import importlib.util
import io
from pathlib import Path
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gen_zh_font', ROOT / 'Tools/gen_zh_font.py')
font = importlib.util.module_from_spec(spec)
spec.loader.exec_module(font)


def run_quietly(argv):
    with contextlib.redirect_stdout(io.StringIO()):
        return font.main(argv)


class GlyphTableTests(unittest.TestCase):
    def test_checked_in_table_parses_and_is_sorted(self):
        glyphs = font.parse_existing(font.OUT_HDR)
        self.assertGreater(len(glyphs), 0)
        self.assertEqual(list(glyphs), sorted(glyphs))
        for bitmap in glyphs.values():
            self.assertEqual(len(bitmap), font.HEIGHT * font.STRIDE)

    def test_ui_strings_have_all_glyphs(self):
        wanted = {ord(ch) for ch in font.collect_chars()}
        glyphs = set(font.parse_existing(font.OUT_HDR))
        missing = sorted(wanted - glyphs)
        self.assertEqual(
            missing,
            [],
            '菜单文案有字库缺字，请运行 python Tools/gen_zh_font.py：'
            + ''.join(chr(code) for code in missing),
        )

    def test_render_header_round_trips(self):
        glyphs = font.parse_existing(font.OUT_HDR)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'Drv_Lcd_FontZh.h'
            path.write_text(font.render_header(sorted(glyphs.items())), encoding='utf-8')
            self.assertEqual(font.parse_existing(path), glyphs)

    def test_merge_keeps_existing_without_pillow(self):
        original = font.parse_existing(font.OUT_HDR)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'Drv_Lcd_FontZh.h'
            shutil.copyfile(font.OUT_HDR, path)
            with self._patched(path, Path(folder), lambda: []):
                run_quietly([])
                self.assertEqual(font.parse_existing(path), original)

    def test_prune_keeps_only_referenced(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'Drv_Lcd_FontZh.h'
            shutil.copyfile(font.OUT_HDR, path)
            with self._patched(path, Path(folder), lambda: ['智', '赛']):
                run_quietly(['--prune'])
                self.assertEqual(sorted(font.parse_existing(path)), [ord('智'), ord('赛')])

    @contextlib.contextmanager
    def _patched(self, out_hdr, root, collect_chars):
        old = font.OUT_HDR, font.ROOT, font.collect_chars
        try:
            font.OUT_HDR, font.ROOT, font.collect_chars = out_hdr, root, collect_chars
            yield
        finally:
            font.OUT_HDR, font.ROOT, font.collect_chars = old


if __name__ == '__main__':
    unittest.main()
