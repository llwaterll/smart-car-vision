import importlib.util
from pathlib import Path
import struct
import unittest
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location('car_host', Path(__file__).resolve().parents[1] / 'Tools/car_host.py')
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)


def telemetry_frame(left, right, state, frame_type=0xA5):
    """按固件 A5/A6 打包帧：类型 + 两个 int16 大端 + 状态 + XOR。"""
    body = bytes([frame_type]) + struct.pack('>hh', left, right) + bytes([state])
    return body + bytes([body[0] ^ body[1] ^ body[2] ^ body[3] ^ body[4] ^ body[5]])


class ProtocolTests(unittest.TestCase):
    def test_crc_golden(self):
        self.assertEqual(host.crc8(b'123456789'), 0xf4)
        self.assertEqual(host.packet(2), bytes.fromhex('c3 3c 02 00 2a'))

    def test_signed_and_range(self):
        self.assertEqual(host.wheels(80, -80)[4:8], struct.pack('>hh', 80, -80))
        self.assertEqual(host.wheels(900, -900)[4:8], struct.pack('>hh', 900, -900))
        with self.assertRaises(ValueError): host.wheels(901, 0)

    def test_pid_validation(self):
        self.assertEqual(host.pid_packet(1, '2', '.12', '0')[4:-1], bytes.fromhex('01 00 c8 00 0c 00 00'))
        for value in ['nan', 'inf', '-1', '0.001', '656']:
            with self.assertRaises(ValueError): host.pid_packet(0, value, '0', '0')

    def test_openloop_validation(self):
        self.assertEqual(host.openloop_packet(0, 1500)[2:], bytes.fromhex('06 03 00 05 dc 2a'))
        self.assertEqual(host.openloop_packet(1, -1500)[4:-1], bytes.fromhex('01 fa 24'))
        for pwm in (3001, -3001):
            with self.assertRaises(ValueError): host.openloop_packet(0, pwm)
        with self.assertRaises(ValueError): host.openloop_packet(2, 0)

    def test_differential_mix(self):
        self.assertEqual(host.mix(1, 0, 80), (80, 80))
        self.assertEqual(host.mix(0, 1, 80), (-80, 80))
        self.assertEqual(host.mix(0, 0, 80), (0, 0))
        l, r = host.mix(1, 1, 180)
        self.assertLess(l, r)
        self.assertLessEqual(max(abs(l), abs(r)), 180)

    def test_fragmented_telemetry(self):
        frame = telemetry_frame(3, 0, 0)
        reader = host.TelemetryReader()
        self.assertEqual(reader.feed(b'garbage' + frame[:3]), [])
        result = reader.feed(frame[3:] + frame)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][3], 3)
        reader.feed(b'x' * 10000)
        self.assertLessEqual(len(reader.buffer), 4096)

    def test_slow_frame_updates_position(self):
        reader = host.TelemetryReader()
        self.assertEqual(reader.feed(telemetry_frame(123, -456, 0, host.TelemetryReader.SLOW)), [])
        self.assertEqual(reader.last_slow, (1.23, -4.56))

class Var:
    def __init__(self, value): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value

class Port:
    def __init__(self): self.sent = []; self.input = b''; self.closed = False
    def write(self, data): self.sent.append(data); return len(data)
    def read(self, count): data, self.input = self.input[:count], self.input[count:]; return data
    @property
    def in_waiting(self): return len(self.input)
    def close(self): self.closed = True

class DrivingSafetyTests(unittest.TestCase):
    def setUp(self):
        self.app = host.Console.__new__(host.Console)
        self.app.port = Port()
        self.app.armed = False
        self.app.pending_arm = False
        self.app.stop_retries = 0
        self.app.stop_retry_at = 0
        self.app.keys = set()
        self.app.sample = None
        self.app.last_rx = host.time.monotonic()
        self.app.next_tx = 0
        # 这些用例只验证使能/停车安全，不验证空闲心跳；把心跳截止放到无穷远，
        # 保持原有“发送序列只含使能/停车帧”的断言。
        self.app.next_hb = float('inf')
        self.app.rpm = Var(60)
        self.app.status = Var('')
        self.app.values = Var('')
        self.app.actual_pid = Var('')
        self.app.log_file = None
        self.app.reader = host.TelemetryReader()
        self.app.root = SimpleNamespace(after=lambda *a: None, focus_get=lambda: None)
        self.app.tk = SimpleNamespace(TclError=RuntimeError)
        self.app.target = [0, 0]
        self.app.local_gains = [[8.0, 0.3, 0.0], [8.0, 0.3, 0.0]]

    def telemetry(self, state, fault=0):
        self.app.port.input += telemetry_frame(0, 0, state)
        self.app.poll()

    def test_cannot_arm_without_stopped_telemetry(self):
        self.app.arm()
        self.assertEqual(self.app.port.sent, [])
        self.telemetry(2, 1)
        self.app.arm()
        self.assertEqual(self.app.port.sent, [])

    def test_requires_confirmation_then_release_brakes(self):
        self.telemetry(0)
        self.app.arm()
        self.assertFalse(self.app.armed)
        self.app.set_key('w', True)
        self.assertEqual(self.app.port.sent, [host.packet(1)])
        self.telemetry(1)
        self.app.set_key('w', True)
        self.assertEqual(self.app.port.sent[-1], host.wheels(60, 60))
        self.app.set_key('w', False)
        self.assertEqual(self.app.port.sent[-1], host.wheels(0, 0))

    def test_fault_and_focus_loss_never_resume(self):
        self.app.armed = True
        self.app.keys.add('w')
        self.telemetry(2, 1)
        self.assertFalse(self.app.armed)
        self.assertFalse(self.app.keys)
        self.telemetry(1)
        self.assertFalse(self.app.armed)
        self.app.armed = True
        self.app.focus_check()
        self.assertEqual(self.app.port.sent[-1], host.packet(2))
        self.assertFalse(self.app.armed)

    def test_stale_telemetry_disarms(self):
        self.app.armed = True
        self.app.last_rx -= 1
        self.app.poll()
        self.assertEqual(self.app.port.sent[-1], host.packet(2))
        self.assertFalse(self.app.armed)

    def test_stop_retries_after_fragment_timeout_and_blocks_arm(self):
        self.app.armed = True
        self.app.stop()
        self.telemetry(0)
        self.app.arm()
        self.assertNotIn(host.packet(1), self.app.port.sent)
        self.app.stop_retry_at = 0
        self.app.poll()
        self.assertEqual(self.app.port.sent, [host.packet(2), host.packet(2)])

if __name__ == '__main__': unittest.main()
