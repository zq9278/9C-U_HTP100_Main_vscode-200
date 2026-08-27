import math
import struct
import unittest

from protocol import (
    CMD_HELLO, PROFILE_KEYS, RSP_TELEMETRY, FrameParser, build_frame,
    build_screen_work_frame, crc16, decode_profile, decode_telemetry, encode_profile,
    encode_profile_field,
)


class ProtocolTests(unittest.TestCase):
    def test_frame_round_trip_in_fragmented_screen_stream(self):
        frame = build_frame(CMD_HELLO, 7, b"abc")
        parser = FrameParser()
        first, ignored1 = parser.feed(b"\x5A\xA5screen" + frame[:5])
        second, ignored2 = parser.feed(frame[5:])
        self.assertEqual(first, [])
        self.assertEqual(len(second), 1)
        self.assertEqual(second[0].command, CMD_HELLO)
        self.assertEqual(second[0].sequence, 7)
        self.assertEqual(second[0].payload, b"abc")
        self.assertGreater(ignored1 + ignored2, 0)

    def test_crc_reference(self):
        self.assertEqual(crc16(b"123456789"), 0x4B37)

    def test_profile_round_trip(self):
        values = {key: float(index + 1) for index, key in enumerate(PROFILE_KEYS)}
        index, decoded = decode_profile(encode_profile(3, values))
        self.assertEqual(index, 3)
        for key in PROFILE_KEYS:
            self.assertTrue(math.isclose(values[key], decoded[key], rel_tol=1e-6))

    def test_short_ram_field_payload(self):
        payload = encode_profile_field(2, "kp", 123.5)
        self.assertEqual(len(payload), 6)
        profile, field, value = struct.unpack("<BBf", payload)
        self.assertEqual((profile, field), (2, 9))
        self.assertAlmostEqual(value, 123.5)

    def test_telemetry_layout(self):
        payload = struct.pack("<IffiiBBBHBB", 123, 350.0, 348.5, 1000, 900,
                              2, 1, 5, 0x0402, 0, 1)
        frame = build_frame(RSP_TELEMETRY, 2, payload)
        parsed, _ = FrameParser().feed(frame)
        data = decode_telemetry(parsed[0].payload)
        self.assertEqual(data["fault"], 0x0402)
        self.assertAlmostEqual(data["pressure_mmhg"], 348.5)

    def test_extended_telemetry_layout(self):
        payload = struct.pack("<IffiiBBBHBB", 123, 350.0, 348.5, 1000, 900,
                              2, 1, 5, 0, 1, 1)
        payload += struct.pack("<BBBBBHH", 2, 1, 1, 0, 1, 88, 4117)
        data = decode_telemetry(payload)
        self.assertEqual(data["eye"], 2)
        self.assertEqual(data["home_valid"], 1)
        self.assertEqual(data["debug_mode"], 1)
        self.assertEqual(data["battery_mv"], 4117)

    def test_screen_work_frame_is_separate(self):
        frame = build_screen_work_frame(0x1037, 350.0)
        self.assertEqual(frame[:3], b"\x5A\xA5\x0D")
        self.assertEqual(frame[3:5], b"\x10\x37")
        self.assertEqual(frame[-2:], b"\xFF\xFF")


if __name__ == "__main__":
    unittest.main()
