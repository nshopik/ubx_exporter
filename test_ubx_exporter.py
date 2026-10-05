import struct
import unittest
from unittest import mock

from prometheus_client import REGISTRY
from pyubx2 import UBXReader

import ubx_exporter


def ubx_frame(cls_id, payload):
    body = cls_id + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


class TestNavDop(unittest.TestCase):
    def test_dop_gauges_match_receiver(self):
        # Raw NAV-DOP fields are DOP * 100, as the receiver sends them.
        raw = struct.pack("<IHHHHHHH", 0, 141, 129, 70, 100, 80, 60, 50)
        msg = UBXReader.parse(ubx_frame(b"\x01\x04", raw))
        with mock.patch.object(ubx_exporter, "poll", return_value={"NAV-DOP": msg}):
            ubx_exporter.update_basic("/dev/test", None, None, {})

        for metric, want in [
            ("ublox_dop_geometric", 1.41),
            ("ublox_dop_position", 1.29),
            ("ublox_dop_time", 0.70),
            ("ublox_dop_vertical", 1.00),
            ("ublox_dop_horizontal", 0.80),
            ("ublox_dop_northing", 0.60),
            ("ublox_dop_easting", 0.50),
        ]:
            with self.subTest(metric=metric):
                got = REGISTRY.get_sample_value(metric, {"port": "/dev/test"})
                self.assertAlmostEqual(got, want)


if __name__ == "__main__":
    unittest.main()
