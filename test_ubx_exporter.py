import math
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


def nav_sat_sv(gnss_id, sv_id, cno, pr_res_dm, used):
    # flags: qualityInd 7 (bits 0-2), svUsed bit 3; prRes is in 0.1 m.
    return struct.pack("<BBBbhhI", gnss_id, sv_id, cno, 40, 100, pr_res_dm, 0x07 | used << 3)


def nav_sat(svs):
    return ubx_frame(b"\x01\x35", struct.pack("<IBBH", 0, 1, len(svs), 0) + b"".join(svs))


def spoofed_receiver_frames():
    """Frames shaped like the 2026-10-05 spoofing incident: GPS used with
    ~267 m residuals, GLONASS tracked but rejected, Galileo listed with no
    signal, 27.8 m/s ground speed on a fixed antenna."""
    svs = [
        nav_sat_sv(0, 5, 37, 2670, 1),
        nav_sat_sv(0, 7, 38, -2900, 1),
        nav_sat_sv(0, 9, 36, 2600, 1),
        nav_sat_sv(6, 1, 34, 0, 0),
        nav_sat_sv(6, 2, 32, 0, 0),
        nav_sat_sv(2, 3, 0, 0, 0),
    ]
    pvt = bytearray(92)
    struct.pack_into("<i", pvt, 60, 27800)
    return {
        # Raw NAV-DOP fields are DOP * 100, as the receiver sends them.
        "NAV-DOP": ubx_frame(b"\x01\x04", struct.pack("<IHHHHHHH", 0, 141, 129, 70, 100, 80, 60, 50)),
        # flags2 bits 3-4 = spoofDetState 2 (spoofing indicated).
        "NAV-STATUS": ubx_frame(b"\x01\x03", struct.pack("<IBBBBII", 0, 3, 0x0D, 0, 2 << 3, 0, 0)),
        "NAV-SAT": nav_sat(svs),
        "NAV-PVT": ubx_frame(b"\x01\x07", bytes(pvt)),
    }


class TestUpdateBasic(unittest.TestCase):
    def test_gauges_match_receiver(self):
        got = {k: UBXReader.parse(v) for k, v in spoofed_receiver_frames().items()}
        with mock.patch.object(ubx_exporter, "poll", return_value=got):
            ubx_exporter.update_basic("/dev/test", None, None, {})

        nan = float("nan")
        for metric, labels, want in [
            ("ublox_dop_geometric", {}, 1.41),
            ("ublox_dop_position", {}, 1.29),
            ("ublox_dop_time", {}, 0.70),
            ("ublox_dop_vertical", {}, 1.00),
            ("ublox_dop_horizontal", {}, 0.80),
            ("ublox_dop_northing", {}, 0.60),
            ("ublox_dop_easting", {}, 0.50),
            ("ublox_spoof_detection_state", {}, 2),
            ("ublox_pvt_ground_speed_mm_per_s", {}, 27800),
            ("ublox_sat_count", {}, 6),
            ("ublox_sat_tracked", {"gnss": "GPS"}, 3),
            ("ublox_sat_tracked", {"gnss": "GLONASS"}, 2),
            ("ublox_sat_tracked", {"gnss": "Galileo"}, 0),
            ("ublox_sat_used", {"gnss": "GPS"}, 3),
            ("ublox_sat_used", {"gnss": "GLONASS"}, 0),
            ("ublox_sat_used", {"gnss": "Galileo"}, 0),
            ("ublox_sat_pr_residual_median_m", {"gnss": "GPS"}, 267.0),
            ("ublox_sat_pr_residual_max_m", {"gnss": "GPS"}, 290.0),
            ("ublox_sat_pr_residual_median_m", {"gnss": "GLONASS"}, nan),
            ("ublox_sat_pr_residual_max_m", {"gnss": "GLONASS"}, nan),
        ]:
            with self.subTest(metric=metric, **labels):
                got = REGISTRY.get_sample_value(metric, {"port": "/dev/test", **labels})
                if math.isnan(want):
                    self.assertTrue(math.isnan(got), f"{got} is not NaN")
                else:
                    self.assertAlmostEqual(got, want)

    def test_dropped_constellation_reads_zero(self):
        frames = spoofed_receiver_frames()
        gps_only = nav_sat([nav_sat_sv(0, 5, 37, 30, 1)])
        for sat in (frames["NAV-SAT"], gps_only):
            with mock.patch.object(ubx_exporter, "poll", return_value={"NAV-SAT": UBXReader.parse(sat)}):
                ubx_exporter.update_basic("/dev/drop", None, None, {})

        labels = {"port": "/dev/drop", "gnss": "GLONASS"}
        self.assertEqual(REGISTRY.get_sample_value("ublox_sat_tracked", labels), 0)
        self.assertEqual(REGISTRY.get_sample_value("ublox_sat_used", labels), 0)
        self.assertTrue(math.isnan(REGISTRY.get_sample_value("ublox_sat_pr_residual_median_m", labels)))
        self.assertTrue(math.isnan(REGISTRY.get_sample_value("ublox_sat_pr_residual_max_m", labels)))
        self.assertEqual(REGISTRY.get_sample_value("ublox_sat_used", {**labels, "gnss": "GPS"}), 1)


if __name__ == "__main__":
    unittest.main()
