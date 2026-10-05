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


def nav_sat_sv(gnss_id, sv_id, cno, pr_res_dm, used, elev=40, azim=100, quality=7):
    # flags: qualityInd bits 0-2, svUsed bit 3; prRes is in 0.1 m.
    return struct.pack("<BBBbhhI", gnss_id, sv_id, cno, elev, azim, pr_res_dm, quality | used << 3)


def nav_sat(svs):
    return ubx_frame(b"\x01\x35", struct.pack("<IBBH", 0, 1, len(svs), 0) + b"".join(svs))


def spoofed_receiver_frames():
    """Frames shaped like the 2026-10-05 spoofing incident: GPS used with
    ~267 m residuals, GLONASS tracked but rejected, Galileo listed with no
    signal, 27.8 m/s ground speed on a fixed antenna."""
    svs = [
        nav_sat_sv(0, 5, 37, 2670, 1),
        nav_sat_sv(0, 7, 38, -2900, 1, elev=12, azim=251),
        nav_sat_sv(0, 9, 36, 2600, 1),
        nav_sat_sv(6, 1, 34, 0, 0),
        nav_sat_sv(6, 2, 32, -15, 0, elev=65, azim=310, quality=4),
        nav_sat_sv(2, 3, 0, 0, 0, elev=-5, azim=0, quality=1),
        nav_sat_sv(0, 25, 35, -32768, 0, elev=10, azim=164),
        nav_sat_sv(0, 29, 35, -32768, 1, elev=62, azim=108),
    ]
    pvt = bytearray(92)
    struct.pack_into("<i", pvt, 60, 27800)
    return {
        # flags bits 0-1 = jammingState 3 (critical).
        "MON-RF": ubx_frame(b"\x0a\x38", struct.pack("<BBH", 0, 1, 0) + struct.pack(
            "<BBBBIIHHBbBbB3x", 0, 3, 2, 1, 0, 0, 92, 5800, 14, 0, 120, 0, 118)),
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
            ("ublox_jamming_state", {}, 3),
            ("ublox_pvt_ground_speed_mm_per_s", {}, 27800),
            ("ublox_sat_count", {}, 8),
            ("ublox_sat_tracked", {"gnss": "GPS"}, 5),
            ("ublox_sat_tracked", {"gnss": "GLONASS"}, 2),
            ("ublox_sat_tracked", {"gnss": "Galileo"}, 0),
            ("ublox_sat_used", {"gnss": "GPS"}, 4),
            ("ublox_sat_used", {"gnss": "GLONASS"}, 0),
            ("ublox_sat_used", {"gnss": "Galileo"}, 0),
            ("ublox_sat_pr_residual_median_m", {"gnss": "GPS"}, 267.0),
            ("ublox_sat_pr_residual_max_m", {"gnss": "GPS"}, 290.0),
            ("ublox_sat_pr_residual_median_m", {"gnss": "GLONASS"}, nan),
            ("ublox_sat_pr_residual_max_m", {"gnss": "GLONASS"}, nan),
            ("ublox_sv_cno_dbhz", {"gnss": "GPS", "svid": "5"}, 37),
            ("ublox_sv_pr_residual_m", {"gnss": "GPS", "svid": "5"}, 267.0),
            ("ublox_sv_used", {"gnss": "GPS", "svid": "5"}, 1),
            ("ublox_sv_cno_dbhz", {"gnss": "GPS", "svid": "7"}, 38),
            ("ublox_sv_elevation_deg", {"gnss": "GPS", "svid": "7"}, 12),
            ("ublox_sv_azimuth_deg", {"gnss": "GPS", "svid": "7"}, 251),
            ("ublox_sv_pr_residual_m", {"gnss": "GPS", "svid": "7"}, -290.0),
            ("ublox_sv_quality", {"gnss": "GPS", "svid": "7"}, 7),
            ("ublox_sv_cno_dbhz", {"gnss": "GLONASS", "svid": "2"}, 32),
            ("ublox_sv_elevation_deg", {"gnss": "GLONASS", "svid": "2"}, 65),
            ("ublox_sv_azimuth_deg", {"gnss": "GLONASS", "svid": "2"}, 310),
            ("ublox_sv_pr_residual_m", {"gnss": "GLONASS", "svid": "2"}, -1.5),
            ("ublox_sv_quality", {"gnss": "GLONASS", "svid": "2"}, 4),
            ("ublox_sv_used", {"gnss": "GLONASS", "svid": "2"}, 0),
            ("ublox_sv_cno_dbhz", {"gnss": "Galileo", "svid": "3"}, 0),
            ("ublox_sv_elevation_deg", {"gnss": "Galileo", "svid": "3"}, -5),
            ("ublox_sv_quality", {"gnss": "Galileo", "svid": "3"}, 1),
            ("ublox_sv_pr_residual_m", {"gnss": "GPS", "svid": "25"}, nan),
            ("ublox_sv_pr_residual_m", {"gnss": "GPS", "svid": "29"}, nan),
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
        sv5 = {"port": "/dev/drop", "gnss": "GPS", "svid": "5"}
        self.assertEqual(REGISTRY.get_sample_value("ublox_sv_pr_residual_m", sv5), 3.0)
        for metric in ("ublox_sv_cno_dbhz", "ublox_sv_elevation_deg", "ublox_sv_azimuth_deg",
                       "ublox_sv_pr_residual_m", "ublox_sv_quality", "ublox_sv_used"):
            for gone in ({**sv5, "svid": "7"}, {**sv5, "gnss": "GLONASS", "svid": "1"}):
                with self.subTest(metric=metric, **gone):
                    self.assertIsNone(REGISTRY.get_sample_value(metric, gone))


if __name__ == "__main__":
    unittest.main()
