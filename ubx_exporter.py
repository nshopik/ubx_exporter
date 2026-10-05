#!/usr/bin/env python3
"""
Prometheus exporter for u-blox GNSS receivers.

Polls UBX-MON-RF, UBX-NAV-STATUS, UBX-NAV-SAT on a fast interval and
UBX-MON-SPAN on a slower one. Exposes everything as Prometheus gauges so
Grafana can render the spectrum as a waterfall heatmap and the health
metrics as time-series.

Install:
    pip install pyubx2 pyserial prometheus_client

Run:
    ./ubx_exporter.py /dev/ttyS5
    ./ubx_exporter.py /dev/ttyS5,/dev/ttyS6                 # multi-port
    ./ubx_exporter.py /dev/ttyS5 --listen 9021 --span-interval 60

Scrape:
    curl http://localhost:9021/metrics

When more than one port is given (comma-separated), each is polled by its
own worker thread; metrics carry a `port="…"` label so Grafana can pick
which receiver to plot. The HTTP listener and the --baud / --basic-interval
/ --span-interval values are shared across all ports.
"""

import argparse
import statistics
import sys
import threading
import time

import serial
from pyubx2 import UBXMessage, UBXReader, POLL, ERR_IGNORE
from prometheus_client import start_http_server, Gauge


LBL = ["port"]
LBL_BLK = ["port", "rf_block"]
LBL_GNSS = ["port", "gnss"]
LBL_SV = ["port", "gnss", "svid"]

# NAV-SAT gnssId -> constellation name for the `gnss` label.
GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou",
              4: "IMES", 5: "QZSS", 6: "GLONASS", 7: "NavIC"}
# port -> gnssIds exported so far; a constellation that leaves NAV-SAT reads 0/NaN, not stale.
seen_gnss = {}
# port -> (gnss, svid) pairs in the last NAV-SAT; a satellite missing from the next one loses its series.
seen_sv = {}

m_fix       = Gauge("ublox_fix", "GPS fix (0=none 2=2D 3=3D 5=time)", LBL)
m_ant       = Gauge("ublox_antenna_status", "Antenna status (1=DK 2=OK 3=SHORT 4=OPEN)", LBL)
m_magi      = Gauge("ublox_mag_i", "ADC I magnitude (>200 = saturating)", LBL)
m_magq      = Gauge("ublox_mag_q", "ADC Q magnitude (>200 = saturating)", LBL)
m_noise     = Gauge("ublox_noise_per_ms", "Noise level (lower=better; <70 good)", LBL)
m_agc       = Gauge("ublox_agc_count", "AGC counter (0-8191)", LBL)
m_jam       = Gauge("ublox_jam_indicator", "CW (narrow-band) interference indicator (0-255); threshold from an unjammed baseline", LBL)
m_jam_state = Gauge("ublox_jamming_state", "MON-RF flags jammingState (broadband): 0=unknown or disabled, 1=ok, 2=warning, 3=critical", LBL)
m_nsv       = Gauge("ublox_sat_count", "Satellites in NAV-SAT report", LBL)
m_cno_above = Gauge("ublox_sat_cno_above_threshold",
                    "Number of sats with C/N0 above given threshold (dB-Hz)",
                    LBL + ["threshold"])
m_max_cno   = Gauge("ublox_sat_max_cno", "Best C/N0 seen this poll (dB-Hz)", LBL)
m_sat_tracked = Gauge(
    "ublox_sat_tracked",
    "NAV-SAT satellites tracked (C/N0 > 0) per constellation. A healthy multi-constellation "
    "antenna tracks every configured constellation; one constellation tracked while the others "
    "drop to 0 or go unused (ublox_sat_used) is a spoofing or band-limited-jamming signature.",
    LBL_GNSS,
)
m_sat_used = Gauge(
    "ublox_sat_used",
    "NAV-SAT satellites used in the navigation solution (svUsed flag) per constellation. "
    "Strong satellites tracked but not used in one constellation while another is fully used "
    "means the receiver rejected them as inconsistent with the solution: suspect spoofing.",
    LBL_GNSS,
)
m_sat_prres_median = Gauge(
    "ublox_sat_pr_residual_median_m",
    "NAV-SAT median absolute pseudorange residual of used satellites per constellation, meters. "
    "How far each measured range sits from the solved position/time. Healthy: under 10 m. "
    "Tens to hundreds of meters across a whole constellation means the ranges disagree with "
    "the sky: spoofing or severe multipath. NaN when no satellite of the constellation is used.",
    LBL_GNSS,
)
m_sat_prres_max = Gauge(
    "ublox_sat_pr_residual_max_m",
    "NAV-SAT maximum absolute pseudorange residual of used satellites per constellation, "
    "meters. A single outlier points at one bad satellite or multipath; compare with the "
    "median to tell that apart from a constellation-wide offset. NaN when none is used.",
    LBL_GNSS,
)
m_sv_cno = Gauge("ublox_sv_cno_dbhz", "NAV-SAT per-satellite carrier-to-noise density, dB-Hz", LBL_SV)
m_sv_elev = Gauge("ublox_sv_elevation_deg", "NAV-SAT per-satellite elevation, degrees", LBL_SV)
m_sv_azim = Gauge("ublox_sv_azimuth_deg", "NAV-SAT per-satellite azimuth, degrees", LBL_SV)
m_sv_prres = Gauge(
    "ublox_sv_pr_residual_m",
    "NAV-SAT per-satellite pseudorange residual as reported (signed), meters, used or not.",
    LBL_SV,
)
m_sv_quality = Gauge(
    "ublox_sv_quality",
    "NAV-SAT per-satellite qualityInd: 0=no signal 1=searching 2=acquired 3=unusable "
    "4=code locked 5-7=code and carrier locked",
    LBL_SV,
)
m_sv_used = Gauge("ublox_sv_used", "NAV-SAT per-satellite svUsed flag (1=used in the solution)", LBL_SV)
SV_GAUGES = (m_sv_cno, m_sv_elev, m_sv_azim, m_sv_prres, m_sv_quality, m_sv_used)
m_uptime    = Gauge("ublox_sample_age_seconds",
                    "Seconds since last successful sample (for each message family)",
                    LBL + ["family"])

m_spec      = Gauge("ublox_spectrum_amplitude",
                    "UBX-MON-SPAN bin amplitude (unitless ~dB)",
                    LBL_BLK + ["bin", "freq_mhz"])
m_spec_ctr  = Gauge("ublox_spectrum_center_mhz", "MON-SPAN center frequency", LBL_BLK)
m_spec_span = Gauge("ublox_spectrum_span_mhz",   "MON-SPAN span width",       LBL_BLK)
m_spec_pga  = Gauge("ublox_spectrum_pga",        "MON-SPAN PGA setting",      LBL_BLK)

# --- Timing-critical metrics (NAV-PVT, TIM-TP, NAV-CLOCK, NAV-DOP) -----------
# HELP strings are written long-form on purpose: they show up in Prometheus
# and in tools like `curl /metrics` so an operator can understand what the
# value means without consulting external docs.

m_pvt_tacc = Gauge(
    "ublox_pvt_time_accuracy_ns",
    "NAV-PVT time accuracy estimate, nanoseconds. THE headline timing-quality "
    "metric: how far off the PPS edge can be from true GNSS time right now. "
    "Healthy <50 ns; warning >100 ns; PPS unusable for serious timing >1000 ns. "
    "Degrades when sat geometry weakens (tDOP rises) or signal quality drops.",
    LBL,
)
m_pvt_hacc = Gauge(
    "ublox_pvt_horizontal_accuracy_mm",
    "NAV-PVT horizontal position accuracy, millimeters. For a stationary timing "
    "receiver this should stabilize at single-digit meters once survey-in completes. "
    "Sudden growth indicates multipath, signal masking, or the antenna physically moved.",
    LBL,
)
m_pvt_vacc = Gauge(
    "ublox_pvt_vertical_accuracy_mm",
    "NAV-PVT vertical position accuracy, millimeters. Vertical is always worse than "
    "horizontal due to satellite geometry (no sats below horizon). Use trend, not absolute.",
    LBL,
)
m_pvt_sats_used = Gauge(
    "ublox_pvt_sats_used",
    "Satellites actually used in the position/time solution (distinct from total visible "
    "in NAV-SAT). Need >=4 for a 3D fix, >=5 for redundancy/RAIM. Falling below 4 means "
    "the receiver is about to lose fix even if many sats appear visible.",
    LBL,
)
m_pvt_fix_ok = Gauge(
    "ublox_pvt_gnss_fix_ok",
    "NAV-PVT gnssFixOK flag: 1 if the fix is within configured DOP/accuracy masks, 0 if "
    "the receiver has a fix but doesn't trust it. Watch for this flapping to 0 while "
    "fix_type still reports 3 — means quality crashed without the fix being formally lost.",
    LBL,
)

m_pvt_lat = Gauge(
    "ublox_pvt_latitude",
    "NAV-PVT reported latitude, degrees. For a stationary timing receiver this should be "
    "rock-stable once survey-in completes. A sudden shift while hAcc remains low is a strong "
    "spoofing indicator — a real degradation would show hAcc climbing first.",
    LBL,
)
m_pvt_lon = Gauge(
    "ublox_pvt_longitude",
    "NAV-PVT reported longitude, degrees. Same rationale as latitude: deviation from the "
    "known baseline position without a corresponding hAcc increase suggests spoofing.",
    LBL,
)
m_pvt_hmsl = Gauge(
    "ublox_pvt_height_msl_mm",
    "NAV-PVT height above mean sea level, millimeters. More operationally useful than "
    "ellipsoidal height. For a roof-mounted antenna this should be constant; drift indicates "
    "multipath or spoofing.",
    LBL,
)

m_pvt_gspeed = Gauge(
    "ublox_pvt_ground_speed_mm_per_s",
    "NAV-PVT 2D ground speed, millimeters per second. A fixed timing antenna should read "
    "near 0 (tens of mm/s of noise). A sustained speed of meters per second on a fixed "
    "antenna means the receiver is following a synthetic trajectory: spoofing.",
    LBL,
)
m_spoof = Gauge(
    "ublox_spoof_detection_state",
    "NAV-STATUS flags2 spoofDetState: 0=unknown or deactivated, 1=no spoofing indicated, "
    "2=spoofing indicated, 3=multiple spoofing indications. The receiver's own detector; "
    "it misses many attacks, so 1 is not proof of a clean sky. Cross-check per-constellation "
    "usage, pseudorange residuals and ground speed.",
    LBL,
)

m_tp_qerr = Gauge(
    "ublox_pps_quantization_error_ps",
    "TIM-TP quantization error, picoseconds. The residual error between the PPS edge the "
    "chip *wanted* to emit and the one it actually emitted, due to clock-domain crossing. "
    "Apply this as a correction to PPS timestamps for sub-ns accuracy. Healthy: within "
    "+/-50 ps. Spikes >1000 ps mean PPS jitter is leaking into downstream chrony/ptp4l.",
    LBL,
)
m_tp_timebase = Gauge(
    "ublox_pps_time_base",
    "TIM-TP time base: 0=GNSS time (e.g. GPS), 1=UTC. Matches the receiver's PPS-source "
    "configuration. If this disagrees with what chrony expects, you get a constant N-second "
    "offset (currently 18 s GPS-UTC, 37 s TAI-UTC).",
    LBL,
)
m_tp_utc_valid = Gauge(
    "ublox_pps_utc_valid",
    "TIM-TP utc flag: 1 if UTC offset known and PPS aligned to UTC. 0 right after cold "
    "start until the receiver decodes the UTC offset from the GPS navigation message "
    "(can take up to 12.5 minutes). Stale 0 indicates the receiver hasn't decoded the "
    "leap-second offset and PPS may be in GPS-time, not UTC.",
    LBL,
)
m_tp_raim = Gauge(
    "ublox_pps_raim_unavailable",
    "TIM-TP raim flag: 1 if RAIM (Receiver Autonomous Integrity Monitoring) is NOT "
    "available for this pulse. RAIM cross-checks sats to detect a bad measurement; "
    "without it, a single faulty satellite can corrupt the PPS without being noticed.",
    LBL,
)

m_clk_bias = Gauge(
    "ublox_clock_bias_ns",
    "NAV-CLOCK receiver clock bias, nanoseconds. The offset between the chip's internal "
    "TCXO-derived time and true GNSS time, as continuously estimated. The receiver steers "
    "PPS to correct for this; large absolute values mean the TCXO has drifted far and "
    "the receiver is working hard to compensate.",
    LBL,
)
m_clk_drift = Gauge(
    "ublox_clock_drift_ns_per_s",
    "NAV-CLOCK receiver clock drift rate, ns/s (== ppb relative). Indicates how fast the "
    "TCXO is moving against GNSS time. Typical TCXO: 5-50 ppb at constant temperature, "
    "swings up to several hundred ppb across temperature. Sudden changes correlate with "
    "thermal events (fan failure, environmental change) or TCXO aging.",
    LBL,
)
m_clk_tacc = Gauge(
    "ublox_clock_time_accuracy_ns",
    "NAV-CLOCK time accuracy estimate, nanoseconds. Same concept as ublox_pvt_time_accuracy_ns "
    "but reported from the clock-solver perspective. Usually tracks closely with PVT version; "
    "divergence between the two indicates a software/firmware issue.",
    LBL,
)
m_clk_facc = Gauge(
    "ublox_clock_frequency_accuracy_ps_per_s",
    "NAV-CLOCK frequency accuracy estimate, ps/s. How well the receiver knows its own "
    "TCXO frequency. Lower = better disciplining headroom. Worsens during signal outages "
    "(holdover) since there's no GNSS reference to refine the estimate.",
    LBL,
)

m_dop_g = Gauge("ublox_dop_geometric",  "NAV-DOP geometric DOP. Combined 3D position + time uncertainty multiplier due to satellite geometry alone (independent of signal quality). <2 excellent, <5 good, >10 poor.", LBL)
m_dop_p = Gauge("ublox_dop_position",   "NAV-DOP position DOP (3D). Position-only geometric multiplier. <2 good for survey, <5 acceptable.", LBL)
m_dop_t = Gauge(
    "ublox_dop_time",
    "NAV-DOP time DOP (tDOP). The geometric multiplier on time-fix uncertainty: "
    "tAcc roughly proportional to tDOP * range_error. For a timing receiver this is "
    "the most important DOP. <1.5 excellent, <2.5 good, >5 reception is degrading. "
    "Combined with C/N0 trends, tDOP rising means losing high-elevation sats.",
    LBL,
)
m_dop_v = Gauge("ublox_dop_vertical",   "NAV-DOP vertical DOP. Vertical-axis uncertainty multiplier (always worst since no sats below horizon).", LBL)
m_dop_h = Gauge("ublox_dop_horizontal", "NAV-DOP horizontal DOP. Horizontal-plane uncertainty multiplier.", LBL)
m_dop_n = Gauge("ublox_dop_northing",   "NAV-DOP northing DOP. North-component uncertainty multiplier.", LBL)
m_dop_e = Gauge("ublox_dop_easting",    "NAV-DOP easting DOP. East-component uncertainty multiplier.", LBL)


def field(msg, base, n=1):
    return getattr(msg, f"{base}_{n:02d}", None)


def poll(ser, ubr, want, window):
    ser.reset_input_buffer()
    for cls, ident in want:
        ser.write(UBXMessage(cls, ident, POLL).serialize())
    ser.flush()
    out = {}
    deadline = time.monotonic() + window
    while time.monotonic() < deadline and len(out) < len(want):
        try:
            _raw, parsed = ubr.read()
        except Exception:
            continue
        if parsed is None:
            time.sleep(0.01)
            continue
        if parsed.identity in {i for _, i in want}:
            out[parsed.identity] = parsed
    return out


def update_basic(port, ser, ubr, last_seen):
    want = [
        ("MON", "MON-RF"),
        ("NAV", "NAV-STATUS"),
        ("NAV", "NAV-SAT"),
        ("NAV", "NAV-PVT"),
        ("NAV", "NAV-CLOCK"),
        ("NAV", "NAV-DOP"),
        ("TIM", "TIM-TP"),
    ]
    got = poll(ser, ubr, want, window=8.0)
    now = time.monotonic()

    if "MON-RF" in got:
        rf = got["MON-RF"]
        m_ant.labels(port).set(field(rf, "antStatus"))
        m_magi.labels(port).set(field(rf, "magI"))
        m_magq.labels(port).set(field(rf, "magQ"))
        m_noise.labels(port).set(field(rf, "noisePerMS"))
        m_agc.labels(port).set(field(rf, "agcCnt"))
        m_jam.labels(port).set(field(rf, "jamInd"))
        m_jam_state.labels(port).set(field(rf, "jammingState"))
        last_seen["MON-RF"] = now

    if "NAV-STATUS" in got:
        m_fix.labels(port).set(got["NAV-STATUS"].gpsFix)
        m_spoof.labels(port).set(got["NAV-STATUS"].spoofDetState)
        last_seen["NAV-STATUS"] = now

    if "NAV-SAT" in got:
        sat = got["NAV-SAT"]
        cnos = [field(sat, "cno", i) or 0 for i in range(1, sat.numSvs + 1)]
        m_nsv.labels(port).set(len(cnos))
        for thr in (20, 30, 35, 40):
            m_cno_above.labels(port, str(thr)).set(sum(1 for c in cnos if c > thr))
        m_max_cno.labels(port).set(max(cnos) if cnos else 0)
        seen = seen_gnss.setdefault(port, set())
        per_gnss = {g: {"tracked": 0, "res": []} for g in seen}
        svs = set()
        for i in range(1, sat.numSvs + 1):
            gnss_id = field(sat, "gnssId", i)
            key = (GNSS_NAMES.get(gnss_id, str(gnss_id)), str(field(sat, "svId", i)))
            svs.add(key)
            for gauge, base in zip(SV_GAUGES, ("cno", "elev", "azim", "prRes", "qualityInd", "svUsed")):
                gauge.labels(port, *key).set(field(sat, base, i))
            sv = per_gnss.setdefault(gnss_id, {"tracked": 0, "res": []})
            sv["tracked"] += field(sat, "cno", i) > 0
            if field(sat, "svUsed", i):
                sv["res"].append(abs(field(sat, "prRes", i)))
        for gnss_id, sv in per_gnss.items():
            name = GNSS_NAMES.get(gnss_id, str(gnss_id))
            res = sv["res"]
            m_sat_tracked.labels(port, name).set(sv["tracked"])
            m_sat_used.labels(port, name).set(len(res))
            m_sat_prres_median.labels(port, name).set(statistics.median(res) if res else float("nan"))
            m_sat_prres_max.labels(port, name).set(max(res) if res else float("nan"))
        seen.update(per_gnss)
        for key in seen_sv.get(port, set()) - svs:
            for gauge in SV_GAUGES:
                gauge.remove(port, *key)
        seen_sv[port] = svs
        last_seen["NAV-SAT"] = now

    if "NAV-PVT" in got:
        pvt = got["NAV-PVT"]
        m_pvt_tacc.labels(port).set(pvt.tAcc)
        m_pvt_hacc.labels(port).set(pvt.hAcc)
        m_pvt_vacc.labels(port).set(pvt.vAcc)
        m_pvt_sats_used.labels(port).set(pvt.numSV)
        m_pvt_fix_ok.labels(port).set(getattr(pvt, "gnssFixOk", 0))
        m_pvt_lat.labels(port).set(pvt.lat)
        m_pvt_lon.labels(port).set(pvt.lon)
        m_pvt_hmsl.labels(port).set(pvt.hMSL)
        m_pvt_gspeed.labels(port).set(pvt.gSpeed)
        last_seen["NAV-PVT"] = now

    if "NAV-CLOCK" in got:
        clk = got["NAV-CLOCK"]
        m_clk_bias.labels(port).set(clk.clkB)
        m_clk_drift.labels(port).set(clk.clkD)
        m_clk_tacc.labels(port).set(clk.tAcc)
        m_clk_facc.labels(port).set(clk.fAcc)
        last_seen["NAV-CLOCK"] = now

    if "NAV-DOP" in got:
        dop = got["NAV-DOP"]
        m_dop_g.labels(port).set(dop.gDOP)
        m_dop_p.labels(port).set(dop.pDOP)
        m_dop_t.labels(port).set(dop.tDOP)
        m_dop_v.labels(port).set(dop.vDOP)
        m_dop_h.labels(port).set(dop.hDOP)
        m_dop_n.labels(port).set(dop.nDOP)
        m_dop_e.labels(port).set(dop.eDOP)
        last_seen["NAV-DOP"] = now

    if "TIM-TP" in got:
        tp = got["TIM-TP"]
        m_tp_qerr.labels(port).set(tp.qErr)
        # pyubx2 splits TIM-TP's flags byte into named sub-fields rather than
        # exposing the composite byte. Use named attrs; fall back to payload
        # byte 14 if the names differ across pyubx2 versions.
        timebase = getattr(tp, "timeBase", None)
        utc      = getattr(tp, "utc", None)
        raim     = getattr(tp, "raim", None)
        if timebase is None or utc is None or raim is None:
            flags = tp.payload[14] if len(tp.payload) > 14 else 0
            timebase = flags & 0x01
            utc      = (flags >> 1) & 0x01
            raim     = (flags >> 2) & 0x03
        m_tp_timebase.labels(port).set(int(timebase))
        m_tp_utc_valid.labels(port).set(int(utc))
        # raim is 2 bits: 0=not avail/disabled, 1=not active, 2=active.
        # Expose "unavailable" as boolean: raim == 0.
        m_tp_raim.labels(port).set(1 if int(raim) == 0 else 0)
        last_seen["TIM-TP"] = now

    for family, ts in last_seen.items():
        m_uptime.labels(port, family).set(now - ts)


def update_span(port, ser, ubr):
    got = poll(ser, ubr, [("MON", "MON-SPAN")], window=8.0)
    if "MON-SPAN" not in got:
        print("update_span: MON-SPAN not received in window", file=sys.stderr, flush=True)
        return
    msg = got["MON-SPAN"]
    for blk in range(1, msg.numRfBlocks + 1):
        center = getattr(msg, f"center_{blk:02d}", 0)
        span   = getattr(msg, f"span_{blk:02d}",   0)
        res    = getattr(msg, f"res_{blk:02d}",    0)
        pga    = getattr(msg, f"pga_{blk:02d}",    0)
        m_spec_ctr.labels(port, str(blk)).set(center / 1e6)
        m_spec_span.labels(port, str(blk)).set(span / 1e6)
        m_spec_pga.labels(port, str(blk)).set(pga)
        f_lo = center - span // 2
        # pyubx2 exposes the whole 256-bin spectrum as one iterable attribute
        # per block (spectrum_01 — list of ints), not 256 separate scalars.
        spectrum = getattr(msg, f"spectrum_{blk:02d}", ())
        for i, val in enumerate(spectrum):
            freq = (f_lo + i * res) / 1e6
            m_spec.labels(port, str(blk), str(i), f"{freq:.3f}").set(val)


def run_port(port, baud, basic_interval, span_interval):
    """Poll one serial port forever. Runs in its own thread; metrics are
    distinguished by the `port` label so multiple workers can publish to
    the same registry without collision."""
    last_seen = {"MON-RF": 0, "NAV-STATUS": 0, "NAV-SAT": 0,
                 "NAV-PVT": 0, "NAV-CLOCK": 0, "NAV-DOP": 0, "TIM-TP": 0}
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"# [{port}] open failed: {e}", file=sys.stderr, flush=True)
        return
    with ser:
        ubr = UBXReader(ser, protfilter=2, quitonerror=ERR_IGNORE)
        last_span = 0.0
        while True:
            try:
                update_basic(port, ser, ubr, last_seen)
                if time.monotonic() - last_span > span_interval:
                    update_span(port, ser, ubr)
                    last_span = time.monotonic()
                time.sleep(basic_interval)
            except KeyboardInterrupt:
                return
            except Exception as e:
                print(f"# [{port}] poll error: {e}", file=sys.stderr, flush=True)
                time.sleep(5)


def main():
    ap = argparse.ArgumentParser(
        description="u-blox GNSS Prometheus exporter (multi-port).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port",
                    help="serial device(s), comma-separated for multi-port "
                         "(e.g. /dev/ttyS5 or /dev/ttyS5,/dev/ttyS6)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="shared baud rate for all ports (default: 115200)")
    ap.add_argument("--listen", type=int, default=9021,
                    help="Prometheus HTTP listen port (default: 9021)")
    ap.add_argument("--basic-interval", type=float, default=15.0,
                    help="seconds between basic (non-SPAN) polls (default: 15)")
    ap.add_argument("--span-interval",  type=float, default=60.0,
                    help="seconds between MON-SPAN captures (default: 60)")
    args = ap.parse_args()

    ports = [p.strip() for p in args.port.split(",") if p.strip()]
    if not ports:
        print("# no serial ports given", file=sys.stderr, flush=True)
        sys.exit(1)

    start_http_server(args.listen)
    print(f"Exporter listening on :{args.listen}/metrics  (devices={ports})",
          file=sys.stderr, flush=True)

    threads = []
    for p in ports:
        t = threading.Thread(
            target=run_port,
            args=(p, args.baud, args.basic_interval, args.span_interval),
            name=f"poll-{p}",
            daemon=True,
        )
        t.start()
        threads.append(t)

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(1)
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
