# ubx_exporter

Prometheus exporter for u-blox UBX-protocol GNSS receivers. Polls receiver
health (MON-RF), satellite reception (NAV-SAT), and PPS timing quality
(NAV-PVT, TIM-TP, NAV-CLOCK, NAV-DOP) on a configurable interval; captures
the L1 spectrum (MON-SPAN) on a slower interval for waterfall plots in
Grafana. Built and tested against a u-blox NEO-M9N; should work on any UBX
receiver supporting those messages.

Two Grafana dashboards ship with it:

- **`ubx-dashboard-overview.json`** — on-call view, 8 stat tiles (7
  red/yellow/green, jam indicator plain) + 6 h fix-state strip + 30 min
  spectrogram. Green tiles = OK.
- **`ubx-dashboard-detailed.json`** — detail view for forensics, 16 panels
  covering every exposed metric with thresholds and descriptions.

## Requirements

- A u-blox UBX-capable GNSS receiver on a serial device (default baud
  115200; pass `--baud` if yours differs).
- Python 3.8+.
- Linux host with read/write access to the serial device.
- Prometheus to scrape `/metrics`, Grafana to render the dashboards.

## Install

On the host that will run the exporter (one-time):

```bash
# Ensure the user can access the serial device
sudo usermod -aG dialout $(whoami)

# Create a venv somewhere persistent
python3 -m venv /opt/ubx-exporter/env
source /opt/ubx-exporter/env/bin/activate
pip install pyubx2 pyserial prometheus_client

# Drop the script in
cp ubx_exporter.py /usr/local/bin/
chmod +x /usr/local/bin/ubx_exporter.py
```

> **pyserial gotcha:** a PyPI package literally called `serial` exists and
> shadows pyserial. If you see `AttributeError: module 'serial' has no
> attribute 'Serial'`, run `pip uninstall serial && pip install pyserial`.

## Run

### Foreground (smoke test)

```bash
/opt/ubx-exporter/env/bin/python3 /usr/local/bin/ubx_exporter.py /dev/ttyS5
```

Multiple devices (comma-separated):

```bash
/opt/ubx-exporter/env/bin/python3 /usr/local/bin/ubx_exporter.py /dev/ttyS5,/dev/ttyS6
```

| Flag | Default | What it does |
|---|---|---|
| `--listen` | `9021` | HTTP port for `/metrics` |
| `--baud` | `115200` | Shared baud rate for all ports |
| `--basic-interval` | `15` (s) | Cadence for MON-RF / NAV-* / TIM-TP polls |
| `--span-interval` | `60` (s) | Cadence for MON-SPAN spectrum captures |

Verify with `curl http://localhost:9021/metrics`.

> **gpsd will fight you for the serial device.** If gpsd is running and
> bound to your device, stop it (`systemctl stop gpsd gpsd.socket`) before
> starting the exporter, or use a different serial port.

### systemd

Minimal unit at `/etc/systemd/system/ubx-exporter.service`:

```ini
[Unit]
Description=u-blox UBX Prometheus exporter
After=network.target

[Service]
Type=simple
# Runs as root by default; add User= for a dedicated account
ExecStart=/opt/ubx-exporter/env/bin/python3 /usr/local/bin/ubx_exporter.py /dev/ttyS5
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Then `systemctl daemon-reload && systemctl enable --now ubx-exporter`.

## Prometheus scrape config

```yaml
scrape_configs:
  - job_name: ubx_exporter
    scrape_interval: 30s  # keep >= --basic-interval (default 15s)
    static_configs:
      - targets: ['ntp-host.example.com:9021']
```

## Grafana

Dashboards → New → Import → upload the JSON, pick your Prometheus
datasource when prompted. Import the overview dashboard first; that's the
one to keep on a wall display. Drill into the detail dashboard when a tile
goes red.

Each dashboard has `Instance` and `Port` template variables that
auto-populate from the `ublox_fix` series.

## Troubleshooting (the three you'll hit)

1. **All metrics flat / missing** — gpsd is holding the device, OR the
   service is running but the receiver isn't responding. Check
   `journalctl -u ubx-exporter` for `# poll error:` lines and
   `lsof /dev/ttyS5` for competing processes.
2. **TIM-TP / `ublox_pps_*` metrics missing** — your receiver doesn't have
   a PPS output configured. Run `ubxtool -p CFG-TP5 -f /dev/ttyS5 -s 115200`
   (or whatever baud your receiver is at) to inspect the PPS configuration.
3. **MON-SPAN spectrum bins all zero** — almost always means you're on an
   older pyubx2 with a different attribute layout. Upgrade pyubx2 first.

