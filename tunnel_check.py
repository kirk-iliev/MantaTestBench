#!/usr/bin/env python3
"""Read-only EPICS connectivity check — confirms the tunnel + PVs work before a
scan. Starts a PVMonitor from pv_config.json, waits, and prints a table of
connected / value / IOC-timestamp per PV. Writes nothing; safe to run anytime.

    .venv/bin/python tunnel_check.py [pv_config.json] [settle_seconds]

Exit code 0 if every PV connected, 1 otherwise.
"""

import sys
import time

from pv_monitor import PVMonitor, load_pv_config


def main(argv):
    cfg_path = argv[1] if len(argv) > 1 else "pv_config.json"
    wait_s = float(argv[2]) if len(argv) > 2 else 5.0

    pv_map, forwards = load_pv_config(cfg_path)
    if not pv_map:
        print(f"No PVs found in {cfg_path}")
        return 1

    mode = "tunnel" if forwards else "native"
    print(f"config: {cfg_path}   mode: {mode}")
    if forwards:
        for f in forwards:
            print(f"  forward -> {f['host']}:{f['port']}")
    print(f"monitoring {len(pv_map)} PV(s); waiting {wait_s:.0f}s for connections...\n")

    mon = PVMonitor(pv_map, tunnel_cfg=forwards)
    mon.start()
    try:
        time.sleep(wait_s)
        snap = mon.snapshot()
    finally:
        mon.stop()

    okc = 0
    print(f"{'LABEL':<18}{'PV':<26}{'CONN':<6}{'VALUE':<16}{'IOC TIMESTAMP'}")
    print("-" * 86)
    for label, pv in pv_map.items():
        rec = snap.get(label, {})
        conn = bool(rec.get("connected"))
        okc += conn
        val = rec.get("value")
        ts = rec.get("timestamp")
        val_s = "—" if val is None else (str(val)[:14])
        ts_s = "—" if ts is None else f"{ts:.3f}"
        print(f"{label:<18}{pv:<26}{('YES' if conn else 'no'):<6}{val_s:<16}{ts_s}")

    print("-" * 86)
    print(f"{okc}/{len(pv_map)} connected")
    return 0 if okc == len(pv_map) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
