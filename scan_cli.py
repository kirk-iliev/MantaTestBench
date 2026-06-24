#!/usr/bin/env python3
"""Headless 2D quad scan driver.

Usage:
  python scan_cli.py --config scan.json [--pv-config pv_config.json] [--dry-run]

--dry-run validates the scan config and prints the grid without touching
hardware. A real run wires the camera (vmbpy, hardware/Line1 trigger) and EPICS
(forwards from pv_config.json), then runs the engine. Ctrl-C aborts cleanly
(quads are restored).
"""

import argparse
import queue
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from scan_config import load_scan_config, validate_config, generate_grid
from scan_engine import run_scan, ScanAborted
from scan_io import MonitorWriterIO, ScanFrameSaver
from epics_control import PVWriter
from pv_monitor import PVMonitor, load_pv_config


class _CameraFrameSource:
    """FrameSource backed by a vmbpy hardware-triggered stream. Each completed
    frame is pushed to a queue; next_triggered_frame pops the next one."""
    def __init__(self, cam, abort_event=None):
        self._cam = cam
        self._q = queue.Queue(maxsize=4)
        self._abort = abort_event

    def _handler(self, cam, stream, frame):
        import vmbpy
        if frame.get_status() == vmbpy.FrameStatus.Complete:
            try:
                self._q.put_nowait(frame.as_numpy_ndarray().copy())
            except queue.Full:
                pass
        cam.queue_frame(frame)

    def start(self):
        import vmbpy
        cam = self._cam
        cam.TriggerSelector.set("FrameStart")
        cam.TriggerMode.set("On")
        cam.TriggerSource.set("Line1")
        cam.TriggerActivation.set("RisingEdge")
        # 12-bit for quantitative profiles (full 0–4095 range); comes back as
        # uint16 and is saved as a 16-bit TIFF. Match the GUI; fall back to the
        # camera default if Mono12 is rejected.
        try:
            cam.set_pixel_format(vmbpy.PixelFormat.Mono12)
        except Exception as exc:
            print(f"  warning: could not set Mono12 (using default): {exc}")
        cam.start_streaming(self._handler)

    def stop(self):
        try:
            self._cam.stop_streaming()
        except Exception:
            pass

    def next_triggered_frame(self, timeout):
        # Discard frames captured while the magnets were ramping/settling so we
        # return a genuinely post-settle shot, not a stale queued one. Mirrors
        # the GUI _SignalFrameSource, which drops its latched frame before
        # waiting. Without this drain, a shot triggered during the move sits in
        # the queue and gets mis-attributed to the new setpoint.
        while True:
            try:
                self._q.get_nowait()
            except queue.Empty:
                break
        # Poll in short slices so a Ctrl-C (which sets the abort event) takes
        # effect promptly instead of only after the full trigger timeout.
        deadline = time.monotonic() + timeout
        while True:
            if self._abort is not None and self._abort.is_set():
                raise ScanAborted("aborted by user")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no triggered frame within {timeout}s")
            try:
                return self._q.get(timeout=min(remaining, 0.2))
            except queue.Empty:
                continue


def _make_epics_io(cfg, pv_config_path):
    _pv_map, forwards = load_pv_config(pv_config_path)
    pvs = [cfg.q1.setpoint_pv, cfg.q1.rbv_pv, cfg.q2.setpoint_pv, cfg.q2.rbv_pv]
    # Beam-metadata PVs are monitored too (so they're cached for per-frame
    # snapshots) but are NOT part of the connectivity gate — they're best-effort.
    mon_pvs = pvs + [p for p in cfg.beam_meta_pvs if p not in pvs]
    monitor = PVMonitor({pv: pv for pv in mon_pvs}, tunnel_cfg=forwards)
    monitor.start()
    writer = PVWriter(forwards=forwards)
    return MonitorWriterIO(monitor, writer), monitor, writer


def main(argv=None):
    ap = argparse.ArgumentParser(description="Automated 2D quad scan")
    ap.add_argument("--config", required=True, help="scan.json")
    ap.add_argument("--pv-config", default="pv_config.json",
                    help="EPICS connection config (forwards/native)")
    ap.add_argument("--dry-run", action="store_true",
                    help="validate + print the grid, no hardware")
    args = ap.parse_args(argv)

    cfg = load_scan_config(args.config)
    validate_config(cfg)
    grid = generate_grid(cfg)

    if args.dry_run:
        print(f"Scan grid: {len(grid)} points "
              f"({cfg.q1.points} x {cfg.q2.points}), {cfg.frames_per_point} frame(s)/point")
        for (i, j, q1, q2) in grid:
            print(f"  ({i},{j})  Q1={q1:.4g}  Q2={q2:.4g}")
        return 0

    import vmbpy
    run_dir = Path(cfg.output_dir) / ("scan_" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=False)

    abort = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: abort.set())

    io, monitor, writer = _make_epics_io(cfg, args.pv_config)
    try:
        pvs = [cfg.q1.setpoint_pv, cfg.q1.rbv_pv, cfg.q2.setpoint_pv, cfg.q2.rbv_pv]
        if not io.wait_connected(pvs, 10.0):
            print("Scan aborted: required PVs did not connect within 10s "
                  "(SSH tunnels up? IOC reachable?)")
            return 1
        with vmbpy.VmbSystem.get_instance() as vmb:
            cam = vmb.get_all_cameras()[0]
            with cam:
                src = _CameraFrameSource(cam, abort_event=abort)
                src.start()
                try:
                    saver = ScanFrameSaver(str(run_dir))
                    res = run_scan(cfg, io, src, saver, str(run_dir),
                                   abort_event=abort,
                                   progress_cb=lambda i, j: print(f"  point ({i},{j}) done"))
                finally:
                    src.stop()
                print(f"Scan {res['status']}: {res['frames']} frames -> {run_dir}")
                if res["failure"]:
                    print(f"  reason: {res['failure']} at point {res['failure_point']}")
                return 0 if res["status"] == "completed" else 1
    finally:
        monitor.stop()
        writer.close()


if __name__ == "__main__":
    sys.exit(main())
