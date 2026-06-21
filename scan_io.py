#!/usr/bin/env python3
"""Real implementations of the scan engine's injected interfaces:

  * MonitorWriterIO — EpicsIO backed by a PVMonitor (reads, from its cache)
    plus a PVWriter (writes). Construct the PVMonitor with label == pv name so
    the cache is keyed by PV.
  * ScanFrameSaver — FrameSaver writing a 16-bit TIFF (cv2) plus a text sidecar
    per frame into a run directory.
"""

from pathlib import Path

import cv2


class MonitorWriterIO:
    def __init__(self, monitor, writer):
        self._monitor = monitor
        self._writer = writer

    def put(self, pv, value):
        self._writer.put(pv, value)        # raises WriteError on failure

    def get(self, pv):
        rec = self._monitor.snapshot().get(pv)
        if rec is None or not rec.get("connected"):
            return None
        v = rec.get("value")
        return None if v is None else float(v)

    def connected(self, pvs):
        snap = self._monitor.snapshot()
        return all(snap.get(pv, {}).get("connected") for pv in pvs)


class ScanFrameSaver:
    def __init__(self, run_dir):
        self._run_dir = Path(run_dir)

    def save(self, frame, indices, setpoints, rbvs, timestamps):
        i, j, k = indices
        stem = f"q1_{i}_q2_{j}_shot_{k}"
        tiff = self._run_dir / f"{stem}.tiff"
        if not cv2.imwrite(str(tiff), frame):  # uint16 -> 16-bit TIFF
            raise RuntimeError(f"cv2.imwrite failed for {tiff}")
        lines = []
        for key, val in setpoints.items():
            lines.append(f"{key}_setpoint: {val}")
        for key, val in rbvs.items():
            lines.append(f"{key}_rbv: {val}")
        lines.append(f"wall_timestamp: {timestamps['wall']}")
        lines.append(f"indices: {i},{j},{k}")
        (self._run_dir / f"{stem}.txt").write_text("\n".join(lines) + "\n")
        return tiff.name
