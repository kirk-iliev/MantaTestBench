#!/usr/bin/env python3
"""Real implementations of the scan engine's injected interfaces:

  * MonitorWriterIO — EpicsIO backed by a PVMonitor (reads, from its cache)
    plus a PVWriter (writes). Construct the PVMonitor with label == pv name so
    the cache is keyed by PV.
  * ScanFrameSaver — FrameSaver writing a 16-bit TIFF (cv2) plus a text sidecar
    per frame into a run directory.
"""

import time
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

    def get_raw(self, pv):
        """Raw value without float coercion: scalars as float, waveforms as a
        plain list (JSON-serializable), None if unset/disconnected."""
        rec = self._monitor.snapshot().get(pv)
        if rec is None or not rec.get("connected"):
            return None
        v = rec.get("value")
        if v is None:
            return None
        if hasattr(v, "tolist"):          # numpy array (waveform)
            return v.tolist()
        if isinstance(v, (list, tuple)):
            return list(v)
        try:
            return float(v)
        except (TypeError, ValueError):
            return v

    def get_timestamp(self, pv):
        rec = self._monitor.snapshot().get(pv)
        if rec is None or not rec.get("connected"):
            return None
        return rec.get("timestamp")

    def wait_connected(self, pvs, timeout, poll=0.2):
        """Poll connected() until all pvs are connected or timeout elapses.
        Returns True if all connected within timeout, else False."""
        deadline = time.monotonic() + timeout
        while True:
            if self.connected(pvs):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll)

    def close(self):
        self._writer.close()


class ScanFrameSaver:
    def __init__(self, run_dir):
        self._run_dir = Path(run_dir)

    def save(self, frame, indices, setpoints, rbvs, timestamps,
             beam=None, decoded=None):
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
        for key, val in timestamps.items():
            lines.append(f"{key}_timestamp: {val}")
        if decoded:
            for key, val in decoded.items():
                lines.append(f"timinjreq_{key}: {val}")
        if beam:
            for pv, rec in beam.items():
                lines.append(f"beam[{pv}]: {rec.get('value')} (ts={rec.get('timestamp')})")
        lines.append(f"indices: {i},{j},{k}")
        (self._run_dir / f"{stem}.txt").write_text("\n".join(lines) + "\n")
        return tiff.name
