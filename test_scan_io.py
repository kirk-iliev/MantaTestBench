#!/usr/bin/env python3
"""Tests for scan_io.ScanFrameSaver (TIFF + sidecar to a temp dir) and
MonitorWriterIO.get/connected against a fake monitor. No camera, no IOC."""

import tempfile
from pathlib import Path

import numpy as np

from scan_io import ScanFrameSaver, MonitorWriterIO


class FakeMonitor:
    def __init__(self, snap):
        self._snap = snap
    def snapshot(self):
        return self._snap


class FakeWriter:
    def __init__(self):
        self.puts = []
    def put(self, pv, value, timeout=5.0):
        self.puts.append((pv, value))


def test_saver_writes_tiff_and_sidecar():
    with tempfile.TemporaryDirectory() as tmp:
        saver = ScanFrameSaver(tmp)
        frame = np.arange(4, dtype=np.uint16).reshape(2, 2)
        fname = saver.save(frame, (1, 2, 0),
                           {"q1": 0.5, "q2": 10.0},
                           {"q1": 0.49, "q2": 9.98}, {"wall": 123.0})
        tiff = Path(tmp) / fname
        sidecar = tiff.with_suffix(".txt")
        assert tiff.exists() and "q1_1_q2_2_shot_0" in fname, fname
        assert sidecar.exists()
        text = sidecar.read_text()
        assert "q1_setpoint: 0.5" in text and "q2_rbv: 9.98" in text, text
    print("ok  test_saver_writes_tiff_and_sidecar")


def test_io_get_and_connected():
    snap = {"A:SP": {"value": 1.0, "connected": True},
            "A:RBV": {"value": 0.99, "connected": True},
            "B:SP": {"value": 2.0, "connected": False}}
    io = MonitorWriterIO(FakeMonitor(snap), FakeWriter())
    assert io.get("A:RBV") == 0.99
    assert io.get("B:SP") is None          # disconnected -> None
    assert io.connected(["A:SP", "A:RBV"]) is True
    assert io.connected(["A:SP", "B:SP"]) is False
    io.put("A:SP", 1.5)
    assert io._writer.puts == [("A:SP", 1.5)]
    print("ok  test_io_get_and_connected")


if __name__ == "__main__":
    test_saver_writes_tiff_and_sidecar()
    test_io_get_and_connected()
    print("\nall passed")
