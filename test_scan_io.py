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
        self.closed = False
    def put(self, pv, value, timeout=5.0):
        self.puts.append((pv, value))
    def close(self):
        self.closed = True


def test_saver_writes_tiff_and_sidecar():
    with tempfile.TemporaryDirectory() as tmp:
        saver = ScanFrameSaver(tmp)
        frame = np.arange(4, dtype=np.uint16).reshape(2, 2)
        fname = saver.save(frame, (1, 2, 0),
                           {"q1": 0.5, "q2": 10.0},
                           {"q1": 0.49, "q2": 9.98},
                           {"wall": 123.0, "q1_ioc": 555.0, "q2_ioc": 666.0})
        tiff = Path(tmp) / fname
        sidecar = tiff.with_suffix(".txt")
        assert tiff.exists() and "q1_1_q2_2_shot_0" in fname, fname
        assert sidecar.exists()
        text = sidecar.read_text()
        assert "q1_setpoint: 0.5" in text and "q2_rbv: 9.98" in text, text
        assert "wall_timestamp: 123.0" in text, text
        assert "q1_ioc_timestamp: 555.0" in text, text
        assert "q2_ioc_timestamp: 666.0" in text, text
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


def test_io_close_closes_writer():
    w = FakeWriter()
    io = MonitorWriterIO(FakeMonitor({}), w)
    io.close()
    assert w.closed is True
    print("ok  test_io_close_closes_writer")


def test_io_get_timestamp():
    snap = {"A:RBV": {"value": 0.99, "timestamp": 555.0, "connected": True},
            "B:RBV": {"value": 1.0, "timestamp": 666.0, "connected": False}}
    io = MonitorWriterIO(FakeMonitor(snap), FakeWriter())
    assert io.get_timestamp("A:RBV") == 555.0
    assert io.get_timestamp("B:RBV") is None   # disconnected -> None
    assert io.get_timestamp("missing") is None
    print("ok  test_io_get_timestamp")


def test_wait_connected_polls_until_ready():
    # Monitor reports disconnected for the first 2 snapshots, then connected.
    class FlipMonitor:
        def __init__(self):
            self.n = 0
        def snapshot(self):
            self.n += 1
            conn = self.n >= 3
            return {"A:SP": {"value": 1.0, "connected": conn}}
    io = MonitorWriterIO(FlipMonitor(), FakeWriter())
    assert io.wait_connected(["A:SP"], timeout=2.0, poll=0.01) is True
    # Never-connects monitor times out and returns False quickly.
    io2 = MonitorWriterIO(FakeMonitor({"A:SP": {"value": 1.0, "connected": False}}), FakeWriter())
    assert io2.wait_connected(["A:SP"], timeout=0.1, poll=0.01) is False
    print("ok  test_wait_connected_polls_until_ready")


if __name__ == "__main__":
    test_saver_writes_tiff_and_sidecar()
    test_io_get_and_connected()
    test_io_close_closes_writer()
    test_io_get_timestamp()
    test_wait_connected_polls_until_ready()
    print("\nall passed")
