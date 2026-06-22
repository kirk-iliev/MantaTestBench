#!/usr/bin/env python3
"""Tests for scan_engine.run_scan with fake EPICS/frames/saver — no hardware.
Verifies raster order, limit refusal, settle/trigger fail-fast + restore,
mid-scan abort + restore, frames_per_point, and manifest output."""

import csv
import tempfile
import threading
from pathlib import Path

import numpy as np

from scan_config import AxisConfig, ScanConfig
from scan_engine import run_scan, EpicsIO


class FakeIO:
    """Setpoint writes update an internal map; reads return the setpoint as the
    readback (instant settle) unless settle_fail is set. ``disconnected`` makes
    connected() False."""
    def __init__(self, settle_fail=False, disconnected=False):
        self.values = {}
        self.put_log = []
        self.settle_fail = settle_fail
        self.disconnected = disconnected
    def put(self, pv, value):
        self.put_log.append((pv, value))
        self.values[pv] = value
    def get(self, pv):
        if pv.endswith("RBV"):
            sp = self.values.get(pv.replace("RBV", "SP"))
            if self.settle_fail:
                return None
            return sp
        return self.values.get(pv)
    def connected(self, pvs):
        return not self.disconnected
    def get_timestamp(self, pv):
        return 1234.5 if self.get(pv) is not None else None


class FakeFrames:
    def __init__(self, fail_after=None):
        self.n = 0
        self.fail_after = fail_after
    def next_triggered_frame(self, timeout):
        if self.fail_after is not None and self.n >= self.fail_after:
            raise TimeoutError("no beam")
        self.n += 1
        return np.full((2, 2), self.n, dtype=np.uint16)


class FakeSaver:
    def __init__(self):
        self.saves = []
    def save(self, frame, indices, setpoints, rbvs, timestamps):
        self.saves.append((indices, dict(setpoints), dict(rbvs)))
        return f"img_{indices[0]}_{indices[1]}_{indices[2]}.tiff"


def _axis(sp, rbv, **kw):
    base = dict(setpoint_pv=sp, rbv_pv=rbv, min=0.0, max=1.0, points=2,
                limit_min=-100.0, limit_max=100.0, settle_tol=0.001)
    base.update(kw)
    return AxisConfig(**base)


def _cfg(**kw):
    base = dict(settle_timeout_s=0.2, settle_poll_s=0.01, frames_per_point=1,
                trigger_timeout_s=1.0, restore_on_finish=True, output_dir="scans")
    base.update(kw)
    return ScanConfig(q1=_axis("Q1:SP", "Q1:RBV", min=0.0, max=1.0, points=2),
                      q2=_axis("Q2:SP", "Q2:RBV", min=10.0, max=11.0, points=2),
                      **base)


def test_completes_in_raster_order():
    io, frames, saver = FakeIO(), FakeFrames(), FakeSaver()
    # pre-set so restore has a baseline
    io.values["Q1:SP"] = -0.5; io.values["Q2:SP"] = -0.5
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(), io, frames, saver, tmp)
    assert res["status"] == "completed", res
    visited = [s[0] for s in saver.saves]
    assert visited == [(0,0,0),(0,1,0),(1,0,0),(1,1,0)], visited
    sps = [(s[1]["q1"], s[1]["q2"]) for s in saver.saves]
    assert sps == [(0.0,10.0),(0.0,11.0),(1.0,10.0),(1.0,11.0)], sps
    # restore drove setpoints back to pre-scan -0.5
    assert io.values["Q1:SP"] == -0.5 and io.values["Q2:SP"] == -0.5
    print("ok  test_completes_in_raster_order")


def test_preflight_refuses_when_disconnected():
    io = FakeIO(disconnected=True)
    with tempfile.TemporaryDirectory() as tmp:
        raised = False
        try:
            run_scan(_cfg(), io, FakeFrames(), FakeSaver(), tmp)
        except Exception:
            raised = True
    assert raised, "expected preflight failure when PVs disconnected"
    print("ok  test_preflight_refuses_when_disconnected")


def test_settle_timeout_fails_and_restores():
    io = FakeIO(settle_fail=True)
    io.values["Q1:SP"] = 0.3; io.values["Q2:SP"] = 0.3
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(), io, FakeFrames(), FakeSaver(), tmp)
    assert res["status"] == "failed", res
    assert "settle" in (res["failure"] or "").lower(), res["failure"]
    assert io.values["Q1:SP"] == 0.3 and io.values["Q2:SP"] == 0.3  # restored
    print("ok  test_settle_timeout_fails_and_restores")


def test_trigger_timeout_fails_and_restores():
    io = FakeIO()
    io.values["Q1:SP"] = 0.0; io.values["Q2:SP"] = 0.0
    frames = FakeFrames(fail_after=0)   # first capture raises
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(), io, frames, FakeSaver(), tmp)
    assert res["status"] == "failed", res
    assert io.values["Q1:SP"] == 0.0 and io.values["Q2:SP"] == 0.0  # restored
    print("ok  test_trigger_timeout_fails_and_restores")


def test_abort_midscan_restores_and_writes_manifest():
    io = FakeIO()
    io.values["Q1:SP"] = 9.0; io.values["Q2:SP"] = 9.0
    ev = threading.Event()
    saver = FakeSaver()
    def progress(i, j):
        ev.set()   # abort after the first completed point
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(), io, FakeFrames(), saver, tmp, abort_event=ev,
                       progress_cb=progress)
        manifest = Path(tmp) / "manifest.csv"
        assert manifest.exists()
        rows = list(csv.DictReader(manifest.open()))
    assert res["status"] == "aborted", res
    assert len(saver.saves) == 1, saver.saves         # only first point done
    assert len(rows) == 1
    assert io.values["Q1:SP"] == 9.0 and io.values["Q2:SP"] == 9.0  # restored
    assert "q1_ioc_timestamp" in rows[0] and rows[0]["q1_ioc_timestamp"] == "1234.5", rows[0]
    assert "q2_ioc_timestamp" in rows[0] and rows[0]["q2_ioc_timestamp"] == "1234.5", rows[0]
    print("ok  test_abort_midscan_restores_and_writes_manifest")


def test_frames_per_point():
    io = FakeIO(); io.values["Q1:SP"] = 0.0; io.values["Q2:SP"] = 0.0
    saver = FakeSaver()
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(frames_per_point=3), io, FakeFrames(), saver, tmp)
    assert res["status"] == "completed"
    assert len(saver.saves) == 4 * 3, len(saver.saves)   # 2x2 grid x 3 shots
    shots = sorted(s[0] for s in saver.saves if s[0][0] == 0 and s[0][1] == 0)
    assert shots == [(0,0,0),(0,0,1),(0,0,2)], shots
    print("ok  test_frames_per_point")


def test_validate_refuses_range_outside_limits():
    # Scan range exceeds the axis limits -> run_scan raises pre-loop (no writes).
    io = FakeIO(); io.values["Q1:SP"] = 0.0; io.values["Q2:SP"] = 0.0
    cfg = _cfg(); cfg.q1.limit_max = 0.5   # q1 range is [0,1], now exceeds limit 0.5
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        raised = False
        try:
            run_scan(cfg, io, FakeFrames(), FakeSaver(), tmp)
        except ValueError:
            raised = True
    assert raised, "expected ValueError for scan range outside limits"
    assert io.put_log == [], "no setpoint writes should occur on a refused scan"
    print("ok  test_validate_refuses_range_outside_limits")


def test_set_axis_raises_limit_error_directly():
    from scan_engine import _set_axis, LimitError
    ax = AxisConfig(setpoint_pv="Q:SP", rbv_pv="Q:RBV", min=0.0, max=1.0, points=2,
                    limit_min=-1.0, limit_max=1.0, settle_tol=0.01)
    io = FakeIO()
    raised = False
    try:
        _set_axis(io, ax, 5.0)   # 5.0 outside [-1,1]
    except LimitError:
        raised = True
    assert raised, "expected LimitError for out-of-limit write"
    assert io.put_log == [], "no put on a rejected limit"
    print("ok  test_set_axis_raises_limit_error_directly")


def test_fault_restores_even_when_restore_on_finish_false():
    io = FakeIO(settle_fail=True)      # forces a settle timeout fault
    io.values["Q1:SP"] = 0.3; io.values["Q2:SP"] = 0.3
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        res = run_scan(_cfg(restore_on_finish=False), io, FakeFrames(), FakeSaver(), tmp)
    assert res["status"] == "failed", res
    assert io.values["Q1:SP"] == 0.3 and io.values["Q2:SP"] == 0.3, "fault must restore even with restore_on_finish=False"
    print("ok  test_fault_restores_even_when_restore_on_finish_false")


if __name__ == "__main__":
    test_completes_in_raster_order()
    test_preflight_refuses_when_disconnected()
    test_settle_timeout_fails_and_restores()
    test_trigger_timeout_fails_and_restores()
    test_abort_midscan_restores_and_writes_manifest()
    test_frames_per_point()
    test_validate_refuses_range_outside_limits()
    test_set_axis_raises_limit_error_directly()
    test_fault_restores_even_when_restore_on_finish_false()
    print("\nall passed")
