#!/usr/bin/env python3
"""Tests for scan_config: axis point generation, raster grid order, validation,
and JSON loading. Pure logic, no hardware."""

import json
import tempfile
from pathlib import Path

from scan_config import (AxisConfig, ScanConfig, axis_points, generate_grid,
                         validate_config, load_scan_config)


def _axis(**kw):
    base = dict(setpoint_pv="SP", rbv_pv="RBV", min=0.0, max=2.0, points=3,
                limit_min=-5.0, limit_max=5.0, settle_tol=0.02)
    base.update(kw)
    return AxisConfig(**base)


def _cfg(q1=None, q2=None, **kw):
    base = dict(settle_timeout_s=10.0, settle_poll_s=0.1, frames_per_point=1,
                trigger_timeout_s=30.0, restore_on_finish=True, output_dir="scans")
    base.update(kw)
    return ScanConfig(q1=q1 or _axis(), q2=q2 or _axis(), **base)


def test_axis_points():
    assert axis_points(_axis(min=0.0, max=2.0, points=3)) == [0.0, 1.0, 2.0]
    assert axis_points(_axis(min=5.0, max=5.0, points=1)) == [5.0]  # single point
    print("ok  test_axis_points")


def test_raster_grid_order():
    cfg = _cfg(q1=_axis(min=0.0, max=1.0, points=2),
               q2=_axis(min=10.0, max=12.0, points=3))
    grid = generate_grid(cfg)
    # outer Q1, inner Q2 low->high, Q2 reset each Q1 step
    assert grid == [
        (0, 0, 0.0, 10.0), (0, 1, 0.0, 11.0), (0, 2, 0.0, 12.0),
        (1, 0, 1.0, 10.0), (1, 1, 1.0, 11.0), (1, 2, 1.0, 12.0),
    ], grid
    print("ok  test_raster_grid_order")


def test_validate_rejects_range_outside_limits():
    bad = _cfg(q1=_axis(min=-10.0, max=2.0, limit_min=-5.0, limit_max=5.0))
    try:
        validate_config(bad)
        raised = False
    except ValueError:
        raised = True
    assert raised, "expected ValueError for range below limit_min"
    print("ok  test_validate_rejects_range_outside_limits")


def test_validate_rejects_bad_points_and_frames():
    for bad in (_cfg(q1=_axis(points=0)), _cfg(frames_per_point=0)):
        try:
            validate_config(bad)
            ok = False
        except ValueError:
            ok = True
        assert ok
    print("ok  test_validate_rejects_bad_points_and_frames")


def test_load_scan_config():
    doc = {
        "axes": {
            "Q1": {"setpoint_pv": "A:SP", "rbv_pv": "A:RBV", "min": 0.0, "max": 2.0,
                   "points": 3, "limit_min": -5.0, "limit_max": 5.0, "settle_tol": 0.02},
            "Q2": {"setpoint_pv": "B:SP", "rbv_pv": "B:RBV", "min": -1.0, "max": 1.0,
                   "points": 5, "limit_min": -5.0, "limit_max": 5.0, "settle_tol": 0.05},
        },
        "settle": {"timeout_s": 8.0, "poll_s": 0.2},
        "frames_per_point": 2, "trigger_timeout_s": 20.0,
        "restore_on_finish": True, "output_dir": "scans",
    }
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "scan.json"
        p.write_text(json.dumps(doc))
        cfg = load_scan_config(p)
    assert cfg.q1.setpoint_pv == "A:SP" and cfg.q2.points == 5
    assert cfg.settle_timeout_s == 8.0 and cfg.frames_per_point == 2
    validate_config(cfg)  # the loaded config is valid
    print("ok  test_load_scan_config")


if __name__ == "__main__":
    test_axis_points()
    test_raster_grid_order()
    test_validate_rejects_range_outside_limits()
    test_validate_rejects_bad_points_and_frames()
    test_load_scan_config()
    print("\nall passed")
