#!/usr/bin/env python3
"""Scan configuration: dataclasses, JSON loading, raster grid generation, and
validation. Pure logic — no hardware, no I/O beyond reading the JSON file."""

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class AxisConfig:
    setpoint_pv: str
    rbv_pv: str
    min: float
    max: float
    points: int
    limit_min: float
    limit_max: float
    settle_tol: float


@dataclass
class ScanConfig:
    q1: AxisConfig
    q2: AxisConfig
    settle_timeout_s: float
    settle_poll_s: float
    frames_per_point: int
    trigger_timeout_s: float
    restore_on_finish: bool
    output_dir: str
    # Extra PVs snapshotted (best-effort) at every frame for beam provenance,
    # e.g. ["TimInjReq", "EG______BIAS___AM01"]. A "TimInjReq" waveform is also
    # decoded into named columns (bucket/bunches/mode/inhibit/seq). Never gates
    # the scan: an unreadable meta PV is logged as null, not a fault.
    beam_meta_pvs: list = field(default_factory=list)


def _axis_from(d):
    return AxisConfig(
        setpoint_pv=str(d["setpoint_pv"]), rbv_pv=str(d["rbv_pv"]),
        min=float(d["min"]), max=float(d["max"]), points=int(d["points"]),
        limit_min=float(d["limit_min"]), limit_max=float(d["limit_max"]),
        settle_tol=float(d["settle_tol"]))


def load_scan_config(path) -> ScanConfig:
    doc = json.loads(Path(path).read_text())
    axes = doc["axes"]
    settle = doc.get("settle", {})
    return ScanConfig(
        q1=_axis_from(axes["Q1"]), q2=_axis_from(axes["Q2"]),
        settle_timeout_s=float(settle.get("timeout_s", 10.0)),
        settle_poll_s=float(settle.get("poll_s", 0.1)),
        frames_per_point=int(doc.get("frames_per_point", 1)),
        trigger_timeout_s=float(doc.get("trigger_timeout_s", 30.0)),
        restore_on_finish=bool(doc.get("restore_on_finish", True)),
        output_dir=str(doc.get("output_dir", "scans")),
        beam_meta_pvs=[str(p) for p in doc.get("beam_meta_pvs", [])])


def axis_points(axis: AxisConfig) -> list:
    if axis.points == 1:
        return [axis.min]
    step = (axis.max - axis.min) / (axis.points - 1)
    return [axis.min + i * step for i in range(axis.points)]


def generate_grid(cfg: ScanConfig) -> list:
    q1pts = axis_points(cfg.q1)
    q2pts = axis_points(cfg.q2)
    grid = []
    for i, q1 in enumerate(q1pts):          # outer
        for j, q2 in enumerate(q2pts):      # inner, low->high, reset each i
            grid.append((i, j, q1, q2))
    return grid


def _validate_axis(name, ax: AxisConfig):
    if ax.points < 1:
        raise ValueError(f"{name}: points must be >= 1 (got {ax.points})")
    if ax.min > ax.max:
        raise ValueError(f"{name}: min {ax.min} > max {ax.max}")
    if ax.limit_min > ax.limit_max:
        raise ValueError(f"{name}: limit_min > limit_max")
    if ax.min < ax.limit_min or ax.max > ax.limit_max:
        raise ValueError(
            f"{name}: scan range [{ax.min}, {ax.max}] outside limits "
            f"[{ax.limit_min}, {ax.limit_max}]")
    if ax.settle_tol <= 0:
        raise ValueError(f"{name}: settle_tol must be > 0")


def validate_config(cfg: ScanConfig) -> None:
    _validate_axis("Q1", cfg.q1)
    _validate_axis("Q2", cfg.q2)
    if cfg.frames_per_point < 1:
        raise ValueError("frames_per_point must be >= 1")
    if cfg.settle_timeout_s <= 0 or cfg.settle_poll_s <= 0:
        raise ValueError("settle timeouts must be > 0")
    if cfg.trigger_timeout_s <= 0:
        raise ValueError("trigger_timeout_s must be > 0")
