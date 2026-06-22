#!/usr/bin/env python3
"""Headless 2D quad scan engine. Pure logic — depends only on injected
EpicsIO / FrameSource / FrameSaver, so the whole set->settle->acquire->save->
restore loop is unit-testable with fakes (no camera, no IOC).

Safety: every setpoint write is limit-checked; the scan refuses to start if any
grid point is out of limits or a required PV is disconnected; any fault stops
the scan and restores both quads to their pre-scan setpoints (fail-fast).
"""

import csv
import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Protocol

from scan_config import ScanConfig, validate_config, generate_grid


class EpicsIO(Protocol):
    def put(self, pv: str, value: float) -> None: ...
    def get(self, pv: str): ...                       # -> float | None
    def get_raw(self, pv: str): ...                   # -> scalar|list|None (no float coercion; for waveforms)
    def connected(self, pvs) -> bool: ...
    def get_timestamp(self, pv: str): ...             # -> float | None (IOC source timestamp)


class FrameSource(Protocol):
    def next_triggered_frame(self, timeout: float): ...   # -> ndarray; raises on timeout


class FrameSaver(Protocol):
    def save(self, frame, indices, setpoints, rbvs, timestamps,
             beam=None, decoded=None) -> str: ...


# TimInjReq is a 7-element injection-request waveform (ALS dual-EVG timing
# system). Index map (0-based here; the MML help is 1-based):
#   0 target bucket (1-328)   1 gun bunches (1-16)   2 injection mode
#   3 gun inhibit (0 fires/1 disabled)   4,5 field-sync delays   6 sequence number
def _decode_timinjreq(value):
    """Decode a TimInjReq waveform into named fields, or {} if not decodable."""
    if value is None:
        return {}
    try:
        seq = list(value)
    except TypeError:
        return {}
    if len(seq) < 7:
        return {}
    return {"target_bucket": seq[0], "gun_bunches": seq[1], "inj_mode": seq[2],
            "gun_inhibit": seq[3], "inj_seq": seq[6]}


def _snapshot_beam(io, pvs):
    """Best-effort {pv: {value, timestamp}} snapshot; never raises."""
    out = {}
    for pv in pvs:
        try:
            val = io.get_raw(pv)
        except Exception:
            val = None
        try:
            ts = io.get_timestamp(pv)
        except Exception:
            ts = None
        out[pv] = {"value": val, "timestamp": ts}
    return out


class ScanError(Exception): ...
class SettleTimeout(ScanError): ...
class ScanAborted(ScanError): ...
class PreflightError(ScanError): ...
class LimitError(ScanError): ...


def _check_abort(ev):
    if ev is not None and ev.is_set():
        raise ScanAborted("aborted by user")


def _set_axis(io, axis, value):
    if not (axis.limit_min <= value <= axis.limit_max):
        raise LimitError(
            f"{axis.setpoint_pv}={value} outside limits "
            f"[{axis.limit_min}, {axis.limit_max}]")
    io.put(axis.setpoint_pv, value)


def _settle(io, axis, setpoint, cfg, abort_event):
    deadline = time.monotonic() + cfg.settle_timeout_s
    while True:
        _check_abort(abort_event)
        rbv = io.get(axis.rbv_pv)
        if rbv is not None and abs(rbv - setpoint) <= axis.settle_tol:
            return
        if time.monotonic() > deadline:
            raise SettleTimeout(
                f"{axis.rbv_pv} did not settle to {setpoint} within "
                f"{cfg.settle_timeout_s}s (last={rbv})")
        time.sleep(cfg.settle_poll_s)


def _restore(io, cfg, pre):
    for axis in (cfg.q1, cfg.q2):
        v = pre.get(axis.setpoint_pv)
        if v is not None:
            try:
                io.put(axis.setpoint_pv, v)
            except Exception:
                pass   # best-effort; restore must not raise out of finally


def _write_manifest(run_dir, rows):
    cols = ["i", "j", "k", "q1_setpoint", "q2_setpoint", "q1_rbv", "q2_rbv",
            "q1_ioc_timestamp", "q2_ioc_timestamp",
            "target_bucket", "gun_bunches", "inj_mode", "gun_inhibit", "inj_seq",
            "wall_timestamp", "filename", "status", "beam_meta"]
    path = Path(run_dir) / "manifest.csv"
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _write_config_snapshot(run_dir, cfg):
    snap = {"q1": asdict(cfg.q1), "q2": asdict(cfg.q2),
            "settle": {"timeout_s": cfg.settle_timeout_s, "poll_s": cfg.settle_poll_s},
            "frames_per_point": cfg.frames_per_point,
            "trigger_timeout_s": cfg.trigger_timeout_s,
            "restore_on_finish": cfg.restore_on_finish}
    (Path(run_dir) / "scan_config.json").write_text(json.dumps(snap, indent=2))


def failed_result(failure, failure_point=None):
    """Build the canonical 'failed' scan-result dict — same schema run_scan
    returns — for callers that bail before run_scan runs (e.g. PVs never
    connected) or that catch an exception around it."""
    return {"status": "failed", "failure": failure,
            "failure_point": failure_point, "frames": 0, "rows": []}


def run_scan(cfg: ScanConfig, io, frames, saver, run_dir,
             abort_event=None, progress_cb=None) -> dict:
    validate_config(cfg)                       # raises ValueError on bad config
    grid = generate_grid(cfg)
    pvs = [cfg.q1.setpoint_pv, cfg.q1.rbv_pv, cfg.q2.setpoint_pv, cfg.q2.rbv_pv]
    if not io.connected(pvs):
        raise PreflightError(f"required PVs not connected: {pvs}")

    pre = {cfg.q1.setpoint_pv: io.get(cfg.q1.setpoint_pv),
           cfg.q2.setpoint_pv: io.get(cfg.q2.setpoint_pv)}

    rows = []
    status, failure, failure_point = "completed", None, None
    try:
        for (i, j, q1, q2) in grid:
            _check_abort(abort_event)
            _set_axis(io, cfg.q1, q1)
            _set_axis(io, cfg.q2, q2)
            _settle(io, cfg.q1, q1, cfg, abort_event)
            _settle(io, cfg.q2, q2, cfg, abort_event)
            for k in range(cfg.frames_per_point):
                _check_abort(abort_event)
                frame = frames.next_triggered_frame(cfg.trigger_timeout_s)
                rbvs = {"q1": io.get(cfg.q1.rbv_pv), "q2": io.get(cfg.q2.rbv_pv)}
                ioc = {"q1": io.get_timestamp(cfg.q1.rbv_pv),
                       "q2": io.get_timestamp(cfg.q2.rbv_pv)}
                setpoints = {"q1": q1, "q2": q2}
                ts = {"wall": time.time(), "q1_ioc": ioc["q1"], "q2_ioc": ioc["q2"]}
                beam = _snapshot_beam(io, cfg.beam_meta_pvs)
                decoded = _decode_timinjreq(beam.get("TimInjReq", {}).get("value"))
                fname = saver.save(frame, (i, j, k), setpoints, rbvs, ts,
                                   beam=beam, decoded=decoded)
                rows.append({"i": i, "j": j, "k": k, "q1_setpoint": q1,
                             "q2_setpoint": q2, "q1_rbv": rbvs["q1"], "q2_rbv": rbvs["q2"],
                             "q1_ioc_timestamp": ioc["q1"], "q2_ioc_timestamp": ioc["q2"],
                             "target_bucket": decoded.get("target_bucket"),
                             "gun_bunches": decoded.get("gun_bunches"),
                             "inj_mode": decoded.get("inj_mode"),
                             "gun_inhibit": decoded.get("gun_inhibit"),
                             "inj_seq": decoded.get("inj_seq"),
                             "wall_timestamp": ts["wall"], "filename": fname,
                             "status": "ok", "beam_meta": json.dumps(beam)})
            if progress_cb is not None:
                progress_cb(i, j)
    except ScanAborted as e:
        status, failure, failure_point = "aborted", str(e), (i, j)
    except Exception as e:
        status, failure, failure_point = "failed", f"{type(e).__name__}: {e}", (i, j)
    finally:
        if status != "completed" or cfg.restore_on_finish:
            _restore(io, cfg, pre)
        try:
            _write_manifest(run_dir, rows)
            _write_config_snapshot(run_dir, cfg)
        except Exception:
            pass

    return {"status": status, "failure": failure, "failure_point": failure_point,
            "frames": len(rows), "rows": rows}
