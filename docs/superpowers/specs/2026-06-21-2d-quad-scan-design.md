# Automated 2D Quadrupole Scan — Design

**Date:** 2026-06-21
**Status:** Approved design, pre-implementation

## Problem

The test bench images the beam spot on a Manta camera. To characterize the beam
optics we want a **2D quadrupole scan**: step two quadrupoles (Q1, Q2) over a
grid of current setpoints within predefined limits, and capture a beam image at
every (Q1, Q2) pair. For an N-step Q1 axis and M-step Q2 axis that is N×M
acquisitions. Doing this by hand is painful and error-prone.

Today the system is **observe-only**: the camera waits on an external hardware
trigger (`Line1`) and saves what arrives; `pv_monitor.py` only *reads* EPICS PVs.
A scan is **closed-loop control** — it must *actuate* the magnets (write
setpoints), which nothing in the repo can do yet. The metadata foundation (the
multi-forward EPICS capture, which records Q1/Q2 readbacks into each image's
sidecar) is already in place and is the right substrate for this.

## Goals

- Step Q1×Q2 over a configured grid within per-quad current limits and capture a
  beam image at each point.
- Each captured image is a **real beam shot** (next hardware/Line1-triggered
  frame after the magnets settle), not a software snapshot.
- Safe by construction: every write is limit-checked; any fault stops the scan
  and restores the quads.
- The scan logic is **headless and unit-testable** with no camera and no live
  accelerator.
- Runnable from a CLI and from a "Scan" button in the existing GUI.

## Non-Goals

- No software-triggered (beam-independent) acquisition mode in v1.
- No on-the-fly analysis (emittance fitting, spot-size extraction) — the scan
  produces raw images + metadata; analysis is downstream/offline.
- No automatic magnet hysteresis pre-cycling/standardization in v1.
- No multi-axis (>2D) scans.

## Decisions (locked during brainstorming)

| Decision | Choice |
|---|---|
| Structure | Separable headless **engine** + CLI driver + thin GUI "Scan" button (dependency-injected interfaces) |
| Capture model | After settle, wait for the **next beam/Line1-triggered frame(s)** |
| Frames per point | Configurable `frames_per_point`, default 1; raw frames kept (no averaging) |
| Settle | Poll readback (RBV) until within tolerance of setpoint, bounded by a timeout |
| Grid order | **Raster, unidirectional** (inner axis Q2 always low→high; Q2 resets to min each Q1 step) |
| Grid spec | Per axis `min/max/points` |
| Failure policy | **Fail-fast**: any fault → stop + restore quads to pre-scan values |
| Restore | Quads restored to pre-scan values on normal completion too (configurable) |
| Limits | Grid generated within each quad `[limit_min, limit_max]`; every write hard-refused if out of limits |

## Architecture

### Components

- **`scan_engine.py`** (new) — pure logic. No camera, Qt, or EPICS imports. Owns
  grid generation, the set→settle→acquire→save→restore loop, limit enforcement,
  fail-fast, and cooperative abort. Depends only on the three injected
  interfaces below.
- **`epics_control.py`** (new) — EPICS **write** (`put`) that works in native and
  tunnel mode. Native: caproto `Context` put. Tunnel: a `WriteNotifyRequest` on a
  `VirtualCircuit`, reusing `pv_monitor.py`'s socket helpers
  (`_connect_epics_socket`, `_open_channel_safe`) and forwards config.
- **`scan_cli.py`** (new) — headless driver: `python scan_cli.py --config scan.json`.
- **GUI "Scan" panel** — small addition to `test_bench_gui.py`: grid fields +
  Run/Stop, reusing the live image display and the `frame_ready` signal.
- **Reused unchanged:** `pv_monitor.py` (read RBVs from its monitor cache),
  `test_bench_gui.py`'s `frame_ready` signal + TIFF/sidecar save pipeline, the
  `_configure_trigger` (Line1) setup. The arming/auto-save code already lives on
  this branch (superset of the `arm-auto-capture` branch) — reuse in place.

### The three injected interfaces (Approach B seam)

```
EpicsController:
    set(axis, value) -> None          # limit-checked put of a setpoint PV
    read(rbv_pv) -> float | None      # latest readback value
    wait_settle(axis, setpoint, tol, timeout) -> bool   # poll RBV within tol
    connected() -> bool               # all required PVs connected

FrameSource:
    next_triggered_frame(timeout) -> ndarray   # next beam-triggered frame, or raises on timeout

FrameSaver:
    save(frame, indices, setpoints, rbvs, timestamps) -> str   # writes TIFF+sidecar, returns path
```

- **GUI** supplies an `EpicsController` backed by `pv_monitor` (reads) +
  `epics_control` (writes), a `FrameSource` that taps the existing `frame_ready`
  signal (latches the next frame after settle), and a `FrameSaver` wrapping the
  existing save pipeline.
- **CLI** supplies a `FrameSource` that opens the camera headless with the
  existing Line1 trigger config; same `EpicsController`/`FrameSaver`.
- **Tests** supply fakes for all three.

### The engine loop (`run_scan`)

```
run_scan(scan_config, epics, frames, saver, abort_event=None, progress_cb=None):
  1. Generate raster grid from per-axis min/max/points.
  2. Validate EVERY grid point within [limit_min, limit_max] for its axis;
     refuse to start otherwise. Refuse to start if epics.connected() is False.
  3. Snapshot pre-scan setpoints for both quads (for restore).
  4. try:
       for i in Q1 points (outer):
         for j in Q2 points (inner, always min->high):
           if abort_event set: raise Aborted
           epics.set("Q1", q1[i]); epics.set("Q2", q2[j])
           if not epics.wait_settle("Q1", q1[i], tol, timeout): raise SettleTimeout
           if not epics.wait_settle("Q2", q2[j], tol, timeout): raise SettleTimeout
           for k in range(frames_per_point):
             if abort_event set: raise Aborted
             frame = frames.next_triggered_frame(trigger_timeout)   # raises on timeout
             rbvs = {Q1: epics.read(q1_rbv), Q2: epics.read(q2_rbv)}
             path = saver.save(frame, (i,j,k), setpoints, rbvs, timestamps)
             manifest.append(row(i,j,k, setpoints, rbvs, ts, path, status="ok"))
           progress_cb(i, j)
     finally:
       restore quads to pre-scan setpoints (if restore_on_finish)
       write manifest.csv and scan_config.json
  5. On any raised fault: record the failing (i,j) + reason, restore (via finally),
     re-raise/report. (Fail-fast.)
```

Faults that trigger fail-fast: settle timeout, trigger/beam timeout, PV
disconnect, rejected/failed write, out-of-limit (caught at validation).

## Configuration (`scan.json`)

```json
{
  "axes": {
    "Q1": {
      "setpoint_pv": "LTB:Q1_1:Setpoint",
      "rbv_pv": "LTB:Q1_1:Readback",
      "min": -2.0, "max": 2.0, "points": 9,
      "limit_min": -5.0, "limit_max": 5.0,
      "settle_tol": 0.02
    },
    "Q2": {
      "setpoint_pv": "LTB:Q1_2:Setpoint",
      "rbv_pv": "LTB:Q1_2:Readback",
      "min": -2.0, "max": 2.0, "points": 9,
      "limit_min": -5.0, "limit_max": 5.0,
      "settle_tol": 0.02
    }
  },
  "settle": { "timeout_s": 10.0, "poll_s": 0.1 },
  "frames_per_point": 1,
  "trigger_timeout_s": 30.0,
  "restore_on_finish": true,
  "output_dir": "scans",
  "beam_meta_pvs": ["TimInjReq", "EG______BIAS___AM01"]
}
```

- `beam_meta_pvs` (optional, default `[]`): extra PVs snapshotted **best-effort**
  at every frame for beam provenance — they are monitored but are *not* part of
  the connectivity gate, so an unreadable one logs `null` rather than aborting
  the scan. A `TimInjReq` waveform is additionally decoded into named manifest
  columns: `target_bucket, gun_bunches, inj_mode, gun_inhibit, inj_seq` (ALS
  dual-EVG injection request; see `srinjectoneshot.m`). The full raw snapshot is
  also written as a JSON `beam_meta` column and into each `.txt` sidecar. For a
  Linac-screen scan the useful set is the gun shot params (`TimInjReq`) + gun
  bias readback (`EG______BIAS___AM01`). Confirm every PV name with `caget`
  first — these come from a static MML snapshot, not a live IOC.

- `min/max/points` define the scan range; `limit_min/limit_max` are the hard
  safety clamps the range must fall within (validated at start).
- `settle_tol` is per-axis (quads may have different precision).
- The EPICS **connection** config (`forwards`/native) is reused from the existing
  `pv_config.json` `_epics` block — NOT duplicated here. The scan reads/writes
  through the same forwards.
- Actual PV names, ranges, and limits are placeholders here; the real values come
  from the machine (confirm setpoint + readback PV names per quad via `cainfo`).

## Output Layout

```
scans/scan_<YYYYmmdd-HHMMSS>/
  q1_<i>_q2_<j>_shot_<k>.tiff      # raw frame (existing TIFF writer)
  q1_<i>_q2_<j>_shot_<k>.txt       # sidecar (existing format; already carries EPICS readbacks)
  manifest.csv                     # one row per frame
  scan_config.json                 # exact parameters used for this run
```

`manifest.csv` columns:
`i, j, k, q1_setpoint, q2_setpoint, q1_rbv, q2_rbv, ioc_timestamp, wall_timestamp, filename, status`

## Error Handling / Safety / Abort

- **Limit clamp:** every `epics.set` is refused (raises) if the value is outside
  `[limit_min, limit_max]`. Grid validation at start guarantees no in-range point
  violates this; the per-write check is defense in depth.
- **Pre-flight:** refuse to start if any grid point is out of limits or any
  required PV (setpoint or RBV, both quads) is disconnected.
- **Cooperative abort:** `abort_event` (a `threading.Event`) is checked between
  points and inside the settle/trigger waits. The GUI Stop button and the CLI
  `Ctrl-C` handler set it. Abort routes through the same restore path as a fault.
- **Restore:** on completion, fault, or abort, quads are driven back to their
  pre-scan setpoints (when `restore_on_finish`).
- **Partial-data preservation:** the manifest and config snapshot are written in
  the `finally` block, so a scan stopped early still leaves a usable record of
  what was captured.

## Testing

Unit tests for `scan_engine` with fake `EpicsController` / `FrameSource` /
`FrameSaver` — **no camera, no IOC**:

- Raster order: visits points inner-axis-low→high, Q2 reset each Q1 step.
- Limit refusal: a grid point outside `[limit_min, limit_max]` refuses to start;
  a forced out-of-range `set` raises.
- Settle timeout → fail-fast + restore called with pre-scan values.
- Trigger/beam timeout → fail-fast + restore.
- Mid-scan abort (`abort_event` set after K points) → stops, restore called,
  manifest written with the points done so far.
- `frames_per_point`: K frames saved per point with correct shot indices.
- Manifest correctness: row count = points×frames, columns populated.

Plus a no-network test for the `epics_control` tunnel `put` path: pointed at a
dead host, it never blocks or raises into the caller (mirrors the existing
`pv_monitor` tunnel test).

Tests are script-style (plain `assert` + `print("ok ...")`), run via
`.venv/bin/python test_scan_engine.py`, consistent with the repo's existing
test convention.

## Rollout (build order, dependency-ordered)

1. `epics_control.py` — EPICS write (native + tunnel) + its no-network test.
2. `scan_engine.py` — grid + loop + limits + fail-fast/restore + abort, against
   the three interfaces; full fake-driven unit tests.
3. `scan_cli.py` — headless driver wiring real EPICS + a headless camera
   `FrameSource` + the existing saver.
4. GUI "Scan" panel — Run/Stop + grid fields, `FrameSource` tapping `frame_ready`.
```
