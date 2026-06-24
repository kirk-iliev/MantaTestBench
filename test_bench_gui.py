#!/usr/bin/env python3
"""
Test Bench GUI — Manta G-235B Hardware Trigger Validation

Live triggered frame display with adjustable camera settings and
frame + sidecar metadata saving. Intermediate step toward the full
OTR emittance measurement system.
"""

import sys
import time
import queue
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import cv2
import vmbpy

from pv_monitor import PVMonitor, load_pv_config
from scan_config import AxisConfig, ScanConfig, validate_config, load_scan_config
from scan_engine import run_scan, failed_result, ScanAborted
from scan_io import MonitorWriterIO, ScanFrameSaver
from epics_control import PVWriter

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QGroupBox, QFormLayout, QDoubleSpinBox, QSpinBox, QPushButton,
    QPlainTextEdit, QFileDialog, QSizePolicy, QCheckBox, QLineEdit,
    QScrollArea, QFrame,
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap


# ──────────────────────────────────────────────────────────────────────────────
# Camera Worker
# ──────────────────────────────────────────────────────────────────────────────

class CameraWorker(QThread):
    """
    Owns the vmbpy session on its own thread.

    Signals
    -------
    frame_ready(np.ndarray)   : emitted for every complete frame received
    stats_updated(dict)       : emitted ~1 Hz with FPS / frame counts
    initialized(dict)         : emitted once after camera is configured,
                                carries initial exposure / gain / max_fps
    error_occurred(str)       : any fatal or per-feature error
    """

    frame_ready    = pyqtSignal(np.ndarray)
    stats_updated  = pyqtSignal(dict)
    initialized    = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)

    def __init__(self, vmb: vmbpy.VmbSystem, pv_monitor=None):
        super().__init__()
        self._vmb        = vmb
        self._pv_monitor = pv_monitor   # optional EPICS metadata source; may be None
        self._stop_event = threading.Event()
        self._cmd_queue  = queue.Queue()
        self._lock       = threading.Lock()

        # per-second FPS bucket
        self._fps_count     = 0
        self._fps_last_time = 0.0
        self._system_fps    = 0.0

        # cumulative counters
        self._total_frames   = 0
        self._dropped_frames = 0

        # acquisition gating (used in free-run mode)
        self._emit_frames = threading.Event()
        self._emit_frames.set()   # streaming on by default
        self._snap_pending = threading.Event()

        # auto-save (armed for the duration of a run)
        self._armed        = threading.Event()
        self._save_queue   = queue.Queue()   # (np.ndarray, meta dict) → saver thread
        self._saver        = None
        self._run_dir      = None
        self._shot_index   = 0
        self._saved_count  = 0
        self._save_meta    = {}              # exposure/gain/desc snapshot at arm time
        self._pixel_format = "unknown"

    # ── public API (called from GUI thread) ───────────────────────────────

    def set_feature(self, name: str, value):
        """Queue a camera feature change to be applied on the worker thread."""
        self._cmd_queue.put((name, value))

    def pause_stream(self):
        """Suppress frame emission without stopping the vmbpy stream."""
        self._emit_frames.clear()

    def resume_stream(self):
        self._snap_pending.clear()
        self._emit_frames.set()

    def snap_frame(self):
        """Emit exactly one frame then auto-pause."""
        self._snap_pending.set()
        self._emit_frames.set()

    def arm(self, run_dir: str, meta: dict):
        """Begin auto-saving every triggered frame into run_dir."""
        self._run_dir     = run_dir
        self._save_meta   = meta
        self._shot_index  = 0
        self._saved_count = 0
        self._armed.set()           # set last: gate fields are ready before callback sees it

    def disarm(self):
        self._armed.clear()

    def stop(self):
        self._stop_event.set()

    # ── thread entry point ────────────────────────────────────────────────

    def run(self):
        try:
            cams = self._vmb.get_all_cameras()
            if not cams:
                self.error_occurred.emit("No cameras found.")
                return

            with cams[0] as cam:
                self._configure_trigger(cam)

                # read initial values so the GUI can mirror them
                init = self._read_initial_settings(cam)
                self.initialized.emit(init)

                self._fps_last_time = time.time()
                last_stat_time = time.time()

                # writer runs on its own thread so disk I/O never stalls
                # acquisition or the GUI, even at the fastest shot rates
                self._saver = threading.Thread(target=self._saver_loop, daemon=True)
                self._saver.start()

                cam.start_streaming(self._frame_handler)
                try:
                    while not self._stop_event.is_set():
                        self._drain_command_queue(cam)

                        now = time.time()
                        if now - last_stat_time >= 1.0:
                            last_stat_time = now
                            self._emit_stats(cam)

                        self._stop_event.wait(timeout=0.05)
                finally:
                    cam.stop_streaming()
                    self._save_queue.put(None)          # sentinel → drain & exit saver
                    self._saver.join(timeout=5.0)

        except Exception as exc:
            self.error_occurred.emit(str(exc))

    # ── private helpers ───────────────────────────────────────────────────

    def _configure_trigger(self, cam):
        cam.TriggerSelector.set("FrameStart")
        cam.TriggerMode.set("On")
        cam.TriggerSource.set("Line1")
        cam.TriggerActivation.set("RisingEdge")
        cam.AcquisitionMode.set("Continuous")

        # 12-bit for quantitative OTR profiles (full 0–4095 dynamic range);
        # comes back as uint16 and is saved as a 16-bit TIFF. Fall back
        # silently to whatever the camera defaults to if Mono12 is rejected.
        try:
            cam.set_pixel_format(vmbpy.PixelFormat.Mono12)
        except Exception as exc:
            self.error_occurred.emit(f"Could not set Mono12 (using default): {exc}")
        try:
            self._pixel_format = str(cam.get_pixel_format())
        except Exception:
            self._pixel_format = "unknown"

    def _read_initial_settings(self, cam) -> dict:
        init = {}
        for feat, key in [("ExposureTime", "exposure_us"), ("Gain", "gain_db")]:
            try:
                init[key] = cam.get_feature_by_name(feat).get()
            except Exception:
                init[key] = 0.0
        try:
            init["max_fps"] = cam.get_feature_by_name("ResultingFrameRate").get()
        except Exception:
            init["max_fps"] = 0.0
        return init

    def _drain_command_queue(self, cam):
        while not self._cmd_queue.empty():
            feat_name, value = self._cmd_queue.get_nowait()
            try:
                cam.get_feature_by_name(feat_name).set(value)
            except Exception as exc:
                self.error_occurred.emit(f"Could not set {feat_name}: {exc}")

    def _emit_stats(self, cam):
        try:
            cam_fps = cam.get_feature_by_name("StatFrameRate").get()
        except Exception:
            cam_fps = 0.0
        try:
            max_fps = cam.get_feature_by_name("ResultingFrameRate").get()
        except Exception:
            max_fps = 0.0

        with self._lock:
            stats = {
                "system_fps": self._system_fps,
                "camera_fps": cam_fps,
                "max_fps":    max_fps,
                "total":      self._total_frames,
                "dropped":    self._dropped_frames,
                "saved":      self._saved_count,
            }
        self.stats_updated.emit(stats)

    def _frame_handler(self, cam, stream, frame):
        """Called by vmbpy on its internal callback thread."""
        if frame.get_status() == vmbpy.FrameStatus.Complete:
            arr = frame.as_numpy_ndarray().copy()

            with self._lock:
                self._total_frames += 1

            # Auto-save runs independently of the display gate: every
            # triggered frame is written while armed, even if the live
            # view is paused. The frame id / camera timestamp are read
            # here (before the buffer is requeued) so a missed egun shot
            # shows up as a gap in the saved sequence.
            if self._armed.is_set():
                self._shot_index += 1
                stamp = datetime.now()
                # Snapshot EPICS PVs as close to the trigger as possible. This is
                # just a locked dict copy from the monitor's cache — no network in
                # the hot path — and is {} when no monitor / no PVs are connected.
                epics = self._pv_monitor.snapshot() if self._pv_monitor else {}
                meta = {
                    "run_dir":      self._run_dir,
                    "shot_index":   self._shot_index,
                    "iso":          stamp.isoformat(),
                    "stem_time":    stamp.strftime("%Y%m%d_%H%M%S_")
                                    + f"{stamp.microsecond // 1000:03d}",
                    "frame_id":     frame.get_id(),
                    "cam_timestamp": frame.get_timestamp(),
                    "pixel_format": self._pixel_format,
                    "epics":        epics,
                    **self._save_meta,   # exposure_us, gain_db, description
                }
                self._save_queue.put((arr, meta))

            if self._emit_frames.is_set():
                self.frame_ready.emit(arr)

                with self._lock:
                    self._fps_count += 1
                    now     = time.time()
                    elapsed = now - self._fps_last_time
                    if elapsed >= 1.0:
                        self._system_fps    = self._fps_count / elapsed
                        self._fps_count     = 0
                        self._fps_last_time = now

                # snap mode: one frame emitted, then auto-pause
                if self._snap_pending.is_set():
                    self._snap_pending.clear()
                    self._emit_frames.clear()
        else:
            with self._lock:
                self._dropped_frames += 1

        cam.queue_frame(frame)

    # ── saver thread ──────────────────────────────────────────────────────

    def _saver_loop(self):
        """Drain the save queue, writing each frame + sidecar to disk."""
        while True:
            item = self._save_queue.get()
            if item is None:          # sentinel from stop()
                break
            arr, meta = item
            try:
                self._write_shot(arr, meta)
                with self._lock:
                    self._saved_count += 1
            except Exception as exc:
                self.error_occurred.emit(f"Save failed (shot {meta['shot_index']}): {exc}")

    def _write_shot(self, arr, meta):
        stem     = f"shot_{meta['shot_index']:05d}_{meta['stem_time']}"
        img_path = Path(meta["run_dir"]) / f"{stem}.tiff"
        txt_path = Path(meta["run_dir"]) / f"{stem}.txt"

        cv2.imwrite(str(img_path), arr)   # uint16 → 16-bit TIFF

        lines = [
            f"timestamp:     {meta['iso']}",
            f"shot_index:    {meta['shot_index']}",
            f"frame_id:      {meta['frame_id']}",
            f"cam_timestamp: {meta['cam_timestamp']}",
            f"exposure_us:   {meta.get('exposure_us', 0.0):.1f}",
            f"gain_db:       {meta.get('gain_db', 0.0):.1f}",
            f"pixel_format:  {meta['pixel_format']}",
            f"description:   {meta.get('description', '')}",
        ]

        # EPICS readbacks captured at trigger time, each with its IOC-set
        # timestamp so a value's staleness relative to the shot is visible.
        epics = meta.get("epics", {})
        for label, rec in epics.items():
            if rec.get("connected"):
                ts = rec.get("timestamp")
                ts_str = datetime.fromtimestamp(ts).isoformat() if ts else "no-timestamp"
                lines.append(f"epics.{label}: {rec['value']} @ {ts_str}")
            else:
                lines.append(f"epics.{label}: disconnected")

        txt_path.write_text("\n".join(lines) + "\n")


# ──────────────────────────────────────────────────────────────────────────────
# Scan helpers
# ──────────────────────────────────────────────────────────────────────────────

class _SignalFrameSource:
    """FrameSource that latches the next frame emitted by CameraWorker.frame_ready.

    Connect ``worker.frame_ready`` to ``on_frame``; call ``next_triggered_frame``
    from the scan runner thread to block until a frame arrives (or timeout).
    """

    def __init__(self, abort_event=None):
        self._lock  = threading.Lock()
        self._cond  = threading.Condition(self._lock)
        self._frame = None
        self._abort = abort_event

    def on_frame(self, frame: np.ndarray):
        """Slot — connect to CameraWorker.frame_ready."""
        with self._cond:
            self._frame = frame
            self._cond.notify_all()

    def next_triggered_frame(self, timeout: float) -> np.ndarray:
        """Block until a new frame arrives, then return it. Polls the abort event
        in short slices so a Stop during the wait takes effect promptly rather
        than only after ``timeout``. Raises ScanAborted / TimeoutError."""
        deadline = time.monotonic() + timeout
        with self._cond:
            self._frame = None          # discard any stale frame
            while True:
                if self._abort is not None and self._abort.is_set():
                    raise ScanAborted("aborted by user")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no triggered frame within {timeout}s")
                if self._cond.wait_for(lambda: self._frame is not None,
                                       timeout=min(remaining, 0.2)):
                    return self._frame


class _ScanRunner(QThread):
    """QThread that runs run_scan() off the GUI thread.

    Emits ``progress(i, j)`` after each grid row and ``finished_scan(dict)``
    when done (always, even on exception — so the GUI is always cleaned up).
    """

    progress     = pyqtSignal(int, int)
    finished_scan = pyqtSignal(dict)

    def __init__(self, cfg, io, src, run_dir: str, abort_event, pvs=None, connect_timeout=10.0):
        super().__init__()
        self._cfg, self._io, self._src = cfg, io, src
        self._run_dir, self._abort     = run_dir, abort_event
        self._pvs = pvs or []
        self._connect_timeout = connect_timeout

    def run(self):
        if not self._io.wait_connected(self._pvs, self._connect_timeout):
            self.finished_scan.emit(failed_result("required PVs did not connect"))
            return
        saver = ScanFrameSaver(self._run_dir)
        try:
            res = run_scan(
                self._cfg, self._io, self._src, saver, self._run_dir,
                abort_event=self._abort,
                progress_cb=lambda i, j: self.progress.emit(i, j),
            )
        except Exception as exc:
            res = failed_result(f"{type(exc).__name__}: {exc}")
        self.finished_scan.emit(res)


# ──────────────────────────────────────────────────────────────────────────────
# Main Window
# ──────────────────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manta G-235B — Test Bench")
        self.resize(1100, 700)

        self._current_frame: np.ndarray | None = None
        self._save_dir = str(Path.home() / "Desktop")
        self._hw_trigger_active = True   # starts in hardware trigger mode
        self._auto_stretch = False

        self._vmb = vmbpy.VmbSystem.get_instance()
        self._vmb.__enter__()

        self._pv_monitor = self._init_pv_monitor()

        self._build_ui()
        self._start_camera()

    def _init_pv_monitor(self):
        """Start the optional EPICS PV monitor from pv_config.json.

        Returns a started PVMonitor, or None if there's nothing to monitor or
        anything goes wrong. The camera tool runs identically either way.
        """
        cfg_path = Path(__file__).parent / "pv_config.json"
        try:
            pv_map, tunnel_cfg = load_pv_config(cfg_path)
            if not pv_map:
                return None
            mon = PVMonitor(pv_map, tunnel_cfg=tunnel_cfg)
            mon.start()
            # tunnel_cfg is a list of {host,port} forwards (or None for native).
            if tunnel_cfg:
                mode = "tunnel " + ", ".join(
                    f"{f['host']}:{f['port']}" for f in tunnel_cfg)
            else:
                mode = "native"
            print(f"EPICS metadata: monitoring {mon.total_count()} PV(s) "
                  f"from {cfg_path.name} ({mode} mode)")
            return mon
        except Exception as exc:
            print(f"EPICS metadata disabled (monitor init failed: {exc})")
            return None

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        root.addWidget(self._build_live_view(), stretch=4)
        root.addWidget(self._build_right_panel())

        # Frame counters pinned to the right of the status bar (permanent
        # widgets, so they aren't clobbered by showMessage() status text).
        self._lbl_acquired = QLabel("Acquired: 0")
        self._lbl_dropped  = QLabel("Dropped: 0")
        self.statusBar().addPermanentWidget(self._lbl_acquired)
        self.statusBar().addPermanentWidget(self._lbl_dropped)
        self.statusBar().showMessage("Starting camera…")

    def _build_live_view(self) -> QLabel:
        lbl = QLabel("Waiting for hardware trigger…")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setMinimumSize(640, 480)
        lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lbl.setStyleSheet("background: #1a1a1a; color: #666; font-size: 14px;")
        self._view_label = lbl
        return lbl

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(340)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self._build_camera_settings_group())
        layout.addWidget(self._build_save_group())
        layout.addWidget(self._build_scan_group())
        layout.addStretch()

        # Wrap the controls in a scroll area so the panel's tall content
        # (stats + camera + save + scan groups) does not force the whole
        # window taller than the display. The scroll area's own minimum
        # height is small, so the window stays resizable and the controls
        # scroll instead of clipping off-screen.
        scroll = QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFixedWidth(340 + 18)   # panel width + room for the scrollbar
        return scroll

    def _build_camera_settings_group(self) -> QGroupBox:
        group = QGroupBox("Camera Settings")
        form  = QFormLayout(group)

        # Exposure
        self._exp_spin = QDoubleSpinBox()
        self._exp_spin.setRange(10.0, 1_000_000.0)
        self._exp_spin.setValue(5000.0)
        self._exp_spin.setSuffix(" µs")
        self._exp_spin.setSingleStep(500.0)
        self._exp_spin.setDecimals(1)
        exp_apply = QPushButton("Apply")
        exp_apply.setFixedWidth(55)
        exp_apply.clicked.connect(self._apply_exposure)
        exp_row = QHBoxLayout()
        exp_row.addWidget(self._exp_spin)
        exp_row.addWidget(exp_apply)

        # Gain
        self._gain_spin = QDoubleSpinBox()
        self._gain_spin.setRange(0.0, 40.0)
        self._gain_spin.setValue(0.0)
        self._gain_spin.setSuffix(" dB")
        self._gain_spin.setSingleStep(1.0)
        self._gain_spin.setDecimals(1)
        gain_apply = QPushButton("Apply")
        gain_apply.setFixedWidth(55)
        gain_apply.clicked.connect(self._apply_gain)
        gain_row = QHBoxLayout()
        gain_row.addWidget(self._gain_spin)
        gain_row.addWidget(gain_apply)

        # Trigger mode toggle
        self._trigger_btn = QPushButton("Mode: Hardware Trigger (Line1)")
        self._trigger_btn.setCheckable(True)
        self._trigger_btn.setChecked(False)   # unchecked = hardware trigger
        self._trigger_btn.clicked.connect(self._toggle_trigger_mode)

        # Free-run acquisition controls (enabled only in free-run mode)
        self._stream_btn = QPushButton("Stop Stream")
        self._stream_btn.setCheckable(True)
        self._stream_btn.setEnabled(False)
        self._stream_btn.clicked.connect(self._toggle_stream)

        self._snap_btn = QPushButton("Snap (Single Frame)")
        self._snap_btn.setEnabled(False)
        self._snap_btn.clicked.connect(self._snap_one_frame)

        acq_row = QHBoxLayout()
        acq_row.addWidget(self._stream_btn)
        acq_row.addWidget(self._snap_btn)

        stretch_chk = QCheckBox("Auto-stretch display")
        stretch_chk.setChecked(False)
        stretch_chk.stateChanged.connect(
            lambda state: setattr(self, '_auto_stretch', bool(state))
        )

        form.addRow("Exposure:", exp_row)
        form.addRow("Gain:",     gain_row)
        form.addRow(self._trigger_btn)
        form.addRow(acq_row)
        form.addRow(stretch_chk)
        return group

    def _build_save_group(self) -> QGroupBox:
        group  = QGroupBox("Save Frame + Metadata")
        layout = QVBoxLayout(group)

        # Save directory row
        dir_row = QHBoxLayout()
        self._dir_label = QLabel(self._save_dir)
        self._dir_label.setWordWrap(True)
        self._dir_label.setStyleSheet("font-size: 10px; color: #777;")
        browse_btn = QPushButton("Browse")
        browse_btn.setFixedWidth(60)
        browse_btn.clicked.connect(self._browse_dir)
        dir_row.addWidget(self._dir_label, stretch=1)
        dir_row.addWidget(browse_btn)
        layout.addLayout(dir_row)

        # Description
        layout.addWidget(QLabel("Description:"))
        self._desc_edit = QPlainTextEdit()
        self._desc_edit.setFixedHeight(72)
        self._desc_edit.setPlaceholderText("Notes about this measurement…")
        layout.addWidget(self._desc_edit)

        # Arm/Record: while armed, every triggered frame auto-saves into a
        # fresh run_<timestamp> subfolder. This is the emittance-scan path.
        self._arm_btn = QPushButton("Arm Auto-Save")
        self._arm_btn.setCheckable(True)
        self._arm_btn.clicked.connect(self._toggle_arm)
        layout.addWidget(self._arm_btn)

        # Manual single-shot save (one-offs / alignment checks)
        save_btn = QPushButton("Save Frame + Metadata")
        save_btn.clicked.connect(self._save_frame)
        layout.addWidget(save_btn)

        # Status label
        self._save_status = QLabel("")
        self._save_status.setWordWrap(True)
        self._save_status.setStyleSheet("font-size: 10px;")
        layout.addWidget(self._save_status)

        return group

    def _build_scan_group(self) -> QGroupBox:
        """Return the '2D Scan' group box. Try to pre-fill from scan.json."""
        group  = QGroupBox("2D Scan")
        layout = QVBoxLayout(group)

        # Attempt to load defaults from scan.json (optional; fails gracefully).
        sc = None
        try:
            sc = load_scan_config(Path(__file__).parent / "scan.json")
        except Exception:
            pass

        def _dbl(val, lo=-1e6, hi=1e6, step=0.1, dec=3):
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setValue(val)
            w.setSingleStep(step)
            w.setDecimals(dec)
            return w

        def _spin(val, lo=1, hi=9999):
            w = QSpinBox()
            w.setRange(lo, hi)
            w.setValue(val)
            return w

        def _ledit(text):
            w = QLineEdit(text)
            w.setMinimumWidth(200)      # fit full PV names without squeezing
            return w

        form = QFormLayout()
        # Stack long fields (PV names) under their label so they get the panel's
        # full width instead of sharing a narrow row; numeric rows stay inline.
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        layout.addLayout(form)

        # ── Q1 axis ──────────────────────────────────────────────────────
        self._scan_q1_sp_pv  = _ledit(sc.q1.setpoint_pv  if sc else "")
        self._scan_q1_rbv_pv = _ledit(sc.q1.rbv_pv       if sc else "")
        self._scan_q1_min    = _dbl(sc.q1.min             if sc else 0.0)
        self._scan_q1_max    = _dbl(sc.q1.max             if sc else 1.0)
        self._scan_q1_pts    = _spin(sc.q1.points         if sc else 3)
        self._scan_q1_lmin   = _dbl(sc.q1.limit_min       if sc else -10.0)
        self._scan_q1_lmax   = _dbl(sc.q1.limit_max       if sc else  10.0)
        self._scan_q1_tol    = _dbl(sc.q1.settle_tol      if sc else  0.01, lo=1e-6, hi=1e3, step=0.001, dec=4)

        form.addRow("Q1 setpt PV:", self._scan_q1_sp_pv)
        form.addRow("Q1 RBV PV:",   self._scan_q1_rbv_pv)

        q1_rng = QHBoxLayout()
        q1_rng.addWidget(self._scan_q1_min)
        q1_rng.addWidget(QLabel("→"))
        q1_rng.addWidget(self._scan_q1_max)
        q1_rng.addWidget(QLabel("N:"))
        q1_rng.addWidget(self._scan_q1_pts)
        form.addRow("Q1 range:", q1_rng)

        q1_lim = QHBoxLayout()
        q1_lim.addWidget(self._scan_q1_lmin)
        q1_lim.addWidget(QLabel("→"))
        q1_lim.addWidget(self._scan_q1_lmax)
        q1_lim.addWidget(QLabel("±"))
        q1_lim.addWidget(self._scan_q1_tol)
        form.addRow("Q1 lim/tol:", q1_lim)

        # ── Q2 axis ──────────────────────────────────────────────────────
        self._scan_q2_sp_pv  = _ledit(sc.q2.setpoint_pv  if sc else "")
        self._scan_q2_rbv_pv = _ledit(sc.q2.rbv_pv       if sc else "")
        self._scan_q2_min    = _dbl(sc.q2.min             if sc else 0.0)
        self._scan_q2_max    = _dbl(sc.q2.max             if sc else 1.0)
        self._scan_q2_pts    = _spin(sc.q2.points         if sc else 3)
        self._scan_q2_lmin   = _dbl(sc.q2.limit_min       if sc else -10.0)
        self._scan_q2_lmax   = _dbl(sc.q2.limit_max       if sc else  10.0)
        self._scan_q2_tol    = _dbl(sc.q2.settle_tol      if sc else  0.01, lo=1e-6, hi=1e3, step=0.001, dec=4)

        form.addRow("Q2 setpt PV:", self._scan_q2_sp_pv)
        form.addRow("Q2 RBV PV:",   self._scan_q2_rbv_pv)

        q2_rng = QHBoxLayout()
        q2_rng.addWidget(self._scan_q2_min)
        q2_rng.addWidget(QLabel("→"))
        q2_rng.addWidget(self._scan_q2_max)
        q2_rng.addWidget(QLabel("N:"))
        q2_rng.addWidget(self._scan_q2_pts)
        form.addRow("Q2 range:", q2_rng)

        q2_lim = QHBoxLayout()
        q2_lim.addWidget(self._scan_q2_lmin)
        q2_lim.addWidget(QLabel("→"))
        q2_lim.addWidget(self._scan_q2_lmax)
        q2_lim.addWidget(QLabel("±"))
        q2_lim.addWidget(self._scan_q2_tol)
        form.addRow("Q2 lim/tol:", q2_lim)

        # ── Global params ─────────────────────────────────────────────────
        self._scan_fpp    = _spin(sc.frames_per_point if sc else 1)
        self._scan_settle = _dbl(sc.settle_timeout_s  if sc else 10.0, lo=0.1, hi=3600.0, step=1.0, dec=1)
        self._scan_trig   = _dbl(sc.trigger_timeout_s if sc else 30.0, lo=0.1, hi=3600.0, step=1.0, dec=1)

        # Comma-separated beam-metadata PVs snapshotted (best-effort) per frame
        # for provenance, e.g. "TimInjReq, EG______BIAS___AM01". A "TimInjReq"
        # waveform is also decoded into named columns. Empty = none recorded.
        self._scan_beam_pvs = _ledit(", ".join(sc.beam_meta_pvs) if sc else "")

        form.addRow("Frames/pt:",   self._scan_fpp)
        form.addRow("Settle tmo s:", self._scan_settle)
        form.addRow("Trig tmo s:",   self._scan_trig)
        form.addRow("Beam meta PVs:", self._scan_beam_pvs)

        # ── Run / Stop ────────────────────────────────────────────────────
        self._scan_run_btn  = QPushButton("Run Scan")
        self._scan_stop_btn = QPushButton("Stop")
        self._scan_stop_btn.setEnabled(False)
        self._scan_run_btn.clicked.connect(self._start_scan)
        self._scan_stop_btn.clicked.connect(self._stop_scan)
        btn_row = QHBoxLayout()
        btn_row.addWidget(self._scan_run_btn)
        btn_row.addWidget(self._scan_stop_btn)
        layout.addLayout(btn_row)

        return group

    def _build_scan_config_from_fields(self) -> ScanConfig:
        """Read UI spinbox/lineedit fields into AxisConfig / ScanConfig."""
        q1 = AxisConfig(
            setpoint_pv=self._scan_q1_sp_pv.text().strip(),
            rbv_pv=self._scan_q1_rbv_pv.text().strip(),
            min=self._scan_q1_min.value(),
            max=self._scan_q1_max.value(),
            points=self._scan_q1_pts.value(),
            limit_min=self._scan_q1_lmin.value(),
            limit_max=self._scan_q1_lmax.value(),
            settle_tol=self._scan_q1_tol.value(),
        )
        q2 = AxisConfig(
            setpoint_pv=self._scan_q2_sp_pv.text().strip(),
            rbv_pv=self._scan_q2_rbv_pv.text().strip(),
            min=self._scan_q2_min.value(),
            max=self._scan_q2_max.value(),
            points=self._scan_q2_pts.value(),
            limit_min=self._scan_q2_lmin.value(),
            limit_max=self._scan_q2_lmax.value(),
            settle_tol=self._scan_q2_tol.value(),
        )
        return ScanConfig(
            q1=q1, q2=q2,
            settle_timeout_s=self._scan_settle.value(),
            settle_poll_s=0.1,
            frames_per_point=self._scan_fpp.value(),
            trigger_timeout_s=self._scan_trig.value(),
            restore_on_finish=True,
            output_dir=str(self._save_dir),
            beam_meta_pvs=[p.strip() for p in self._scan_beam_pvs.text().split(",")
                           if p.strip()],
        )

    def _start_scan(self):
        if getattr(self, "_worker", None) is None:
            self.statusBar().showMessage("Start the camera before running a scan")
            return
        cfg = self._build_scan_config_from_fields()
        try:
            validate_config(cfg)
        except ValueError as e:
            self.statusBar().showMessage(f"Scan config invalid: {e}")
            return

        pvs = [cfg.q1.setpoint_pv, cfg.q1.rbv_pv,
               cfg.q2.setpoint_pv, cfg.q2.rbv_pv]
        if not all(p for p in pvs):
            self.statusBar().showMessage("Scan: fill in all PV name fields")
            return

        _pv_map, forwards = load_pv_config(Path(__file__).parent / "pv_config.json")

        # Build a dedicated PVMonitor keyed by PV name (label == pv name) so
        # MonitorWriterIO.get(pv) and .connected(pvs) can look up by PV name.
        # self._pv_monitor uses labels from pv_config.json (e.g. "q1_current_a")
        # which do not match raw PV names, so it cannot serve the scan's
        # connectivity checks or readback calls.
        # Monitor the quad PVs plus any beam-metadata PVs (TimInjReq, gun bias,
        # …) so they're cached for per-frame snapshots. Beam-meta PVs are NOT
        # added to the `pvs` connectivity gate below — they're best-effort.
        mon_pvs = pvs + [p for p in cfg.beam_meta_pvs if p not in pvs]
        scan_monitor = PVMonitor({pv: pv for pv in mon_pvs}, tunnel_cfg=forwards)
        scan_monitor.start()
        io = MonitorWriterIO(scan_monitor, PVWriter(forwards=forwards))

        run_dir = (Path(self._save_dir)
                   / ("scan_" + datetime.now().strftime("%Y%m%d-%H%M%S")))
        run_dir.mkdir(parents=True, exist_ok=False)

        self._scan_monitor = scan_monitor
        self._scan_io      = io
        self._scan_abort   = threading.Event()
        self._scan_src     = _SignalFrameSource(self._scan_abort)
        # _SignalFrameSource is a plain object (not a QObject), so this is a
        # direct connection: on_frame runs on the thread that emits frame_ready
        # (vmbpy's callback thread), while next_triggered_frame blocks on the
        # _ScanRunner thread — the two rendezvous via the source's Condition.
        # If _SignalFrameSource ever becomes a QObject this flips to a queued
        # connection and the scan thread would block forever; keep it a plain object.
        self._worker.frame_ready.connect(self._scan_src.on_frame)

        self._scan_runner = _ScanRunner(
            cfg, io, self._scan_src, str(run_dir), self._scan_abort,
            pvs=pvs, connect_timeout=10.0)
        self._scan_runner.progress.connect(
            lambda i, j: self.statusBar().showMessage(f"Scan point ({i},{j})"))
        self._scan_runner.finished_scan.connect(self._on_scan_finished)
        self._scan_runner.start()

        self._scan_run_btn.setEnabled(False)
        self._scan_stop_btn.setEnabled(True)
        self.statusBar().showMessage(f"Scan started → {run_dir.name}")

    def _stop_scan(self):
        if getattr(self, "_scan_abort", None) is not None:
            self._scan_abort.set()

    def _on_scan_finished(self, res: dict):
        # run() has already returned by the time this queued slot fires, but
        # wait() makes the thread's lifetime explicit before the next scan
        # reassigns self._scan_runner (dropping a still-running QThread crashes).
        if getattr(self, "_scan_runner", None) is not None:
            self._scan_runner.wait(3000)
        try:
            self._worker.frame_ready.disconnect(self._scan_src.on_frame)
        except Exception:
            pass
        if getattr(self, "_scan_monitor", None) is not None:
            self._scan_monitor.stop()
            self._scan_monitor = None
        if getattr(self, "_scan_io", None) is not None:
            try:
                self._scan_io.close()
            except Exception:
                pass
            self._scan_io = None
        self._scan_run_btn.setEnabled(True)
        self._scan_stop_btn.setEnabled(False)
        msg = f"Scan {res['status']}: {res['frames']} frames"
        if res.get("failure"):
            msg += f" — {res['failure']} at {res['failure_point']}"
        self.statusBar().showMessage(msg)

    # ── Camera startup ────────────────────────────────────────────────────

    def _start_camera(self):
        self._worker = CameraWorker(self._vmb, self._pv_monitor)
        self._worker.frame_ready.connect(self._on_frame_ready)
        self._worker.stats_updated.connect(self._on_stats_updated)
        self._worker.initialized.connect(self._on_camera_initialized)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.start()

    # ── Slots ─────────────────────────────────────────────────────────────

    def _on_camera_initialized(self, init: dict):
        self._exp_spin.setValue(init.get("exposure_us", 5000.0))
        self._gain_spin.setValue(init.get("gain_db", 0.0))
        self.statusBar().showMessage("Hardware trigger mode — waiting for Line1 signal.")

    def _on_frame_ready(self, frame: np.ndarray):
        self._current_frame = frame  # always raw — untouched for saving

        # Scale to fit label while preserving aspect ratio
        lw, lh = self._view_label.width(), self._view_label.height()
        h, w   = frame.shape[:2]
        scale  = min(lw / w, lh / h)
        dw     = max(1, int(w * scale))
        dh     = max(1, int(h * scale))
        disp   = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)

        # Convert to uint8 for display regardless of camera bit depth.
        # Mono12 / Mono12Packed come back as uint16; shift right 4 bits
        # (12-bit range 0–4095 → 8-bit range 0–255) before handing to QImage,
        # which expects exactly 1 byte per pixel for Format_Grayscale8.
        if disp.dtype == np.uint16:
            disp = (disp >> 4).astype(np.uint8)

        # Optional min-max stretch: fills the 0-255 range regardless of how
        # little of the dynamic range the signal actually uses.
        if self._auto_stretch:
            cv2.normalize(disp, disp, 0, 255, cv2.NORM_MINMAX)

        if disp.ndim == 2:
            qimg = QImage(disp.tobytes(), dw, dh, dw, QImage.Format.Format_Grayscale8)
        else:
            disp = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
            qimg = QImage(disp.tobytes(), dw, dh, dw * 3, QImage.Format.Format_RGB888)

        self._view_label.setPixmap(QPixmap.fromImage(qimg))

    def _on_stats_updated(self, stats: dict):
        self._lbl_acquired.setText(f"Acquired: {stats['total']}")
        dropped = stats['dropped']
        self._lbl_dropped.setText(f"Dropped: {dropped}")
        self._lbl_dropped.setStyleSheet("color: #e55;" if dropped > 0 else "")

    def _on_error(self, msg: str):
        self.statusBar().showMessage(f"Error: {msg}")

    def _toggle_trigger_mode(self):
        self._hw_trigger_active = not self._hw_trigger_active
        if self._hw_trigger_active:
            # restore hardware trigger on Line1
            self._worker.set_feature("TriggerSelector",   "FrameStart")
            self._worker.set_feature("TriggerSource",     "Line1")
            self._worker.set_feature("TriggerActivation", "RisingEdge")
            self._worker.set_feature("TriggerMode",       "On")
            self._worker.resume_stream()   # ensure emission is on when switching back
            self._trigger_btn.setText("Mode: Hardware Trigger (Line1)")
            self._trigger_btn.setChecked(False)
            self._view_label.setText("Waiting for hardware trigger…")
            self.statusBar().showMessage("Hardware trigger mode — waiting for Line1 signal.")
            # disable free-run controls
            self._stream_btn.setEnabled(False)
            self._snap_btn.setEnabled(False)
        else:
            # free-run: disable trigger, camera streams continuously
            self._worker.set_feature("TriggerMode", "Off")
            self._worker.resume_stream()   # start emitting immediately
            self._trigger_btn.setText("Mode: Free Run (Software)")
            self._trigger_btn.setChecked(True)
            self._stream_btn.setText("Stop Stream")
            self._stream_btn.setChecked(False)
            self._stream_btn.setEnabled(True)
            self._snap_btn.setEnabled(True)
            self.statusBar().showMessage("Free-run mode — continuous acquisition.")

    def _toggle_stream(self):
        if self._stream_btn.isChecked():
            self._worker.pause_stream()
            self._stream_btn.setText("Start Stream")
            self.statusBar().showMessage("Stream paused.")
        else:
            self._worker.resume_stream()
            self._stream_btn.setText("Stop Stream")
            self.statusBar().showMessage("Free-run mode — continuous acquisition.")

    def _snap_one_frame(self):
        # if stream was paused, snap resumes emission for exactly one frame
        self._worker.snap_frame()
        self._stream_btn.setText("Start Stream")
        self._stream_btn.setChecked(True)
        self.statusBar().showMessage("Snap — waiting for next frame…")

    def _apply_exposure(self):
        self._worker.set_feature("ExposureTime", self._exp_spin.value())

    def _apply_gain(self):
        self._worker.set_feature("Gain", self._gain_spin.value())

    def _browse_dir(self):
        path = QFileDialog.getExistingDirectory(self, "Select Save Directory", self._save_dir)
        if path:
            self._save_dir = path
            self._dir_label.setText(path)

    def _toggle_arm(self):
        if self._arm_btn.isChecked():
            # Start a run: every triggered frame from now on is saved.
            run_dir = Path(self._save_dir) / ("run_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
            try:
                run_dir.mkdir(parents=True, exist_ok=False)
            except Exception as exc:
                self._arm_btn.setChecked(False)
                self._save_status.setStyleSheet("color: #e77; font-size: 10px;")
                self._save_status.setText(f"Could not create run folder:\n{exc}")
                return

            meta = {
                "exposure_us": self._exp_spin.value(),
                "gain_db":     self._gain_spin.value(),
                "description": self._desc_edit.toPlainText().strip(),
            }
            self._worker.arm(str(run_dir), meta)

            self._arm_btn.setText("Disarm (Recording…)")
            self._arm_btn.setStyleSheet("color: #e55; font-weight: bold;")
            self._save_status.setStyleSheet("color: #4a9; font-size: 10px;")
            self._save_status.setText(f"Armed — saving to:\n{run_dir.name}")
            self.statusBar().showMessage(f"Recording every trigger → {run_dir}")
        else:
            self._worker.disarm()
            self._arm_btn.setText("Arm Auto-Save")
            self._arm_btn.setStyleSheet("")
            self._save_status.setText("Disarmed.")
            self.statusBar().showMessage("Auto-save disarmed.")

    def _save_frame(self):
        if self._current_frame is None:
            self._save_status.setStyleSheet("color: #e77; font-size: 10px;")
            self._save_status.setText("No frame captured yet.")
            return

        now  = datetime.now()
        # millisecond precision avoids collisions at test-bench rates
        stem = "frame_" + now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond // 1000:03d}"

        img_path = Path(self._save_dir) / f"{stem}.tiff"
        txt_path = Path(self._save_dir) / f"{stem}.txt"

        cv2.imwrite(str(img_path), self._current_frame)

        lines = [
            f"timestamp:   {now.isoformat()}",
            f"description: {self._desc_edit.toPlainText().strip()}",
            f"exposure_us: {self._exp_spin.value():.1f}",
            f"gain_db:     {self._gain_spin.value():.1f}",
        ]
        txt_path.write_text("\n".join(lines) + "\n")

        self._save_status.setStyleSheet("color: #4a9; font-size: 10px;")
        self._save_status.setText(f"Saved:\n{stem}")

    # ── Cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        # Abort any in-progress scan and wait for its thread to finish.
        if getattr(self, "_scan_abort", None) is not None:
            self._scan_abort.set()
        if getattr(self, "_scan_runner", None) is not None:
            self._scan_runner.wait(3000)
        if getattr(self, "_scan_src", None) is not None:
            try:
                self._worker.frame_ready.disconnect(self._scan_src.on_frame)
            except Exception:
                pass
        if getattr(self, "_scan_monitor", None) is not None:
            self._scan_monitor.stop()
            self._scan_monitor = None
        if getattr(self, "_scan_io", None) is not None:
            try:
                self._scan_io.close()
            except Exception:
                pass
            self._scan_io = None
        self._worker.stop()
        self._worker.wait(3000)
        if self._pv_monitor is not None:
            self._pv_monitor.stop()
        self._vmb.__exit__(None, None, None)
        event.accept()


# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
