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

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QGroupBox, QFormLayout, QDoubleSpinBox, QPushButton,
    QPlainTextEdit, QFileDialog, QSizePolicy, QCheckBox,
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

    def __init__(self, vmb: vmbpy.VmbSystem):
        super().__init__()
        self._vmb        = vmb
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

        except Exception as exc:
            self.error_occurred.emit(str(exc))

    # ── private helpers ───────────────────────────────────────────────────

    def _configure_trigger(self, cam):
        cam.TriggerSelector.set("FrameStart")
        cam.TriggerMode.set("On")
        cam.TriggerSource.set("Line1")
        cam.TriggerActivation.set("RisingEdge")
        cam.AcquisitionMode.set("Continuous")

    def _read_initial_settings(self, cam) -> dict:
        init = {}
        for feat, key in [("ExposureTimeAbs", "exposure_us"), ("Gain", "gain_db")]:
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
            }
        self.stats_updated.emit(stats)

    def _frame_handler(self, cam, stream, frame):
        """Called by vmbpy on its internal callback thread."""
        if frame.get_status() == vmbpy.FrameStatus.Complete:
            if self._emit_frames.is_set():
                arr = frame.as_numpy_ndarray().copy()
                self.frame_ready.emit(arr)

                with self._lock:
                    self._fps_count    += 1
                    self._total_frames += 1
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

        self._build_ui()
        self._start_camera()

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        root.addWidget(self._build_live_view(), stretch=4)
        root.addWidget(self._build_right_panel())

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
        panel.setFixedWidth(285)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self._build_stats_group())
        layout.addWidget(self._build_camera_settings_group())
        layout.addWidget(self._build_save_group())
        layout.addStretch()
        return panel

    def _build_stats_group(self) -> QGroupBox:
        group = QGroupBox("Stats")
        form  = QFormLayout(group)

        self._lbl_sys_fps  = QLabel("—")
        self._lbl_cam_fps  = QLabel("—")
        self._lbl_max_fps  = QLabel("—")
        self._lbl_total    = QLabel("0")
        self._lbl_dropped  = QLabel("0")

        form.addRow("System FPS:",   self._lbl_sys_fps)
        form.addRow("Camera FPS:",   self._lbl_cam_fps)
        form.addRow("Max Possible:", self._lbl_max_fps)
        form.addRow("Total Frames:", self._lbl_total)
        form.addRow("Dropped:",      self._lbl_dropped)
        return group

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

        # Save button
        save_btn = QPushButton("Save Frame + Metadata")
        save_btn.clicked.connect(self._save_frame)
        layout.addWidget(save_btn)

        # Status label
        self._save_status = QLabel("")
        self._save_status.setWordWrap(True)
        self._save_status.setStyleSheet("font-size: 10px;")
        layout.addWidget(self._save_status)

        return group

    # ── Camera startup ────────────────────────────────────────────────────

    def _start_camera(self):
        self._worker = CameraWorker(self._vmb)
        self._worker.frame_ready.connect(self._on_frame_ready)
        self._worker.stats_updated.connect(self._on_stats_updated)
        self._worker.initialized.connect(self._on_camera_initialized)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.start()

    # ── Slots ─────────────────────────────────────────────────────────────

    def _on_camera_initialized(self, init: dict):
        self._exp_spin.setValue(init.get("exposure_us", 5000.0))
        self._gain_spin.setValue(init.get("gain_db", 0.0))
        self._lbl_max_fps.setText(f"{init.get('max_fps', 0.0):.2f}")
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
        self._lbl_sys_fps.setText(f"{stats['system_fps']:.2f}")
        self._lbl_cam_fps.setText(f"{stats['camera_fps']:.2f}")
        self._lbl_max_fps.setText(f"{stats['max_fps']:.2f}")
        self._lbl_total.setText(str(stats['total']))
        dropped = stats['dropped']
        self._lbl_dropped.setText(str(dropped))
        color = "color: #e55;" if dropped > 0 else ""
        self._lbl_dropped.setStyleSheet(color)

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
        self._worker.set_feature("ExposureTimeAbs", self._exp_spin.value())

    def _apply_gain(self):
        self._worker.set_feature("Gain", self._gain_spin.value())

    def _browse_dir(self):
        path = QFileDialog.getExistingDirectory(self, "Select Save Directory", self._save_dir)
        if path:
            self._save_dir = path
            self._dir_label.setText(path)

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
        self._worker.stop()
        self._worker.wait(3000)
        self._vmb.__exit__(None, None, None)
        event.accept()


# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
