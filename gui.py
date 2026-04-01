"""
PLECS Buck Converter PID Auto-Tuner — GUI
==========================================
PyQt5 + matplotlib GUI for real-time monitoring and control of PID auto-tuning.

Usage:
    python gui.py
"""

import sys
import math
import threading
import io
import csv
from pathlib import Path
from typing import List, Tuple, Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QSpinBox,
    QTextEdit, QSplitter, QFileDialog, QMessageBox, QLineEdit,
    QProgressBar, QSizePolicy, QScrollArea
)
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QPixmap, QImage, QFont, QColor, QPalette

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from auto_tune import (
    TuningConfig, AutoTuner, CompensatorDesign, TuningResult,
    PlecsRpc, PidTuner, ScopeCsvParser, ResponseAnalyzer, select_best_result
)
from analyze import plot_animation, load_iter_csv, get_vout_column


# ---------------------------------------------------------------------------
# Matplotlib Canvases
# ---------------------------------------------------------------------------

class WaveformCanvas(FigureCanvasQTAgg):
    """Live waveform plot with dark theme, ghost traces, and OS/US bands."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 4), dpi=90)
        self.fig.patch.set_facecolor('#1a1a2e')
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.history: List[dict] = []  # [{result, time, vout}, ...]
        self._draw_empty()

    def _draw_empty(self, target_os: float = 5.0, target_us: float = 5.0):
        v_tgt = 5.0
        v_os = v_tgt * (1 + target_os / 100)
        v_us = v_tgt * (1 - target_us / 100)
        ax = self.ax
        ax.clear()
        ax.set_facecolor('#16213e')
        ax.axhspan(v_tgt, v_os, color='#ff6b6b', alpha=0.12)
        ax.axhspan(v_us, v_tgt, color='#ffa500', alpha=0.12)
        ax.axhline(y=v_os, color='#ff6b6b', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_us, color='#ffa500', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_tgt, color='#aaa', linestyle=':', lw=1.0, alpha=0.7)
        ax.set_xlim(0, 10)
        ax.set_ylim(max(3.8, v_us - 0.2), min(6.2, v_os + 0.2))
        ax.set_xlabel('Time (ms)', color='#ccc', fontsize=10)
        ax.set_ylabel('Output Voltage (V)', color='#ccc', fontsize=10)
        ax.set_title('Waiting for iteration...', color='#888', fontsize=11)
        ax.tick_params(colors='#ccc')
        for sp in ax.spines.values():
            sp.set_edgecolor('#444466')
        ax.grid(True, alpha=0.15, color='#8888aa')
        self.fig.tight_layout()
        self.draw()

    def update_plot(self, history: List[dict], current_idx: int,
                    target_os: float = 5.0, target_us: float = 5.0):
        v_tgt = 5.0
        v_os = v_tgt * (1 + target_os / 100)
        v_us = v_tgt * (1 - target_us / 100)
        ax = self.ax
        ax.clear()
        ax.set_facecolor('#16213e')

        # OS/US bands with dynamic labels
        ax.axhspan(v_tgt, v_os, color='#ff6b6b', alpha=0.12,
                   label=f'{target_os:.1f}% OS limit ({v_os:.3f}V)')
        ax.axhspan(v_us, v_tgt, color='#ffa500', alpha=0.12,
                   label=f'{target_us:.1f}% US limit ({v_us:.3f}V)')
        ax.axhline(y=v_os, color='#ff6b6b', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_us, color='#ffa500', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_tgt, color='#aaa', linestyle=':', lw=1.0, alpha=0.7)

        # Ghost traces
        for i in range(current_idx):
            entry = history[i]
            t_ms = [t * 1000 for t in entry['time']]
            gc = '#00ff88' if entry['result'].status == 'PASS' else '#4488ff'
            ax.plot(t_ms, entry['vout'], color=gc, lw=0.6, alpha=0.20)

        # Current waveform
        cur = history[current_idx]
        r = cur['result']
        sc = '#00ff88' if r.status == 'PASS' else '#ff6b6b'
        t_ms = [t * 1000 for t in cur['time']]
        vout = cur['vout']
        ax.plot(t_ms, vout, color=sc, lw=2.0, label=f'Iter {r.iter_num} ({r.status})', zorder=5)

        # Peak / valley markers
        if vout:
            v_peak = max(vout)
            v_valley = min(vout)
            ax.scatter([t_ms[vout.index(v_peak)]], [v_peak], color='#ff4444', s=50, zorder=6)
            ax.scatter([t_ms[vout.index(v_valley)]], [v_valley], color='#ffaa00', s=50, zorder=6)

        ax.set_xlim(min(t_ms), max(t_ms))
        v_lo = max(3.8, min(vout) - 0.05) if vout else 4.0
        v_hi = min(6.2, max(vout) + 0.05) if vout else 5.6
        ax.set_ylim(v_lo, v_hi)
        ax.set_xlabel('Time (ms)', color='#ccc', fontsize=10)
        ax.set_ylabel('Output Voltage (V)', color='#ccc', fontsize=10)
        ax.set_title(
            f"Iter {r.iter_num}  —  Kp={r.Kp:.4f}  Ki={r.Ki:.1f}  "
            f"Kd={r.Kd:.2e}  Kf={r.Kf:.0f}\n"
            f"OS={r.overshoot:.1f}%  US={r.undershoot:.1f}%  "
            f"Osc={r.osc_count}  →  {r.status}",
            color=sc, fontsize=10, fontweight='bold')
        ax.tick_params(colors='#ccc')
        for sp in ax.spines.values():
            sp.set_edgecolor('#444466')
        ax.legend(loc='upper right', facecolor='#1a1a2e', edgecolor='#444466',
                  labelcolor='#ccc', fontsize=8)
        ax.grid(True, alpha=0.15, color='#8888aa')
        self.fig.tight_layout()
        self.draw()


class MetricsCanvas(FigureCanvasQTAgg):
    """OS/US line chart and oscillation bar chart."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 2.5), dpi=90)
        self.fig.patch.set_facecolor('#1a1a2e')
        self.ax_os, self.ax_osc = self.fig.subplots(1, 2)
        super().__init__(self.fig)
        self.setParent(parent)
        self._draw_empty()

    def _draw_empty(self):
        for ax in (self.ax_os, self.ax_osc):
            ax.clear()
            ax.set_facecolor('#16213e')
            ax.tick_params(colors='#ccc')
            for sp in ax.spines.values():
                sp.set_edgecolor('#444466')
            ax.grid(True, alpha=0.15, color='#8888aa')
        self.ax_os.set_title('Overshoot / Undershoot', color='#ccc', fontsize=9)
        self.ax_osc.set_title('Oscillations', color='#ccc', fontsize=9)
        self.fig.tight_layout()
        self.draw()

    def update_metrics(self, results: List[TuningResult]):
        if not results:
            return
        iters = [r.iter_num for r in results]
        os_vals = [r.overshoot for r in results]
        us_vals = [r.undershoot for r in results]
        osc_vals = [r.osc_count for r in results]

        ax = self.ax_os
        ax.clear()
        ax.set_facecolor('#16213e')
        ax.plot(iters, os_vals, 'r-o', label='OS%', ms=4, lw=1.2)
        ax.plot(iters, us_vals, color='orange', marker='s', label='US%', ms=4, lw=1.2)
        ax.axhline(y=5.0, color='#ff6b6b', linestyle='--', alpha=0.5, lw=0.8)
        ax.set_xlabel('Iteration', color='#ccc', fontsize=8)
        ax.set_ylabel('%', color='#ccc', fontsize=8)
        ax.set_title('Overshoot / Undershoot', color='#ccc', fontsize=9)
        ax.legend(facecolor='#1a1a2e', edgecolor='#444466', labelcolor='#ccc', fontsize=7)
        ax.tick_params(colors='#ccc', labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor('#444466')
        ax.grid(True, alpha=0.15, color='#8888aa')

        ax2 = self.ax_osc
        ax2.clear()
        ax2.set_facecolor('#16213e')
        colors = ['#00ff88' if o <= 2 else '#ff6b6b' for o in osc_vals]
        ax2.bar(iters, osc_vals, color=colors, width=0.8)
        ax2.axhline(y=2, color='#888', linestyle='--', lw=0.8)
        ax2.set_xlabel('Iteration', color='#ccc', fontsize=8)
        ax2.set_ylabel('Count', color='#ccc', fontsize=8)
        ax2.set_title('Oscillations', color='#ccc', fontsize=9)
        ax2.tick_params(colors='#ccc', labelsize=8)
        for sp in ax2.spines.values():
            sp.set_edgecolor('#444466')
        ax2.grid(True, alpha=0.15, color='#8888aa')

        self.fig.tight_layout()
        self.draw()


# ---------------------------------------------------------------------------
# Worker Thread
# ---------------------------------------------------------------------------

class TunerWorker(QObject):
    """Background worker for running tuning iterations on a QThread."""

    iteration_complete = pyqtSignal(object)  # dict with result + waveform
    tuning_finished = pyqtSignal(bool)       # True = PASS found
    log_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, config: TuningConfig):
        super().__init__()
        self.config = config
        self._pause_event = threading.Event()
        self._pause_event.set()  # not paused
        self._stop_flag = False
        self._auto_tuner: Optional[AutoTuner] = None

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def stop(self):
        self._stop_flag = True
        self._pause_event.set()  # unblock if paused

    def _read_waveform(self, iter_num: int):
        """Read saved iter CSV and return time/vout/il lists."""
        csv_path = Path(self.config.results_dir) / f"iter_{iter_num:03d}.csv"
        if not csv_path.exists():
            return [], [], []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)
        if len(rows) < 2:
            return [], [], []
        header = rows[0]
        vout_col = 2
        for i, h in enumerate(header):
            if 'voltage' in h.lower() or 'vout' in h.lower():
                vout_col = i
                break
        time_vals, vout_vals, il_vals = [], [], []
        for r in rows[1:]:
            try:
                vals = [float(c) for c in r]
                time_vals.append(vals[0])
                il_vals.append(vals[1] if len(vals) > 1 else 0)
                vout_vals.append(vals[vout_col] if len(vals) > vout_col else 0)
            except ValueError:
                pass
        return time_vals, vout_vals, il_vals

    def _emit_best_summary(self, results: List[TuningResult]) -> Optional[TuningResult]:
        """Log the best iteration after the full search completes."""
        best = select_best_result(results)
        if best is None:
            return None
        self.log_message.emit(
            f"Best iteration: {best.iter_num} | "
            f"OS={best.overshoot:.2f}% US={best.undershoot:.2f}% "
            f"Osc={best.osc_count} | "
            f"Kp={best.Kp:.5f} Ki={best.Ki:.2f} Kd={best.Kd:.2e} Kf={best.Kf:.0f}"
        )
        return best

    @pyqtSlot()
    def run_auto_tune(self):
        """Main auto-tuning loop with pause/stop support."""
        self._stop_flag = False
        try:
            at = AutoTuner(self.config)
            self._auto_tuner = at
            self.log_message.emit("Connecting to PLECS...")
            at.setup()
            self.log_message.emit("Connected. Starting tuning loop.")

            Kp, Ki, Kd, Kf = at.tuner.get_initial_params()
            self.log_message.emit(
                f"Initial: Kp={Kp:.5f} Ki={Ki:.2f} Kd={Kd:.2e} Kf={Kf:.0f}")

            for i in range(self.config.max_iterations):
                # Pause / stop checks
                self._pause_event.wait()
                if self._stop_flag:
                    self.log_message.emit("Stopped by user.")
                    break

                result = at.run_iteration(i, Kp, Ki, Kd, Kf)
                at.results.append(result)

                # Read waveform from saved CSV
                t, v, il = self._read_waveform(i)
                self.iteration_complete.emit({
                    'result': result,
                    'time': t,
                    'vout': v,
                    'il': il,
                })

                msg = (f"Iter {i}: OS={result.overshoot:.1f}% "
                       f"US={result.undershoot:.1f}% Osc={result.osc_count} "
                       f"→ {result.status}")
                self.log_message.emit(msg)

                if i < self.config.max_iterations - 1:
                    Kp, Ki, Kd, Kf = at.tuner.adjust(
                        Kp, Ki, Kd, Kf,
                        result.overshoot, result.undershoot, result.osc_count)

            at.save_log()
            best = self._emit_best_summary(at.results)
            self.log_message.emit("Max iterations reached.")
            self.tuning_finished.emit(best is not None and best.status == "PASS")

        except Exception as e:
            self.error_occurred.emit(str(e))
            self.tuning_finished.emit(False)

    @pyqtSlot(float, float, float, float, int)
    def run_single(self, Kp, Ki, Kd, Kf, iter_num):
        """Run a single iteration with user-supplied parameters."""
        self._stop_flag = False
        try:
            if self._auto_tuner is None:
                at = AutoTuner(self.config)
                self._auto_tuner = at
                self.log_message.emit("Connecting to PLECS...")
                at.setup()
                self.log_message.emit("Connected.")
            at = self._auto_tuner

            result = at.run_iteration(iter_num, Kp, Ki, Kd, Kf)
            at.results.append(result)

            t, v, il = self._read_waveform(iter_num)
            self.iteration_complete.emit({
                'result': result, 'time': t, 'vout': v, 'il': il,
            })
            msg = (f"Single iter {iter_num}: OS={result.overshoot:.1f}% "
                   f"US={result.undershoot:.1f}% Osc={result.osc_count} "
                   f"→ {result.status}")
            self.log_message.emit(msg)
            self.tuning_finished.emit(result.status == "PASS")
        except Exception as e:
            self.error_occurred.emit(str(e))
            self.tuning_finished.emit(False)


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class BuckTunerGui(QMainWindow):

    # Signal to trigger single iteration in worker thread
    _run_single_sig = pyqtSignal(float, float, float, float, int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PLECS Buck Converter PID Auto-Tuner")
        screen = QApplication.primaryScreen().availableGeometry()
        w = min(1400, screen.width() - 40)
        h = min(780, screen.height() - 60)
        self.resize(w, h)

        self._results: List[TuningResult] = []
        self._waveform_history: List[dict] = []
        self._iter_counter = 0
        self._worker: Optional[TunerWorker] = None
        self._thread: Optional[QThread] = None
        self._run_mode: Optional[str] = None
        self._auto_tune_completed = False

        self._build_ui()
        self._apply_dark_theme()

    # ---- UI Construction ----

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # Left panel
        left_inner = QWidget()
        left_inner.setFixedWidth(330)
        left_layout = QVBoxLayout(left_inner)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(4)

        # Circuit image
        grp_img = QGroupBox("Circuit")
        img_layout = QVBoxLayout(grp_img)
        self.circuit_label = QLabel("No image captured")
        self.circuit_label.setAlignment(Qt.AlignCenter)
        self.circuit_label.setMinimumHeight(100)
        self.circuit_label.setStyleSheet("background: #16213e; color: #888;")
        img_layout.addWidget(self.circuit_label)
        btn_capture = QPushButton("Capture from PLECS")
        btn_capture.clicked.connect(self.on_capture_circuit)
        img_layout.addWidget(btn_capture)
        left_layout.addWidget(grp_img)

        # PID parameters
        grp_pid = QGroupBox("PID Parameters")
        pid_layout = QVBoxLayout(grp_pid)
        self.spin_kp = self._make_spin("Kp:", 0, 10, 6, 0.001, 0.317, pid_layout)
        self.spin_ki = self._make_spin("Ki:", 0, 100000, 2, 100, 3765, pid_layout)
        self.spin_kd = self._make_spin("Kd:", 0, 0.01, 9, 1e-7, 4.785e-6, pid_layout)
        self.spin_kf = self._make_spin("Kf:", 0, 2000000, 0, 1000, 551786, pid_layout)
        left_layout.addWidget(grp_pid)

        # Design variables
        grp_dv = QGroupBox("Design Variables")
        dv_layout = QVBoxLayout(grp_dv)
        self.spin_wc = self._make_spin("wc (rad/s):", 47140, 314159, 0, 1000, 157080, dv_layout)
        self.spin_phim = self._make_spin("phi_m (deg):", 30, 80, 1, 1, 60, dv_layout)
        btn_compute = QPushButton("Compute PID from wc / phi_m")
        btn_compute.clicked.connect(self.on_compute_pid)
        dv_layout.addWidget(btn_compute)
        left_layout.addWidget(grp_dv)

        # Targets
        grp_tgt = QGroupBox("Targets")
        tgt_layout = QVBoxLayout(grp_tgt)
        self.spin_tgt_os = self._make_spin("Target OS%:", 0, 50, 1, 0.5, 5.0, tgt_layout)
        self.spin_tgt_us = self._make_spin("Target US%:", 0, 50, 1, 0.5, 5.0, tgt_layout)
        self.spin_max_osc = self._make_spin("Max Osc:", 0, 20, 0, 1, 2, tgt_layout)
        self.spin_max_iter = self._make_spin("Max Iter:", 1, 200, 0, 5, 60, tgt_layout)
        left_layout.addWidget(grp_tgt)

        # Controls
        grp_ctrl = QGroupBox("Controls")
        ctrl_layout = QVBoxLayout(grp_ctrl)

        self.btn_start = QPushButton("Start Auto-Tune")
        self.btn_start.clicked.connect(self.on_start_auto_tune)
        ctrl_layout.addWidget(self.btn_start)

        self.btn_single = QPushButton("Run Single Iteration")
        self.btn_single.clicked.connect(self.on_run_single)
        ctrl_layout.addWidget(self.btn_single)

        row_ps = QHBoxLayout()
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.clicked.connect(self.on_pause)
        self.btn_pause.setEnabled(False)
        row_ps.addWidget(self.btn_pause)
        self.btn_resume = QPushButton("Resume")
        self.btn_resume.clicked.connect(self.on_resume)
        self.btn_resume.setEnabled(False)
        row_ps.addWidget(self.btn_resume)
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)
        row_ps.addWidget(self.btn_stop)
        ctrl_layout.addLayout(row_ps)

        self.btn_gif = QPushButton("Save Animation GIF")
        self.btn_gif.clicked.connect(self.on_save_gif)
        ctrl_layout.addWidget(self.btn_gif)

        self.btn_reset = QPushButton("Reset to Defaults")
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_reset.setStyleSheet(
            "QPushButton { background: #3a1a1a; color: #ff9999; border: 1px solid #664444; }"
            "QPushButton:hover { background: #5a2a2a; }")
        ctrl_layout.addWidget(self.btn_reset)

        left_layout.addWidget(grp_ctrl)

        # Log
        grp_log = QGroupBox("Log")
        log_layout = QVBoxLayout(grp_log)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(110)
        self.log_text.setStyleSheet("background: #0d1117; color: #c9d1d9; font-size: 9pt;")
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(grp_log)

        left_layout.addStretch()

        left_scroll = QScrollArea()
        left_scroll.setWidget(left_inner)
        left_scroll.setWidgetResizable(False)
        left_scroll.setFixedWidth(350)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setFrameShape(left_scroll.NoFrame)
        main_layout.addWidget(left_scroll)

        # Right panel
        right_splitter = QSplitter(Qt.Vertical)
        self.waveform_canvas = WaveformCanvas()
        self.metrics_canvas = MetricsCanvas()
        right_splitter.addWidget(self.waveform_canvas)
        right_splitter.addWidget(self.metrics_canvas)
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 1)
        main_layout.addWidget(right_splitter, stretch=1)

        # Status bar
        self.statusBar().showMessage("Ready")

    def _make_spin(self, label, lo, hi, decimals, step, default, layout):
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setFixedWidth(95)
        row.addWidget(lbl)
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(default)
        spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        row.addWidget(spin)
        layout.addLayout(row)
        return spin

    def _apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #1a1a2e; color: #e0e0e0; }
            QGroupBox { border: 1px solid #444466; border-radius: 4px;
                        margin-top: 6px; padding-top: 10px; color: #aaa; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
            QPushButton { background: #2d3a5c; color: #e0e0e0; border: 1px solid #444466;
                          border-radius: 3px; padding: 3px 8px; }
            QPushButton:hover { background: #3d4a7c; }
            QPushButton:disabled { background: #222; color: #555; }
            QDoubleSpinBox, QSpinBox, QLineEdit {
                background: #0d1117; color: #c9d1d9; border: 1px solid #444466;
                border-radius: 2px; padding: 2px 4px; }
            QLabel { color: #c0c0c0; }
            QStatusBar { color: #aaa; }
            QSplitter::handle { background: #444466; height: 3px; }
        """)

    # ---- Slots ----

    def _make_config(self) -> TuningConfig:
        cfg = TuningConfig()
        cfg.target_overshoot = self.spin_tgt_os.value()
        cfg.target_undershoot = self.spin_tgt_us.value()
        cfg.max_oscillations = int(self.spin_max_osc.value())
        cfg.max_iterations = int(self.spin_max_iter.value())
        cfg.wc_initial = self.spin_wc.value()
        cfg.phi_m_initial = math.radians(self.spin_phim.value())
        return cfg

    def _start_worker(self, config: TuningConfig):
        self._cleanup_worker()
        self._thread = QThread()
        self._worker = TunerWorker(config)
        self._worker.moveToThread(self._thread)
        self._worker.iteration_complete.connect(self.on_iteration_complete)
        self._worker.tuning_finished.connect(self.on_tuning_finished)
        self._worker.log_message.connect(self.on_log)
        self._worker.error_occurred.connect(self.on_error)
        self._run_single_sig.connect(self._worker.run_single)
        self._thread.start()

    def _cleanup_worker(self):
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(3000)
        self._worker = None
        self._thread = None

    def on_start_auto_tune(self):
        if self._auto_tune_completed:
            QMessageBox.information(
                self,
                "Auto-Tune Completed",
                "Auto-tune has already completed. To start over, click Reset to Defaults first.",
            )
            self.statusBar().showMessage("Auto-tune already completed.")
            return

        cfg = self._make_config()
        self._results.clear()
        self._waveform_history.clear()
        self._iter_counter = 0
        self.waveform_canvas._draw_empty(
            target_os=cfg.target_overshoot,
            target_us=cfg.target_undershoot)
        self.metrics_canvas._draw_empty()
        self.log_text.clear()

        self._run_mode = "auto"
        self._start_worker(cfg)
        self._set_running(True)
        # Invoke in worker thread
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(self._worker, "run_auto_tune", Qt.QueuedConnection)

    def on_run_single(self):
        if self._worker is None:
            cfg = self._make_config()
            self._start_worker(cfg)

        Kp = self.spin_kp.value()
        Ki = self.spin_ki.value()
        Kd = self.spin_kd.value()
        Kf = self.spin_kf.value()
        self._run_mode = "single"
        self._set_running(True)
        self._run_single_sig.emit(Kp, Ki, Kd, Kf, self._iter_counter)

    def on_pause(self):
        if self._worker:
            self._worker.pause()
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(True)
            self.statusBar().showMessage("Paused")
            self.on_log("Paused by user.")

    def on_resume(self):
        if self._worker:
            self._worker.resume()
            self.btn_pause.setEnabled(True)
            self.btn_resume.setEnabled(False)
            self.statusBar().showMessage("Resumed")
            self.on_log("Resumed.")

    def on_stop(self):
        if self._worker:
            self._worker.stop()
            self.statusBar().showMessage("Stopping...")

    def on_reset(self):
        """Stop any running tune and reset everything to defaults."""
        self._cleanup_worker()

        self._results.clear()
        self._waveform_history.clear()
        self._iter_counter = 0
        self._run_mode = None
        self._auto_tune_completed = False

        # Reset spinboxes to defaults
        cfg = TuningConfig()
        comp = CompensatorDesign()
        ref_Kp, ref_Ki, ref_Kd, ref_Kf = comp.reference_design()
        self.spin_kp.setValue(ref_Kp)
        self.spin_ki.setValue(ref_Ki)
        self.spin_kd.setValue(ref_Kd)
        self.spin_kf.setValue(ref_Kf)
        self.spin_wc.setValue(cfg.wc_initial)
        self.spin_phim.setValue(60.0)
        self.spin_tgt_os.setValue(cfg.target_overshoot)
        self.spin_tgt_us.setValue(cfg.target_undershoot)
        self.spin_max_osc.setValue(cfg.max_oscillations)
        self.spin_max_iter.setValue(cfg.max_iterations)

        # Clear plots
        self.waveform_canvas._draw_empty(
            target_os=cfg.target_overshoot,
            target_us=cfg.target_undershoot)
        self.metrics_canvas._draw_empty()

        self.log_text.clear()
        self._set_running(False)
        self.statusBar().showMessage("Reset to defaults.")

    def on_compute_pid(self):
        wc = self.spin_wc.value()
        phi_m = math.radians(self.spin_phim.value())
        comp = CompensatorDesign()
        Kp, Ki, Kd, Kf = comp.compute(wc, phi_m)
        self.spin_kp.setValue(Kp)
        self.spin_ki.setValue(Ki)
        self.spin_kd.setValue(Kd)
        self.spin_kf.setValue(Kf)
        self.on_log(f"Computed: Kp={Kp:.5f} Ki={Ki:.2f} Kd={Kd:.2e} Kf={Kf:.0f}")

    def on_capture_circuit(self):
        # Try PLECS RPC image capture first, then fallback to file dialog
        try:
            import xmlrpc.client
            server = xmlrpc.client.ServerProxy('http://127.0.0.1:1080/RPC2', allow_none=True)
            # Try various RPC calls for image export
            try:
                res = server.plecs.webserver('getImage', 'synchronous buck', {})
                if res and isinstance(res, (bytes, xmlrpc.client.Binary)):
                    img_data = res.data if hasattr(res, 'data') else res
                    pixmap = QPixmap()
                    pixmap.loadFromData(img_data)
                    if not pixmap.isNull():
                        self.circuit_label.setPixmap(
                            pixmap.scaled(320, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                        self.on_log("Circuit image captured from PLECS.")
                        return
            except Exception:
                pass
        except Exception:
            pass

        # Fallback: file dialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Select circuit screenshot", "", "Images (*.png *.jpg *.bmp)")
        if path:
            pixmap = QPixmap(path)
            self.circuit_label.setPixmap(
                pixmap.scaled(320, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.on_log(f"Loaded circuit image: {path}")

    def on_save_gif(self):
        results_dir = TuningConfig().results_dir
        out_path = str(Path(results_dir) / "animation.gif")
        self.on_log("Generating animation GIF...")
        self.statusBar().showMessage("Generating GIF...")

        def _gen():
            try:
                plot_animation(out_path)
                return out_path
            except Exception as e:
                return str(e)

        import concurrent.futures
        future = concurrent.futures.ThreadPoolExecutor(max_workers=1).submit(_gen)
        future.add_done_callback(
            lambda f: QMetaObject.invokeMethod(
                self, "_on_gif_done", Qt.QueuedConnection,
                Q_ARG(str, f.result())))

    @pyqtSlot(str)
    def _on_gif_done(self, result):
        self.on_log(f"GIF saved: {result}")
        self.statusBar().showMessage(f"GIF saved: {result}")

    @pyqtSlot(object)
    def on_iteration_complete(self, data: dict):
        result = data['result']
        self._results.append(result)
        self._waveform_history.append(data)
        self._iter_counter = result.iter_num + 1

        # Update PID fields
        self.spin_kp.setValue(result.Kp)
        self.spin_ki.setValue(result.Ki)
        self.spin_kd.setValue(result.Kd)
        self.spin_kf.setValue(result.Kf)

        # Update plots
        idx = len(self._waveform_history) - 1
        if data['time'] and data['vout']:
            self.waveform_canvas.update_plot(
                self._waveform_history, idx,
                target_os=self.spin_tgt_os.value(),
                target_us=self.spin_tgt_us.value())
        self.metrics_canvas.update_metrics(self._results)

        # Status bar
        self.statusBar().showMessage(
            f"Iter {result.iter_num} — {result.status} — "
            f"OS={result.overshoot:.1f}% US={result.undershoot:.1f}% "
            f"Osc={result.osc_count}")

    def _show_best_iteration(self):
        """Switch plots and parameter fields to the best iteration found."""
        best = select_best_result(self._results)
        if best is None:
            return
        best_idx = next(
            (idx for idx, item in enumerate(self._waveform_history)
             if item['result'].iter_num == best.iter_num),
            None,
        )
        if best_idx is None:
            return

        self.spin_kp.setValue(best.Kp)
        self.spin_ki.setValue(best.Ki)
        self.spin_kd.setValue(best.Kd)
        self.spin_kf.setValue(best.Kf)
        self.waveform_canvas.update_plot(
            self._waveform_history, best_idx,
            target_os=self.spin_tgt_os.value(),
            target_us=self.spin_tgt_us.value())
        self.metrics_canvas.update_metrics(self._results)
        self.statusBar().showMessage(
            f"Best iter {best.iter_num} | {best.status} | "
            f"OS={best.overshoot:.2f}% US={best.undershoot:.2f}% Osc={best.osc_count}"
        )
        self.on_log(
            f"Showing best iteration {best.iter_num}: "
            f"OS={best.overshoot:.2f}% US={best.undershoot:.2f}% Osc={best.osc_count}"
        )

    @pyqtSlot(bool)
    def on_tuning_finished(self, success: bool):
        self._set_running(False)
        if self._run_mode == "auto":
            self._auto_tune_completed = True
            self._show_best_iteration()
        if success:
            self.on_log("=== SEARCH COMPLETE: best iteration meets target ===")
        else:
            self.on_log("=== SEARCH COMPLETE: showing best iteration found ===")
        self._run_mode = None
        self._cleanup_worker()

    @pyqtSlot(str)
    def on_log(self, msg: str):
        self.log_text.append(msg)

    @pyqtSlot(str)
    def on_error(self, msg: str):
        self.log_text.append(f"ERROR: {msg}")
        self.statusBar().showMessage(f"Error: {msg}")
        QMessageBox.critical(self, "Error", msg)
        self._set_running(False)

    def _set_running(self, running: bool):
        self.btn_start.setEnabled(not running)
        self.btn_single.setEnabled(not running)
        self.btn_pause.setEnabled(running)
        self.btn_resume.setEnabled(False)
        self.btn_stop.setEnabled(running)
        self.spin_kp.setReadOnly(running)
        self.spin_ki.setReadOnly(running)
        self.spin_kd.setReadOnly(running)
        self.spin_kf.setReadOnly(running)

    def closeEvent(self, event):
        self._cleanup_worker()
        event.accept()


# Need to import these for QMetaObject invocation
from PyQt5.QtCore import QMetaObject, Q_ARG


def main():
    app = QApplication(sys.argv)
    window = BuckTunerGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
