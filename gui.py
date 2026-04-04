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
import time
import shutil
import re
from pathlib import Path
from typing import List, Tuple, Optional, Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QCheckBox,
    QTextEdit, QSplitter, QFileDialog, QMessageBox, QLineEdit, QToolButton,
    QProgressBar, QSizePolicy, QScrollArea
)
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, Qt, QSize, QTimer
from PyQt5.QtGui import QPixmap, QImage, QFont, QColor, QPalette

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator

from auto_tune import (
    TuningConfig, AutoTuner, CompensatorDesign, TuningResult,
    PlecsRpc, PidTuner, ScopeCsvParser, ResponseAnalyzer, select_best_result
)
from analyze import plot_animation
from bode_plot import BodeResult, draw_bode_axes, run_loop_gain_analysis
from iteration_export import (
    copy_best_frame,
    save_iteration_frame,
    write_bode_workbook,
    write_time_workbook,
)


# ---------------------------------------------------------------------------
# Matplotlib Canvases
# ---------------------------------------------------------------------------

class WaveformCanvas(FigureCanvasQTAgg):
    """Live waveform plot with dark theme, ghost traces, and OS/US bands."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 4), dpi=90)
        self.fig.patch.set_facecolor('#141416')
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.history: List[dict] = []  # [{result, time, vout}, ...]
        self._draw_empty()

    @staticmethod
    def _moving_average(values: List[float], window: int = 50) -> List[float]:
        if not values:
            return []
        window = max(1, min(window, len(values)))
        out: List[float] = []
        running = 0.0
        buf: List[float] = []
        for val in values:
            buf.append(val)
            running += val
            if len(buf) > window:
                running -= buf.pop(0)
            out.append(running / len(buf))
        return out

    def _draw_empty(self, target_os: float = 5.0, target_us: float = 5.0):
        v_tgt = 5.0
        v_os = v_tgt * (1 + target_os / 100)
        v_us = v_tgt * (1 - target_us / 100)
        ax = self.ax
        ax.clear()
        ax.set_facecolor('#1c1c1f')
        ax.axhspan(v_tgt, v_os, color='#f05050', alpha=0.10)
        ax.axhspan(v_us, v_tgt, color='#e0a030', alpha=0.10)
        ax.axhline(y=v_os, color='#f05050', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_us, color='#e0a030', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_tgt, color='#707076', linestyle=':', lw=1.0, alpha=0.7)
        ax.set_xlim(0, 10)
        ax.set_ylim(4.4, 5.6)
        ax.set_xlabel('Time (ms)', color='#9a9aa0', fontsize=10)
        ax.set_ylabel('Output Voltage (V)', color='#9a9aa0', fontsize=10)
        ax.set_title('Waiting for iteration...', color='#707076', fontsize=11)
        ax.tick_params(colors='#9a9aa0')
        for sp in ax.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax.grid(True, alpha=0.2, color='#3a3a40')
        self.fig.tight_layout()
        self.draw()

    def update_plot(self, history: List[dict], current_idx: int,
                    target_os: float = 5.0, target_us: float = 5.0):
        v_tgt = 5.0
        v_os = v_tgt * (1 + target_os / 100)
        v_us = v_tgt * (1 - target_us / 100)
        ax = self.ax
        ax.clear()
        ax.set_facecolor('#1c1c1f')

        # OS/US bands with dynamic labels
        ax.axhspan(v_tgt, v_os, color='#f05050', alpha=0.10,
                   label=f'{target_os:.1f}% OS limit ({v_os:.3f}V)')
        ax.axhspan(v_us, v_tgt, color='#e0a030', alpha=0.10,
                   label=f'{target_us:.1f}% US limit ({v_us:.3f}V)')
        ax.axhline(y=v_os, color='#f05050', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_us, color='#e0a030', linestyle='--', lw=0.8, alpha=0.6)
        ax.axhline(y=v_tgt, color='#707076', linestyle=':', lw=1.0, alpha=0.7)

        # Ghost traces
        for i in range(current_idx):
            entry = history[i]
            t_ms = [t * 1000 for t in entry['time']]
            gc = '#00d4aa' if entry['result'].status == 'PASS' else '#5b8af0'
            ax.plot(t_ms, entry['vout'], color=gc, lw=0.6, alpha=0.20)

        # Current waveform
        cur = history[current_idx]
        r = cur['result']
        sc = '#00d4aa' if r.status == 'PASS' else '#f05050'
        t_ms = [t * 1000 for t in cur['time']]
        vout = cur['vout']
        ax.plot(t_ms, vout, color=sc, lw=2.0, label=f'Iter {r.iter_num} ({r.status})', zorder=5)
        vout_filt = self._moving_average(vout, 50)
        ax.plot(
            t_ms,
            vout_filt,
            color='#7fb3ff',
            lw=2.4,
            alpha=1.0,
            linestyle='--',
            dashes=(6, 3),
            label='50-sample moving average',
            zorder=7,
        )

        # Peak / valley markers
        if vout:
            v_peak = max(vout)
            v_valley = min(vout)
            ax.scatter([t_ms[vout.index(v_peak)]], [v_peak], color='#f05050', s=50, zorder=6)
            ax.scatter([t_ms[vout.index(v_valley)]], [v_valley], color='#e0a030', s=50, zorder=6)

        ax.set_xlim(min(t_ms), max(t_ms))
        ax.set_ylim(4.4, 5.6)
        ax.set_xlabel('Time (ms)', color='#9a9aa0', fontsize=10)
        ax.set_ylabel('Output Voltage (V)', color='#9a9aa0', fontsize=10)
        ax.set_title(
            f"Iter {r.iter_num}  -  Kp={r.Kp:.4f}  Ki={r.Ki:.1f}  "
            f"Kd={r.Kd:.2e}  Kf={r.Kf:.0f}\n"
            f"OS={r.overshoot:.1f}%  US={r.undershoot:.1f}%  "
            f"Osc={r.osc_count}  Ts={r.settling_time*1000:.3f} ms  ->  {r.status}",
            color=sc, fontsize=10, fontweight='bold')
        ax.tick_params(colors='#9a9aa0')
        for sp in ax.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax.legend(loc='upper right', facecolor='#141416', edgecolor='#2a2a2e',
                  labelcolor='#9a9aa0', fontsize=8)
        ax.grid(True, alpha=0.2, color='#3a3a40')
        self.fig.tight_layout()
        self.draw()


class BodeCanvas(FigureCanvasQTAgg):
    """Loop-gain bode plot with fc / PM / GM markers."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(7.2, 4), dpi=90)
        self.fig.patch.set_facecolor('#141416')
        self.ax_mag, self.ax_phase = self.fig.subplots(2, 1, sharex=True)
        super().__init__(self.fig)
        self.setParent(parent)
        self._draw_empty()

    def _draw_empty(self):
        draw_bode_axes(self.ax_mag, self.ax_phase, None, "Loop Gain Bode")
        self.fig.tight_layout()
        self.draw()

    def update_bode(self, bode: Optional[BodeResult], iter_num: Optional[int] = None):
        title = "Loop Gain Bode"
        if iter_num is not None:
            title = f"Loop Gain Bode\nIter {iter_num}"
        draw_bode_axes(self.ax_mag, self.ax_phase, bode, title)
        self.fig.tight_layout()
        self.draw()


class MetricsCanvas(FigureCanvasQTAgg):
    """OS/US line chart, oscillation bar chart, and settling-time trend."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 3.1), dpi=90)
        self.fig.patch.set_facecolor('#141416')
        self.ax_os, self.ax_osc, self.ax_ts = self.fig.subplots(1, 3)
        super().__init__(self.fig)
        self.setParent(parent)
        self._draw_empty()

    def _draw_empty(self):
        for ax in (self.ax_os, self.ax_osc, self.ax_ts):
            ax.clear()
            ax.set_facecolor('#1c1c1f')
            ax.tick_params(colors='#9a9aa0')
            for sp in ax.spines.values():
                sp.set_edgecolor('#2a2a2e')
            ax.grid(True, alpha=0.2, color='#3a3a40')
        self.ax_os.set_title('Overshoot / Undershoot', color='#9a9aa0', fontsize=9)
        self.ax_osc.set_title('Oscillations', color='#9a9aa0', fontsize=9)
        self.ax_ts.set_title('Settling Time', color='#9a9aa0', fontsize=9)
        self.fig.tight_layout()
        self.fig.subplots_adjust(hspace=0.45)
        self.draw()

    def update_metrics(self, results: List[TuningResult]):
        if not results:
            return
        iters = [r.iter_num for r in results]
        os_vals = [r.overshoot for r in results]
        us_vals = [r.undershoot for r in results]
        osc_vals = [r.osc_count for r in results]
        ts_vals = [r.settling_time * 1000.0 for r in results]

        ax = self.ax_os
        ax.clear()
        ax.set_facecolor('#1c1c1f')
        ax.plot(iters, os_vals, color='#f05050', marker='o', label='OS%', ms=4, lw=1.2)
        ax.plot(iters, us_vals, color='#e0a030', marker='s', label='US%', ms=4, lw=1.2)
        ax.axhline(y=5.0, color='#f05050', linestyle='--', alpha=0.5, lw=0.8)
        ax.set_xlabel('Iteration', color='#9a9aa0', fontsize=8)
        ax.set_ylabel('%', color='#9a9aa0', fontsize=8)
        ax.set_title('Overshoot / Undershoot', color='#9a9aa0', fontsize=9)
        ax.legend(facecolor='#141416', edgecolor='#2a2a2e', labelcolor='#9a9aa0', fontsize=7)
        ax.tick_params(colors='#9a9aa0', labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax.grid(True, alpha=0.2, color='#3a3a40')

        ax2 = self.ax_osc
        ax2.clear()
        ax2.set_facecolor('#1c1c1f')
        colors = ['#00d4aa' if o <= 2 else '#f05050' for o in osc_vals]
        ax2.bar(iters, osc_vals, color=colors, width=0.8)
        ax2.axhline(y=2, color='#707076', linestyle='--', lw=0.8)
        ax2.set_xlabel('Iteration', color='#9a9aa0', fontsize=8)
        ax2.set_ylabel('Count', color='#9a9aa0', fontsize=8)
        ax2.set_title('Oscillations', color='#9a9aa0', fontsize=9)
        ax2.tick_params(colors='#9a9aa0', labelsize=8)
        ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
        for sp in ax2.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax2.grid(True, alpha=0.2, color='#3a3a40')

        ax3 = self.ax_ts
        ax3.clear()
        ax3.set_facecolor('#1c1c1f')
        ax3.plot(iters, ts_vals, color='#5b8af0', marker='D', ms=4, lw=1.2)
        ax3.set_xlabel('Iteration', color='#9a9aa0', fontsize=8)
        ax3.set_ylabel('ms', color='#9a9aa0', fontsize=8)
        ax3.set_title('Settling Time', color='#9a9aa0', fontsize=9)
        ax3.tick_params(colors='#9a9aa0', labelsize=8)
        for sp in ax3.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax3.grid(True, alpha=0.2, color='#3a3a40')

        self.fig.tight_layout()
        self.fig.subplots_adjust(hspace=0.45)
        self.draw()


class CollapsibleSection(QWidget):
    """Simple click-to-toggle section for dense control panels."""

    def __init__(self, title: str, expanded: bool = True, parent=None):
        super().__init__(parent)
        self.toggle_button = QToolButton(self)
        self.toggle_button.setText(title)
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(expanded)
        self.toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.toggle_button.clicked.connect(self._on_toggled)

        self.content = QWidget(self)
        self.content.setVisible(expanded)
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(10, 4, 6, 6)
        self.content_layout.setSpacing(4)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.content)

    def _on_toggled(self, checked: bool) -> None:
        self.toggle_button.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        self.content.setVisible(checked)


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
        self._ui_update_interval_sec = 1.0
        self._last_ui_emit_ts = 0.0
        self._pending_iteration_payload: Optional[dict] = None
        self._pending_log_message: Optional[str] = None
        self._waveform_store: Dict[int, Dict] = {}
        self._bode_store: Dict[int, BodeResult] = {}

    def _run_bode_for_iteration(self, iter_num: int) -> Optional[BodeResult]:
        if not getattr(self.config, "run_bode_analysis", False):
            return None
        if self._auto_tuner is None or self._auto_tuner.plecs.server is None:
            return None
        bode = run_loop_gain_analysis(
            self._auto_tuner.plecs.server,
            self.config.model_id,
            getattr(self.config, "bode_freq_start_hz", 1e3),
            getattr(self.config, "bode_freq_stop_hz", 1e5),
            int(getattr(self.config, "bode_num_points", 31)),
        )
        fc_text = f"{bode.metrics.crossover_hz / 1000:.2f} kHz" if bode.metrics.crossover_hz is not None else "n/a"
        pm_text = f"{bode.metrics.phase_margin_deg:.1f} deg" if bode.metrics.phase_margin_deg is not None else "n/a"
        self.log_message.emit(
            f"Bode iter {iter_num}: fc={fc_text} PM={pm_text} | "
            f"time total={bode.elapsed_s:.1f}s coarse={bode.coarse_elapsed_s:.1f}s "
            f"dense={bode.dense_elapsed_s:.1f}s"
        )
        return bode

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def stop(self):
        self._stop_flag = True
        self._pause_event.set()  # unblock if paused

    @staticmethod
    def _waveform_from_data(header: List[str], rows: List[List[float]]):
        vout_col = 2
        for i, h in enumerate(header):
            if 'voltage' in h.lower() or 'vout' in h.lower():
                vout_col = i
                break
        time_vals, vout_vals, il_vals = [], [], []
        for vals in rows:
            time_vals.append(vals[0])
            il_vals.append(vals[1] if len(vals) > 1 else 0)
            vout_vals.append(vals[vout_col] if len(vals) > vout_col else 0)
        return time_vals, vout_vals, il_vals

    def _write_workbooks(self, results: List[TuningResult]) -> None:
        results_dir = Path(self.config.results_dir)
        results_dir.mkdir(parents=True, exist_ok=True)
        write_time_workbook(results_dir / "time_iterations.xlsx", results, self._waveform_store)
        if self._bode_store:
            write_bode_workbook(results_dir / "bode_iterations.xlsx", self._bode_store)

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

    def _queue_iteration_update(self, payload: dict, message: str) -> None:
        """Throttle GUI refreshes while keeping the newest iteration data."""
        self._pending_iteration_payload = payload
        self._pending_log_message = message
        now = time.monotonic()
        if now - self._last_ui_emit_ts >= self._ui_update_interval_sec:
            self._flush_iteration_update()

    def _flush_iteration_update(self) -> None:
        """Emit the latest queued iteration update to the GUI."""
        if self._pending_iteration_payload is not None:
            self.iteration_complete.emit(self._pending_iteration_payload)
            self._pending_iteration_payload = None
        if self._pending_log_message is not None:
            self.log_message.emit(self._pending_log_message)
            self._pending_log_message = None
        self._last_ui_emit_ts = time.monotonic()

    @pyqtSlot()
    def run_auto_tune(self):
        """Main auto-tuning loop with pause/stop support."""
        self._stop_flag = False
        self._last_ui_emit_ts = 0.0
        self._pending_iteration_payload = None
        self._pending_log_message = None
        self._waveform_store = {}
        self._bode_store = {}
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

                phase = getattr(at.tuner, "phase", "unknown")
                result = at.run_iteration(i, Kp, Ki, Kd, Kf)
                at.results.append(result)

                header = at.last_header
                data_rows = at.last_data
                t, v, il = self._waveform_from_data(header, data_rows)
                bode = self._run_bode_for_iteration(i)
                self._waveform_store[i] = {'header': header, 'data': data_rows}
                if bode is not None:
                    self._bode_store[i] = bode
                figures_dir = Path(self.config.results_dir)
                figures_dir.mkdir(parents=True, exist_ok=True)
                save_iteration_frame(
                    figures_dir / f"iter{i + 1}.png",
                    result,
                    t,
                    v,
                    bode,
                    self.config.target_overshoot,
                    self.config.target_undershoot,
                )
                payload = {
                    'result': result,
                    'time': t,
                    'vout': v,
                    'il': il,
                    'bode': bode,
                }

                msg = (
                    f"Time Iter {i}: phase={phase} | OS={result.overshoot:.1f}% "
                    f"US={result.undershoot:.1f}% Osc={result.osc_count} "
                    f"→ {result.status}\n"
                    f"Kp={result.Kp:.5f} Ki={result.Ki:.2f} "
                    f"Kd={result.Kd:.2e} Kf={result.Kf:.0f}"
                )
                self._queue_iteration_update(payload, msg)

                if i < self.config.max_iterations - 1:
                    Kp, Ki, Kd, Kf = at.tuner.adjust(
                        Kp, Ki, Kd, Kf,
                        result.overshoot, result.undershoot, result.osc_count, result.settling_time)

            self._flush_iteration_update()
            self._write_workbooks(at.results)
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

            phase = getattr(at.tuner, "phase", "unknown")
            result = at.run_iteration(iter_num, Kp, Ki, Kd, Kf)
            at.results.append(result)

            header = at.last_header
            data_rows = at.last_data
            t, v, il = self._waveform_from_data(header, data_rows)
            bode = self._run_bode_for_iteration(iter_num)
            self._waveform_store[iter_num] = {'header': header, 'data': data_rows}
            if bode is not None:
                self._bode_store[iter_num] = bode
            figures_dir = Path(self.config.results_dir)
            figures_dir.mkdir(parents=True, exist_ok=True)
            save_iteration_frame(
                figures_dir / f"iter{iter_num + 1}.png",
                result,
                t,
                v,
                bode,
                self.config.target_overshoot,
                self.config.target_undershoot,
            )
            self._write_workbooks(at.results)
            self.iteration_complete.emit({
                'result': result, 'time': t, 'vout': v, 'il': il, 'bode': bode,
            })
            msg = (
                f"Time Iter {iter_num}: phase={phase} | OS={result.overshoot:.1f}% "
                f"US={result.undershoot:.1f}% Osc={result.osc_count} "
                f"→ {result.status}\n"
                f"Kp={result.Kp:.5f} Ki={result.Ki:.2f} "
                f"Kd={result.Kd:.2e} Kf={result.Kf:.0f}\n\n"
            )
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
        w = min(1500, screen.width() - 40)
        h = min(940, screen.height() - 60)
        self.resize(w, h)

        self._results: List[TuningResult] = []
        self._waveform_history: List[dict] = []
        self._iter_counter = 0
        self._worker: Optional[TunerWorker] = None
        self._thread: Optional[QThread] = None
        self._run_mode: Optional[str] = None
        self._auto_tune_completed = False
        self._circuit_pixmap: Optional[QPixmap] = None
        self._results_root_dir = Path(TuningConfig().results_dir)
        self._current_results_dir: Optional[Path] = None
        self._model_sync_ready = False
        self._model_sync_timer = QTimer(self)
        self._model_sync_timer.setSingleShot(True)
        self._model_sync_timer.timeout.connect(self._sync_gui_to_plecs_file)

        self._build_ui()
        self._connect_model_sync_signals()
        self._model_sync_ready = True
        self._queue_model_sync()
        self._apply_dark_theme()

    # ---- UI Construction ----

    def _build_ui(self):
        cfg = TuningConfig()
        comp = CompensatorDesign()
        init_Kp, init_Ki, init_Kd, init_Kf = comp.compute(cfg.wc_initial, cfg.phi_m_initial)
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
        self.circuit_label.setMinimumHeight(180)
        self.circuit_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.circuit_label.setStyleSheet("background: #1c1c1f; color: #707076;")
        img_layout.addWidget(self.circuit_label)
        btn_capture = QPushButton("Capture from PLECS")
        btn_capture.clicked.connect(self.on_capture_circuit)
        img_layout.addWidget(btn_capture)
        left_layout.addWidget(grp_img)

        # PID parameters
        grp_pid = CollapsibleSection("PID Parameters", expanded=True)
        pid_layout = grp_pid.content_layout
        self.spin_kp = self._make_spin("Kp:", 0, 10, 6, 0.001, init_Kp, pid_layout)
        self.spin_ki = self._make_spin("Ki:", 0, 100000, 2, 100, init_Ki, pid_layout)
        self.spin_kd = self._make_spin("Kd:", 0, 0.01, 9, 1e-7, init_Kd, pid_layout)
        self.spin_kf = self._make_spin("Kf:", 0, 2000000, 0, 1000, init_Kf, pid_layout)
        left_layout.addWidget(grp_pid)

        # Design variables
        grp_dv = CollapsibleSection("Design Variables", expanded=True)
        dv_layout = grp_dv.content_layout
        self.spin_wc = self._make_spin("wc (rad/s):", 47140, 314159, 0, 1000, cfg.wc_initial, dv_layout)
        self.spin_phim = self._make_spin("phi_m (deg):", 30, 80, 1, 1, math.degrees(cfg.phi_m_initial), dv_layout)
        btn_compute = QPushButton("Compute PID from wc / phi_m")
        btn_compute.clicked.connect(self.on_compute_pid)
        dv_layout.addWidget(btn_compute)
        left_layout.addWidget(grp_dv)

        # Targets
        grp_tgt = CollapsibleSection("Targets", expanded=True)
        tgt_layout = grp_tgt.content_layout
        self.spin_tgt_os = self._make_spin("Target OS%:", 0, 50, 1, 0.5, 4.0, tgt_layout)
        self.spin_tgt_us = self._make_spin("Target US%:", 0, 50, 1, 0.5, 4.0, tgt_layout)
        self.spin_max_osc = self._make_spin("Max Osc:", 0, 20, 0, 1, 0, tgt_layout)
        self.spin_tgt_settle = self._make_spin("Max Ts (ms):", 0.01, 10, 3, 0.05, cfg.target_settling_time * 1000, tgt_layout)
        self.spin_max_iter = self._make_spin("Max Iter:", 1, 200, 0, 5, 50, tgt_layout)
        left_layout.addWidget(grp_tgt)

        grp_bode = CollapsibleSection("Bode Analysis", expanded=True)
        bode_layout = grp_bode.content_layout
        self.chk_run_bode = QCheckBox("Run bode plot analysis")
        self.chk_run_bode.setChecked(True)
        bode_layout.addWidget(self.chk_run_bode)
        self.spin_bode_f_start = self._make_spin("Start f (Hz):", 10, 1e7, 0, 100, 1000, bode_layout)
        self.spin_bode_f_stop = self._make_spin("Stop f (Hz):", 100, 1e7, 0, 1000, 100000, bode_layout)
        self.spin_bode_points = self._make_spin("Points:", 5, 500, 0, 1, 31, bode_layout)
        left_layout.addWidget(grp_bode)

        # Controls
        grp_ctrl = QGroupBox("Controls")
        ctrl_layout = QVBoxLayout(grp_ctrl)

        self.btn_start = QPushButton("Start Auto-Tune")
        self.btn_start.clicked.connect(self.on_start_auto_tune)
        self.btn_start.setStyleSheet(
            "QPushButton { background: #0a3a30; color: #e8e8eb; border: 1px solid #00d4aa; }"
            "QPushButton:hover { background: #0f4a3d; }"
            "QPushButton:pressed { background: #082a22; }"
            "QPushButton:disabled { background: #1a1a1c; color: #505055; border: 1px solid #2a2a2e; }"
        )
        ctrl_layout.addWidget(self.btn_start)

        self.btn_single = QPushButton("Run Single Iteration")
        self.btn_single.clicked.connect(self.on_run_single)
        self.btn_single.setStyleSheet(
            "QPushButton { background: #1a2540; color: #e8e8eb; border: 1px solid #5b8af0; }"
            "QPushButton:hover { background: #223055; }"
            "QPushButton:pressed { background: #141d30; }"
            "QPushButton:disabled { background: #1a1a1c; color: #505055; border: 1px solid #2a2a2e; }"
        )
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
            "QPushButton { background: #2a1518; color: #f07070; border: 1px solid #4a2025; }"
            "QPushButton:hover { background: #3a2028; }")
        ctrl_layout.addWidget(self.btn_reset)

        left_layout.addWidget(grp_ctrl)

        # Log
        grp_log = QGroupBox("Log")
        log_layout = QVBoxLayout(grp_log)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(150)
        self.log_text.setStyleSheet("background: #111113; color: #c0c0c4; font-size: 9.5pt;")
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(grp_log)

        left_scroll = QScrollArea()
        left_scroll.setWidget(left_inner)
        left_scroll.setWidgetResizable(True)
        left_scroll.setFixedWidth(350)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setFrameShape(left_scroll.NoFrame)
        main_layout.addWidget(left_scroll)

        # Right panel
        right_splitter = QSplitter(Qt.Vertical)
        top_splitter = QSplitter(Qt.Horizontal)
        self.waveform_canvas = WaveformCanvas(top_splitter)
        self.bode_canvas = BodeCanvas(top_splitter)
        self.metrics_canvas = MetricsCanvas(right_splitter)
        top_splitter.addWidget(self.waveform_canvas)
        top_splitter.addWidget(self.bode_canvas)
        top_splitter.setStretchFactor(0, 3)
        top_splitter.setStretchFactor(1, 2)
        right_splitter.addWidget(top_splitter)
        right_splitter.addWidget(self.metrics_canvas)
        top_splitter.setChildrenCollapsible(False)
        right_splitter.setChildrenCollapsible(False)
        right_splitter.setCollapsible(0, False)
        right_splitter.setCollapsible(1, False)
        top_splitter.setMinimumHeight(360)
        self.metrics_canvas.setMinimumHeight(220)
        self.metrics_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.MinimumExpanding)
        right_splitter.setStretchFactor(0, 5)
        right_splitter.setStretchFactor(1, 3)
        right_splitter.setSizes([700, 240])
        main_layout.addWidget(right_splitter, stretch=1)

        # Status bar
        self.statusBar().showMessage("Ready")
        QTimer.singleShot(0, self._load_default_circuit_image)

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

    def _set_circuit_pixmap(self, pixmap: QPixmap) -> None:
        """Render the circuit image fully inside the preview area."""
        if pixmap.isNull():
            return
        self._circuit_pixmap = pixmap
        target_width = max(1, self.circuit_label.contentsRect().width())
        if target_width <= 1:
            target_width = max(1, self.circuit_label.width())
        scaled = pixmap.scaledToWidth(target_width, Qt.SmoothTransformation)
        self.circuit_label.setMinimumHeight(scaled.height())
        self.circuit_label.setMaximumHeight(scaled.height())
        self.circuit_label.setMinimumSize(QSize(target_width, scaled.height()))
        scaled = pixmap.scaled(
            QSize(target_width, scaled.height()),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.circuit_label.setPixmap(scaled)

    def _load_default_circuit_image(self) -> None:
        """Load the checked-in circuit screenshot if it exists."""
        image_path = Path(__file__).resolve().parent / "synchronous buck.png"
        if not image_path.exists():
            return
        pixmap = QPixmap(str(image_path))
        if pixmap.isNull():
            return
        self._set_circuit_pixmap(pixmap)

    def _apply_dark_theme(self):
        self.setStyleSheet("""
            * { font-family: "Segoe UI", "Inter", sans-serif; }
            QMainWindow, QWidget { background: #141416; color: #e8e8eb; }
            QGroupBox { border: 1px solid #2a2a2e; border-radius: 4px;
                        margin-top: 6px; padding-top: 10px; color: #707076; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
            QPushButton { background: #1f1f23; color: #e8e8eb; border: 1px solid #3a3a40;
                          border-radius: 4px; padding: 5px 12px; }
            QPushButton:hover { background: #2a2a30; }
            QPushButton:pressed { background: #18181b; }
            QPushButton:disabled { background: #1a1a1c; color: #505055; }
            QToolButton { background: #1f1f23; color: #e8e8eb; border: 1px solid #2a2a2e;
                          border-radius: 4px; padding: 5px 8px; text-align: left;
                          font-weight: 600; }
            QToolButton:hover { background: #2a2a30; border: 1px solid #3a3a40; }
            QDoubleSpinBox, QSpinBox, QLineEdit {
                background: #111113; color: #e8e8eb; border: 1px solid #2a2a2e;
                border-radius: 3px; padding: 3px 5px; }
            QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {
                border: 1px solid #00d4aa; }
            QLabel { color: #9a9aa0; }
            QStatusBar { color: #707076; }
            QSplitter::handle { background: #2a2a2e; height: 3px; }
            QScrollBar:vertical { background: #141416; width: 8px; border: none; }
            QScrollBar::handle:vertical { background: #2a2a2e; border-radius: 4px;
                                          min-height: 20px; }
            QScrollBar::handle:vertical:hover { background: #3a3a40; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
        """)

    # ---- Slots ----

    def _make_config(self) -> TuningConfig:
        cfg = TuningConfig()
        cfg.target_overshoot = self.spin_tgt_os.value()
        cfg.target_undershoot = self.spin_tgt_us.value()
        cfg.max_oscillations = int(self.spin_max_osc.value())
        cfg.target_settling_time = self.spin_tgt_settle.value() / 1000.0
        cfg.max_iterations = int(self.spin_max_iter.value())
        cfg.wc_initial = self.spin_wc.value()
        cfg.phi_m_initial = math.radians(self.spin_phim.value())
        cfg.run_bode_analysis = self.chk_run_bode.isChecked()
        cfg.bode_freq_start_hz = self.spin_bode_f_start.value()
        cfg.bode_freq_stop_hz = self.spin_bode_f_stop.value()
        cfg.bode_num_points = int(self.spin_bode_points.value())
        if self._current_results_dir is not None:
            cfg.results_dir = str(self._current_results_dir)
        return cfg

    def _connect_model_sync_signals(self) -> None:
        widgets = [
            self.spin_kp, self.spin_ki, self.spin_kd, self.spin_kf,
            self.spin_wc, self.spin_phim,
            self.spin_tgt_os, self.spin_tgt_us, self.spin_max_osc,
            self.spin_tgt_settle, self.spin_max_iter,
            self.spin_bode_f_start, self.spin_bode_f_stop, self.spin_bode_points,
        ]
        for widget in widgets:
            widget.valueChanged.connect(self._queue_model_sync)
        self.chk_run_bode.toggled.connect(self._queue_model_sync)

    def _queue_model_sync(self, *_args) -> None:
        if not self._model_sync_ready:
            return
        self._model_sync_timer.start(150)

    @staticmethod
    def _replace_analysis_param(content: str, analysis_name: str, variable: str, value: str) -> str:
        pattern = (
            rf'(?ms)(Analysis\s*\{{.*?Name\s+"{re.escape(analysis_name)}".*?^\s*'
            rf'{re.escape(variable)}\s+)\"[^\"]*\"'
        )
        return re.sub(pattern, rf'\1"{value}"', content, count=1)

    @staticmethod
    def _build_gui_metadata_block(values: Dict[str, str]) -> str:
        lines = [
            "% GUI_METADATA_BEGIN",
            *[f"% {key} = {value}" for key, value in values.items()],
            "% GUI_METADATA_END",
        ]
        return "\\n".join(lines)

    def _sync_gui_to_plecs_file(self) -> None:
        model_path = Path(TuningConfig().plecs_model)
        if not model_path.exists():
            return

        try:
            content = model_path.read_text(encoding="utf-8")
            content = re.sub(r'Ki\s*=\s*[\d.eE+-]+;', f'Ki = {self.spin_ki.value():.12g};', content)
            content = re.sub(r'Kf\s*=\s*[\d.eE+-]+;', f'Kf = {self.spin_kf.value():.12g};', content)
            content = re.sub(r'Kd\s*=\s*[\d.eE+-]+;', f'Kd = {self.spin_kd.value():.12g};', content)
            content = re.sub(r'Kp\s*=\s*[\d.eE+-]+;', f'Kp = {self.spin_kp.value():.12g};', content)

            freq_range = f"[{self.spin_bode_f_start.value():.12g} {self.spin_bode_f_stop.value():.12g}]"
            content = self._replace_analysis_param(
                content, "Loop Gain (Frequency Response)", "FrequencyRange", freq_range
            )
            content = self._replace_analysis_param(
                content, "Loop Gain (Frequency Response)", "NumPoints", str(int(self.spin_bode_points.value()))
            )

            meta_values = {
                "wc_rad_per_s": f"{self.spin_wc.value():.12g}",
                "phi_m_deg": f"{self.spin_phim.value():.12g}",
                "target_os_pct": f"{self.spin_tgt_os.value():.12g}",
                "target_us_pct": f"{self.spin_tgt_us.value():.12g}",
                "max_osc": f"{int(self.spin_max_osc.value())}",
                "max_ts_ms": f"{self.spin_tgt_settle.value():.12g}",
                "max_iter": f"{int(self.spin_max_iter.value())}",
                "run_bode_analysis": "1" if self.chk_run_bode.isChecked() else "0",
                "bode_start_hz": f"{self.spin_bode_f_start.value():.12g}",
                "bode_stop_hz": f"{self.spin_bode_f_stop.value():.12g}",
                "bode_points": f"{int(self.spin_bode_points.value())}",
            }
            metadata = self._build_gui_metadata_block(meta_values)
            script_pattern = r'(?ms)(Script\s*\{\s*Name\s+"Script"\s*Script\s+)\"[^\"]*\"'
            if re.search(script_pattern, content):
                content = re.sub(script_pattern, rf'\1"{metadata}"', content, count=1)

            model_path.write_text(content, encoding="utf-8")
        except Exception:
            # Keep GUI responsive even if metadata sync fails.
            pass

    def _prune_old_run_folders(self) -> None:
        self._results_root_dir.mkdir(parents=True, exist_ok=True)
        run_dirs = sorted(
            [p for p in self._results_root_dir.iterdir() if p.is_dir() and p.name.startswith("figures_")],
            key=lambda p: p.stat().st_mtime,
        )
        while len(run_dirs) > 5:
            oldest = run_dirs.pop(0)
            shutil.rmtree(oldest, ignore_errors=True)

    def _prepare_run_results_dir(self) -> Path:
        self._results_root_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("figures_%m%d_%H%M")
        run_dir = self._results_root_dir / timestamp
        suffix = 1
        while run_dir.exists():
            run_dir = self._results_root_dir / f"{timestamp}_{suffix:02d}"
            suffix += 1
        run_dir.mkdir(parents=True, exist_ok=True)
        self._current_results_dir = run_dir
        self._prune_old_run_folders()
        return run_dir

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

    def _clear_bode_results(self):
        self._prepare_run_results_dir()

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
        self._clear_bode_results()
        cfg.results_dir = str(self._current_results_dir)
        self.waveform_canvas._draw_empty(
            target_os=cfg.target_overshoot,
            target_us=cfg.target_undershoot)
        self.bode_canvas._draw_empty()
        self.metrics_canvas._draw_empty()
        self.log_text.clear()

        self._run_mode = "auto"
        self._start_worker(cfg)
        self._set_running(True)
        # Invoke in worker thread
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(self._worker, "run_auto_tune", Qt.QueuedConnection)

    def on_run_single(self):
        if self._iter_counter == 0:
            self._clear_bode_results()
        cfg = self._make_config()
        cfg.results_dir = str(self._current_results_dir)
        if self._worker is None:
            self._start_worker(cfg)
        else:
            self._worker.config = cfg

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
        self._current_results_dir = None

        # Reset spinboxes to defaults
        cfg = TuningConfig()
        comp = CompensatorDesign()
        ref_Kp, ref_Ki, ref_Kd, ref_Kf = comp.compute(cfg.wc_initial, cfg.phi_m_initial)
        self.spin_kp.setValue(ref_Kp)
        self.spin_ki.setValue(ref_Ki)
        self.spin_kd.setValue(ref_Kd)
        self.spin_kf.setValue(ref_Kf)
        self.spin_wc.setValue(cfg.wc_initial)
        self.spin_phim.setValue(math.degrees(cfg.phi_m_initial))
        self.spin_tgt_os.setValue(cfg.target_overshoot)
        self.spin_tgt_us.setValue(cfg.target_undershoot)
        self.spin_max_osc.setValue(cfg.max_oscillations)
        self.spin_tgt_settle.setValue(cfg.target_settling_time * 1000.0)
        self.spin_max_iter.setValue(cfg.max_iterations)
        self.chk_run_bode.setChecked(True)
        self.spin_bode_f_start.setValue(1000)
        self.spin_bode_f_stop.setValue(100000)
        self.spin_bode_points.setValue(31)

        # Clear plots
        self.waveform_canvas._draw_empty(
            target_os=cfg.target_overshoot,
            target_us=cfg.target_undershoot)
        self.bode_canvas._draw_empty()
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
                        self._set_circuit_pixmap(pixmap)
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
            self._set_circuit_pixmap(pixmap)
            self.on_log(f"Loaded circuit image: {path}")

    def on_save_gif(self):
        results_dir = self._current_results_dir or self._results_root_dir
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
        self.bode_canvas.update_bode(data.get('bode'), result.iter_num)
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
        self.bode_canvas.update_bode(self._waveform_history[best_idx].get('bode'), best.iter_num)
        self.metrics_canvas.update_metrics(self._results)
        self.statusBar().showMessage(
            f"Best iter {best.iter_num} | {best.status} | "
            f"OS={best.overshoot:.2f}% US={best.undershoot:.2f}% Osc={best.osc_count}"
        )
        self.on_log(
            f"Showing best iteration {best.iter_num}: "
            f"OS={best.overshoot:.2f}% US={best.undershoot:.2f}% Osc={best.osc_count}"
        )
        copy_best_frame(
            self._current_results_dir or self._results_root_dir,
            best.iter_num,
            (self._current_results_dir or self._results_root_dir) / "best_iteration.png",
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
        if msg.startswith("Bode iter") and self.log_text.toPlainText().strip():
            self.log_text.append("")
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
        self.chk_run_bode.setEnabled(not running)
        self.spin_bode_f_start.setReadOnly(running)
        self.spin_bode_f_stop.setReadOnly(running)
        self.spin_bode_points.setReadOnly(running)

    def closeEvent(self, event):
        self._cleanup_worker()
        event.accept()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._circuit_pixmap is not None:
            self._set_circuit_pixmap(self._circuit_pixmap)


# Need to import these for QMetaObject invocation
from PyQt5.QtCore import QMetaObject, Q_ARG


def main():
    app = QApplication(sys.argv)
    window = BuckTunerGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
