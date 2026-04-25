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
import time
import shutil
import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QCheckBox,
    QTextEdit, QSplitter, QFileDialog, QMessageBox, QLineEdit, QToolButton,
    QProgressBar, QSizePolicy, QScrollArea, QGridLayout, QTableWidget,
    QTableWidgetItem, QHeaderView, QAbstractItemView, QComboBox
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
from bode_plot import BodeResult, draw_bode_axes, run_loop_gain_analysis, run_ltspice_loop_gain_analysis
from iteration_export import (
    copy_best_frame,
    save_iteration_frame,
    write_bode_workbook,
    write_time_workbook,
)
from ltspice_backend import import_pyltspice


def validate_config_values(config: TuningConfig) -> List[str]:
    """Return user-facing validation errors before starting a run."""
    errors: List[str] = []
    backend = config.backend
    if backend == "plecs":
        if not Path(config.plecs_model).exists():
            errors.append(f"PLECS model not found: {config.plecs_model}")
        if config.plecs_exe and not Path(config.plecs_exe).exists():
            errors.append(f"PLECS executable not found: {config.plecs_exe}")
    elif backend == "ltspice":
        if not config.ltspice_exe or not Path(config.ltspice_exe).exists():
            errors.append(
                "LTspice executable not found. Set LTSPICE_EXE or choose the installed LTspice.exe in the GUI."
            )
        for label, path in (
            ("LTspice schematic", config.ltspice_asc_model),
            ("LTspice transient netlist", config.ltspice_netlist_model),
            ("LTspice Bode netlist", config.ltspice_bode_netlist_model),
        ):
            if not Path(path).exists():
                errors.append(f"{label} not found: {path}")
        try:
            import_pyltspice()
        except ImportError as exc:
            errors.append(str(exc))
    if config.max_iterations < 1:
        errors.append("Max Iter must be at least 1.")
    if config.target_overshoot < 0 or config.target_undershoot < 0:
        errors.append("Overshoot and undershoot targets must be non-negative.")
    if config.target_settling_time <= 0:
        errors.append("Settling-time target must be positive.")
    if config.run_bode_analysis:
        if config.bode_freq_stop_hz <= config.bode_freq_start_hz:
            errors.append("Bode Stop f (Hz) must be greater than Start f (Hz).")
        if config.bode_coarse_num_points < 5:
            errors.append("Bode point count must be at least 5.")
        if backend == "plecs":
            if config.bode_dense_num_points < 5:
                errors.append("Dense Bode point count must be at least 5.")
            if config.bode_extraction_cycles < 1:
                errors.append("Bode extraction cycles must be at least 1.")
    return errors


# ---------------------------------------------------------------------------
# Matplotlib Canvases
# ---------------------------------------------------------------------------

class WaveformCanvas(FigureCanvasQTAgg):
    """Live waveform plot with dark theme, ghost traces, and OS/US bands."""

    MAX_GHOST_TRACES = 10
    GHOST_RENDER_POINTS = 500
    CURRENT_RENDER_POINTS = 2500

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 4), dpi=90)
        self.fig.patch.set_facecolor('#141416')
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.history: List[dict] = []  # [{result, time, vout}, ...]
        self._draw_empty()

    def _apply_layout(self) -> None:
        width_px, height_px = self.fig.get_size_inches() * self.fig.dpi
        left = max(0.055, 58.0 / max(width_px, 1.0))
        right = 1.0 - max(0.010, 12.0 / max(width_px, 1.0))
        bottom = max(0.105, 42.0 / max(height_px, 1.0))
        top = 1.0 - max(0.105, 48.0 / max(height_px, 1.0))
        self.fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top)

    @staticmethod
    def _moving_average(values: List[float], window: int = 10) -> List[float]:
        if not values:
            return []
        window = max(1, min(window, len(values)))
        cumsum = [0.0]
        for value in values:
            cumsum.append(cumsum[-1] + value)
        half = window // 2
        out: List[float] = []
        for i in range(len(values)):
            lo = max(0, i - half)
            hi = min(len(values), i + half + 1)
            out.append((cumsum[hi] - cumsum[lo]) / (hi - lo))
        return out

    @staticmethod
    def _thin_xy(time_vals: List[float], y_vals: List[float], max_points: int) -> Tuple[List[float], List[float]]:
        n = min(len(time_vals), len(y_vals))
        if n <= max_points or max_points < 2:
            return [time_vals[i] * 1000.0 for i in range(n)], y_vals[:n]
        last_idx = n - 1
        idxs = [round(i * last_idx / (max_points - 1)) for i in range(max_points)]
        return [time_vals[i] * 1000.0 for i in idxs], [y_vals[i] for i in idxs]

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
        self._apply_layout()
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

        # Ghost traces are useful context, but plotting every LTspice point from
        # every prior iteration makes row selection feel sluggish.
        ghost_start = max(0, current_idx - self.MAX_GHOST_TRACES)
        for i in range(ghost_start, current_idx):
            entry = history[i]
            t_ms, ghost_vout = self._thin_xy(
                entry['time'],
                entry['vout'],
                self.GHOST_RENDER_POINTS,
            )
            gc = '#00d4aa' if entry['result'].status == 'PASS' else '#5b8af0'
            ax.plot(t_ms, ghost_vout, color=gc, lw=0.6, alpha=0.20)

        # Current waveform
        cur = history[current_idx]
        r = cur['result']
        sc = '#00d4aa' if r.status == 'PASS' else '#f05050'
        cur_time = cur['time']
        cur_vout = cur['vout']
        vout_filt_full = self._moving_average(cur_vout, 10)
        t_ms, vout = self._thin_xy(cur_time, cur_vout, self.CURRENT_RENDER_POINTS)
        filt_t_ms, vout_filt = self._thin_xy(cur_time, vout_filt_full, self.CURRENT_RENDER_POINTS)
        ax.plot(t_ms, vout, color=sc, lw=2.0, label=f'Iter {r.iter_num} ({r.status})', zorder=5)
        ax.plot(
            filt_t_ms,
            vout_filt,
            color='#7fb3ff',
            lw=2.4,
            alpha=1.0,
            linestyle='--',
            dashes=(6, 3),
            label='10-sample moving average',
            zorder=7,
        )

        # Peak / valley markers
        if cur_vout:
            v_peak = max(cur_vout)
            v_valley = min(cur_vout)
            peak_idx = cur_vout.index(v_peak)
            valley_idx = cur_vout.index(v_valley)
            ax.scatter([cur_time[peak_idx] * 1000.0], [v_peak], color='#f05050', s=50, zorder=6)
            ax.scatter([cur_time[valley_idx] * 1000.0], [v_valley], color='#e0a030', s=50, zorder=6)

        ax.set_xlim(min(t_ms), max(t_ms))
        ax.set_ylim(4.4, 5.6)
        ax.set_xlabel('Time (ms)', color='#9a9aa0', fontsize=10)
        ax.set_ylabel('Output Voltage (V)', color='#9a9aa0', fontsize=10)
        ax.set_title(
            f"Iter {r.iter_num}  -  Kp={r.Kp:.4f}  Ki={r.Ki:.1f}  "
            f"Kd={r.Kd:.2e}  Kf={r.Kf:.0f}\n"
            f"OS={r.overshoot:.1f}%  US={r.undershoot:.1f}%  "
            f"Osc={r.osc_count}  Ts={r.settling_time*1000:.3f} ms  ->  {r.status}",
            color=sc, fontsize=9, fontweight='bold', pad=7)
        ax.tick_params(colors='#9a9aa0')
        for sp in ax.spines.values():
            sp.set_edgecolor('#2a2a2e')
        ax.legend(loc='upper right', facecolor='#141416', edgecolor='#2a2a2e',
                  labelcolor='#9a9aa0', fontsize=8)
        ax.grid(True, alpha=0.2, color='#3a3a40')
        self._apply_layout()
        self.draw_idle()


class BodeCanvas(FigureCanvasQTAgg):
    """Loop-gain bode plot with fc / PM / GM markers."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(7.2, 4), dpi=90)
        self.fig.patch.set_facecolor('#141416')
        self.ax_mag, self.ax_phase = self.fig.subplots(2, 1, sharex=True)
        super().__init__(self.fig)
        self.setParent(parent)
        self._draw_empty()

    def _apply_layout(self) -> None:
        width_px, height_px = self.fig.get_size_inches() * self.fig.dpi
        left = max(0.115, 58.0 / max(width_px, 1.0))
        right = 1.0 - max(0.012, 10.0 / max(width_px, 1.0))
        bottom = max(0.105, 38.0 / max(height_px, 1.0))
        top = 1.0 - max(0.105, 42.0 / max(height_px, 1.0))
        hspace = max(0.12, min(0.22, 30.0 / max(height_px, 1.0)))
        self.fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top, hspace=hspace)

    def _draw_empty(self):
        draw_bode_axes(self.ax_mag, self.ax_phase, None, "Loop Gain Bode")
        self._apply_layout()
        self.draw()

    def update_bode(self, bode: Optional[BodeResult], iter_num: Optional[int] = None):
        title = "Loop Gain Bode"
        if iter_num is not None:
            title = f"Loop Gain Bode\nIter {iter_num}"
        draw_bode_axes(self.ax_mag, self.ax_phase, bode, title)
        self._apply_layout()
        self.draw_idle()


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

    def update_metrics(self, results: List[TuningResult],
                       target_os: float = 5.0,
                       target_us: float = 5.0,
                       max_osc: int = 0,
                       target_ts_ms: float = 0.1):
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
        ax.axhline(y=target_os, color='#f05050', linestyle='--', alpha=0.5, lw=0.8,
                   label=f'OS target {target_os:.1f}%')
        ax.axhline(y=target_us, color='#e0a030', linestyle=':', alpha=0.5, lw=0.8,
                   label=f'US target {target_us:.1f}%')
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
        colors = ['#00d4aa' if o <= max_osc else '#f05050' for o in osc_vals]
        ax2.bar(iters, osc_vals, color=colors, width=0.8)
        ax2.axhline(y=max_osc, color='#707076', linestyle='--', lw=0.8)
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
        ax3.axhline(y=target_ts_ms, color='#707076', linestyle='--', lw=0.8)
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

    def __init__(self, title: str, expanded: bool = True, parent=None, compact: bool = False):
        super().__init__(parent)
        self.toggle_button = QToolButton(self)
        self.toggle_button.setText(title)
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(expanded)
        self.toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toggle_button.setIconSize(QSize(8 if compact else 12, 8 if compact else 12))
        self.toggle_button.setProperty("compact", compact)
        if compact:
            self.toggle_button.setFixedHeight(22)
        self.toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.toggle_button.clicked.connect(self._on_toggled)

        self.content = QWidget(self)
        self.content.setVisible(expanded)
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(8 if compact else 10, 3 if compact else 4, 6, 5 if compact else 6)
        self.content_layout.setSpacing(3 if compact else 4)

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
    status_changed = pyqtSignal(object)

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

    def _run_bode_for_iteration(self, iter_num: int, Kp: float, Ki: float, Kd: float, Kf: float) -> Optional[BodeResult]:
        if not getattr(self.config, "run_bode_analysis", False):
            return None
        backend = self.config.backend
        if backend == "ltspice":
            bode = run_ltspice_loop_gain_analysis(
                self.config,
                Kp,
                Ki,
                Kd,
                Kf,
                getattr(self.config, "bode_freq_start_hz", 1e3),
                getattr(self.config, "bode_freq_stop_hz", 1e5),
                int(getattr(self.config, "bode_coarse_num_points", 31)),
                int(getattr(self.config, "bode_dense_num_points", 51)),
            )
            fc_text = f"{bode.metrics.crossover_hz / 1000:.2f} kHz" if bode.metrics.crossover_hz is not None else "n/a"
            pm_text = f"{bode.metrics.phase_margin_deg:.1f} deg" if bode.metrics.phase_margin_deg is not None else "n/a"
            self.log_message.emit(
                f"LTspice Bode iter {iter_num}: fc={fc_text} PM={pm_text} | "
                f"time total={bode.elapsed_s:.1f}s coarse={bode.coarse_elapsed_s:.1f}s "
                f"dense={bode.dense_elapsed_s:.1f}s points={len(bode.freq_hz)}"
            )
            return bode
        if self._auto_tuner is None or self._auto_tuner.plecs.server is None:
            return None
        bode = run_loop_gain_analysis(
            self._auto_tuner.plecs.server,
            self.config.model_id,
            getattr(self.config, "bode_freq_start_hz", 1e3),
            getattr(self.config, "bode_freq_stop_hz", 1e5),
            int(getattr(self.config, "bode_coarse_num_points", 31)),
        )
        fc_text = f"{bode.metrics.crossover_hz / 1000:.2f} kHz" if bode.metrics.crossover_hz is not None else "n/a"
        pm_text = f"{bode.metrics.phase_margin_deg:.1f} deg" if bode.metrics.phase_margin_deg is not None else "n/a"
        self.log_message.emit(
            f"PLECS Bode iter {iter_num}: fc={fc_text} PM={pm_text} | "
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

    @staticmethod
    def _decimate_rows(rows: List[List[float]], max_points: int = 5000) -> List[List[float]]:
        if len(rows) <= max_points or max_points < 2:
            return rows
        last_idx = len(rows) - 1
        return [rows[round(i * last_idx / (max_points - 1))] for i in range(max_points)]

    def _write_workbooks(self, results: List[TuningResult]) -> None:
        results_dir = Path(self.config.results_dir)
        results_dir.mkdir(parents=True, exist_ok=True)
        write_time_workbook(results_dir / "data_time_iterations.xlsx", results, self._waveform_store)
        if self._bode_store:
            write_bode_workbook(results_dir / "data_bode_iterations.xlsx", self._bode_store)

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
            backend_label = "LTspice" if self.config.backend == "ltspice" else "PLECS"
            self.log_message.emit(f"Preparing {backend_label}...")
            self.status_changed.emit({'backend': f'{backend_label} starting', 'mode': 'Auto'})
            at.setup()
            self.status_changed.emit({
                'backend': f'{backend_label} ready',
                'mode': 'Auto',
                'results_dir': str(self.config.results_dir),
                'work_model_path': str(at.work_model_path or ''),
            })
            self.log_message.emit(f"{backend_label} ready. Starting tuning loop.")
            self.log_message.emit(f"Results directory: {self.config.results_dir}")
            self.log_message.emit(f"Working model copy: {at.work_model_path}")

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
                plot_rows = self._decimate_rows(data_rows)
                t, v, il = self._waveform_from_data(header, plot_rows)
                bode = self._run_bode_for_iteration(i, result.Kp, result.Ki, result.Kd, result.Kf)
                self._waveform_store[i] = {'header': header, 'data': plot_rows}
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
                    'phase': phase,
                    'backend': self.config.backend,
                    'results_dir': str(self.config.results_dir),
                    'work_model_path': str(at.work_model_path or ''),
                }
                bode_metrics = {
                    'fc': bode.metrics.crossover_hz if bode is not None else None,
                    'pm': bode.metrics.phase_margin_deg if bode is not None else None,
                    'gm': bode.metrics.gain_margin_db if bode is not None else None,
                }
                self._waveform_store[i].update({
                    'phase': phase,
                    'bode_metrics': bode_metrics,
                    'targets': {
                        'target_os': self.config.target_overshoot,
                        'target_us': self.config.target_undershoot,
                        'max_osc': self.config.max_oscillations,
                        'target_ts': self.config.target_settling_time,
                    },
                    'backend': self.config.backend,
                    'results_dir': str(self.config.results_dir),
                    'work_model_path': str(at.work_model_path or ''),
                })

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
            at = self._auto_tuner
            backend_label = "LTspice" if self.config.backend == "ltspice" else "PLECS"
            self.log_message.emit(f"Preparing {backend_label}...")
            self.status_changed.emit({'backend': f'{backend_label} starting', 'mode': 'Single'})
            at.config = self.config
            at.setup()
            self.status_changed.emit({
                'backend': f'{backend_label} ready',
                'mode': 'Single',
                'results_dir': str(self.config.results_dir),
                'work_model_path': str(at.work_model_path or ''),
            })
            self.log_message.emit(f"{backend_label} ready. Model reloaded with current GUI Bode settings.")
            self.log_message.emit(f"Results directory: {self.config.results_dir}")
            self.log_message.emit(f"Working model copy: {at.work_model_path}")

            phase = getattr(at.tuner, "phase", "unknown")
            result = at.run_iteration(iter_num, Kp, Ki, Kd, Kf)
            at.results.append(result)

            header = at.last_header
            data_rows = at.last_data
            plot_rows = self._decimate_rows(data_rows)
            t, v, il = self._waveform_from_data(header, plot_rows)
            bode = self._run_bode_for_iteration(iter_num, result.Kp, result.Ki, result.Kd, result.Kf)
            self._waveform_store[iter_num] = {'header': header, 'data': plot_rows}
            if bode is not None:
                self._bode_store[iter_num] = bode
            self._waveform_store[iter_num].update({
                'phase': phase,
                'bode_metrics': {
                    'fc': bode.metrics.crossover_hz if bode is not None else None,
                    'pm': bode.metrics.phase_margin_deg if bode is not None else None,
                    'gm': bode.metrics.gain_margin_db if bode is not None else None,
                },
                'targets': {
                    'target_os': self.config.target_overshoot,
                    'target_us': self.config.target_undershoot,
                    'max_osc': self.config.max_oscillations,
                    'target_ts': self.config.target_settling_time,
                },
                'backend': self.config.backend,
                'results_dir': str(self.config.results_dir),
                'work_model_path': str(at.work_model_path or ''),
            })
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
            next_Kp, next_Ki, next_Kd, next_Kf = at.tuner.adjust(
                Kp, Ki, Kd, Kf,
                result.overshoot, result.undershoot, result.osc_count, result.settling_time
            )
            self.iteration_complete.emit({
                'result': result, 'time': t, 'vout': v, 'il': il, 'bode': bode,
                'phase': phase,
                'backend': self.config.backend,
                'results_dir': str(self.config.results_dir),
                'work_model_path': str(at.work_model_path or ''),
                'next_params': (next_Kp, next_Ki, next_Kd, next_Kf),
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
        self.setWindowTitle("Buck Converter PID Auto-Tuner")
        screen = QApplication.primaryScreen().availableGeometry()
        self.setGeometry(screen)

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
        self._best_result: Optional[TuningResult] = None
        self._pid_design_dirty = False
        self._model_sync_ready = False
        self._model_sync_timer = QTimer(self)
        self._model_sync_timer.setSingleShot(True)
        self._model_sync_timer.timeout.connect(self._sync_gui_to_model)

        self._build_ui()
        self._connect_model_sync_signals()
        self._model_sync_ready = True
        self._queue_model_sync()
        self._apply_dark_theme()

    # ---- UI Construction ----

    def _make_status_value(self, text: str = "-") -> QLabel:
        label = QLabel(text)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        label.setMinimumWidth(110)
        label.setStyleSheet("color: #e8e8eb; font-weight: 600;")
        return label

    def _build_status_panel(self) -> CollapsibleSection:
        panel = CollapsibleSection("Run Status", expanded=True, compact=True)
        layout = QGridLayout()
        layout.setContentsMargins(10, 12, 10, 8)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(5)
        panel.content_layout.addLayout(layout)

        self.lbl_plecs_status = self._make_status_value("Idle")
        self.lbl_run_mode = self._make_status_value("Ready")
        self.lbl_iter_status = self._make_status_value("0 / 0")
        self.lbl_search_phase = self._make_status_value("-")
        self.lbl_current_result = self._make_status_value("-")
        self.lbl_best_result = self._make_status_value("-")
        self.lbl_results_dir = self._make_status_value(str(self._results_root_dir))
        self.lbl_work_model = self._make_status_value("-")
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)

        items = [
            ("Backend", self.lbl_plecs_status),
            ("Mode", self.lbl_run_mode),
            ("Iter", self.lbl_iter_status),
            ("Phase", self.lbl_search_phase),
            ("Current", self.lbl_current_result),
            ("Best", self.lbl_best_result),
            ("Results", self.lbl_results_dir),
            ("Work model", self.lbl_work_model),
        ]
        for idx, (name, widget) in enumerate(items):
            row = idx // 2
            col = (idx % 2) * 2
            name_label = QLabel(name)
            name_label.setStyleSheet("color: #707076;")
            layout.addWidget(name_label, row, col)
            layout.addWidget(widget, row, col + 1)

        layout.addWidget(self.progress, 4, 0, 1, 4)
        return panel

    def _build_history_panel(self) -> CollapsibleSection:
        panel = CollapsibleSection("Iteration History", expanded=True, compact=True)
        layout = panel.content_layout
        self.history_table = QTableWidget(0, 15)
        self.history_table.setHorizontalHeaderLabels([
            "Iter", "Phase", "Status", "OS%", "US%", "Osc", "Ts ms",
            "Kp", "Ki", "Kd", "Kf", "fc Hz", "PM deg", "GM dB", "Result dir",
        ])
        self.history_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.history_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.history_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.history_table.verticalHeader().setVisible(False)
        self.history_table.setAlternatingRowColors(True)
        self.history_table.setMinimumHeight(150)
        self.history_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.history_table.horizontalHeader().setStretchLastSection(True)
        self.history_table.cellClicked.connect(self.on_history_row_selected)
        layout.addWidget(self.history_table)
        return panel

    def _add_path_row(self, layout: QGridLayout, row: int, label: str, value: str, file_filter: str):
        name = QLabel(label)
        edit = QLineEdit(value)
        edit.setMinimumWidth(120)
        edit.setToolTip(value)
        edit.textChanged.connect(lambda text, e=edit: e.setToolTip(text))
        button = QPushButton("...")
        button.setFixedWidth(30)
        button.clicked.connect(lambda _checked=False, e=edit, t=label, f=file_filter: self._browse_path(e, t, f))
        layout.addWidget(name, row, 0)
        layout.addWidget(edit, row, 1)
        layout.addWidget(button, row, 2)
        return [name, edit, button], edit

    def _build_backend_panel(self, cfg: TuningConfig) -> CollapsibleSection:
        panel = CollapsibleSection("Simulation Backend", expanded=True)
        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        panel.content_layout.addLayout(layout)

        layout.addWidget(QLabel("Backend:"), 0, 0)
        self.combo_backend = QComboBox()
        self.combo_backend.addItem("PLECS", "plecs")
        self.combo_backend.addItem("LTspice", "ltspice")
        self.combo_backend.setCurrentIndex(1 if cfg.backend == "ltspice" else 0)
        layout.addWidget(self.combo_backend, 0, 1, 1, 2)

        self._plecs_path_widgets = []
        self._ltspice_path_widgets = []
        widgets, self.edit_plecs_model = self._add_path_row(
            layout, 1, "PLECS model:", cfg.plecs_model, "PLECS Models (*.plecs);;All Files (*)")
        self._plecs_path_widgets.extend(widgets)
        widgets, self.edit_plecs_exe = self._add_path_row(
            layout, 2, "PLECS exe:", cfg.plecs_exe, "Executables (*.exe);;All Files (*)")
        self._plecs_path_widgets.extend(widgets)

        widgets, self.edit_ltspice_exe = self._add_path_row(
            layout, 3, "LTspice exe:", cfg.ltspice_exe, "Executables (*.exe);;All Files (*)")
        self._ltspice_path_widgets.extend(widgets)
        widgets, self.edit_ltspice_asc = self._add_path_row(
            layout, 4, "LTspice asc:", cfg.ltspice_asc_model, "LTspice Schematics (*.asc);;All Files (*)")
        self._ltspice_path_widgets.extend(widgets)
        widgets, self.edit_ltspice_net = self._add_path_row(
            layout, 5, "LTspice tran:", cfg.ltspice_netlist_model, "SPICE Netlists (*.cir *.net);;All Files (*)")
        self._ltspice_path_widgets.extend(widgets)
        widgets, self.edit_ltspice_bode = self._add_path_row(
            layout, 6, "LTspice Bode:", cfg.ltspice_bode_netlist_model, "SPICE Netlists (*.cir *.net);;All Files (*)")
        self._ltspice_path_widgets.extend(widgets)

        self.combo_backend.currentIndexChanged.connect(self._on_backend_changed)
        QTimer.singleShot(0, self._on_backend_changed)
        return panel

    def _build_ui(self):
        cfg = TuningConfig()
        comp = CompensatorDesign()
        init_Kp, init_Ki, init_Kd, init_Kf = comp.compute(cfg.wc_initial, cfg.phi_m_initial)
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        main_splitter = QSplitter(Qt.Horizontal, central)
        main_layout.addWidget(main_splitter)

        # Left panel
        left_inner = QWidget()
        left_inner.setMinimumWidth(280)
        left_layout = QVBoxLayout(left_inner)
        left_layout.setContentsMargins(8, 4, 6, 4)
        left_layout.setSpacing(4)

        left_layout.addWidget(self._build_backend_panel(cfg))

        # Circuit image
        grp_img = CollapsibleSection("Circuit", expanded=False)
        img_layout = grp_img.content_layout
        self.circuit_label = QLabel("No image captured")
        self.circuit_label.setAlignment(Qt.AlignCenter)
        self.circuit_label.setMinimumHeight(165)
        self.circuit_label.setMaximumHeight(165)
        self.circuit_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.circuit_label.setStyleSheet("background: #1c1c1f; color: #707076;")
        img_layout.addWidget(self.circuit_label)
        self.btn_open_model = QPushButton("Open Model")
        self.btn_open_model.clicked.connect(self.on_open_model)
        img_layout.addWidget(self.btn_open_model)
        self.btn_capture = QPushButton("Capture from PLECS")
        btn_capture = self.btn_capture
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
        self.lbl_pid_sync = QLabel("PID matches design variables")
        self.lbl_pid_sync.setStyleSheet("color: #00d4aa;")
        dv_layout.addWidget(self.lbl_pid_sync)
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
        self.spin_max_iter = self._make_spin("Max Iter:", 1, 200, 0, 5, 40, tgt_layout)
        left_layout.addWidget(grp_tgt)

        grp_bode = CollapsibleSection("Bode Analysis", expanded=True)
        bode_layout = grp_bode.content_layout
        self.chk_run_bode = QCheckBox("Run bode plot analysis")
        self.chk_run_bode.setChecked(cfg.run_bode_analysis)
        bode_layout.addWidget(self.chk_run_bode)
        self.spin_bode_f_start = self._make_spin("Start f (Hz):", 10, 1e7, 0, 100, 1000, bode_layout)
        self.spin_bode_f_stop = self._make_spin("Stop f (Hz):", 100, 1e7, 0, 1000, 100000, bode_layout)
        self.spin_bode_cycles = self._make_spin("Extraction Cycles:", 1, 500, 0, 1, cfg.bode_extraction_cycles, bode_layout)
        self.spin_bode_coarse_points = self._make_spin("Coarse Num Points:", 5, 500, 0, 1, cfg.bode_coarse_num_points, bode_layout)
        self.spin_bode_dense_points = self._make_spin("Dense Num Points:", 5, 500, 0, 1, cfg.bode_dense_num_points, bode_layout)
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
        left_scroll.setMinimumWidth(292)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setFrameShape(left_scroll.NoFrame)
        main_splitter.addWidget(left_scroll)

        # Right panel
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(6, 6, 8, 6)
        right_layout.setSpacing(6)
        right_layout.addWidget(self._build_status_panel())

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
        right_layout.addWidget(right_splitter, 1)
        right_layout.addWidget(self._build_history_panel())
        main_splitter.addWidget(right_panel)
        main_splitter.setChildrenCollapsible(False)
        main_splitter.setStretchFactor(0, 0)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([312, 1000])

        # Status bar
        self.statusBar().showMessage("Ready")
        QTimer.singleShot(0, self._load_default_circuit_image)

    def _make_spin(self, label, lo, hi, decimals, step, default, layout):
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setFixedWidth(96)
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
        """Render the full circuit image inside the preview area without cropping."""
        if pixmap.isNull():
            return
        self._circuit_pixmap = pixmap
        label_size = self.circuit_label.contentsRect().size()
        target_width = max(1, label_size.width() - 4)
        target_height = max(1, label_size.height() - 4)
        scaled = pixmap.scaled(
            QSize(target_width, target_height),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.circuit_label.setPixmap(scaled)

    def on_open_model(self) -> None:
        cfg = self._make_config()
        if cfg.backend == "ltspice":
            model_path = Path(cfg.ltspice_asc_model).resolve()
            label = "LTspice schematic"
        else:
            model_path = Path(cfg.plecs_model).resolve()
            label = "PLECS model"
        if not model_path.exists():
            QMessageBox.warning(self, "Model Not Found", f"{label} not found:\n{model_path}")
            return
        try:
            os.startfile(str(model_path))
            self.on_log(f"Opened {label}: {model_path}")
        except Exception as exc:
            QMessageBox.warning(self, "Cannot Open Model", f"Could not open {label}:\n{exc}")

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
            QToolButton[compact="true"] { padding: 2px 7px; min-height: 18px; max-height: 22px;
                                          font-size: 8.5pt; border-radius: 3px; }
            QToolButton:hover { background: #2a2a30; border: 1px solid #3a3a40; }
            QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
                background: #111113; color: #e8e8eb; border: 1px solid #2a2a2e;
                border-radius: 3px; padding: 3px 5px; }
            QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
                border: 1px solid #00d4aa; }
            QLabel { color: #9a9aa0; }
            QStatusBar { color: #707076; }
            QProgressBar { background: #111113; color: #e8e8eb; border: 1px solid #2a2a2e;
                           border-radius: 3px; text-align: center; height: 16px; }
            QProgressBar::chunk { background: #00a889; border-radius: 2px; }
            QTableWidget { background: #111113; color: #d8d8dc; gridline-color: #2a2a2e;
                           alternate-background-color: #17171a; selection-background-color: #263447; }
            QHeaderView::section { background: #1f1f23; color: #9a9aa0; border: 1px solid #2a2a2e;
                                   padding: 3px 5px; }
            QSplitter::handle { background: #2a2a2e; height: 3px; }
            QScrollBar:vertical { background: #141416; width: 8px; border: none; }
            QScrollBar::handle:vertical { background: #2a2a2e; border-radius: 4px;
                                          min-height: 20px; }
            QScrollBar::handle:vertical:hover { background: #3a3a40; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
        """)

    # ---- Slots ----

    def _selected_backend(self) -> str:
        if not hasattr(self, "combo_backend"):
            return TuningConfig().backend
        return str(self.combo_backend.currentData() or "plecs")

    def _browse_path(self, edit: QLineEdit, title: str, file_filter: str) -> None:
        current = Path(edit.text()).expanduser()
        start_dir = str(current.parent if current.parent.exists() else Path.cwd())
        path, _ = QFileDialog.getOpenFileName(self, title, start_dir, file_filter)
        if path:
            edit.setText(path)

    def _on_backend_changed(self, *_args) -> None:
        backend = self._selected_backend()
        show_plecs = backend == "plecs"
        for widget in getattr(self, "_plecs_path_widgets", []):
            widget.setVisible(show_plecs)
        for widget in getattr(self, "_ltspice_path_widgets", []):
            widget.setVisible(not show_plecs)
        if hasattr(self, "btn_capture"):
            self.btn_capture.setEnabled(show_plecs)
        if hasattr(self, "spin_bode_cycles"):
            self.spin_bode_cycles.setEnabled(show_plecs)
            self.spin_bode_dense_points.setEnabled(True)
        backend_label = "PLECS" if show_plecs else "LTspice"
        self._update_status_panel(backend=f"{backend_label} selected")

    def _make_config(self) -> TuningConfig:
        cfg = TuningConfig()
        cfg.sim_backend = self._selected_backend()
        cfg.plecs_model = self.edit_plecs_model.text().strip()
        cfg.plecs_exe = self.edit_plecs_exe.text().strip()
        cfg.ltspice_exe = self.edit_ltspice_exe.text().strip()
        cfg.ltspice_asc_model = self.edit_ltspice_asc.text().strip()
        cfg.ltspice_netlist_model = self.edit_ltspice_net.text().strip()
        cfg.ltspice_bode_netlist_model = self.edit_ltspice_bode.text().strip()
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
        cfg.bode_extraction_cycles = int(self.spin_bode_cycles.value())
        cfg.bode_coarse_num_points = int(self.spin_bode_coarse_points.value())
        cfg.bode_dense_num_points = int(self.spin_bode_dense_points.value())
        if self._current_results_dir is not None:
            cfg.results_dir = str(self._current_results_dir)
        return cfg

    def _validate_or_warn(self, config: TuningConfig) -> bool:
        errors = validate_config_values(config)
        if not errors:
            return True
        QMessageBox.warning(
            self,
            "Cannot Start Run",
            "Please fix these settings before running:\n\n" + "\n".join(f"- {e}" for e in errors),
        )
        self.statusBar().showMessage("Run settings need attention.")
        return False

    def _metrics_targets(self) -> Tuple[float, float, int, float]:
        return (
            self.spin_tgt_os.value(),
            self.spin_tgt_us.value(),
            int(self.spin_max_osc.value()),
            self.spin_tgt_settle.value(),
        )

    @staticmethod
    def _fmt_optional(value, fmt: str) -> str:
        if value is None:
            return "n/a"
        try:
            return format(value, fmt)
        except Exception:
            return str(value)

    def _set_pid_dirty(self, *_args) -> None:
        if not getattr(self, "lbl_pid_sync", None):
            return
        self._pid_design_dirty = True
        self.lbl_pid_sync.setText("PID needs recompute from wc / phi_m")
        self.lbl_pid_sync.setStyleSheet("color: #e0a030;")

    def _set_pid_clean(self) -> None:
        self._pid_design_dirty = False
        self.lbl_pid_sync.setText("PID matches design variables")
        self.lbl_pid_sync.setStyleSheet("color: #00d4aa;")

    def _update_status_panel(self, **values) -> None:
        if 'backend' in values:
            self.lbl_plecs_status.setText(str(values['backend']))
        if 'plecs' in values:
            self.lbl_plecs_status.setText(str(values['plecs']))
        if 'mode' in values:
            self.lbl_run_mode.setText(str(values['mode']))
        if 'phase' in values:
            self.lbl_search_phase.setText(str(values['phase']))
        if 'results_dir' in values and values['results_dir']:
            self.lbl_results_dir.setText(str(values['results_dir']))
            self.lbl_results_dir.setToolTip(str(values['results_dir']))
        if 'work_model_path' in values and values['work_model_path']:
            self.lbl_work_model.setText(Path(str(values['work_model_path'])).name)
            self.lbl_work_model.setToolTip(str(values['work_model_path']))
        if 'iter_num' in values or 'max_iterations' in values:
            iter_num = int(values.get('iter_num', max(0, self._iter_counter)))
            max_iter = int(values.get('max_iterations', max(1, self.spin_max_iter.value())))
            self.lbl_iter_status.setText(f"{iter_num} / {max_iter}")
            self.progress.setRange(0, max(1, max_iter))
            self.progress.setValue(max(0, min(iter_num, max_iter)))
        if 'current_result' in values and values['current_result'] is None:
            self.lbl_current_result.setText("-")
        elif 'current_result' in values and values['current_result'] is not None:
            r = values['current_result']
            self.lbl_current_result.setText(
                f"{r.status} OS={r.overshoot:.1f}% US={r.undershoot:.1f}% Osc={r.osc_count}"
            )
        if 'best_result' in values and values['best_result'] is None:
            self.lbl_best_result.setText("-")
        elif 'best_result' in values and values['best_result'] is not None:
            b = values['best_result']
            self.lbl_best_result.setText(
                f"Iter {b.iter_num} {b.status} OS={b.overshoot:.1f}% US={b.undershoot:.1f}%"
            )

    @pyqtSlot(object)
    def on_worker_status(self, data: dict) -> None:
        self._update_status_panel(**data)

    def _clear_history_table(self) -> None:
        self.history_table.setRowCount(0)

    def _append_history_row(self, data: dict) -> None:
        result: TuningResult = data['result']
        bode = data.get('bode')
        metrics = bode.metrics if bode is not None else None
        row = self.history_table.rowCount()
        self.history_table.insertRow(row)
        values = [
            str(result.iter_num),
            str(data.get('phase', '-')),
            result.status,
            f"{result.overshoot:.2f}",
            f"{result.undershoot:.2f}",
            str(result.osc_count),
            f"{result.settling_time * 1000.0:.3f}",
            f"{result.Kp:.5f}",
            f"{result.Ki:.2f}",
            f"{result.Kd:.2e}",
            f"{result.Kf:.0f}",
            self._fmt_optional(metrics.crossover_hz if metrics else None, ".0f"),
            self._fmt_optional(metrics.phase_margin_deg if metrics else None, ".1f"),
            self._fmt_optional(metrics.gain_margin_db if metrics else None, ".1f"),
            str(data.get('results_dir', '')),
        ]
        for col, text in enumerate(values):
            item = QTableWidgetItem(text)
            item.setData(Qt.UserRole, len(self._waveform_history) - 1)
            if result.status == "PASS":
                item.setForeground(QColor("#00d4aa"))
            self.history_table.setItem(row, col, item)
        self.history_table.selectRow(row)

    def _show_history_index(self, idx: int) -> None:
        if idx < 0 or idx >= len(self._waveform_history):
            return
        data = self._waveform_history[idx]
        result = data['result']
        self.spin_kp.setValue(result.Kp)
        self.spin_ki.setValue(result.Ki)
        self.spin_kd.setValue(result.Kd)
        self.spin_kf.setValue(result.Kf)
        target_os, target_us, max_osc, target_ts_ms = self._metrics_targets()
        if data.get('time') and data.get('vout'):
            self.waveform_canvas.update_plot(
                self._waveform_history,
                idx,
                target_os=target_os,
                target_us=target_us,
            )
        self.bode_canvas.update_bode(data.get('bode'), result.iter_num)
        self.metrics_canvas.update_metrics(
            self._results,
            target_os=target_os,
            target_us=target_us,
            max_osc=max_osc,
            target_ts_ms=target_ts_ms,
        )
        self._update_status_panel(
            phase=data.get('phase', '-'),
            current_result=result,
            iter_num=result.iter_num + 1,
            max_iterations=max(1, self.spin_max_iter.value()),
        )

    def on_history_row_selected(self, row: int, _col: int) -> None:
        item = self.history_table.item(row, 0)
        if item is None:
            return
        idx = item.data(Qt.UserRole)
        self._show_history_index(int(idx) if idx is not None else row)

    def _connect_model_sync_signals(self) -> None:
        widgets = [
            self.spin_kp, self.spin_ki, self.spin_kd, self.spin_kf,
            self.spin_wc, self.spin_phim,
            self.spin_tgt_os, self.spin_tgt_us, self.spin_max_osc,
            self.spin_tgt_settle, self.spin_max_iter,
            self.spin_bode_f_start, self.spin_bode_f_stop,
            self.spin_bode_cycles, self.spin_bode_coarse_points, self.spin_bode_dense_points,
        ]
        for widget in widgets:
            widget.valueChanged.connect(self._queue_model_sync)
        self.chk_run_bode.toggled.connect(self._queue_model_sync)
        self.combo_backend.currentIndexChanged.connect(self._queue_model_sync)
        for edit in (
            self.edit_plecs_model, self.edit_plecs_exe,
            self.edit_ltspice_exe, self.edit_ltspice_asc,
            self.edit_ltspice_net, self.edit_ltspice_bode,
        ):
            edit.textChanged.connect(self._queue_model_sync)
        self.spin_wc.valueChanged.connect(self._set_pid_dirty)
        self.spin_phim.valueChanged.connect(self._set_pid_dirty)

    def _queue_model_sync(self, *_args) -> None:
        return

    def _sync_gui_to_model(self) -> None:
        # The GUI deliberately does not write source models. AutoTuner copies
        # the selected PLECS/LTspice template to a work directory for each run.
        return

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
        self._worker.status_changed.connect(self.on_worker_status)
        self._run_single_sig.connect(self._worker.run_single)
        self._thread.start()

    def _cleanup_worker(self):
        if self._worker:
            self._worker.stop()
            try:
                self._run_single_sig.disconnect(self._worker.run_single)
            except TypeError:
                pass
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

        self._model_sync_timer.stop()
        self._sync_gui_to_model()
        cfg = self._make_config()
        if not self._validate_or_warn(cfg):
            return
        self._results.clear()
        self._waveform_history.clear()
        self._best_result = None
        self._iter_counter = 0
        self._clear_bode_results()
        cfg.results_dir = str(self._current_results_dir)
        self._clear_history_table()
        self._update_status_panel(
            backend=f"{'LTspice' if cfg.backend == 'ltspice' else 'PLECS'} queued",
            mode="Auto",
            phase="bootstrap",
            current_result=None,
            best_result=None,
            iter_num=0,
            max_iterations=cfg.max_iterations,
            results_dir=cfg.results_dir,
            work_model_path="-",
        )
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
        self._model_sync_timer.stop()
        self._sync_gui_to_model()
        cfg = self._make_config()
        if not self._validate_or_warn(cfg):
            return
        cfg.results_dir = str(self._current_results_dir)
        self._update_status_panel(
            backend=f"{'LTspice' if cfg.backend == 'ltspice' else 'PLECS'} queued",
            mode="Single",
            iter_num=self._iter_counter,
            max_iterations=cfg.max_iterations,
            results_dir=cfg.results_dir,
        )
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
        self._best_result = None
        self._current_results_dir = None
        self._clear_history_table()

        # Reset spinboxes to defaults
        cfg = TuningConfig()
        comp = CompensatorDesign()
        ref_Kp, ref_Ki, ref_Kd, ref_Kf = comp.compute(cfg.wc_initial, cfg.phi_m_initial)
        self.combo_backend.setCurrentIndex(1 if cfg.backend == "ltspice" else 0)
        self.edit_plecs_model.setText(cfg.plecs_model)
        self.edit_plecs_exe.setText(cfg.plecs_exe)
        self.edit_ltspice_exe.setText(cfg.ltspice_exe)
        self.edit_ltspice_asc.setText(cfg.ltspice_asc_model)
        self.edit_ltspice_net.setText(cfg.ltspice_netlist_model)
        self.edit_ltspice_bode.setText(cfg.ltspice_bode_netlist_model)
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
        self.chk_run_bode.setChecked(cfg.run_bode_analysis)
        self.spin_bode_f_start.setValue(1000)
        self.spin_bode_f_stop.setValue(100000)
        self.spin_bode_cycles.setValue(cfg.bode_extraction_cycles)
        self.spin_bode_coarse_points.setValue(cfg.bode_coarse_num_points)
        self.spin_bode_dense_points.setValue(cfg.bode_dense_num_points)
        self._set_pid_clean()

        # Clear plots
        self.waveform_canvas._draw_empty(
            target_os=cfg.target_overshoot,
            target_us=cfg.target_undershoot)
        self.bode_canvas._draw_empty()
        self.metrics_canvas._draw_empty()

        self.log_text.clear()
        self._set_running(False)
        self._update_status_panel(
            backend=f"{'LTspice' if cfg.backend == 'ltspice' else 'PLECS'} selected",
            mode="Ready",
            phase="-",
            current_result=None,
            best_result=None,
            iter_num=0,
            max_iterations=cfg.max_iterations,
            results_dir=str(self._results_root_dir),
            work_model_path="-",
        )
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
        self._set_pid_clean()
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
        self._best_result = select_best_result(self._results)

        # Update PID fields: show next iteration's params if available, else current
        if 'next_params' in data:
            next_Kp, next_Ki, next_Kd, next_Kf = data['next_params']
            self.spin_kp.setValue(next_Kp)
            self.spin_ki.setValue(next_Ki)
            self.spin_kd.setValue(next_Kd)
            self.spin_kf.setValue(next_Kf)
        else:
            self.spin_kp.setValue(result.Kp)
            self.spin_ki.setValue(result.Ki)
            self.spin_kd.setValue(result.Kd)
            self.spin_kf.setValue(result.Kf)

        # Update plots
        idx = len(self._waveform_history) - 1
        target_os, target_us, max_osc, target_ts_ms = self._metrics_targets()
        if data['time'] and data['vout']:
            self.waveform_canvas.update_plot(
                self._waveform_history, idx,
                target_os=target_os,
                target_us=target_us)
        self.bode_canvas.update_bode(data.get('bode'), result.iter_num)
        self.metrics_canvas.update_metrics(
            self._results,
            target_os=target_os,
            target_us=target_us,
            max_osc=max_osc,
            target_ts_ms=target_ts_ms,
        )
        self._append_history_row(data)
        self._update_status_panel(
            backend=f"{'LTspice' if data.get('backend') == 'ltspice' else 'PLECS'} ready",
            mode="Auto" if self._run_mode == "auto" else "Single",
            phase=data.get('phase', '-'),
            current_result=result,
            best_result=self._best_result,
            iter_num=self._iter_counter,
            max_iterations=max(1, int(self.spin_max_iter.value())),
            results_dir=data.get('results_dir', ''),
            work_model_path=data.get('work_model_path', ''),
        )

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
        self._best_result = best
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
        target_os, target_us, max_osc, target_ts_ms = self._metrics_targets()
        self.waveform_canvas.update_plot(
            self._waveform_history, best_idx,
            target_os=target_os,
            target_us=target_us)
        self.bode_canvas.update_bode(self._waveform_history[best_idx].get('bode'), best.iter_num)
        self.metrics_canvas.update_metrics(
            self._results,
            target_os=target_os,
            target_us=target_us,
            max_osc=max_osc,
            target_ts_ms=target_ts_ms,
        )
        self._update_status_panel(
            phase=self._waveform_history[best_idx].get('phase', '-'),
            current_result=best,
            best_result=best,
        )
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
        self._update_status_panel(
            mode="Complete" if success else "Stopped / Review",
            backend=self.lbl_plecs_status.text(),
            best_result=self._best_result,
        )
        if self._run_mode == "auto":
            self._auto_tune_completed = True
            self._show_best_iteration()
            if success:
                self.on_log("=== SEARCH COMPLETE: best iteration meets target ===")
            else:
                self.on_log("=== SEARCH COMPLETE: showing best iteration found ===")
            self._run_mode = None
            self._cleanup_worker()
        else:
            # Single mode: keep worker alive to preserve tuner state between presses
            if success:
                self.on_log("=== PASS ===")
            self._run_mode = None

    @pyqtSlot(str)
    def on_log(self, msg: str):
        if "Bode iter" in msg and self.log_text.toPlainText().strip():
            self.log_text.append("")
        self.log_text.append(msg)

    @pyqtSlot(str)
    def on_error(self, msg: str):
        self.log_text.append(f"ERROR: {msg}")
        self.statusBar().showMessage(f"Error: {msg}")
        self._update_status_panel(backend="Error", mode="Error")
        QMessageBox.critical(self, "Error", msg)
        self._set_running(False)

    def _set_running(self, running: bool):
        self.btn_start.setEnabled(not running)
        self.btn_single.setEnabled(not running)
        self.btn_pause.setEnabled(running)
        self.btn_resume.setEnabled(False)
        self.btn_stop.setEnabled(running)
        self.combo_backend.setEnabled(not running)
        for edit in (
            self.edit_plecs_model, self.edit_plecs_exe,
            self.edit_ltspice_exe, self.edit_ltspice_asc,
            self.edit_ltspice_net, self.edit_ltspice_bode,
        ):
            edit.setReadOnly(running)
        self.spin_kp.setReadOnly(running)
        self.spin_ki.setReadOnly(running)
        self.spin_kd.setReadOnly(running)
        self.spin_kf.setReadOnly(running)
        self.chk_run_bode.setEnabled(not running)
        self.spin_bode_f_start.setReadOnly(running)
        self.spin_bode_f_stop.setReadOnly(running)
        self.spin_bode_cycles.setReadOnly(running)
        self.spin_bode_coarse_points.setReadOnly(running)
        self.spin_bode_dense_points.setReadOnly(running)

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
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
