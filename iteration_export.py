from __future__ import annotations

import shutil
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import xlsxwriter

from auto_tune import TuningResult
from bode_plot import BodeResult, draw_bode_axes


def write_time_workbook(path: Path, summary_rows: List[TuningResult], waveforms: Dict[int, Dict]) -> None:
    workbook = xlsxwriter.Workbook(str(path))
    ws_summary = workbook.add_worksheet("Summary")
    headers = ["Iter", "Kp", "Ki", "Kd", "Kf", "Overshoot", "Undershoot", "OscCount", "SettlingTime", "Status"]
    for col, name in enumerate(headers):
        ws_summary.write(0, col, name)
    for row_idx, r in enumerate(summary_rows, start=1):
        vals = [r.iter_num, r.Kp, r.Ki, r.Kd, r.Kf, r.overshoot, r.undershoot, r.osc_count, r.settling_time, r.status]
        for col, val in enumerate(vals):
            ws_summary.write(row_idx, col, val)

    for iter_num in sorted(waveforms):
        item = waveforms[iter_num]
        ws = workbook.add_worksheet(f"Iter_{iter_num + 1:03d}")
        header = item["header"]
        data = item["data"]
        for col, name in enumerate(header):
            ws.write(0, col, name)
        for row_idx, row in enumerate(data, start=1):
            for col, val in enumerate(row):
                ws.write(row_idx, col, val)
    workbook.close()


def write_bode_workbook(path: Path, bodes: Dict[int, BodeResult]) -> None:
    workbook = xlsxwriter.Workbook(str(path))
    ws_summary = workbook.add_worksheet("Summary")
    summary_headers = ["Iter", "Crossover_Hz", "PhaseMargin_deg", "GainMargin_dB", "AnalysisTime_s"]
    for col, name in enumerate(summary_headers):
        ws_summary.write(0, col, name)

    summary_row = 1
    for iter_num in sorted(bodes):
        bode = bodes[iter_num]
        ws_summary.write(summary_row, 0, iter_num + 1)
        ws_summary.write(summary_row, 1, bode.metrics.crossover_hz)
        ws_summary.write(summary_row, 2, bode.metrics.phase_margin_deg)
        ws_summary.write(summary_row, 3, bode.metrics.gain_margin_db)
        ws_summary.write(summary_row, 4, bode.elapsed_s)
        summary_row += 1

        ws = workbook.add_worksheet(f"Iter_{iter_num + 1:03d}")
        headers = ["Frequency_Hz", "Magnitude_dB", "Phase_deg", "Real", "Imag"]
        for col, name in enumerate(headers):
            ws.write(0, col, name)
        for row_idx, row in enumerate(zip(bode.freq_hz, bode.mag_db, bode.phase_deg, bode.real_vals, bode.imag_vals), start=1):
            for col, val in enumerate(row):
                ws.write(row_idx, col, val)
    workbook.close()


def save_iteration_frame(
    output_path: Path,
    result: TuningResult,
    time_vals: List[float],
    vout_vals: List[float],
    bode: Optional[BodeResult],
    target_os: float,
    target_us: float,
) -> None:
    display_iter = result.iter_num + 1
    t_ms = [t * 1000.0 for t in time_vals]
    fig = plt.figure(figsize=(16, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0], height_ratios=[1, 1])
    ax_ts = fig.add_subplot(gs[:, 0])
    ax_mag = fig.add_subplot(gs[0, 1])
    ax_phase = fig.add_subplot(gs[1, 1], sharex=ax_mag)

    v_tgt = 5.0
    v_os = v_tgt * (1 + target_os / 100.0)
    v_us = v_tgt * (1 - target_us / 100.0)
    ax_ts.axhspan(v_tgt, v_os, color='#f05050', alpha=0.10, label=f'{target_os:.1f}% OS limit')
    ax_ts.axhspan(v_us, v_tgt, color='#e0a030', alpha=0.10, label=f'{target_us:.1f}% US limit')
    ax_ts.axhline(y=v_os, color='#f05050', linestyle='--', lw=0.8, alpha=0.6)
    ax_ts.axhline(y=v_us, color='#e0a030', linestyle='--', lw=0.8, alpha=0.6)
    ax_ts.axhline(y=v_tgt, color='#707076', linestyle=':', lw=1.0, alpha=0.7)

    color = '#00d4aa' if result.status == 'PASS' else '#f05050'
    ax_ts.plot(t_ms, vout_vals, color=color, lw=2.0, label=f'Iter {display_iter} ({result.status})')
    if vout_vals:
        v_peak = max(vout_vals)
        v_valley = min(vout_vals)
        ax_ts.scatter([t_ms[vout_vals.index(v_peak)]], [v_peak], color='#f05050', s=50, zorder=6)
        ax_ts.scatter([t_ms[vout_vals.index(v_valley)]], [v_valley], color='#e0a030', s=50, zorder=6)
    ax_ts.set_ylim(4.4, 5.6)
    if t_ms:
        ax_ts.set_xlim(min(t_ms), max(t_ms))
    ax_ts.set_xlabel('Time (ms)')
    ax_ts.set_ylabel('Output Voltage (V)')
    ax_ts.set_title(
        f"Iter {display_iter}  -  Kp={result.Kp:.4f}  Ki={result.Ki:.1f}  "
        f"Kd={result.Kd:.2e}  Kf={result.Kf:.0f}\n"
        f"OS={result.overshoot:.1f}%  US={result.undershoot:.1f}%  "
        f"Osc={result.osc_count}  Ts={result.settling_time*1000:.3f} ms  ->  {result.status}",
        color=color, fontsize=11, fontweight='bold'
    )
    ax_ts.legend(loc='upper right')
    ax_ts.grid(True, alpha=0.2)

    draw_bode_axes(ax_mag, ax_phase, bode, f"Loop Gain Bode\nIter {display_iter}")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def copy_best_frame(frames_dir: Path, best_iter_num: int, output_path: Path) -> bool:
    src = frames_dir / f"iter{best_iter_num + 1}.png"
    if not src.exists():
        return False
    shutil.copyfile(src, output_path)
    return True
