"""
PLECS loop-gain frequency-response helpers.

Usage:
    python bode_plot.py
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import xmlrpc.client as x

from auto_tune import TuningConfig


@dataclass
class BodeMetrics:
    crossover_hz: Optional[float]
    phase_margin_deg: Optional[float]
    gain_margin_db: Optional[float]
    phase_crossover_hz: Optional[float]


@dataclass
class BodeResult:
    freq_hz: List[float]
    mag_db: List[float]
    phase_deg: List[float]
    real_vals: List[float]
    imag_vals: List[float]
    metrics: BodeMetrics
    elapsed_s: float
    coarse_elapsed_s: float = 0.0
    dense_elapsed_s: float = 0.0
    requested_start_hz: Optional[float] = None
    requested_stop_hz: Optional[float] = None


def format_frequency(freq_hz: Optional[float]) -> str:
    if freq_hz is None:
        return "n/a"
    abs_f = abs(freq_hz)
    if abs_f >= 1e6:
        return f"{freq_hz / 1e6:.2f} MHz"
    if abs_f >= 1e3:
        return f"{freq_hz / 1e3:.2f} kHz"
    return f"{freq_hz:.0f} Hz"


def _run_frequency_response(server, model_id: str, analysis_name: str) -> BodeResult:
    t0 = time.perf_counter()
    res = server.plecs.analyze(model_id, analysis_name)
    elapsed_s = time.perf_counter() - t0

    freq_hz = list(res["F"])
    real_vals = list(res["Gr"][0])
    imag_vals = list(res["Gi"][0])
    response = [complex(re, im) for re, im in zip(real_vals, imag_vals)]
    mag_db = [20.0 * math.log10(max(abs(h), 1e-18)) for h in response]
    phase_deg = [math.degrees(math.atan2(h.imag, h.real)) for h in response]
    metrics = compute_metrics(freq_hz, mag_db, phase_deg)
    return BodeResult(
        freq_hz=freq_hz,
        mag_db=mag_db,
        phase_deg=phase_deg,
        real_vals=real_vals,
        imag_vals=imag_vals,
        metrics=metrics,
        elapsed_s=elapsed_s,
        coarse_elapsed_s=elapsed_s if analysis_name == "Loop Gain (Frequency Response)" else 0.0,
        dense_elapsed_s=elapsed_s if analysis_name == "Loop Gain (Peak Dense)" else 0.0,
        requested_start_hz=None,
        requested_stop_hz=None,
    )


def _merge_bode_results(*bodes: BodeResult) -> BodeResult:
    merged: Dict[float, Tuple[float, float, float, float]] = {}
    total_elapsed = 0.0
    coarse_elapsed = 0.0
    dense_elapsed = 0.0
    for bode in bodes:
        total_elapsed += bode.elapsed_s
        coarse_elapsed += bode.coarse_elapsed_s
        dense_elapsed += bode.dense_elapsed_s
        for f, mag, phase, re, im in zip(
            bode.freq_hz, bode.mag_db, bode.phase_deg, bode.real_vals, bode.imag_vals
        ):
            merged[round(f, 9)] = (f, mag, phase, re, im)

    rows = sorted(merged.values(), key=lambda row: row[0])
    freq_hz = [row[0] for row in rows]
    mag_db = [row[1] for row in rows]
    phase_deg = [row[2] for row in rows]
    real_vals = [row[3] for row in rows]
    imag_vals = [row[4] for row in rows]
    return BodeResult(
        freq_hz=freq_hz,
        mag_db=mag_db,
        phase_deg=phase_deg,
        real_vals=real_vals,
        imag_vals=imag_vals,
        metrics=compute_metrics(freq_hz, mag_db, phase_deg),
        elapsed_s=total_elapsed,
        coarse_elapsed_s=coarse_elapsed,
        dense_elapsed_s=dense_elapsed,
        requested_start_hz=None,
        requested_stop_hz=None,
    )


def _slice_bode_result(bode: BodeResult, freq_start_hz: float, freq_stop_hz: float) -> BodeResult:
    rows = [
        (f, mag, phase, re, im)
        for f, mag, phase, re, im in zip(
            bode.freq_hz, bode.mag_db, bode.phase_deg, bode.real_vals, bode.imag_vals
        )
        if freq_start_hz <= f <= freq_stop_hz
    ]
    if not rows:
        return bode

    freq_hz = [row[0] for row in rows]
    mag_db = [row[1] for row in rows]
    phase_deg = [row[2] for row in rows]
    real_vals = [row[3] for row in rows]
    imag_vals = [row[4] for row in rows]
    return BodeResult(
        freq_hz=freq_hz,
        mag_db=mag_db,
        phase_deg=phase_deg,
        real_vals=real_vals,
        imag_vals=imag_vals,
        metrics=compute_metrics(freq_hz, mag_db, phase_deg),
        elapsed_s=bode.elapsed_s,
        coarse_elapsed_s=bode.coarse_elapsed_s,
        dense_elapsed_s=bode.dense_elapsed_s,
        requested_start_hz=freq_start_hz,
        requested_stop_hz=freq_stop_hz,
    )


def interpolate_x_at_y(x1: float, y1: float, x2: float, y2: float, y_target: float) -> float:
    if y2 == y1:
        return x1
    return x1 + (x2 - x1) * ((y_target - y1) / (y2 - y1))


def interpolate_y_at_x(freq_hz: List[float], y_vals: List[float], x_target: float) -> Optional[float]:
    if not freq_hz:
        return None
    if x_target < freq_hz[0] or x_target > freq_hz[-1]:
        return None
    for i in range(len(freq_hz) - 1):
        x1 = freq_hz[i]
        x2 = freq_hz[i + 1]
        if x1 <= x_target <= x2:
            lx1 = math.log10(x1)
            lx2 = math.log10(x2)
            lxt = math.log10(x_target)
            if lx2 == lx1:
                return y_vals[i]
            frac = (lxt - lx1) / (lx2 - lx1)
            return y_vals[i] + frac * (y_vals[i + 1] - y_vals[i])
    return y_vals[-1]


def find_crossover(freq_hz: List[float], mag_db: List[float]) -> Optional[float]:
    for i in range(len(freq_hz) - 1):
        y1 = mag_db[i]
        y2 = mag_db[i + 1]
        if y1 == 0.0:
            return freq_hz[i]
        if (y1 > 0.0 and y2 < 0.0) or (y1 < 0.0 and y2 > 0.0):
            x1 = math.log10(freq_hz[i])
            x2 = math.log10(freq_hz[i + 1])
            x = interpolate_x_at_y(x1, y1, x2, y2, 0.0)
            return 10.0 ** x
    return None


def find_phase_crossover(freq_hz: List[float], phase_deg: List[float], target_deg: float = -180.0) -> Optional[float]:
    for i in range(len(freq_hz) - 1):
        y1 = phase_deg[i]
        y2 = phase_deg[i + 1]
        if y1 == target_deg:
            return freq_hz[i]
        if (y1 > target_deg and y2 < target_deg) or (y1 < target_deg and y2 > target_deg):
            x1 = math.log10(freq_hz[i])
            x2 = math.log10(freq_hz[i + 1])
            x = interpolate_x_at_y(x1, y1, x2, y2, target_deg)
            return 10.0 ** x
    return None


def compute_metrics(freq_hz: List[float], mag_db: List[float], phase_deg: List[float]) -> BodeMetrics:
    crossover_hz = find_crossover(freq_hz, mag_db)
    phase_margin_deg = None
    if crossover_hz is not None:
        phase_at_fc = interpolate_y_at_x(freq_hz, phase_deg, crossover_hz)
        if phase_at_fc is not None:
            phase_margin_deg = 180.0 + phase_at_fc

    phase_crossover_hz = find_phase_crossover(freq_hz, phase_deg, -180.0)
    gain_margin_db = None
    if phase_crossover_hz is not None:
        mag_at_pc = interpolate_y_at_x(freq_hz, mag_db, phase_crossover_hz)
        if mag_at_pc is not None:
            gain_margin_db = -mag_at_pc

    return BodeMetrics(
        crossover_hz=crossover_hz,
        phase_margin_deg=phase_margin_deg,
        gain_margin_db=gain_margin_db,
        phase_crossover_hz=phase_crossover_hz,
    )


def run_loop_gain_analysis(
    server,
    model_id: str,
    freq_start_hz: float,
    freq_stop_hz: float,
    num_points: int,
) -> BodeResult:
    # Match the official PLECS loop-gain demo polarity for negative-feedback insertion.
    # Bode is measured in a "clean" operating mode: the 1A load pulse generator is
    # temporarily commented out during loop-gain extraction, then restored.
    load_block = f"{model_id}/1A Load"
    original_comment_status = None
    try:
        try:
            original_comment_status = server.plecs.get(load_block, "CommentStatus")
        except Exception:
            original_comment_status = "Active"

        server.plecs.set(f"{model_id}/Loop Gain Meter", "invert", "2")
        server.plecs.set(load_block, "CommentStatus", "CommentedOut")

        coarse = _run_frequency_response(server, model_id, "Loop Gain (Frequency Response)")
        try:
            dense = _run_frequency_response(server, model_id, "Loop Gain (Peak Dense)")
        except Exception:
            return _slice_bode_result(coarse, freq_start_hz, freq_stop_hz)
        merged = _merge_bode_results(coarse, dense)
        return _slice_bode_result(merged, freq_start_hz, freq_stop_hz)
    finally:
        if original_comment_status is not None:
            try:
                server.plecs.set(load_block, "CommentStatus", original_comment_status)
            except Exception:
                pass


def save_bode_csv(path: Path, bode: BodeResult) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["Frequency_Hz", "Magnitude_dB", "Phase_deg", "Real", "Imag"])
        for row in zip(bode.freq_hz, bode.mag_db, bode.phase_deg, bode.real_vals, bode.imag_vals):
            writer.writerow(row)


def load_bode_csv(path: Path) -> Optional[BodeResult]:
    if not path.exists():
        return None
    freq_hz: List[float] = []
    mag_db: List[float] = []
    phase_deg: List[float] = []
    real_vals: List[float] = []
    imag_vals: List[float] = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            freq_hz.append(float(row["Frequency_Hz"]))
            mag_db.append(float(row["Magnitude_dB"]))
            phase_deg.append(float(row["Phase_deg"]))
            real_vals.append(float(row["Real"]))
            imag_vals.append(float(row["Imag"]))
    return BodeResult(
        freq_hz=freq_hz,
        mag_db=mag_db,
        phase_deg=phase_deg,
        real_vals=real_vals,
        imag_vals=imag_vals,
        metrics=compute_metrics(freq_hz, mag_db, phase_deg),
        elapsed_s=0.0,
        coarse_elapsed_s=0.0,
        dense_elapsed_s=0.0,
        requested_start_hz=None,
        requested_stop_hz=None,
    )


def draw_bode_axes(ax_mag, ax_phase, bode: Optional[BodeResult], title: str = "Loop Gain Bode Plot") -> None:
    for ax in (ax_mag, ax_phase):
        ax.clear()
        ax.set_facecolor("#1c1c1f")
        ax.tick_params(colors="#9a9aa0")
        ax.grid(True, which="both", alpha=0.2, color="#3a3a40")
        for sp in ax.spines.values():
            sp.set_edgecolor("#2a2a2e")

    if bode is None or not bode.freq_hz:
        ax_mag.set_title(title, color="#707076", fontsize=10)
        ax_mag.set_ylabel("Magnitude (dB)", color="#9a9aa0")
        ax_phase.set_ylabel("Phase (deg)", color="#9a9aa0")
        ax_phase.set_xlabel("Frequency (Hz)", color="#9a9aa0")
        return

    ax_mag.semilogx(bode.freq_hz, bode.mag_db, color="#0b84f3", lw=2.0)
    ax_mag.axhline(0.0, color="#888888", ls="--", lw=0.9)
    ax_mag.set_ylabel("Magnitude (dB)", color="#9a9aa0")
    ax_mag.set_title(title, color="#e8e8eb", fontsize=10)

    ax_phase.semilogx(bode.freq_hz, bode.phase_deg, color="#f05a28", lw=2.0)
    ax_phase.axhline(-180.0, color="#888888", ls="--", lw=0.9)
    ax_phase.set_xlabel("Frequency (Hz)", color="#9a9aa0")
    ax_phase.set_ylabel("Phase (deg)", color="#9a9aa0")

    if bode.requested_start_hz is not None and bode.requested_stop_hz is not None:
        ax_mag.set_xlim(bode.requested_start_hz, bode.requested_stop_hz)
        ax_phase.set_xlim(bode.requested_start_hz, bode.requested_stop_hz)

    if bode.metrics.crossover_hz is not None:
        phase_at_fc = interpolate_y_at_x(bode.freq_hz, bode.phase_deg, bode.metrics.crossover_hz)
        ax_mag.scatter([bode.metrics.crossover_hz], [0.0], color="#d62728", s=40, zorder=5)
        if phase_at_fc is not None:
            ax_phase.scatter([bode.metrics.crossover_hz], [phase_at_fc], color="#d62728", s=40, zorder=5)

    if bode.metrics.phase_crossover_hz is not None and bode.metrics.gain_margin_db is not None:
        mag_at_pc = interpolate_y_at_x(bode.freq_hz, bode.mag_db, bode.metrics.phase_crossover_hz)
        if mag_at_pc is not None:
            ax_mag.scatter([bode.metrics.phase_crossover_hz], [mag_at_pc], color="#2ca02c", s=40, zorder=5)
            ax_phase.scatter([bode.metrics.phase_crossover_hz], [-180.0], color="#2ca02c", s=40, zorder=5)

    summary = [
        f"fc = {format_frequency(bode.metrics.crossover_hz)}",
        f"PM = {bode.metrics.phase_margin_deg:.1f} deg" if bode.metrics.phase_margin_deg is not None else "PM = n/a",
        f"GM = {bode.metrics.gain_margin_db:.1f} dB" if bode.metrics.gain_margin_db is not None else "GM = inf / n.a.",
    ]
    if bode.elapsed_s > 0:
        summary.append(f"analysis = {bode.elapsed_s:.1f} s")
    ax_mag.text(
        0.98,
        0.98,
        "\n".join(summary),
        transform=ax_mag.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "#141416", "alpha": 0.92, "edgecolor": "#3a3a40"},
        color="#e8e8eb",
    )


def run_loop_gain_bode() -> Tuple[Path, Path, BodeMetrics, float]:
    cfg = TuningConfig()
    results_dir = Path(cfg.results_dir)
    results_dir.mkdir(exist_ok=True)
    out_png = results_dir / "loop_gain_bode.png"
    out_csv = results_dir / "loop_gain_bode.csv"

    server = x.ServerProxy(cfg.rpc_url, allow_none=True)
    try:
        server.plecs.close(cfg.model_id)
    except Exception:
        pass
    server.plecs.load(str(Path(cfg.plecs_model).resolve()))

    bode = run_loop_gain_analysis(server, cfg.model_id, 1e3, 1e5, 31)
    save_bode_csv(out_csv, bode)

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(8.8, 6.7), sharex=True)
    draw_bode_axes(ax_mag, ax_phase, bode, "Loop Gain Bode Plot\nSynchronous Buck Converter")
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

    return out_png, out_csv, bode.metrics, bode.elapsed_s


if __name__ == "__main__":
    png, csv_path, metrics, elapsed_s = run_loop_gain_bode()
    print(f"Saved plot: {png}")
    print(f"Saved data: {csv_path}")
    print(f"Analysis time: {elapsed_s:.2f} s")
    print(f"Crossover frequency: {metrics.crossover_hz}")
    print(f"Phase margin: {metrics.phase_margin_deg}")
    print(f"Gain margin: {metrics.gain_margin_db}")
