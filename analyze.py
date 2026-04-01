"""
Analyze PLECS Buck Converter Tuning Results
============================================
Generates visualizations from saved iteration CSV files:
1. Animation of Vout response evolution
2. Parameter tuning path
3. Final response with annotations

Usage:
    python analyze.py
"""

import csv
import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


RESULTS_DIR = str((Path(__file__).resolve().parent / "results").resolve())
TARGET_OS = 5.0
TARGET_US = 5.0


def load_tuning_log(results_dir: Optional[str] = None) -> List[Dict]:
    """Load tuning log CSV"""
    log_path = Path(results_dir or RESULTS_DIR) / "tuning_log.csv"
    if not log_path.exists():
        return []

    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            iter_val = (row.get('Iter') or '').strip()
            if not iter_val.isdigit():
                continue
            rows.append(row)
        return rows


def load_iter_csv(iter_num: int, results_dir: Optional[str] = None) -> Tuple[List[str], List[List[float]]]:
    """Load CSV data for a specific iteration"""
    csv_path = Path(results_dir or RESULTS_DIR) / f"iter_{iter_num:03d}.csv"
    if not csv_path.exists():
        return [], []

    with open(csv_path, 'r') as f:
        content = f.read()

    lines = content.strip().split('\n')
    if len(lines) < 2:
        return [], []

    reader = csv.reader(lines)
    rows = list(reader)
    header = rows[0] if rows else []
    data = []
    for r in rows[1:]:
        try:
            data.append([float(c) for c in r])
        except ValueError:
            pass
    return header, data


def get_vout_column(header: List[str]) -> int:
    """Find Vout column index"""
    # Try to find Vout by name
    for i, h in enumerate(header):
        if 'vout' in h.lower() or 'output' in h.lower():
            return i
    # Default: time, IL, Vout pattern
    return 2 if len(header) >= 3 else 1


def get_best_iteration_entry(log: List[Dict]) -> Optional[Dict]:
    """Return the best iteration entry from the tuning log."""
    if not log:
        return None

    def key(row: Dict):
        return (
            max(float(row['Overshoot']), float(row['Undershoot'])),
            float(row['Overshoot']) + float(row['Undershoot']),
            int(row['OscCount']),
            float(row['SettlingTime']),
            int(row['Iter']),
        )

    return min(log, key=key)


def plot_animation(output_path: str = None):
    """Generate animated GIF of Vout response evolution across all iterations"""
    results_dir = str(Path(output_path).resolve().parent) if output_path else RESULTS_DIR
    log = load_tuning_log(results_dir)
    if not log:
        print("No tuning log found. Run auto_tune.py first.")
        return

    n_iters = len(log)
    iterations = list(range(n_iters))

    # Pre-load all waveforms to determine axis limits
    all_time, all_vout = [], []
    waveforms = {}
    for idx in iterations:
        header, data = load_iter_csv(idx, results_dir)
        if data:
            vout_col = get_vout_column(header)
            t = [row[0] * 1000 for row in data]
            v = [row[vout_col] for row in data]
            waveforms[idx] = (t, v)
            all_time.extend(t)
            all_vout.extend(v)

    if not all_vout:
        print("No waveform data found.")
        return

    t_min, t_max = min(all_time), max(all_time)
    v_min = max(4.0, min(all_vout) - 0.05)
    v_max = min(6.0, max(all_vout) + 0.05)

    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#16213e')

    def update(frame):
        ax.clear()
        ax.set_facecolor('#16213e')

        cur_idx = iterations[frame]
        result = log[cur_idx]
        status = result['Status']
        status_color = '#00ff88' if status == 'PASS' else '#ff6b6b'

        # --- 5% OS/US bands ---
        ax.axhspan(5.0, 5.25, color='#ff6b6b', alpha=0.12, label='5% OS limit')
        ax.axhspan(4.75, 5.0, color='#ffa500', alpha=0.12, label='5% US limit')
        ax.axhline(y=5.25, color='#ff6b6b', linestyle='--', linewidth=0.8, alpha=0.6)
        ax.axhline(y=4.75, color='#ffa500', linestyle='--', linewidth=0.8, alpha=0.6)
        ax.axhline(y=5.0,  color='#aaaaaa', linestyle=':',  linewidth=1.0, alpha=0.7)

        # --- Ghost traces of all previous iterations ---
        for prev in iterations[:frame]:
            if prev in waveforms:
                pt, pv = waveforms[prev]
                prev_result = log[prev]
                ghost_color = '#00ff88' if prev_result['Status'] == 'PASS' else '#4488ff'
                ax.plot(pt, pv, color=ghost_color, linewidth=0.6, alpha=0.20)

        # --- Current iteration waveform ---
        if cur_idx in waveforms:
            t, v = waveforms[cur_idx]
            ax.plot(t, v, color=status_color, linewidth=2.0, label=f'Iter {cur_idx} ({status})', zorder=5)

            # Mark peak and valley
            v_peak = max(v)
            v_valley = min(v)
            t_peak = t[v.index(v_peak)]
            t_valley = t[v.index(v_valley)]
            ax.scatter([t_peak],   [v_peak],   color='#ff4444', s=60, zorder=6)
            ax.scatter([t_valley], [v_valley], color='#ffaa00', s=60, zorder=6)

        # --- Annotations ---
        os_val  = float(result['Overshoot'])
        us_val  = float(result['Undershoot'])
        osc_val = result['OscCount']
        kp_val  = float(result['Kp'])
        ki_val  = float(result['Ki'])
        kd_val  = float(result['Kd'])
        kf_val  = float(result['Kf'])

        ax.set_xlim(t_min, t_max)
        ax.set_ylim(v_min, v_max)
        ax.set_xlabel('Time (ms)', color='#cccccc', fontsize=11)
        ax.set_ylabel('Output Voltage (V)', color='#cccccc', fontsize=11)
        ax.tick_params(colors='#cccccc')
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')

        ax.set_title(
            f"Iteration {cur_idx} / {n_iters - 1}  —  "
            f"Kp={kp_val:.4f}  Ki={ki_val:.1f}  Kd={kd_val:.2e}  Kf={kf_val:.0f}\n"
            f"OS={os_val:.1f}%  US={us_val:.1f}%  Osc={osc_val}  →  {status}",
            color=status_color, fontsize=11, fontweight='bold'
        )

        # Progress bar at bottom
        progress = (frame + 1) / len(iterations)
        ax.annotate('', xy=(t_min + progress * (t_max - t_min), v_min + 0.01),
                    xytext=(t_min, v_min + 0.01),
                    arrowprops=dict(arrowstyle='-', color='#5566ff', lw=3))

        ax.legend(loc='upper right', facecolor='#1a1a2e', edgecolor='#444466',
                  labelcolor='#cccccc', fontsize=9)
        ax.grid(True, alpha=0.15, color='#8888aa')

    ani = animation.FuncAnimation(fig, update, frames=len(iterations),
                                  interval=200, repeat=True)

    if output_path:
        try:
            ani.save(output_path, writer='pillow', fps=4)
            print(f"Animation saved: {output_path}")
            best_path = str(Path(output_path).with_name("best_iteration.png"))
            plot_best_iteration(best_path)
        except Exception as e:
            print(f"Animation save failed: {e}")
    else:
        plt.show()
    plt.close()


def plot_path(output_path: str = None):
    """Plot parameter tuning path (4 panels)"""
    log = load_tuning_log()
    if not log:
        print("No tuning log found. Run auto_tune.py first.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('PID Parameter Tuning Path', fontsize=14, fontweight='bold')

    iterations = list(range(len(log)))
    kp_vals = [float(r['Kp']) for r in log]
    ki_vals = [float(r['Ki']) for r in log]
    kd_vals = [float(r['Kd']) for r in log]
    kf_vals = [float(r['Kf']) for r in log]
    os_vals = [float(r['Overshoot']) for r in log]
    us_vals = [float(r['Undershoot']) for r in log]

    colors = ['green' if r['Status'] == 'PASS' else 'red' for r in log]

    # Kp and Ki
    ax1 = axes[0, 0]
    sc1 = ax1.scatter(iterations, kp_vals, c=colors, s=50)
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Kp')
    ax1.set_title('Kp Evolution')
    ax1.grid(True, alpha=0.3)

    ax2 = axes[0, 1]
    ax2.scatter(iterations, ki_vals, c=colors, s=50)
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Ki')
    ax2.set_title('Ki Evolution')
    ax2.grid(True, alpha=0.3)

    # Kd and Kf
    ax3 = axes[1, 0]
    ax3.scatter(iterations, kd_vals, c=colors, s=50)
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Kd')
    ax3.set_title('Kd Evolution')
    ax3.grid(True, alpha=0.3)

    ax4 = axes[1, 1]
    ax4.scatter(iterations, kf_vals, c=colors, s=50)
    ax4.set_xlabel('Iteration')
    ax4.set_ylabel('Kf')
    ax4.set_title('Kf Evolution')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Path plot saved: {output_path}")
    else:
        plt.show()
    plt.close()


def _plot_iteration(output_path: Optional[str], entry: Dict, title_prefix: str,
                    results_dir: Optional[str] = None):
    """Plot one iteration Vout response with annotations."""
    iter_num = int(entry['Iter'])
    header, data = load_iter_csv(iter_num, results_dir)
    if not data:
        print(f"No CSV data found for iteration {iter_num}.")
        return

    vout_col = get_vout_column(header)
    time_vals = [row[0] * 1000 for row in data]
    vout_vals = [row[vout_col] for row in data]
    il_vals = [row[1] if len(header) >= 3 else 0 for row in data]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    ax1.plot(time_vals, vout_vals, 'b-', linewidth=1.5, label='Vout')
    ax1.axhline(y=5.0, color='gray', linestyle=':', label='Target (5V)')
    ax1.axhline(y=5.0 * (1 + TARGET_OS / 100), color='r', linestyle='--', alpha=0.5,
                label=f'+{TARGET_OS:.1f}% ({5.0*(1 + TARGET_OS/100):.2f}V)')
    ax1.axhline(y=5.0 * (1 - TARGET_US / 100), color='r', linestyle='--', alpha=0.5,
                label=f'-{TARGET_US:.1f}% ({5.0*(1 - TARGET_US/100):.2f}V)')

    max_v = max(vout_vals)
    min_v = min(vout_vals)
    ax1.annotate(f'Max: {max_v:.3f}V\nOS: {(max_v-5)/5*100:.1f}%',
                 xy=(time_vals[vout_vals.index(max_v)], max_v),
                 xytext=(time_vals[vout_vals.index(max_v)] + 0.05, max_v + 0.03),
                 arrowprops=dict(arrowstyle='->', color='red'),
                 fontsize=10, color='red')

    ax1.annotate(f'Min: {min_v:.3f}V\nUS: {(5-min_v)/5*100:.1f}%',
                 xy=(time_vals[vout_vals.index(min_v)], min_v),
                 xytext=(time_vals[vout_vals.index(min_v)] + 0.05, min_v - 0.06),
                 arrowprops=dict(arrowstyle='->', color='orange'),
                 fontsize=10, color='orange')

    ax1.set_ylabel('Output Voltage (V)')
    ax1.set_title(
        f"{title_prefix}: Iter {iter_num}, Kp={float(entry['Kp']):.4f}, "
        f"Ki={float(entry['Ki']):.4f}, Kd={float(entry['Kd']):.4f}, Kf={float(entry['Kf']):.4f}"
    )
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(min(time_vals), max(time_vals))

    if any(il_vals):
        ax2.plot(time_vals, il_vals, 'g-', linewidth=1, label='IL')
        ax2.set_ylabel('Inductor Current (A)')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

    ax2.set_xlabel('Time (ms)')
    ax2.set_title(
        f"OS={float(entry['Overshoot']):.1f}%, US={float(entry['Undershoot']):.1f}%, "
        f"Osc={entry['OscCount']} -> {entry['Status']}"
    )

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Plot saved: {output_path}")
    else:
        plt.show()
    plt.close()


def plot_final(output_path: str = None):
    """Plot final iteration Vout response with annotations"""
    results_dir = str(Path(output_path).resolve().parent) if output_path else RESULTS_DIR
    log = load_tuning_log(results_dir)
    if not log:
        print("No tuning log found. Run auto_tune.py first.")
        return

    final = log[-1]
    _plot_iteration(output_path, final, "Final Response", results_dir)


def plot_best_iteration(output_path: str = None):
    """Plot the best iteration Vout response with annotations."""
    results_dir = str(Path(output_path).resolve().parent) if output_path else RESULTS_DIR
    log = load_tuning_log(results_dir)
    if not log:
        print("No tuning log found. Run auto_tune.py first.")
        return

    best = get_best_iteration_entry(log)
    if best is None:
        print("No best iteration found.")
        return

    _plot_iteration(output_path, best, "Best Response", results_dir)


def plot_metrics(output_path: str = None):
    """Plot overshoot/undershoot/oscillation over iterations"""
    log = load_tuning_log()
    if not log:
        print("No tuning log found. Run auto_tune.py first.")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    iterations = list(range(len(log)))
    os_vals = [float(r['Overshoot']) for r in log]
    us_vals = [float(r['Undershoot']) for r in log]
    osc_vals = [int(r['OscCount']) for r in log]

    # Overshoot/Undershoot
    ax1.plot(iterations, os_vals, 'r-o', label='Overshoot (%)', markersize=4)
    ax1.plot(iterations, us_vals, 'orange', marker='s', label='Undershoot (%)', markersize=4)
    ax1.axhline(y=TARGET_OS, color='r', linestyle='--', alpha=0.5, label=f'Target OS ({TARGET_OS}%)')
    ax1.axhline(y=TARGET_US, color='orange', linestyle='--', alpha=0.5, label=f'Target US ({TARGET_US}%)')
    ax1.fill_between(iterations, 0, os_vals, where=[os < TARGET_OS for os in os_vals],
                     color='green', alpha=0.2, label='Pass region')
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Percentage (%)')
    ax1.set_title('Overshoot & Undershoot Evolution')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Oscillations
    ax2.bar(iterations, osc_vals, color=['green' if o <= 2 else 'red' for o in osc_vals])
    ax2.axhline(y=2, color='gray', linestyle='--', label='Max oscillations (2)')
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Oscillation Count')
    ax2.set_title('Oscillation Count')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Metrics plot saved: {output_path}")
    else:
        plt.show()
    plt.close()


def main():
    """Generate all visualizations"""
    print("=" * 60)
    print("PLECS Buck Converter Tuning Analysis")
    print("=" * 60)

    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("\n[1] Generating metrics plot...")
    plot_metrics(str(Path(RESULTS_DIR) / "metrics.png"))

    print("\n[2] Generating parameter path plot...")
    plot_path(str(Path(RESULTS_DIR) / "path.png"))

    print("\n[3] Generating final response plot...")
    plot_final(str(Path(RESULTS_DIR) / "final.png"))

    print("\n[4] Generating animation (this may take a moment)...")
    plot_animation(str(Path(RESULTS_DIR) / "animation.gif"))

    print("\n" + "=" * 60)
    print("Analysis complete! Check the results/ folder:")
    print(f"  - {RESULTS_DIR}/metrics.png")
    print(f"  - {RESULTS_DIR}/path.png")
    print(f"  - {RESULTS_DIR}/final.png")
    print(f"  - {RESULTS_DIR}/animation.gif")
    print("=" * 60)


if __name__ == "__main__":
    main()
