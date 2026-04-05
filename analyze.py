"""
Analyze PLECS Buck Converter Tuning Results
===========================================
Uses exported iteration frames and Excel workbooks to generate summaries:
1. Animation GIF from saved per-iteration images
2. Parameter tuning path
3. Metrics trend
4. Best/final frame exports
"""

from __future__ import annotations

import csv
import os
import shutil
import zipfile
from pathlib import Path
from typing import Dict, List, Optional
from xml.etree import ElementTree as ET

import matplotlib.pyplot as plt
from PIL import Image


RESULTS_DIR = str((Path(__file__).resolve().parent / "results").resolve())
TARGET_OS = 4.0
TARGET_US = 4.0


def _xlsx_shared_strings(zf: zipfile.ZipFile) -> List[str]:
    path = "xl/sharedStrings.xml"
    if path not in zf.namelist():
        return []
    root = ET.fromstring(zf.read(path))
    ns = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    strings: List[str] = []
    for si in root.findall("x:si", ns):
        parts = [node.text or "" for node in si.findall(".//x:t", ns)]
        strings.append("".join(parts))
    return strings


def _xlsx_sheet_path(zf: zipfile.ZipFile, sheet_name: str) -> Optional[str]:
    ns_main = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    ns_rel = {"r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships"}
    workbook = ET.fromstring(zf.read("xl/workbook.xml"))
    rels = ET.fromstring(zf.read("xl/_rels/workbook.xml.rels"))
    rel_map = {
        rel.attrib["Id"]: rel.attrib["Target"]
        for rel in rels
        if rel.tag.endswith("Relationship")
    }
    for sheet in workbook.findall("x:sheets/x:sheet", ns_main):
        if sheet.attrib.get("name") == sheet_name:
            rel_id = sheet.attrib.get(f"{{{ns_rel['r']}}}id")
            target = rel_map.get(rel_id)
            if target:
                return f"xl/{target}" if not target.startswith("xl/") else target
    return None


def _read_summary_from_xlsx(path: Path) -> List[Dict]:
    if not path.exists():
        return []
    with zipfile.ZipFile(path) as zf:
        sheet_path = _xlsx_sheet_path(zf, "Summary")
        if not sheet_path:
            return []
        root = ET.fromstring(zf.read(sheet_path))
        shared = _xlsx_shared_strings(zf)
        ns = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        rows: List[List[str]] = []
        for row in root.findall(".//x:sheetData/x:row", ns):
            values: List[str] = []
            for cell in row.findall("x:c", ns):
                cell_type = cell.attrib.get("t")
                if cell_type == "inlineStr":
                    text = "".join(node.text or "" for node in cell.findall(".//x:t", ns))
                else:
                    value_node = cell.find("x:v", ns)
                    raw = value_node.text if value_node is not None else ""
                    if cell_type == "s" and raw:
                        idx = int(raw)
                        text = shared[idx] if idx < len(shared) else raw
                    else:
                        text = raw or ""
                values.append(text)
            rows.append(values)
    if len(rows) < 2:
        return []
    header = rows[0]
    out: List[Dict] = []
    for vals in rows[1:]:
        if not vals or not vals[0].strip():
            continue
        out.append({header[i]: vals[i] if i < len(vals) else "" for i in range(len(header))})
    return out


def load_tuning_log(results_dir: Optional[str] = None) -> List[Dict]:
    """Load tuning summary from workbook, with CSV fallback for older runs."""
    base = Path(results_dir or RESULTS_DIR)
    workbook_path = base / "data_time_iterations.xlsx"
    if not workbook_path.exists():
        workbook_path = base / "time_iterations.xlsx"
    rows = _read_summary_from_xlsx(workbook_path)
    if rows:
        return rows

    log_path = base / "tuning_log.csv"
    if not log_path.exists():
        return []
    with open(log_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        return [row for row in reader if (row.get("Iter") or "").strip().isdigit()]


def get_best_iteration_entry(log: List[Dict]) -> Optional[Dict]:
    if not log:
        return None

    def key(row: Dict):
        return (
            float(row["Overshoot"]) ** 2 + float(row["Undershoot"]) ** 2,
            max(float(row["Overshoot"]), float(row["Undershoot"])),
            float(row["Overshoot"]) + float(row["Undershoot"]),
            int(row["OscCount"]),
            float(row["SettlingTime"]),
            int(row["Iter"]),
        )

    return min(log, key=key)


def _frame_paths(results_dir: Path) -> List[Path]:
    def frame_key(path: Path) -> int:
        stem = path.stem
        try:
            return int(stem.replace("iter", ""))
        except ValueError:
            return 0

    direct_frames = sorted(results_dir.glob("iter*.png"), key=frame_key)
    if direct_frames:
        return direct_frames
    figures_dir = results_dir / "figures"
    return sorted(figures_dir.glob("iter*.png"), key=frame_key)


def _copy_or_show_frame(frame_path: Path, output_path: Optional[str]) -> None:
    if not frame_path.exists():
        print(f"Frame not found: {frame_path}")
        return
    if output_path:
        shutil.copyfile(frame_path, output_path)
        print(f"Plot saved: {output_path}")
        return
    Image.open(frame_path).show()


def plot_animation(output_path: str = None):
    """Generate GIF directly from saved per-iteration frame images."""
    results_dir = Path(output_path).resolve().parent if output_path else Path(RESULTS_DIR)
    frames = _frame_paths(results_dir)
    if not frames:
        raise FileNotFoundError("No iteration frame images found. Run GUI tuning first.")

    images = [Image.open(path).convert("RGB") for path in frames]
    if output_path:
        images[0].save(
            output_path,
            save_all=True,
            append_images=images[1:],
            duration=250,
            loop=0,
            disposal=2,
            optimize=False,
        )
        print(f"Animation saved: {output_path}")
        best = get_best_iteration_entry(load_tuning_log(str(results_dir)))
        if best is not None:
            plot_best_iteration(str(results_dir / "best_iteration.png"))
    else:
        images[0].show()
    for img in images:
        img.close()


def plot_path(output_path: str = None):
    log = load_tuning_log()
    if not log:
        print("No tuning summary found. Run GUI tuning first.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("PID Parameter Tuning Path", fontsize=14, fontweight="bold")

    iterations = [int(r["Iter"]) + 1 for r in log]
    kp_vals = [float(r["Kp"]) for r in log]
    ki_vals = [float(r["Ki"]) for r in log]
    kd_vals = [float(r["Kd"]) for r in log]
    kf_vals = [float(r["Kf"]) for r in log]
    colors = ["green" if r["Status"] == "PASS" else "red" for r in log]

    ax1 = axes[0, 0]
    ax1.scatter(iterations, kp_vals, c=colors, s=50)
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Kp")
    ax1.set_title("Kp Evolution")
    ax1.grid(True, alpha=0.3)

    ax2 = axes[0, 1]
    ax2.scatter(iterations, ki_vals, c=colors, s=50)
    ax2.set_xlabel("Iteration")
    ax2.set_ylabel("Ki")
    ax2.set_title("Ki Evolution")
    ax2.grid(True, alpha=0.3)

    ax3 = axes[1, 0]
    ax3.scatter(iterations, kd_vals, c=colors, s=50)
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("Kd")
    ax3.set_title("Kd Evolution")
    ax3.grid(True, alpha=0.3)

    ax4 = axes[1, 1]
    ax4.scatter(iterations, kf_vals, c=colors, s=50)
    ax4.set_xlabel("Iteration")
    ax4.set_ylabel("Kf")
    ax4.set_title("Kf Evolution")
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Path plot saved: {output_path}")
    else:
        plt.show()
    plt.close()


def _plot_iteration(output_path: Optional[str], entry: Dict, results_dir: Optional[str] = None):
    iter_num = int(entry["Iter"])
    base = Path(results_dir or RESULTS_DIR)
    frame_path = base / f"iter{iter_num + 1}.png"
    if not frame_path.exists():
        frame_path = base / "figures" / f"iter{iter_num + 1}.png"
    _copy_or_show_frame(frame_path, output_path)


def plot_final(output_path: str = None):
    results_dir = str(Path(output_path).resolve().parent) if output_path else RESULTS_DIR
    log = load_tuning_log(results_dir)
    if not log:
        print("No tuning summary found. Run GUI tuning first.")
        return
    _plot_iteration(output_path, log[-1], results_dir)


def plot_best_iteration(output_path: str = None):
    results_dir = str(Path(output_path).resolve().parent) if output_path else RESULTS_DIR
    log = load_tuning_log(results_dir)
    if not log:
        print("No tuning summary found. Run GUI tuning first.")
        return
    best = get_best_iteration_entry(log)
    if best is None:
        print("No best iteration found.")
        return
    _plot_iteration(output_path, best, results_dir)


def plot_metrics(output_path: str = None):
    log = load_tuning_log()
    if not log:
        print("No tuning summary found. Run GUI tuning first.")
        return

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

    iterations = [int(r["Iter"]) + 1 for r in log]
    os_vals = [float(r["Overshoot"]) for r in log]
    us_vals = [float(r["Undershoot"]) for r in log]
    osc_vals = [int(r["OscCount"]) for r in log]
    ts_vals = [float(r["SettlingTime"]) * 1000.0 for r in log]

    ax1.plot(iterations, os_vals, "r-o", label="Overshoot (%)", markersize=4)
    ax1.plot(iterations, us_vals, color="orange", marker="s", label="Undershoot (%)", markersize=4)
    ax1.axhline(y=TARGET_OS, color="r", linestyle="--", alpha=0.5, label=f"Target OS ({TARGET_OS}%)")
    ax1.axhline(y=TARGET_US, color="orange", linestyle="--", alpha=0.5, label=f"Target US ({TARGET_US}%)")
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Percentage (%)")
    ax1.set_title("Overshoot & Undershoot Evolution")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2.bar(iterations, osc_vals, color=["green" if o <= 0 else "red" for o in osc_vals])
    ax2.axhline(y=0, color="gray", linestyle="--", label="Max oscillations (0)")
    ax2.set_xlabel("Iteration")
    ax2.set_ylabel("Oscillation Count")
    ax2.set_title("Oscillation Count")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    ax3.plot(iterations, ts_vals, color="#3366cc", marker="D", markersize=4)
    ax3.axhline(y=0.1, color="gray", linestyle="--", label="Max Ts (0.1 ms)")
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("Settling Time (ms)")
    ax3.set_title("Settling Time")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Metrics plot saved: {output_path}")
    else:
        plt.show()
    plt.close()


def main():
    print("=" * 60)
    print("PLECS Buck Converter Tuning Analysis")
    print("=" * 60)

    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("\n[1] Generating metrics plot...")
    plot_metrics(str(Path(RESULTS_DIR) / "metrics.png"))

    print("\n[2] Generating parameter path plot...")
    plot_path(str(Path(RESULTS_DIR) / "path.png"))

    print("\n[3] Exporting final response frame...")
    plot_final(str(Path(RESULTS_DIR) / "final.png"))

    print("\n[4] Generating animation GIF...")
    plot_animation(str(Path(RESULTS_DIR) / "animation.gif"))

    print("\n[5] Exporting best iteration frame...")
    plot_best_iteration(str(Path(RESULTS_DIR) / "best_iteration.png"))

    print("\n" + "=" * 60)
    print("Analysis complete! Check the results/ folder.")
    print("=" * 60)


if __name__ == "__main__":
    main()
