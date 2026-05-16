"""Runtime paths for source and PyInstaller-frozen execution."""

from __future__ import annotations

import os
import shutil
import sys
import time
from pathlib import Path


APP_NAME = "BuckPidAutoTuner"
MAX_TUNING_WORK_RUNS = 20


def is_frozen() -> bool:
    """Return True when running from a PyInstaller bundle."""
    return bool(getattr(sys, "frozen", False))


def resource_root() -> Path:
    """Directory containing read-only bundled resources."""
    external_root = os.environ.get("BUCK_AUTOTUNER_RESOURCE_ROOT", "").strip().strip('"')
    if external_root:
        root = Path(external_root).resolve()
        if root.exists():
            return root
    if is_frozen():
        return Path(getattr(sys, "_MEIPASS")).resolve()
    return Path(__file__).resolve().parent


def resource_path(*parts: str) -> Path:
    """Path to a bundled resource such as a model template or image."""
    return resource_root().joinpath(*parts).resolve()


def app_data_root() -> Path:
    """Writable application data directory used by frozen builds."""
    if not is_frozen():
        return Path(__file__).resolve().parent
    local_appdata = os.environ.get("LOCALAPPDATA")
    base = Path(local_appdata) if local_appdata else Path.home() / "AppData" / "Local"
    return (base / APP_NAME).resolve()


def writable_path(*parts: str) -> Path:
    """Path under the writable application data directory."""
    return app_data_root().joinpath(*parts).resolve()


def prune_run_dirs(root: Path, keep: int = MAX_TUNING_WORK_RUNS, prefix: str = "run_") -> None:
    """Keep only the newest run directories under a tuning work folder."""
    root.mkdir(parents=True, exist_ok=True)
    if keep < 0:
        keep = 0

    run_dirs = [path for path in root.iterdir() if path.is_dir() and path.name.startswith(prefix)]
    run_dirs.sort(key=lambda path: (path.stat().st_mtime, path.name))
    stale_count = max(0, len(run_dirs) - keep)
    for run_dir in run_dirs[:stale_count]:
        shutil.rmtree(run_dir)


def create_limited_run_dir(root: Path, prefix: str = "run", keep: int = MAX_TUNING_WORK_RUNS) -> Path:
    """Create a timestamped run directory while capping retained run history."""
    root.mkdir(parents=True, exist_ok=True)
    prune_run_dirs(root, keep=max(0, keep - 1), prefix=f"{prefix}_")

    timestamp = time.strftime(f"{prefix}_%Y%m%d_%H%M%S")
    run_dir = root / timestamp
    suffix = 1
    while run_dir.exists():
        run_dir = root / f"{timestamp}_{suffix:02d}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir
