"""
SIMetrix/SIMPLIS automation helpers for the buck PID auto-tuner.

The visual .sxsch file is bundled for manual inspection/editing. Automated
transient runs use the companion SIMPLIS netlist, following the same
"schematic plus generated netlist" pattern as the LTspice backend.
"""

from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

from app_paths import create_limited_run_dir


def find_default_simplis_exe() -> str:
    """Return the most likely local SIMetrix executable path, or an empty string."""
    env_path = os.environ.get("SIMPLIS_EXE", "").strip().strip('"')
    if env_path and Path(env_path).exists():
        return env_path

    roots = [
        Path(os.environ.get("ProgramFiles", "")),
        Path(os.environ.get("ProgramFiles(x86)", "")),
    ]
    candidates: List[Path] = []
    for root in roots:
        if not root:
            continue
        candidates.extend(sorted(root.glob("SIMetrix*/bin64/SIMetrix.exe"), reverse=True))
        candidates.extend(sorted(root.glob("SIMetrix*/bin/SIMetrix.exe"), reverse=True))
    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())
    return env_path


class SimplisExportParser:
    """Convert SIMetrix text/CSV exports into the canonical project table."""

    TIME_NAMES = ("time", "x", "index")
    VOUT_NAMES = ("vout", "#vout", "v(vout)", "v(3)", ":3")
    IL_NAMES = ("il", "i(l1)", "l1#p", "i(v$iprobe1)", "iprobe")

    @staticmethod
    def _clean_name(name: str) -> str:
        return name.strip().strip('"').strip("'").lower()

    @staticmethod
    def _find_col(names: Sequence[str], tokens: Sequence[str], fallback: int) -> int:
        lowered = [SimplisExportParser._clean_name(name) for name in names]
        for token in tokens:
            token_l = token.lower()
            for idx, name in enumerate(lowered):
                if name == token_l or token_l in name:
                    return idx
        return fallback

    @staticmethod
    def _split_line(line: str) -> List[str]:
        line = line.strip()
        if not line:
            return []
        if "," in line:
            return [part.strip() for part in next(csv.reader([line]))]
        return re.split(r"\s+", line)

    @classmethod
    def parse_text(cls, text: str) -> Tuple[List[str], List[List[float]]]:
        rows: List[List[str]] = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith(("*", "#", ";")):
                continue
            parts = cls._split_line(line)
            if parts:
                rows.append(parts)
        if not rows:
            return [], []

        header: List[str]
        first = rows[0]
        try:
            [float(part) for part in first]
            header = ["Time", "IL", "Vout"][: len(first)]
            data_rows = rows
        except ValueError:
            header = first
            data_rows = rows[1:]

        if len(header) < 2:
            return [], []
        if data_rows:
            try:
                first_data_width = len([float(part) for part in data_rows[0]])
            except ValueError:
                first_data_width = len(data_rows[0])
            header_has_time = any(token in cls._clean_name(name) for name in header for token in cls.TIME_NAMES)
            if not header_has_time and first_data_width == len(header) + 1:
                header = ["Time", *header]

        time_col = cls._find_col(header, cls.TIME_NAMES, 0)
        il_col = cls._find_col(header, cls.IL_NAMES, 1 if len(header) > 1 else 0)
        vout_col = cls._find_col(header, cls.VOUT_NAMES, 2 if len(header) > 2 else len(header) - 1)

        parsed: List[List[float]] = []
        for row in data_rows:
            try:
                values = [float(part) for part in row]
            except ValueError:
                continue
            if max(time_col, il_col, vout_col) >= len(values):
                continue
            parsed.append([values[time_col], values[il_col], values[vout_col]])
        return ["Time", "IL", "Vout"], parsed

    @classmethod
    def parse_file(cls, path: Path) -> Tuple[List[str], List[List[float]]]:
        return cls.parse_text(path.read_text(encoding="utf-8", errors="replace"))


class SimplisSimulationRunner:
    """Prepare SIMPLIS working copies, run transient simulations, and parse output."""

    def __init__(self, config: Any):
        self.config = config
        self.run_dir: Optional[Path] = None
        self.work_schematic_path: Optional[Path] = None
        self.work_netlist_path: Optional[Path] = None
        self.output_path: Optional[Path] = None
        self.script_path: Optional[Path] = None
        self.processed_netlist_path: Optional[Path] = None

    @property
    def simplis_exe(self) -> str:
        return str(getattr(self.config, "simplis_exe", "") or "")

    def prepare_working_model(self) -> Path:
        work_root = Path(getattr(self.config, "simplis_work_dir")).resolve()
        self.run_dir = create_limited_run_dir(work_root)

        schematic_source = Path(getattr(self.config, "simplis_schematic_model")).resolve()
        netlist_source = Path(getattr(self.config, "simplis_netlist_model")).resolve()
        if not schematic_source.exists():
            raise FileNotFoundError(f"SIMPLIS schematic not found: {schematic_source}")
        if not netlist_source.exists():
            raise FileNotFoundError(f"SIMPLIS transient netlist not found: {netlist_source}")

        self.work_schematic_path = self.run_dir / schematic_source.name
        self.work_netlist_path = self.run_dir / netlist_source.name
        shutil.copyfile(schematic_source, self.work_schematic_path)
        shutil.copyfile(netlist_source, self.work_netlist_path)
        return self.work_netlist_path

    def _ensure_ready(self) -> None:
        if self.run_dir is None or self.work_netlist_path is None:
            self.prepare_working_model()
        if not self.simplis_exe or not Path(self.simplis_exe).exists():
            raise FileNotFoundError(
                "SIMetrix/SIMPLIS executable not found. Set SIMPLIS_EXE or choose SIMetrix.exe."
            )

    @staticmethod
    def _replace_var(text: str, name: str, value: Any) -> str:
        value_text = str(value)
        pattern = re.compile(rf"(?im)^(\s*\.VAR\s+{re.escape(name)}\s*=\s*)(.+?)\s*$")
        if pattern.search(text):
            return pattern.sub(rf"\g<1>{value_text}", text)
        return f".VAR {name}={value_text}\n{text}"

    def _patch_netlist(self, Kp: float, Ki: float, Kd: float, Kf: float) -> None:
        if self.work_netlist_path is None:
            raise RuntimeError("SIMPLIS transient netlist was not prepared.")
        text = self.work_netlist_path.read_text(encoding="utf-8")
        params = {
            "Kp": f"{Kp:.17g}",
            "Ki": f"{Ki:.17g}",
            "Kd": f"{Kd:.17g}",
            "Kf": f"{Kf:.17g}",
            "tstop": str(getattr(self.config, "sim_time_span", "3e-3")),
            "load_freq": str(getattr(self.config, "load_pulse_frequency", "250")),
            "load_duty": str(getattr(self.config, "load_pulse_duty_cycle", "0.25")),
            "load_delay": str(getattr(self.config, "load_pulse_delay", "1e-3")),
        }
        for name, value in params.items():
            text = self._replace_var(text, name, value)
        self.work_netlist_path.write_text(text, encoding="utf-8", newline="\n")

    def _write_script(self, iter_num: int) -> Path:
        if self.run_dir is None or self.work_netlist_path is None:
            raise RuntimeError("SIMPLIS runner is not prepared.")
        self.processed_netlist_path = self.run_dir / f"tran_iter_{iter_num + 1}_processed.net"
        self.output_path = self.run_dir / f"tran_iter_{iter_num + 1}.txt"
        self.script_path = self.run_dir / f"run_tran_iter_{iter_num + 1}.sxscr"
        script = f"""Set precision = 15
PreProcessNetlist /simulator SIMPLIS "{self.work_netlist_path}" "{self.processed_netlist_path}"
RunSIMPLIS /noerr "{self.processed_netlist_path}"
Show /noerr /file "{self.output_path}" /force Vec('L1#P'), Vec('#VOUT')
Quit
"""
        self.script_path.write_text(script, encoding="utf-8", newline="\n")
        return self.script_path

    def _run_simetrix_script(self, script_path: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [self.simplis_exe, "/s", str(script_path)],
            cwd=str(self.run_dir),
            timeout=600.0,
            capture_output=True,
            text=True,
        )

    def run_transient(self, iter_num: int, Kp: float, Ki: float, Kd: float, Kf: float) -> Tuple[List[str], List[List[float]]]:
        self._ensure_ready()
        self._patch_netlist(Kp, Ki, Kd, Kf)
        script_path = self._write_script(iter_num)
        result = self._run_simetrix_script(script_path)
        if result.returncode != 0:
            raise RuntimeError(
                "SIMetrix/SIMPLIS run failed "
                f"(exit {result.returncode}). stderr: {result.stderr.strip()}"
            )
        if self.output_path is None or not self.output_path.exists():
            raise RuntimeError(f"SIMPLIS export was not created: {self.output_path}")
        header, rows = SimplisExportParser.parse_file(self.output_path)
        if not rows:
            raise RuntimeError(f"SIMPLIS export did not contain waveform rows: {self.output_path}")
        return header, rows
