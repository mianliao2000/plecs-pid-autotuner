"""
LTspice automation helpers for the buck PID auto-tuner.

The module is intentionally independent from auto_tune.py so PyLTSpice remains
an optional runtime dependency until the LTspice backend is selected.
"""

from __future__ import annotations

import math
import os
import shutil
import time
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple


def normalize_backend(value: str) -> str:
    """Normalize a user/config backend name."""
    backend = (value or "plecs").strip().lower()
    if backend in ("ltspice", "lt-spice", "lt"):
        return "ltspice"
    return "plecs"


def find_default_ltspice_exe() -> str:
    """Return the most likely local LTspice executable path, or an empty string."""
    env_path = os.environ.get("LTSPICE_EXE", "").strip().strip('"')
    if env_path and Path(env_path).exists():
        return env_path

    candidates = [
        Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "ADI" / "LTspice" / "LTspice.exe",
        Path(os.environ.get("ProgramFiles", "")) / "ADI" / "LTspice" / "LTspice.exe",
        Path(os.environ.get("ProgramFiles(x86)", "")) / "ADI" / "LTspice" / "LTspice.exe",
        Path(os.environ.get("ProgramFiles", "")) / "LTC" / "LTspiceXVII" / "XVIIx64.exe",
        Path(os.environ.get("ProgramFiles(x86)", "")) / "LTC" / "LTspiceXVII" / "XVIIx64.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())
    return env_path


def import_pyltspice():
    """
    Import the PyLTSpice/spicelib classes used by this project.

    PyLTSpice 5.x exposes much of the implementation through spicelib, while
    older examples import directly from PyLTSpice. Support both shapes.
    """
    try:
        from PyLTSpice import RawRead, SimRunner, SpiceEditor  # type: ignore

        return SimRunner, SpiceEditor, RawRead
    except Exception as pyltspice_error:
        try:
            from spicelib import RawRead, SimRunner, SpiceEditor  # type: ignore

            return SimRunner, SpiceEditor, RawRead
        except Exception as spicelib_error:
            raise ImportError(
                "PyLTSpice/spicelib is not installed. Install it with: pip install PyLTSpice"
            ) from spicelib_error or pyltspice_error


def _unique_run_dir(root: Path, prefix: str = "run") -> Path:
    timestamp = time.strftime(f"{prefix}_%Y%m%d_%H%M%S")
    run_dir = root / timestamp
    suffix = 1
    while run_dir.exists():
        run_dir = root / f"{timestamp}_{suffix:02d}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def _as_float_list(values: Any) -> List[float]:
    if hasattr(values, "tolist"):
        values = values.tolist()
    result: List[float] = []
    for value in values:
        if isinstance(value, complex):
            value = value.real
        result.append(float(value))
    return result


def _as_complex_list(values: Any) -> List[complex]:
    if hasattr(values, "tolist"):
        values = values.tolist()
    return [complex(v) for v in values]


class LtspiceRawParser:
    """Convert LTspice RAW data into the canonical project table format."""

    @staticmethod
    def _trace_names(raw: Any) -> List[str]:
        try:
            return list(raw.get_trace_names())
        except Exception:
            return []

    @staticmethod
    def find_trace_name(names: Sequence[str], exact: Sequence[str], contains: Sequence[str]) -> Optional[str]:
        lowered = {name.lower(): name for name in names}
        for candidate in exact:
            found = lowered.get(candidate.lower())
            if found is not None:
                return found
        for name in names:
            name_l = name.lower()
            if any(token.lower() in name_l for token in contains):
                return name
        return None

    @staticmethod
    def _wave(trace: Any) -> Any:
        try:
            return trace.get_wave()
        except TypeError:
            return trace.get_wave(0)

    @staticmethod
    def parse_transient_raw(raw_filename: Path) -> Tuple[List[str], List[List[float]]]:
        _, _, RawRead = import_pyltspice()
        raw = RawRead(str(raw_filename))
        try:
            axis = raw.get_axis()
        except Exception:
            axis = raw.get_trace("time")
        if hasattr(axis, "get_wave"):
            axis = LtspiceRawParser._wave(axis)
        time_vals = _as_float_list(axis)

        names = LtspiceRawParser._trace_names(raw)
        vout_name = LtspiceRawParser.find_trace_name(names, ["V(vout)", "v(vout)"], ["vout"])
        il_name = LtspiceRawParser.find_trace_name(names, ["I(L1)", "i(l1)"], ["i(l1)", "l1"])
        if vout_name is None:
            raise ValueError(f"LTspice RAW does not contain V(vout). Available traces: {names}")

        vout_vals = _as_float_list(LtspiceRawParser._wave(raw.get_trace(vout_name)))
        if il_name is not None:
            il_vals = _as_float_list(LtspiceRawParser._wave(raw.get_trace(il_name)))
        else:
            il_vals = [0.0] * len(vout_vals)

        # LTspice can emit a synthetic all-zero t=0 row for .tran uic runs even
        # when the next femtosecond-scale point contains the requested ICs.
        # Keep t=0 for plotting, but seed it with the first physical sample.
        if (
            len(time_vals) > 1
            and abs(time_vals[0]) < 1e-18
            and abs(vout_vals[0]) < 1e-9
            and abs(vout_vals[1]) > 1.0
        ):
            vout_vals[0] = vout_vals[1]
            if il_vals:
                il_vals[0] = il_vals[1]

        n = min(len(time_vals), len(vout_vals), len(il_vals))
        return ["Time", "IL", "Vout"], [[time_vals[i], il_vals[i], vout_vals[i]] for i in range(n)]

    @staticmethod
    def parse_ac_raw(raw_filename: Path) -> Tuple[List[float], List[complex]]:
        _, _, RawRead = import_pyltspice()
        raw = RawRead(str(raw_filename))
        try:
            axis = raw.get_axis()
        except Exception:
            axis_name = LtspiceRawParser.find_trace_name(
                LtspiceRawParser._trace_names(raw),
                ["frequency", "freq"],
                ["freq"],
            )
            axis = raw.get_trace(axis_name or "frequency")
        if hasattr(axis, "get_wave"):
            axis = LtspiceRawParser._wave(axis)
        freq_hz = _as_float_list(axis)

        names = LtspiceRawParser._trace_names(raw)
        loop_name = LtspiceRawParser.find_trace_name(names, ["V(loop)", "v(loop)", "V(out)", "v(out)"], ["loop", "out"])
        if loop_name is None:
            raise ValueError(f"LTspice AC RAW does not contain V(loop)/V(out). Available traces: {names}")
        response = _as_complex_list(LtspiceRawParser._wave(raw.get_trace(loop_name)))
        n = min(len(freq_hz), len(response))
        return freq_hz[:n], response[:n]


class LtspiceSimulationRunner:
    """Prepare LTspice working copies, run simulations, and parse RAW outputs."""

    def __init__(self, config: Any):
        self.config = config
        self.run_dir: Optional[Path] = None
        self.work_asc_path: Optional[Path] = None
        self.work_netlist_path: Optional[Path] = None
        self.work_bode_netlist_path: Optional[Path] = None

    @property
    def ltspice_exe(self) -> str:
        return str(getattr(self.config, "ltspice_exe", "") or "")

    def prepare_working_model(self) -> Path:
        work_root = Path(getattr(self.config, "ltspice_work_dir")).resolve()
        self.run_dir = _unique_run_dir(work_root)

        artifacts = [
            ("ltspice_asc_model", "work_asc_path"),
            ("ltspice_netlist_model", "work_netlist_path"),
            ("ltspice_bode_netlist_model", "work_bode_netlist_path"),
        ]
        for config_attr, instance_attr in artifacts:
            source = Path(getattr(self.config, config_attr)).resolve()
            if not source.exists():
                raise FileNotFoundError(f"LTspice model artifact not found: {source}")
            dest = self.run_dir / source.name
            shutil.copyfile(source, dest)
            setattr(self, instance_attr, dest)

        if self.work_netlist_path is None:
            raise RuntimeError("LTspice transient netlist was not prepared.")
        return self.work_netlist_path

    def _ensure_ready(self) -> None:
        if self.run_dir is None or self.work_netlist_path is None:
            self.prepare_working_model()
        if self.ltspice_exe and not Path(self.ltspice_exe).exists():
            raise FileNotFoundError(f"LTspice executable not found: {self.ltspice_exe}")

    def _new_runner(self):
        SimRunner, _, _ = import_pyltspice()
        kwargs = {
            "output_folder": str(self.run_dir),
            "parallel_sims": 1,
            "timeout": 600.0,
            "verbose": False,
        }
        if self.ltspice_exe:
            kwargs["simulator"] = self.ltspice_exe
        return SimRunner(**kwargs)

    @staticmethod
    def _set_parameters(editor: Any, **params: Any) -> None:
        try:
            editor.set_parameters(**params)
            return
        except Exception:
            pass
        for key, value in params.items():
            editor.set_parameter(key, value)

    def _run_now(self, template: Path, run_filename: str, **params: Any) -> Tuple[Path, Path]:
        self._ensure_ready()
        _, SpiceEditor, _ = import_pyltspice()
        editor = SpiceEditor(str(template))
        self._set_parameters(editor, **params)
        runner = self._new_runner()
        raw_file, log_file = runner.run_now(editor, run_filename=run_filename)
        if raw_file is None:
            raise RuntimeError(f"LTspice simulation failed for {run_filename}")
        return Path(raw_file), Path(log_file)

    def _common_params(self, Kp: float, Ki: float, Kd: float, Kf: float) -> dict:
        return {
            "Kp": f"{Kp:.17g}",
            "Ki": f"{Ki:.17g}",
            "Kd": f"{Kd:.17g}",
            "Kf": f"{Kf:.17g}",
            "tstop": str(getattr(self.config, "sim_time_span", "3e-3")),
            "load_freq": str(getattr(self.config, "load_pulse_frequency", "250")),
            "load_duty": str(getattr(self.config, "load_pulse_duty_cycle", "0.25")),
            "load_delay": str(getattr(self.config, "load_pulse_delay", "1e-3")),
        }

    def run_transient(self, iter_num: int, Kp: float, Ki: float, Kd: float, Kf: float) -> Tuple[List[str], List[List[float]]]:
        if self.work_netlist_path is None:
            self.prepare_working_model()
        raw_file, _log_file = self._run_now(
            self.work_netlist_path,
            f"tran_iter_{iter_num + 1}.net",
            **self._common_params(Kp, Ki, Kd, Kf),
        )
        return LtspiceRawParser.parse_transient_raw(raw_file)

    def run_ac(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        Kf: float,
        freq_start_hz: float,
        freq_stop_hz: float,
        num_points: int,
    ) -> Tuple[List[float], List[complex], float]:
        if self.work_bode_netlist_path is None:
            self.prepare_working_model()
        t0 = time.perf_counter()
        raw_file, _log_file = self._run_now(
            self.work_bode_netlist_path,
            f"bode_{int(time.time() * 1000)}.net",
            Kp=f"{Kp:.17g}",
            Ki=f"{Ki:.17g}",
            Kd=f"{Kd:.17g}",
            Kf=f"{Kf:.17g}",
            fstart=f"{freq_start_hz:.17g}",
            fstop=f"{freq_stop_hz:.17g}",
            bode_points=str(max(5, int(num_points))),
        )
        elapsed_s = time.perf_counter() - t0
        freq_hz, response = LtspiceRawParser.parse_ac_raw(raw_file)
        return freq_hz, response, elapsed_s


def analytic_loop_gain(
    Kp: float,
    Ki: float,
    Kd: float,
    Kf: float,
    freq_hz: Sequence[float],
    vdc: float = 12.0,
    l_h: float = 30e-6,
    cout_f: float = 15e-6,
    rc_ohm: float = 7.5e-3,
    rl_ohm: float = 50e-3,
) -> List[complex]:
    """Reference loop-gain calculation used only by tests and fallbacks."""
    response: List[complex] = []
    for freq in freq_hz:
        s = 1j * 2.0 * math.pi * freq
        plant = vdc * (1.0 + s * rc_ohm * cout_f) / (
            1.0 + s * (rc_ohm + rl_ohm) * cout_f + s * s * l_h * cout_f
        )
        comp = Kp + Ki / s + Kd * Kf * s / (s + Kf)
        response.append(comp * plant)
    return response
