"""
PLECS Buck Converter PID Auto-Tuner
====================================
Connects to PLECS via XML-RPC, runs simulations with different PID parameters,
analyzes output voltage response, and tunes Kp, Ki, Kd, Kf to meet:
- Overshoot < 8%
- Undershoot < 8%
- Oscillations <= 2

Uses analytical Type 3 compensator design: tunes crossover frequency (wc)
and phase margin (phi_m) as design variables, then computes Kp/Ki/Kd/Kf
from the plant transfer function.

Usage:
    python auto_tune.py
"""

import xmlrpc.client as x
import time
import csv
import os
import re
import shutil
import subprocess
import cmath
import math
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict


@dataclass
class TuningResult:
    """Result of a single tuning iteration"""
    iter_num: int
    Kp: float
    Ki: float
    Kd: float
    Kf: float
    overshoot: float
    undershoot: float
    osc_count: int
    settling_time: float
    status: str  # "PASS" or "FAIL"


@dataclass
class PlantParams:
    """Buck converter plant parameters"""
    Vdc: float = 12.0       # Input voltage (V)
    Vout: float = 5.0       # Output voltage (V)
    L: float = 30e-6        # Inductance (H)
    Cout: float = 15e-6     # Output capacitance (F)
    Rc: float = 7.5e-3      # ESR of capacitor (Ohm)
    Rl: float = 50e-3       # Inductor DCR (Ohm)
    fsw: float = 250e3      # Switching frequency (Hz)

    @property
    def Gvd0(self) -> float:
        """DC gain of control-to-output transfer function"""
        return self.Vdc

    @property
    def wesr(self) -> float:
        """ESR zero frequency (rad/s)"""
        return 1.0 / (self.Rc * self.Cout)

    @property
    def w0(self) -> float:
        """LC resonant frequency (rad/s)"""
        return 1.0 / math.sqrt(self.L * self.Cout)

    @property
    def Q(self) -> float:
        """Quality factor"""
        return math.sqrt(self.L / self.Cout) / (self.Rc + self.Rl)

    @property
    def fn(self) -> float:
        """LC resonant frequency (Hz)"""
        return self.w0 / (2 * math.pi)

    @property
    def wsw(self) -> float:
        """Switching frequency (rad/s)"""
        return 2 * math.pi * self.fsw


class CompensatorDesign:
    """
    Analytical Type 3 compensator design for buck converter.

    Computes PID parameters (Kp, Ki, Kd, Kf) from two design variables:
      - wc: crossover frequency (rad/s)
      - phi_m: phase margin (rad)

    PID topology (PLECS parallel form):
      C(s) = Kp + Ki/s + Kd*Kf*s/(s + Kf)

    Design equations (from user's MATLAB reference):
      Plant: Gvd(s) = Gvd0*(1+s/wesr) / (1 + s/(Q*w0) + (s/w0)^2)
      Integral pole: wl = wc/10
      Phase boost: phi_boost = phi_target - phi_plant
      Lead-lag zeros/poles:
        wz = wc * sqrt((1-sin(phi_boost))/(1+sin(phi_boost)))
        wp = wc * sqrt((1+sin(phi_boost))/(1-sin(phi_boost)))
      Controller gain at crossover:
        Gpid0 = 1/|Gvd(jwc)| * sqrt((1+(wc/wp)^2)/(1+(wc/wz)^2))
      PID parameters:
        Ki = Gpid0 * wl
        Kf = wp
        Kp = Gpid0*(1 + wl/wz) - Ki/Kf
        Kd = Gpid0/wz - Kp/Kf
    """

    def __init__(self, plant: PlantParams = None):
        self.plant = plant or PlantParams()

    def Gvd_at_wc(self, wc: float) -> complex:
        """Evaluate plant transfer function Gvd(jwc)"""
        p = self.plant
        jwc = 1j * wc
        num = p.Gvd0 * (1 + jwc / p.wesr)
        den = 1 + jwc / (p.Q * p.w0) + (jwc / p.w0) ** 2
        return num / den

    def compute(self, wc: float, phi_m: float) -> Tuple[float, float, float, float]:
        """
        Compute Kp, Ki, Kd, Kf from design variables wc and phi_m.

        Args:
            wc: crossover frequency (rad/s)
            phi_m: phase margin (rad)

        Returns:
            (Kp, Ki, Kd, Kf)
        """
        # Integral pole frequency
        wl = wc / 10.0

        # Plant gain and phase at crossover
        Gvd_wc = self.Gvd_at_wc(wc)
        gain_plant = abs(Gvd_wc)
        phi_plant = cmath.phase(Gvd_wc)

        # Required phase boost
        phi_target = -math.pi + phi_m
        phi_boost = phi_target - phi_plant

        # Clamp phi_boost to avoid numerical issues
        # sin(phi_boost) must be in (-1, 1) for sqrt to work
        phi_boost = max(-math.pi / 2 + 0.01, min(math.pi / 2 - 0.01, phi_boost))

        sin_pb = math.sin(phi_boost)

        # Lead-lag zero and pole
        wz = wc * math.sqrt((1 - sin_pb) / (1 + sin_pb))
        wp = wc * math.sqrt((1 + sin_pb) / (1 - sin_pb))

        # Controller gain at crossover
        Gpid0 = (1.0 / gain_plant) * math.sqrt((1 + (wc / wp) ** 2) / (1 + (wc / wz) ** 2))

        # Convert to PID parameters
        Ki = Gpid0 * wl
        Kf = wp
        Kp = Gpid0 * (1 + wl / wz) - Ki / Kf
        Kd = Gpid0 / wz - Kp / Kf
        Kd = max(0.0, Kd)  # Derivative must be non-negative (negative = unstable)

        return Kp, Ki, Kd, Kf

    def reference_design(self) -> Tuple[float, float, float, float]:
        """Compute reference design at wc=fsw/10, phi_m=60deg"""
        wc = 2 * math.pi * self.plant.fsw / 10.0
        phi_m = 60 * math.pi / 180.0
        return self.compute(wc, phi_m)


@dataclass
class TuningConfig:
    """
    Configuration for the auto-tuner.

    PID parameters are computed analytically from two design variables:
      - wc: crossover frequency (rad/s), range [w0, 2*pi*fsw/5]
      - phi_m: phase margin (rad), range [30deg, 80deg]

    All Kp/Ki/Kd/Kf bounds are derived by sweeping wc and phi_m
    through the compensator design equations.
    """
    plecs_exe: str = r"C:\Users\liaom\Documents\Plexim\PLECS 4.9 (64 bit)\PLECS.exe"
    plecs_model: str = str((Path(__file__).resolve().parent / "synchronous buck.plecs").resolve())
    rpc_url: str = 'http://127.0.0.1:1080/RPC2'
    model_id: str = "synchronous buck"
    work_dir: str = str((Path(__file__).resolve().parent / "plecs_tuning_work").resolve())
    results_dir: str = str((Path(__file__).resolve().parent / "results").resolve())
    target_overshoot: float = 5.0
    target_undershoot: float = 5.0
    max_oscillations: int = 2
    max_iterations: int = 30
    # --- Design variable ranges ---
    # wc: crossover frequency (rad/s)
    #   min: LC resonance w0 ~ 47,140 rad/s (7.5 kHz)
    #   max: fsw/5 = 2*pi*50kHz ~ 314,159 rad/s
    #   nominal: fsw/10 = 2*pi*25kHz ~ 157,080 rad/s
    wc_min: float = 94248.0   # 2*w0 = 15 kHz — minimum safe (below this, near resonance singularity)
    wc_max: float = 314159.0
    wc_initial: float = 94248.0   # Start at minimum (15 kHz) — intentionally slow/bad
    # phi_m: phase margin (rad)
    #   min: 30 deg (aggressive, more ringing)
    #   max: 80 deg (conservative, slower response)
    #   nominal: 60 deg (standard)
    phi_m_min: float = 0.5236    # 30 deg
    phi_m_max: float = 1.3963    # 80 deg
    phi_m_initial: float = 0.5236  # Start at 30 deg — intentionally low phase margin


class PlecsRpc:
    """PLECS XML-RPC interface wrapper"""

    def __init__(self, url: str, plecs_exe: str = "", model_id: str = ""):
        self.url = url
        self.plecs_exe = plecs_exe
        self.model_id = model_id
        self.server: Optional[x.ServerProxy] = None

    def connect(self, retries: int = 20, delay: float = 1.0) -> 'PlecsRpc':
        """Connect to PLECS RPC server"""
        last_err = None
        for _ in range(retries):
            try:
                self.server = x.ServerProxy(self.url, allow_none=True)
                self.server.plecs.statistics()
                print(f"  Connected to PLECS at {self.url}")
                return self
            except Exception as e:
                last_err = e
                time.sleep(delay)
        raise ConnectionError(f"Failed to connect to PLECS: {last_err}")

    def ensure_plecs_running(self) -> bool:
        """Start PLECS if not running, return True if we started it"""
        try:
            self.connect(retries=1)
            return False  # Already running
        except ConnectionError:
            if not self.plecs_exe:
                raise ConnectionError("PLECS not running and no executable path configured")
            print("  PLECS not running, starting...")
            subprocess.Popen([self.plecs_exe])
            self.connect(retries=30, delay=1.0)
            return True

    def load_model(self, model_path: str) -> None:
        """Load model, closing any existing model with the same name first"""
        self.model_id = Path(model_path).stem
        stats = self.server.plecs.statistics()
        ids = [m['id'] for m in stats.get('models', [])]
        if self.model_id in ids:
            try:
                self.server.plecs.close(self.model_id)
                time.sleep(0.3)
            except Exception:
                pass
        self.server.plecs.load(str(Path(model_path).resolve()))
        time.sleep(1.0)

    def start_simulation(self) -> None:
        """Start simulation"""
        self.server.plecs.webserver('startSimulation', self.model_id, {})

    def get_simulation_state(self) -> str:
        """Get current simulation state"""
        return self.server.plecs.webserver('getSimulationState', self.model_id, {})

    def wait_simulation_done(self, poll_interval: float = 0.2, timeout: float = 60.0) -> None:
        """Wait for simulation to complete"""
        start = time.time()
        while True:
            state = self.get_simulation_state()
            txt = state.get('state', '') if isinstance(state, dict) else str(state)
            if 'running' not in txt.lower():
                break
            if time.time() - start > timeout:
                raise TimeoutError(f"Simulation did not finish within {timeout}s")
            time.sleep(poll_interval)

    def get_scope_csv(self, scope_path: str = '') -> bytes:
        """Get scope CSV data"""
        res = self.server.plecs.webserver('getScopeCsv', scope_path, {})
        if isinstance(res, dict):
            blob = res.get('csv', b'')
        else:
            blob = res
        # Handle XML-RPC binary data
        if hasattr(blob, 'data'):
            return blob.data
        return blob


class PlecsModelEditor:
    """Edit PLECS model file to change parameters"""

    def __init__(self, original_path: str, work_dir: str):
        self.original_path = Path(original_path)
        self.work_dir = Path(work_dir)
        self.work_dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _downgrade_for_plecs_49(content: str) -> str:
        """
        Remove top-level model options that are valid in PLECS 5.x but rejected
        by PLECS 4.9. This keeps the original model untouched and only adjusts
        the generated temp copy used for RPC runs.
        """
        content = re.sub(r'(^\s*Version\s+")5\.0(")', r'\g<1>4.9\2', content, flags=re.MULTILINE)

        unsupported_fields = [
            "JacobianComputation",
            "MixedSignalSolverBypassOperatingPoint",
            "MixedSignalSolverInitialConditions",
            "MixedSignalSolverMaxNewtonIterations",
            "MixedSignalSolverNumThreads",
            "OperatingPointMaxNewtonIterations",
            "OperatingPointRelativeNewtonTolerance",
            "OperatingPointAbsoluteNewtonTolerance",
            "OperatingPointNodalVoltageNewtonTolerance",
        ]

        for field in unsupported_fields:
            content = re.sub(
                rf'^\s*{re.escape(field)}\s+".*?"\s*\r?\n',
                '',
                content,
                flags=re.MULTILINE,
            )
        return content

    def modify_params(self, Kp: float, Ki: float, Kd: float, Kf: float) -> str:
        """Modify PID parameters in .plecs file, return path to modified file"""
        with open(self.original_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Replace parameter values with regex
        # Some params may be split across quoted strings in .plecs format
        # e.g. Kp"\n" = 1; — handle both normal and split cases
        content = re.sub(r'Ki\s*=\s*[\d.e+-]+;', f'Ki = {Ki};', content)
        content = re.sub(r'Kf\s*=\s*[\d.e+-]+;', f'Kf = {Kf};', content)
        content = re.sub(r'Kd\s*=\s*[\d.e+-]+;', f'Kd = {Kd};', content)
        # Kp may be split across quoted string boundary: Kp"\n" = value;
        if re.search(r'Kp\s*=\s*[\d.e+-]+;', content):
            content = re.sub(r'Kp\s*=\s*[\d.e+-]+;', f'Kp = {Kp};', content)
        else:
            content = re.sub(r'Kp"\n"\s*=\s*[\d.e+-]+;', f'Kp = {Kp};"\n"', content)

        # The checked-in model was saved by PLECS 5.0, but this machine has
        # PLECS 4.9 installed. Strip unsupported 5.x-only fields in the temp
        # copy so the model can still be loaded by the local installation.
        content = self._downgrade_for_plecs_49(content)

        # Write to working directory
        out_path = self.work_dir / "temp_model.plecs"
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(content)

        return str(out_path)


class ScopeCsvParser:
    """Parse scope CSV data from PLECS"""

    @staticmethod
    def parse(csv_data: bytes) -> Tuple[List[str], List[List[float]]]:
        """Parse CSV data, return (header, data)"""
        text = csv_data.decode('utf-8', errors='replace')
        lines = text.strip().split('\n')
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


class ResponseAnalyzer:
    """
    Analyze buck converter output voltage response.

    Strategy:
    - OS/US: measured from RAW data vs target voltage. The heavy filter
      smooths out real peaks and gives false low values.
    - Load step detection: auto-detect from the current channel (abrupt IL jump).
    - Oscillation count: use a LIGHT filter (1 switching cycle) to remove ripple,
      then count envelope peaks using local extrema — not zero-crossings.
    - Settling time: on lightly-filtered data within ±2% of target.
    """

    def __init__(self, v_target: float = 5.0,
                 transient_window: float = 0.003,
                 ripple_filter_cycles: int = 4,
                 fsw: float = 250e3):
        self.v_target = v_target          # Target output voltage (V)
        self.transient_window = transient_window  # How long after step to analyse (s)
        self.ripple_filter_cycles = ripple_filter_cycles
        self.fsw = fsw                    # Switching frequency (Hz)

    @staticmethod
    def _moving_average(vals: List[float], window: int) -> List[float]:
        if window <= 1 or len(vals) < window:
            return vals[:]
        cumsum = [0.0]
        for v in vals:
            cumsum.append(cumsum[-1] + v)
        half = window // 2
        result = []
        for i in range(len(vals)):
            lo = max(0, i - half)
            hi = min(len(vals), i + half + 1)
            result.append((cumsum[hi] - cumsum[lo]) / (hi - lo))
        return result

    @staticmethod
    def _find_col(header: List[str], *keywords) -> int:
        for i, h in enumerate(header):
            hl = h.lower()
            if any(k in hl for k in keywords):
                return i
        return -1

    @staticmethod
    def _find_idx(time_vals: List[float], target: float) -> int:
        lo, hi = 0, len(time_vals) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if time_vals[mid] < target:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def _detect_step_times(self, time_vals: List[float],
                           il_vals: List[float]) -> List[float]:
        """
        Detect load step times from the current channel.
        Look for the largest abrupt jumps in IL.
        """
        n = len(il_vals)
        # Use a coarse derivative: compare 50-sample blocks
        block = max(1, n // 200)
        jumps = []
        for i in range(block, n - block, block):
            dil = abs(il_vals[i] - il_vals[i - block])
            jumps.append((dil, time_vals[i]))

        if not jumps:
            return []

        jumps.sort(reverse=True)
        threshold = jumps[0][0] * 0.3  # at least 30% of the biggest jump

        step_times = []
        for mag, t in jumps:
            if mag < threshold:
                break
            # Avoid duplicates within 1ms
            if not any(abs(t - st) < 0.001 for st in step_times):
                step_times.append(t)

        return sorted(step_times)

    def _count_oscillations(self, vout_filt: List[float],
                            time_vals: List[float],
                            start_idx: int, end_idx: int) -> int:
        """
        Count oscillation cycles using envelope local extrema.

        Uses a wide neighborhood (half an LC period) for peak detection to
        avoid counting switching ripple residuals as separate oscillations.
        Also requires minimum time spacing between consecutive peaks/valleys.
        """
        seg = vout_filt[start_idx:end_idx]
        seg_t = time_vals[start_idx:end_idx]
        if len(seg) < 10:
            return 0

        # LC period ~ 1/7500 Hz = 133 us. Use half-period as neighborhood.
        # With avg_dt ~ 0.115us, that's ~580 samples. Use ~1/4 period for
        # a balance between catching real oscillations and filtering ripple.
        total_t = seg_t[-1] - seg_t[0]
        avg_dt = total_t / (len(seg) - 1) if len(seg) > 1 else 1e-6
        lc_period = 1.0 / 7500.0  # ~133 us
        half_n = max(5, int(lc_period / 4.0 / avg_dt))  # ~quarter LC period

        # Find local maxima and minima with wide neighborhood
        peaks, valleys = [], []  # (index, value)
        for i in range(half_n, len(seg) - half_n):
            window = seg[i - half_n:i + half_n + 1]
            if seg[i] == max(window):
                peaks.append((i, seg[i]))
            elif seg[i] == min(window):
                valleys.append((i, seg[i]))

        # Deduplicate: keep only the most extreme peak/valley within each
        # half LC period
        min_spacing = lc_period / 2.0
        def dedupe(extrema, is_peak):
            if not extrema:
                return []
            result = [extrema[0]]
            for idx, val in extrema[1:]:
                t_prev = seg_t[result[-1][0]]
                t_curr = seg_t[idx]
                if t_curr - t_prev < min_spacing:
                    # Keep the more extreme one
                    if (is_peak and val > result[-1][1]) or \
                       (not is_peak and val < result[-1][1]):
                        result[-1] = (idx, val)
                else:
                    result.append((idx, val))
            return result

        peaks = dedupe(peaks, is_peak=True)
        valleys = dedupe(valleys, is_peak=False)

        # Threshold: only count peaks/valleys that deviate > 1% from target
        threshold = self.v_target * 0.01
        sig_peaks   = [p for _, p in peaks   if p - self.v_target >  threshold]
        sig_valleys = [v for _, v in valleys if self.v_target - v >  threshold]

        return max(len(sig_peaks), len(sig_valleys))

    def analyze(self, header: List[str], data: List[List[float]]) -> Tuple[float, float, int, float]:
        """
        Analyze Vout response.
        Returns: (overshoot %, undershoot %, oscillation count, settling time)
        """
        if not data or len(data) < 10:
            return 10.0, 10.0, 5, 0.005

        n = len(data)
        time_col = 0
        vout_col = self._find_col(header, 'voltage', 'vout', 'output')
        if vout_col < 0:
            vout_col = 2 if len(header) >= 3 else 1
        il_col = self._find_col(header, 'current', 'il', 'load')
        if il_col < 0:
            il_col = 1

        time_vals = [row[time_col] for row in data]
        vout_raw  = [row[vout_col] for row in data]
        il_vals   = [row[il_col]   for row in data]

        # Estimate samples per switching cycle for ripple filter
        total_time = time_vals[-1] - time_vals[0]
        avg_dt = total_time / (n - 1) if n > 1 else 1e-6
        samples_per_cycle = max(1, int(1.0 / (self.fsw * avg_dt)))
        ripple_window = samples_per_cycle * self.ripple_filter_cycles
        vout_filt = self._moving_average(vout_raw, ripple_window)

        # Auto-detect load step times
        step_times = self._detect_step_times(time_vals, il_vals)
        if not step_times:
            step_times = [time_vals[n // 2]]

        # Skip startup transient: exclude first 25% of simulation
        # (startup from 0V to Vout takes ~1-2ms and looks like a huge step)
        sim_duration = time_vals[-1] - time_vals[0]
        startup_end = time_vals[0] + sim_duration * 0.25
        steps_after_startup = [t for t in step_times if t > startup_end]
        step_t = steps_after_startup[0] if steps_after_startup else step_times[-1]

        step_idx  = self._find_idx(time_vals, step_t)
        end_idx   = self._find_idx(time_vals, step_t + self.transient_window)
        end_idx   = min(end_idx, n - 1)

        # OS/US: use RAW data (don't filter — filter kills the peaks)
        seg_raw = vout_raw[step_idx:end_idx]
        v_peak   = max(seg_raw) if seg_raw else self.v_target
        v_valley = min(seg_raw) if seg_raw else self.v_target

        overshoot  = max(0.0, (v_peak   - self.v_target) / self.v_target * 100)
        undershoot = max(0.0, (self.v_target - v_valley) / self.v_target * 100)

        # Oscillation count: on lightly-filtered signal, count envelope peaks
        osc_count = self._count_oscillations(vout_filt, time_vals, step_idx, end_idx)

        # Settling time: lightly-filtered within ±2% of target
        v_band = self.v_target * 0.02
        settling_time = self.transient_window
        check_block = max(10, ripple_window)
        for i in range(step_idx, end_idx - check_block):
            block = vout_filt[i:i + check_block]
            if max(block) < self.v_target + v_band and min(block) > self.v_target - v_band:
                settling_time = time_vals[i] - step_t
                break

        print(f"    [Analyzer] Step@{step_t*1000:.2f}ms | "
              f"raw Vmax={v_peak:.3f} Vmin={v_valley:.3f} | "
              f"OS={overshoot:.1f}% US={undershoot:.1f}% Osc={osc_count}")

        return overshoot, undershoot, osc_count, settling_time


class PidTuner:
    """
    PID tuning via analytical compensator design.

    Instead of independently drifting Kp/Ki/Kd/Kf with heuristics,
    this tuner adjusts two design variables:
      - wc: crossover frequency (rad/s)
      - phi_m: phase margin (rad)

    Then recomputes all four PID parameters analytically using
    CompensatorDesign. This guarantees physical consistency.

    Tuning strategy:
    - High overshoot → reduce wc (lower bandwidth) or increase phi_m (more damping)
    - High undershoot → increase wc (higher bandwidth)
    - Oscillations → increase phi_m (more phase margin = more damping)
    - Both OS and US high → adjust phi_m first (resonance issue)
    """

    def __init__(self, config: TuningConfig):
        self.config = config
        self.compensator = CompensatorDesign()
        self.iteration = 0
        self.best_score: float = float('inf')
        self.best_wc: float = config.wc_initial
        self.best_phi_m: float = config.phi_m_initial
        self.stall_count: int = 0
        # Current design variables
        self.wc = config.wc_initial
        self.phi_m = config.phi_m_initial

    def _score(self, overshoot: float, undershoot: float, osc_count: int) -> float:
        """Compute a scalar score (lower is better) for comparison."""
        return (max(0, overshoot - self.config.target_overshoot) +
                max(0, undershoot - self.config.target_undershoot) +
                max(0, osc_count - self.config.max_oscillations) * 3.0)

    def _decay_factor(self) -> float:
        """Reduce adjustment aggressiveness as iterations progress."""
        return max(0.2, 1.0 - self.iteration * 0.05)

    def get_initial_params(self) -> Tuple[float, float, float, float]:
        """Compute initial PID parameters from initial design variables."""
        return self.compensator.compute(self.wc, self.phi_m)

    def adjust(self, Kp: float, Ki: float, Kd: float, Kf: float,
               overshoot: float, undershoot: float, osc_count: int) -> Tuple[float, float, float, float]:
        """
        Adjust design variables (wc, phi_m) based on response analysis,
        then recompute all PID parameters analytically.
        """
        self.iteration += 1
        decay = self._decay_factor()

        # Track best result
        score = self._score(overshoot, undershoot, osc_count)
        if score < self.best_score:
            self.best_score = score
            self.best_wc = self.wc
            self.best_phi_m = self.phi_m
            self.stall_count = 0
        else:
            self.stall_count += 1

        # If stuck for 5 iterations, revert to best and perturb
        if self.stall_count >= 5:
            self.wc = self.best_wc
            self.phi_m = self.best_phi_m
            # If stuck at wc_min with oscillations, force a larger jump upward
            if self.wc <= self.config.wc_min * 1.05 and osc_count > self.config.max_oscillations:
                self.wc = self.config.wc_min * 2.0
                self.phi_m = self.config.phi_m_initial
                print(f"    -> Stuck at wc_min with oscillations: jump to wc={self.wc:.0f}")
            else:
                self.wc *= (1.0 + 0.10 * (1 if undershoot > overshoot else -1))
                self.phi_m += 0.02 * (1 if osc_count > self.config.max_oscillations else -1)
                print(f"    -> Reverting to best (score={self.best_score:.2f}), perturbing")
            self.stall_count = 0
        else:
            os_fail = overshoot > self.config.target_overshoot
            us_fail = undershoot > self.config.target_undershoot
            osc_fail = osc_count > self.config.max_oscillations

            if osc_fail:
                # Oscillations: need more phase margin (more damping)
                self.phi_m += 0.05 * decay
                # Also slightly reduce wc to move away from resonance
                self.wc *= (1.0 - 0.05 * decay)
                print(f"    -> Oscillatory ({osc_count}): increase phi_m, reduce wc (decay={decay:.2f})")

            elif os_fail and not us_fail:
                # Pure overshoot: reduce bandwidth or increase phase margin
                if overshoot > 2 * self.config.target_overshoot:
                    # Very high OS: reduce wc significantly
                    self.wc *= (1.0 - 0.15 * decay)
                    print(f"    -> High overshoot: reduce wc (decay={decay:.2f})")
                else:
                    # Moderate OS: increase phase margin for more damping
                    self.phi_m += 0.03 * decay
                    self.wc *= (1.0 - 0.05 * decay)
                    print(f"    -> Overshoot: increase phi_m, slightly reduce wc (decay={decay:.2f})")

            elif us_fail and not os_fail:
                # Pure undershoot: increase bandwidth
                self.wc *= (1.0 + 0.10 * decay)
                print(f"    -> Undershoot: increase wc (decay={decay:.2f})")

            elif os_fail and us_fail:
                # Both fail: likely near resonance, adjust phi_m primarily
                if undershoot > overshoot * 1.5:
                    # US dominant: need more bandwidth
                    self.wc *= (1.0 + 0.08 * decay)
                    self.phi_m += 0.02 * decay
                    print(f"    -> Both fail (US dominant): increase wc, slight phi_m (decay={decay:.2f})")
                elif overshoot > undershoot * 1.5:
                    # OS dominant: need less bandwidth or more damping
                    self.wc *= (1.0 - 0.08 * decay)
                    self.phi_m += 0.03 * decay
                    print(f"    -> Both fail (OS dominant): reduce wc, increase phi_m (decay={decay:.2f})")
                else:
                    # Balanced: increase phi_m
                    self.phi_m += 0.05 * decay
                    print(f"    -> Both fail (balanced): increase phi_m (decay={decay:.2f})")
            else:
                # Passing but still iterating (shouldn't happen, but safe)
                print(f"    -> Passing, no adjustment needed")

        # Clamp design variables
        self.wc = max(self.config.wc_min, min(self.wc, self.config.wc_max))
        self.phi_m = max(self.config.phi_m_min, min(self.phi_m, self.config.phi_m_max))

        # Recompute PID parameters analytically
        Kp, Ki, Kd, Kf = self.compensator.compute(self.wc, self.phi_m)

        print(f"    -> Design: wc={self.wc:.0f} rad/s ({self.wc/(2*math.pi):.0f} Hz), "
              f"phi_m={math.degrees(self.phi_m):.1f} deg")
        print(f"    -> PID: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")

        return Kp, Ki, Kd, Kf

    def check_pass(self, overshoot: float, undershoot: float, osc_count: int) -> bool:
        """Check if tuning criteria are met"""
        return (overshoot < self.config.target_overshoot and
                undershoot < self.config.target_undershoot and
                osc_count <= self.config.max_oscillations)


class AutoTuner:
    """Main auto-tuning orchestrator"""

    def __init__(self, config: TuningConfig = None):
        self.config = config or TuningConfig()
        self.plecs = PlecsRpc(
            self.config.rpc_url,
            plecs_exe=self.config.plecs_exe,
            model_id=self.config.model_id
        )
        self.model_editor = PlecsModelEditor(
            self.config.plecs_model,
            self.config.work_dir
        )
        self.analyzer = ResponseAnalyzer(v_target=5.0, fsw=250e3)
        self.tuner = PidTuner(self.config)
        self.csv_parser = ScopeCsvParser()
        self.results: List[TuningResult] = []

    def setup(self) -> None:
        """Initialize PLECS connection and results directory"""
        print("=" * 60)
        print("PLECS Buck Converter PID Auto-Tuner")
        print("=" * 60)

        # Create results directory
        Path(self.config.results_dir).mkdir(parents=True, exist_ok=True)

        # Connect to PLECS (starts it if not running)
        print("\n[1] Connecting to PLECS...")
        self.plecs.ensure_plecs_running()

    def run_iteration(self, iter_num: int, Kp: float, Ki: float,
                      Kd: float, Kf: float) -> TuningResult:
        """Run a single tuning iteration"""
        print(f"\n[Iteration {iter_num}]")
        print(f"  Parameters: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")

        # Modify .plecs file
        model_path = self.model_editor.modify_params(Kp, Ki, Kd, Kf)

        # Load model
        print(f"  Loading model...")
        self.plecs.load_model(model_path)
        time.sleep(0.5)

        # Run simulation
        print(f"  Running simulation...")
        try:
            self.plecs.start_simulation()
            self.plecs.wait_simulation_done()
        except Exception as e:
            print(f"  Simulation error: {e}")
            return TuningResult(iter_num, Kp, Ki, Kd, Kf, 10, 10, 5, 0.005, "FAIL")

        # Export scope CSV (use actual loaded model_id, not config)
        scope_path = f'{self.plecs.model_id}/Scope'
        print(f"  Exporting scope data...")
        try:
            csv_data = self.plecs.get_scope_csv(scope_path)
        except Exception as e:
            print(f"  Export error: {e}")
            return TuningResult(iter_num, Kp, Ki, Kd, Kf, 10, 10, 5, 0.005, "FAIL")

        # Save CSV for this iteration
        csv_path = Path(self.config.results_dir) / f"iter_{iter_num:03d}.csv"
        csv_path.write_bytes(csv_data)

        # Parse and analyze
        header, data = self.csv_parser.parse(csv_data)
        overshoot, undershoot, osc_count, settling_time = self.analyzer.analyze(header, data)

        status = "PASS" if self.tuner.check_pass(overshoot, undershoot, osc_count) else "FAIL"

        result = TuningResult(
            iter_num=iter_num,
            Kp=Kp, Ki=Ki, Kd=Kd, Kf=Kf,
            overshoot=overshoot,
            undershoot=undershoot,
            osc_count=osc_count,
            settling_time=settling_time,
            status=status
        )

        print(f"  Results: OS={overshoot:.1f}%, US={undershoot:.1f}%, "
              f"Osc={osc_count}, Settling={settling_time*1000:.1f}ms -> {status}")

        return result

    def save_log(self) -> None:
        """Save tuning log to CSV"""
        log_path = Path(self.config.results_dir) / "tuning_log.csv"
        with open(log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Iter', 'Kp', 'Ki', 'Kd', 'Kf',
                           'Overshoot', 'Undershoot', 'OscCount',
                           'SettlingTime', 'Status'])
            for r in self.results:
                writer.writerow([r.iter_num, r.Kp, r.Ki, r.Kd, r.Kf,
                               r.overshoot, r.undershoot, r.osc_count,
                               r.settling_time, r.status])
        print(f"\n[Log] Saved to {log_path}")

    def tune(self) -> Optional[TuningResult]:
        """Run the full auto-tuning process"""
        self.setup()

        # Compute initial PID parameters from design variables
        Kp, Ki, Kd, Kf = self.tuner.get_initial_params()

        print(f"\n[2] Starting tuning...")
        print(f"    Design: wc={self.config.wc_initial:.0f} rad/s "
              f"({self.config.wc_initial/(2*math.pi):.0f} Hz), "
              f"phi_m={math.degrees(self.config.phi_m_initial):.0f} deg")
        print(f"    Initial: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")
        print(f"    Targets: OS<{self.config.target_overshoot}%, "
              f"US<{self.config.target_undershoot}%, "
              f"Osc<={self.config.max_oscillations}")

        # Print reference design for comparison
        comp = CompensatorDesign()
        ref_Kp, ref_Ki, ref_Kd, ref_Kf = comp.reference_design()
        print(f"    Reference: Kp={ref_Kp:.5f}, Ki={ref_Ki:.2f}, "
              f"Kd={ref_Kd:.2e}, Kf={ref_Kf:.0f}")
        print("=" * 60)

        for i in range(self.config.max_iterations):
            # Run iteration
            result = self.run_iteration(i, Kp, Ki, Kd, Kf)
            self.results.append(result)

            if result.status == "PASS":
                print("\n" + "=" * 60)
                print("*** TUNING SUCCESSFUL ***")
                print(f"Design: wc={self.tuner.wc:.0f} rad/s "
                      f"({self.tuner.wc/(2*math.pi):.0f} Hz), "
                      f"phi_m={math.degrees(self.tuner.phi_m):.1f} deg")
                print(f"Final: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")
                print(f"Overshoot={result.overshoot:.2f}%, "
                      f"Undershoot={result.undershoot:.2f}%, "
                      f"Oscillations={result.osc_count}")
                print("=" * 60)
                self.save_log()
                return result

            # Adjust parameters for next iteration
            Kp, Ki, Kd, Kf = self.tuner.adjust(
                Kp, Ki, Kd, Kf,
                result.overshoot, result.undershoot, result.osc_count
            )

        print(f"\nMax iterations ({self.config.max_iterations}) reached")
        self.save_log()
        return None


def main():
    # Print reference design first
    print("Computing reference compensator design...")
    comp = CompensatorDesign()
    Kp, Ki, Kd, Kf = comp.reference_design()
    print(f"  Reference: Kp={Kp:.5f}, Ki={Ki:.5f}, Kd={Kd:.6e}, Kf={Kf:.5f}")
    print()

    config = TuningConfig()
    autotuner = AutoTuner(config)
    final_result = autotuner.tune()

    if final_result:
        print(f"\nFinal parameters saved to {config.results_dir}")
        print("\nNext steps:")
        print("  python analyze.py  # Generate visualization plots")


if __name__ == "__main__":
    main()
