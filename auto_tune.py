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


def result_priority(result: TuningResult) -> Tuple[float, float, int, float, int]:
    """
    Rank iterations by how small and balanced OS/US are.

    Priority order:
    1. Smaller worst-case deviation between OS and US
    2. Smaller combined OS + US
    3. Fewer oscillations
    4. Faster settling
    5. Earlier iteration number
    """
    return (
        max(result.overshoot, result.undershoot),
        result.overshoot + result.undershoot,
        result.osc_count,
        result.settling_time,
        result.iter_num,
    )


def select_best_result(results: List[TuningResult]) -> Optional[TuningResult]:
    """Return the best iteration seen so far."""
    if not results:
        return None
    return min(results, key=result_priority)


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
    results_dir: str = str((Path(__file__).resolve().parent / "results").resolve())
    sim_time_span: str = "1.2e-3"
    load_pulse_frequency: str = "1000"
    load_pulse_duty_cycle: str = "0.5"
    load_pulse_delay: str = "4e-4"
    cout_v_init: str = "Vout"
    inductor_i_init: str = "Vout/5"
    target_overshoot: float = 5.0
    target_undershoot: float = 5.0
    max_oscillations: int = 2
    max_iterations: int = 101
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

    def meets_targets(self, overshoot: float, undershoot: float, osc_count: int) -> bool:
        """Return True when the measured metrics satisfy the user targets."""
        return (
            overshoot < self.target_overshoot and
            undershoot < self.target_undershoot and
            osc_count <= self.max_oscillations
        )


class PlecsRpc:
    """PLECS XML-RPC interface wrapper"""

    def __init__(self, url: str, plecs_exe: str = "", model_id: str = ""):
        self.url = url
        self.plecs_exe = plecs_exe
        self.model_id = model_id
        self.server: Optional[x.ServerProxy] = None
        self.loaded_model_path: Optional[Path] = None

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

    def load_model(self, model_path: str, force_reload: bool = False) -> None:
        """Load a model once and reuse it unless an explicit reload is requested."""
        resolved_path = Path(model_path).resolve()
        self.model_id = resolved_path.stem
        stats = self.server.plecs.statistics()
        loaded = {
            m['id']: Path(m['filename']).resolve()
            for m in stats.get('models', [])
            if 'id' in m and 'filename' in m
        }
        if not force_reload and loaded.get(self.model_id) == resolved_path:
            self.loaded_model_path = resolved_path
            return
        if force_reload and self.model_id in loaded:
            try:
                self.server.plecs.close(self.model_id)
                time.sleep(0.3)
            except Exception:
                pass
        self.server.plecs.load(str(resolved_path))
        self.loaded_model_path = resolved_path
        time.sleep(1.0)

    def find_loaded_model_path(self, model_id: str) -> Optional[Path]:
        """Return the on-disk path of a loaded model, if present."""
        stats = self.server.plecs.statistics()
        for model in stats.get('models', []):
            if model.get('id') == model_id and model.get('filename'):
                return Path(model['filename']).resolve()
        return None

    def close_model(self, model_id: str) -> bool:
        """Close a loaded model if it is currently open."""
        if self.find_loaded_model_path(model_id) is None:
            return False
        try:
            self.server.plecs.close(model_id)
            time.sleep(0.3)
            return True
        except Exception:
            return False

    def get_parameter(self, component_path: str, parameter: str):
        """Read a model or component parameter."""
        return self.server.plecs.get(component_path, parameter)

    def set_parameter(self, component_path: str, parameter: str, value) -> None:
        """Write a model or component parameter."""
        self.server.plecs.set(component_path, parameter, value)

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

    def simulate(self) -> Dict[str, List[List[float]]]:
        """Run the loaded model and return raw simulation outputs."""
        return self.server.plecs.simulate(self.model_id)


class PlecsModelEditor:
    """Update tunable commands inside the loaded PLECS model."""

    @staticmethod
    def update_initialization_commands(commands: str, Kp: float, Ki: float, Kd: float, Kf: float) -> str:
        """Update only the controller gains inside a loaded model."""
        commands = re.sub(r'Ki\s*=\s*[\d.e+-]+;', f'Ki = {Ki};', commands)
        commands = re.sub(r'Kf\s*=\s*[\d.e+-]+;', f'Kf = {Kf};', commands)
        commands = re.sub(r'Kd\s*=\s*[\d.e+-]+;', f'Kd = {Kd};', commands)
        commands = re.sub(r'Kp\s*=\s*[\d.e+-]+;', f'Kp = {Kp};', commands)
        return commands


class ScopeCsvParser:
    """Convert between PLECS simulation outputs and the CSV shape used by the GUI."""

    @staticmethod
    def from_simulation_result(sim_result: Dict[str, List[List[float]]]) -> Tuple[List[str], List[List[float]]]:
        """
        Map direct PLECS simulation outputs to the project's canonical table:
        Time, IL, Vout

        Signal Output 1 is treated as Vout and Signal Output 2 as IL.
        """
        time_vals = sim_result.get('Time', [])
        values = sim_result.get('Values', [])
        if len(values) < 2:
            raise ValueError("PLECS simulate() did not return two signal outputs.")

        vout_vals = values[0]
        il_vals = values[1]
        n = min(len(time_vals), len(vout_vals), len(il_vals))
        header = ["Time", "IL", "Vout"]
        data = [[time_vals[i], il_vals[i], vout_vals[i]] for i in range(n)]
        return header, data

    @staticmethod
    def to_csv_bytes(header: List[str], data: List[List[float]]) -> bytes:
        """Serialize tabular simulation data to CSV bytes for GUI/history use."""
        lines = [",".join(header)]
        for row in data:
            lines.append(",".join(f"{val:.17g}" for val in row))
        return ("\n".join(lines) + "\n").encode("utf-8")

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
                 transient_window: float = 0.00035,
                 ripple_filter_cycles: int = 4,
                 fsw: float = 250e3,
                 expected_step_times: Optional[List[float]] = None):
        self.v_target = v_target          # Target output voltage (V)
        self.transient_window = transient_window  # How long after step to analyse (s)
        self.ripple_filter_cycles = ripple_filter_cycles
        self.fsw = fsw                    # Switching frequency (Hz)
        self.expected_step_times = expected_step_times or []

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
        duplicate_window = max(8.0 / self.fsw, self.transient_window / 3.0)
        for mag, t in jumps:
            if mag < threshold:
                break
            # Avoid counting the same edge multiple times while still letting
            # closely spaced step-up / step-down events survive.
            if not any(abs(t - st) < duplicate_window for st in step_times):
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

    def _analyze_single_step(self, time_vals: List[float], vout_raw: List[float],
                             vout_filt: List[float], step_t: float,
                             window_end_t: float, ripple_window: int) -> Tuple[float, float, int, float]:
        """Analyze one transient window and return OS/US/osc/settling metrics."""
        n = len(time_vals)
        step_idx = self._find_idx(time_vals, step_t)
        end_idx = min(self._find_idx(time_vals, window_end_t), n - 1)
        if end_idx <= step_idx + 5:
            return 0.0, 0.0, 0, 0.0

        seg_raw = vout_raw[step_idx:end_idx]
        v_peak = max(seg_raw) if seg_raw else self.v_target
        v_valley = min(seg_raw) if seg_raw else self.v_target
        overshoot = max(0.0, (v_peak - self.v_target) / self.v_target * 100)
        undershoot = max(0.0, (self.v_target - v_valley) / self.v_target * 100)
        osc_count = self._count_oscillations(vout_filt, time_vals, step_idx, end_idx)

        v_band = self.v_target * 0.02
        settling_time = max(0.0, window_end_t - step_t)
        check_block = max(10, ripple_window)
        for i in range(step_idx, max(step_idx + 1, end_idx - check_block)):
            block = vout_filt[i:i + check_block]
            if max(block) < self.v_target + v_band and min(block) > self.v_target - v_band:
                settling_time = time_vals[i] - step_t
                break

        return overshoot, undershoot, osc_count, settling_time

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

    def analyze(self, header: List[str], data: List[List[float]]) -> Tuple[float, float, int, float]:
        """
        Analyze Vout response across the first two meaningful transients
        after startup and return the worst-case metrics.
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
        vout_raw = [row[vout_col] for row in data]
        il_vals = [row[il_col] for row in data]

        total_time = time_vals[-1] - time_vals[0]
        avg_dt = total_time / (n - 1) if n > 1 else 1e-6
        samples_per_cycle = max(1, int(1.0 / (self.fsw * avg_dt)))
        ripple_window = samples_per_cycle * self.ripple_filter_cycles
        vout_filt = self._moving_average(vout_raw, ripple_window)

        if self.expected_step_times:
            selected_steps = [
                t for t in self.expected_step_times
                if time_vals[0] <= t <= time_vals[-1]
            ]
        else:
            step_times = self._detect_step_times(time_vals, il_vals)
            if not step_times:
                step_times = [time_vals[n // 2]]
            sim_duration = time_vals[-1] - time_vals[0]
            startup_end = time_vals[0] + sim_duration * 0.30
            steps_after_startup = [t for t in step_times if t > startup_end]
            selected_steps = steps_after_startup[:2] if steps_after_startup else step_times[-1:]

        step_results = []
        for idx, step_t in enumerate(selected_steps):
            next_step_t = selected_steps[idx + 1] if idx + 1 < len(selected_steps) else time_vals[-1]
            window_end_t = min(step_t + self.transient_window, next_step_t - avg_dt)
            if window_end_t <= step_t:
                window_end_t = min(step_t + self.transient_window, time_vals[-1])
            metrics = self._analyze_single_step(
                time_vals, vout_raw, vout_filt, step_t, window_end_t, ripple_window
            )
            step_results.append((step_t, *metrics))

        overshoot = max((r[1] for r in step_results), default=10.0)
        undershoot = max((r[2] for r in step_results), default=10.0)
        osc_count = max((r[3] for r in step_results), default=5)
        settling_time = max((r[4] for r in step_results), default=self.transient_window)

        summary = " | ".join(
            f"{step_t*1000:.2f}ms: OS={os:.1f}% US={us:.1f}% Osc={osc}"
            for step_t, os, us, osc, _ in step_results
        )
        print(f"    [Analyzer] {summary}")

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


class GridRefinePidTuner:
    """
    Two-stage tuner:
    1. Start from a deliberately poor initial point.
    2. Sweep a coarse wc / phi_m grid.
    3. Refine locally around the best coarse point.
    """

    def __init__(self, config: TuningConfig):
        self.config = config
        self.compensator = CompensatorDesign()
        self.wc = config.wc_initial
        self.phi_m = config.phi_m_initial
        self.phase = "bootstrap"
        self.iteration = 0
        self.best_score = float('inf')
        self.best_wc = self.wc
        self.best_phi_m = self.phi_m
        self.best_pass_score = float('inf')
        self.best_pass_wc: Optional[float] = None
        self.best_pass_phi_m: Optional[float] = None
        self.coarse_candidates = self._build_coarse_candidates()
        self.local_candidates: List[Tuple[float, float]] = []

    @staticmethod
    def _linspace(start: float, stop: float, count: int) -> List[float]:
        if count <= 1:
            return [start]
        step = (stop - start) / (count - 1)
        return [start + i * step for i in range(count)]

    def _clamp_design(self, wc: float, phi_m: float) -> Tuple[float, float]:
        wc = max(self.config.wc_min, min(wc, self.config.wc_max))
        phi_m = max(self.config.phi_m_min, min(phi_m, self.config.phi_m_max))
        return wc, phi_m

    def _score(self, overshoot: float, undershoot: float, osc_count: int) -> float:
        return (max(0.0, overshoot - self.config.target_overshoot) +
                max(0.0, undershoot - self.config.target_undershoot) +
                max(0, osc_count - self.config.max_oscillations) * 3.0)

    def _build_coarse_candidates(self) -> List[Tuple[float, float]]:
        wc_vals = self._linspace(self.config.wc_min, self.config.wc_max, 5)
        phi_vals = self._linspace(self.config.phi_m_min, self.config.phi_m_max, 4)
        candidates: List[Tuple[float, float]] = []
        initial = self._clamp_design(self.config.wc_initial, self.config.phi_m_initial)
        for wc in wc_vals:
            for phi_m in phi_vals:
                candidate = self._clamp_design(wc, phi_m)
                if candidate == initial:
                    continue
                candidates.append(candidate)
        return candidates

    def _build_local_candidates(self, center_wc: float, center_phi_m: float) -> List[Tuple[float, float]]:
        wc_scales = [-0.12, -0.06, 0.0, 0.06, 0.12]
        phi_offsets = [math.radians(v) for v in (-8.0, -4.0, 0.0, 4.0, 8.0)]
        center = self._clamp_design(center_wc, center_phi_m)
        candidates: List[Tuple[float, float]] = []
        for wc_scale in wc_scales:
            for phi_delta in phi_offsets:
                candidate = self._clamp_design(center_wc * (1.0 + wc_scale), center_phi_m + phi_delta)
                if candidate == center:
                    continue
                if candidate not in candidates:
                    candidates.append(candidate)
        candidates.append(center)
        return candidates

    def _record_result(self, overshoot: float, undershoot: float, osc_count: int) -> None:
        score = self._score(overshoot, undershoot, osc_count)
        passed = (overshoot < self.config.target_overshoot and
                  undershoot < self.config.target_undershoot and
                  osc_count <= self.config.max_oscillations)
        if score < self.best_score:
            self.best_score = score
            self.best_wc = self.wc
            self.best_phi_m = self.phi_m
        if passed and score < self.best_pass_score:
            self.best_pass_score = score
            self.best_pass_wc = self.wc
            self.best_pass_phi_m = self.phi_m

    def _set_design(self, wc: float, phi_m: float) -> Tuple[float, float, float, float]:
        self.wc, self.phi_m = self._clamp_design(wc, phi_m)
        Kp, Ki, Kd, Kf = self.compensator.compute(self.wc, self.phi_m)
        print(f"    -> Next design ({self.phase}): wc={self.wc:.0f} rad/s ({self.wc/(2*math.pi):.0f} Hz), "
              f"phi_m={math.degrees(self.phi_m):.1f} deg")
        print(f"    -> PID: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")
        return Kp, Ki, Kd, Kf

    def get_initial_params(self) -> Tuple[float, float, float, float]:
        self.phase = "bootstrap"
        return self.compensator.compute(self.wc, self.phi_m)

    def adjust(self, Kp: float, Ki: float, Kd: float, Kf: float,
               overshoot: float, undershoot: float, osc_count: int) -> Tuple[float, float, float, float]:
        self.iteration += 1
        self._record_result(overshoot, undershoot, osc_count)

        if self.phase == "bootstrap":
            self.phase = "coarse_grid"
            wc, phi_m = self.coarse_candidates.pop(0)
            print("    -> Bootstrap complete, starting coarse grid search")
            return self._set_design(wc, phi_m)

        if self.phase == "coarse_grid":
            if self.coarse_candidates:
                wc, phi_m = self.coarse_candidates.pop(0)
                return self._set_design(wc, phi_m)

            seed_wc = self.best_pass_wc if self.best_pass_wc is not None else self.best_wc
            seed_phi = self.best_pass_phi_m if self.best_pass_phi_m is not None else self.best_phi_m
            self.local_candidates = self._build_local_candidates(seed_wc, seed_phi)
            self.phase = "local_refine"
            print("    -> Coarse grid complete, switching to local refinement")
            wc, phi_m = self.local_candidates.pop(0)
            return self._set_design(wc, phi_m)

        if self.local_candidates:
            wc, phi_m = self.local_candidates.pop(0)
            return self._set_design(wc, phi_m)

        seed_wc = self.best_pass_wc if self.best_pass_wc is not None else self.best_wc
        seed_phi = self.best_pass_phi_m if self.best_pass_phi_m is not None else self.best_phi_m
        self.local_candidates = self._build_local_candidates(seed_wc, seed_phi)
        print("    -> Refinement ring exhausted, recentering around best point")
        wc, phi_m = self.local_candidates.pop(0)
        return self._set_design(wc, phi_m)

    def check_pass(self, overshoot: float, undershoot: float, osc_count: int) -> bool:
        metrics_pass = (overshoot < self.config.target_overshoot and
                        undershoot < self.config.target_undershoot and
                        osc_count <= self.config.max_oscillations)
        return metrics_pass and self.phase == "local_refine"


class AutoTuner:
    """Main auto-tuning orchestrator"""

    def __init__(self, config: TuningConfig = None):
        self.config = config or TuningConfig()
        step_up_t = float(self.config.load_pulse_delay)
        step_down_t = step_up_t + float(self.config.load_pulse_duty_cycle) / float(self.config.load_pulse_frequency)
        self.plecs = PlecsRpc(
            self.config.rpc_url,
            plecs_exe=self.config.plecs_exe,
            model_id=self.config.model_id
        )
        self.model_editor = PlecsModelEditor()
        self.analyzer = ResponseAnalyzer(
            v_target=5.0,
            fsw=250e3,
            expected_step_times=[step_up_t, step_down_t],
        )
        self.tuner = GridRefinePidTuner(self.config)
        self.csv_parser = ScopeCsvParser()
        self.results: List[TuningResult] = []

    def _apply_fast_transient_settings(self) -> None:
        """Configure the loaded model for short step-up/step-down tests."""
        model_id = self.plecs.model_id
        self.plecs.set_parameter(model_id, "TimeSpan", self.config.sim_time_span)
        self.plecs.set_parameter(f"{model_id}/Cout", "v_init", self.config.cout_v_init)
        self.plecs.set_parameter(f"{model_id}/L1", "i_init", self.config.inductor_i_init)
        self.plecs.set_parameter(f"{model_id}/1A Load", "f", self.config.load_pulse_frequency)
        self.plecs.set_parameter(f"{model_id}/1A Load", "DutyCycle", self.config.load_pulse_duty_cycle)
        self.plecs.set_parameter(f"{model_id}/1A Load", "Delay", self.config.load_pulse_delay)

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

        target_model_path = Path(self.config.plecs_model).resolve()
        loaded_path = self.plecs.find_loaded_model_path(self.config.model_id)
        if loaded_path is None:
            print(f"  Opening source model...")
            self.plecs.load_model(str(target_model_path))
        else:
            self.plecs.model_id = self.config.model_id
            self.plecs.loaded_model_path = loaded_path
        self._apply_fast_transient_settings()

    def run_iteration(self, iter_num: int, Kp: float, Ki: float,
                      Kd: float, Kf: float) -> TuningResult:
        """Run a single tuning iteration"""
        print(f"\n[Iteration {iter_num}]")
        print(f"  Parameters: Kp={Kp:.5f}, Ki={Ki:.2f}, Kd={Kd:.2e}, Kf={Kf:.0f}")

        loaded_path = self.plecs.find_loaded_model_path(self.config.model_id)

        if loaded_path is None:
            target_model_path = Path(self.config.plecs_model).resolve()
            print(f"  Source model not open, reopening...")
            self.plecs.load_model(str(target_model_path), force_reload=True)
        else:
            self.plecs.model_id = self.config.model_id
            self.plecs.loaded_model_path = loaded_path
            print(f"  Updating parameters in loaded model...")

        self._apply_fast_transient_settings()
        init_cmds = self.plecs.get_parameter(self.plecs.model_id, "InitializationCommands")
        updated_cmds = self.model_editor.update_initialization_commands(init_cmds, Kp, Ki, Kd, Kf)
        self.plecs.set_parameter(self.plecs.model_id, "InitializationCommands", updated_cmds)

        # Run simulation and read direct signal outputs.
        print(f"  Running simulation...")
        try:
            sim_result = self.plecs.simulate()
            header, data = self.csv_parser.from_simulation_result(sim_result)
            csv_data = self.csv_parser.to_csv_bytes(header, data)
        except Exception as e:
            print(f"  Simulation/export error: {e}")
            return TuningResult(iter_num, Kp, Ki, Kd, Kf, 10, 10, 5, 0.005, "FAIL")

        # Save CSV for this iteration
        csv_path = Path(self.config.results_dir) / f"iter_{iter_num:03d}.csv"
        csv_path.write_bytes(csv_data)

        # Analyze direct simulation outputs
        overshoot, undershoot, osc_count, settling_time = self.analyzer.analyze(header, data)

        status = "PASS" if self.config.meets_targets(overshoot, undershoot, osc_count) else "FAIL"

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
        best_result = select_best_result(self.results)
        with open(log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Iter', 'Kp', 'Ki', 'Kd', 'Kf',
                           'Overshoot', 'Undershoot', 'OscCount',
                           'SettlingTime', 'Status'])
            for r in self.results:
                writer.writerow([r.iter_num, r.Kp, r.Ki, r.Kd, r.Kf,
                               r.overshoot, r.undershoot, r.osc_count,
                               r.settling_time, r.status])
            if best_result is not None:
                writer.writerow([])
                writer.writerow([
                    'BestIter',
                    best_result.iter_num,
                    best_result.Kp,
                    best_result.Ki,
                    best_result.Kd,
                    best_result.Kf,
                    best_result.overshoot,
                    best_result.undershoot,
                    best_result.osc_count,
                    best_result.settling_time,
                    best_result.status,
                ])
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

            # Adjust parameters for next iteration
            if i < self.config.max_iterations - 1:
                Kp, Ki, Kd, Kf = self.tuner.adjust(
                    Kp, Ki, Kd, Kf,
                    result.overshoot, result.undershoot, result.osc_count
                )

        print(f"\nMax iterations ({self.config.max_iterations}) reached")
        best_result = select_best_result(self.results)
        if best_result is not None:
            print("\n" + "=" * 60)
            print("*** BEST ITERATION AFTER FULL SEARCH ***")
            print(f"Iteration: {best_result.iter_num}")
            print(f"Kp={best_result.Kp:.5f}, Ki={best_result.Ki:.2f}, "
                  f"Kd={best_result.Kd:.2e}, Kf={best_result.Kf:.0f}")
            print(f"Overshoot={best_result.overshoot:.2f}%, "
                  f"Undershoot={best_result.undershoot:.2f}%, "
                  f"Oscillations={best_result.osc_count}, "
                  f"Settling={best_result.settling_time*1000:.3f} ms")
            print(f"Status={best_result.status}")
            print("=" * 60)
        self.save_log()
        return best_result


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
