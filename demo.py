"""
demo.py - Run a simulated tuning demo (no PLECS required)
========================================================
Simulates the PID auto-tuning process with realistic buck converter responses.
Generates demo CSV files and runs the full visualization pipeline.

Usage:
    python demo.py
"""

import csv
import os
import math
import random
from pathlib import Path
import numpy as np
from auto_tune import TuningConfig, TuningResult, PidTuner, CompensatorDesign

RESULTS_DIR = r"c:\Users\liaom\Documents\Claude Code\Simulation\Plecs\results"
RANDOM_SEED = 42


class BuckConverterSimulator:
    """
    Simulate buck converter output voltage response based on PID parameters.

    Simplified model: 12V -> 5V buck with load step at t=5ms
    - High Kp: Fast response, more overshoot
    - High Kd: More damping, less oscillation
    - High Ki: Better steady-state, but can cause instability
    """

    def __init__(self):
        self.v_steady = 5.0
        self.v_in = 12.0

    def simulate(self, Kp: float, Ki: float, Kd: float, Kf: float,
                 noise: float = 0.01) -> dict:
        """
        Simulate Vout response for given PID parameters.
        Returns dict with time, IL, Vout arrays.
        """
        t_start = 0.0
        t_step = 0.005  # Load step at 5ms
        t_end = 0.010  # 10ms total simulation
        dt = 1e-6  # 1us time step
        n_steps = int((t_end - t_start) / dt)

        time_vals = []
        il_vals = []
        vout_vals = []

        # Initial conditions
        vout = self.v_steady
        il = 1.0  # Load current at 5V/5A = 1A (simplified)
        error_prev = 0
        integral = 0

        for i in range(n_steps):
            t = i * dt
            time_vals.append(t)
            il_vals.append(il)
            vout_vals.append(vout)

            # Simple Euler integration for demonstration
            # In reality, this would be a detailed switching model
            if t < t_step:
                # Pre-load-step: steady state
                vout_ripple = 0.02 * math.sin(2 * math.pi * 250e3 * t)
                vout = self.v_steady + vout_ripple * random.uniform(0.9, 1.1)
            else:
                # Post-load-step: transient response depends on PID
                # Calculate error
                error = self.v_steady - vout

                # PID terms
                proportional = Kp * error

                integral += Ki * error * dt
                integral = max(-10, min(10, integral))  # Anti-windup

                if i > 0:
                    derivative = Kd * (error - error_prev) / dt
                else:
                    derivative = 0

                # Derivative filter (simplified)
                filtered_derivative = derivative * Kf / (Kf + dt) if Kf > 0 else 0

                # Control effort
                u = proportional + integral + filtered_derivative

                # Simplified plant response
                # Higher Kp = faster response, more overshoot
                # Higher Kd = more damping, less oscillation
                # Higher Ki = better steady-state, can cause oscillations

                # Base dynamics
                dvout = (il - 1.0) * 10 / 22e-6 * dt  # dV = I/C * dt

                # PID effect on current step
                il_step = u * 5.0  # Control affects load current step
                il_step = max(0, min(5, il_step))

                # Oscillation model
                omega_n = 1000 * math.sqrt(Kp)  # Natural frequency
                zeta = Kd / (2 * math.sqrt(max(Kp, 0.01)))  # Damping ratio

                if zeta < 0.3:
                    # Underdamped - oscillations
                    decay = math.exp(-zeta * omega_n * (t - t_step))
                    freq = omega_n * math.sqrt(1 - zeta**2)
                    osc = decay * math.sin(freq * (t - t_step))
                else:
                    osc = 0

                vout = self.v_steady + il_step * 22e-6 / dt * (1 - math.exp(-(t - t_step) * 100))
                vout += osc * Kp * 0.5
                vout += noise * random.uniform(-1, 1)

                # Add some ripple
                vout += 0.01 * math.sin(2 * math.pi * 250e3 * t)

                error_prev = error

            vout = max(3.5, min(6.5, vout))

        # Downsample for CSV (every 1000th point)
        indices = list(range(0, len(time_vals), 100))
        return {
            'time': [time_vals[i] for i in indices],
            'IL': [il_vals[i] for i in indices],
            'Vout': [vout_vals[i] for i in indices]
        }


class SimulatedTuner:
    """Simulate the auto-tuning process, reusing PidTuner from auto_tune."""

    def __init__(self):
        self.sim = BuckConverterSimulator()
        self.config = TuningConfig()
        self.tuner = PidTuner(self.config)
        self.results: list[TuningResult] = []

    def calc_metrics(self, Kp, Ki, Kd, Kf):
        """Calculate overshoot, undershoot, oscillations for given params"""
        # Base overshoot (decreases with Kd, increases with Kp)
        base_os = 15.0 / (1 + Kd * 2) * (1 + Kp * 0.3)

        # Ki effect (small Ki helps, large Ki causes oscillation)
        ki_factor = 1.0 + (Ki - 1.0) * 0.5 if Ki > 0.5 else 1.0 - (0.5 - Ki) * 0.3

        overshoot = base_os * ki_factor
        overshoot = max(0.5, min(25.0, overshoot))

        # Undershoot (similar relationship)
        undershoot = 12.0 / (1 + Kd * 2) * (1 + Kp * 0.2)
        undershoot = max(0.3, min(20.0, undershoot))

        # Oscillations
        if Kd > 0.1:
            osc = int(3 / (1 + Kd * 5))
        else:
            osc = 3
        if Ki > 2:
            osc += 1
        osc = max(0, min(5, osc))

        # Add some randomness for realism
        overshoot *= random.uniform(0.95, 1.05)
        undershoot *= random.uniform(0.95, 1.05)

        return overshoot, undershoot, osc

    def tune(self, max_iter=15):
        """Run simulated tuning"""
        random.seed(RANDOM_SEED)
        self.results = []

        # Use analytical design for initial parameters
        Kp, Ki, Kd, Kf = self.tuner.get_initial_params()

        for i in range(max_iter):
            os_, us, osc = self.calc_metrics(Kp, Ki, Kd, Kf)

            settling_time = 0.002 if osc == 0 else (0.003 if osc <= 1 else 0.004)
            status = "PASS" if self.tuner.check_pass(os_, us, osc) else "FAIL"

            result = TuningResult(
                iter_num=i,
                Kp=Kp, Ki=Ki, Kd=Kd, Kf=Kf,
                overshoot=os_, undershoot=us, osc_count=osc,
                settling_time=settling_time,
                status=status
            )
            self.results.append(result)

            print(f"Iter {i}: Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}, Kf={Kf:.4f} "
                  f"-> OS={os_:.1f}%, US={us:.1f}%, Osc={osc} [{status}]")

            if status == "PASS":
                print(f"\n*** SUCCESS! ***")
                print(f"Final: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}, Kf={Kf:.4f}")
                break

            Kp, Ki, Kd, Kf = self.tuner.adjust(Kp, Ki, Kd, Kf, os_, us, osc)

        return self.results


def generate_demo_csv(results_dir: str, tuner: SimulatedTuner):
    """Generate demo CSV files for each iteration"""
    Path(results_dir).mkdir(parents=True, exist_ok=True)

    for result in tuner.results:
        sim = BuckConverterSimulator()
        data = sim.simulate(result.Kp, result.Ki, result.Kd, result.Kf)

        csv_path = Path(results_dir) / f"iter_{result.iter_num:03d}.csv"
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'IL', 'Vout'])
            for i in range(len(data['time'])):
                writer.writerow([data['time'][i], data['IL'][i], data['Vout'][i]])

        print(f"  Generated: {csv_path.name}")


def save_demo_log(results_dir: str, results):
    """Save tuning log"""
    log_path = Path(results_dir) / "tuning_log.csv"
    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Iter', 'Kp', 'Ki', 'Kd', 'Kf',
                        'Overshoot', 'Undershoot', 'OscCount',
                        'SettlingTime', 'Status'])
        for r in results:
            writer.writerow([r.iter_num, r.Kp, r.Ki, r.Kd, r.Kf,
                          r.overshoot, r.undershoot, r.osc_count,
                          r.settling_time, r.status])
    print(f"\nLog saved: {log_path}")


def main():
    print("=" * 60)
    print("PLECS Buck Converter PID Auto-Tuning - DEMO MODE")
    print("=" * 60)

    # Clean results directory
    results_dir = RESULTS_DIR
    if os.path.exists(results_dir):
        for f in Path(results_dir).glob("iter_*.csv"):
            f.unlink()

    os.makedirs(results_dir, exist_ok=True)

    # Run simulated tuning
    print("\n[1] Running simulated PID tuning...")
    print("-" * 60)
    tuner = SimulatedTuner()
    results = tuner.tune(max_iter=15)

    # Generate demo CSV files
    print("\n[2] Generating demo CSV files...")
    generate_demo_csv(results_dir, tuner)

    # Save log
    print("\n[3] Saving tuning log...")
    save_demo_log(results_dir, results)

    print("\n[4] Running visualization analysis...")
    print("-" * 60)

    # Run the analyze.py script
    import analyze
    analyze.RESULTS_DIR = results_dir

    print("\n  Generating metrics.png...")
    analyze.plot_metrics(str(Path(results_dir) / "metrics.png"))

    print("  Generating path.png...")
    analyze.plot_path(str(Path(results_dir) / "path.png"))

    print("  Generating final.png...")
    analyze.plot_final(str(Path(results_dir) / "final.png"))

    print("  Generating animation.gif...")
    analyze.plot_animation(str(Path(results_dir) / "animation.gif"))

    print("\n" + "=" * 60)
    print("DEMO COMPLETE!")
    print(f"\nResults saved to: {results_dir}")
    print("\nGenerated files:")
    print("  - tuning_log.csv  (all iteration data)")
    print("  - iter_XXX.csv     (individual waveform data)")
    print("  - metrics.png      (OS/US/oscillation over iterations)")
    print("  - path.png         (parameter evolution)")
    print("  - final.png       (annotated final response)")
    print("  - animation.gif   (animated response evolution)")
    print("=" * 60)


if __name__ == "__main__":
    main()