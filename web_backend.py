"""FastAPI backend for the Electron Buck PID Auto-Tuner workbench."""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from auto_tune import AutoTuner, CompensatorDesign, TuningConfig, TuningResult, select_best_result
from analyze import plot_animation
from bode_plot import BodeResult, run_loop_gain_analysis, run_ltspice_loop_gain_analysis
from iteration_export import save_iteration_frame, write_bode_workbook, write_time_workbook
from ltspice_backend import import_pyltspice


def validate_config_values(config: TuningConfig) -> List[str]:
    """Return user-facing validation errors before starting a run."""
    errors: List[str] = []
    backend = config.backend
    if backend == "plecs":
        if not Path(config.plecs_model).exists():
            errors.append(f"PLECS model not found: {config.plecs_model}")
        if config.plecs_exe and not Path(config.plecs_exe).exists():
            errors.append(f"PLECS executable not found: {config.plecs_exe}")
    elif backend == "ltspice":
        if not config.ltspice_exe or not Path(config.ltspice_exe).exists():
            errors.append("LTspice executable not found. Set LTSPICE_EXE or choose LTspice.exe.")
        for label, path in (
            ("LTspice schematic", config.ltspice_asc_model),
            ("LTspice transient netlist", config.ltspice_netlist_model),
        ):
            if not Path(path).exists():
                errors.append(f"{label} not found: {path}")
        bode_mode = str(getattr(config, "ltspice_bode_mode", "ac") or "ac").lower()
        if bode_mode in ("switching", "switching_fra", "fra", "transient"):
            if not Path(config.ltspice_bode_netlist_model).exists():
                errors.append(f"LTspice switching Bode netlist not found: {config.ltspice_bode_netlist_model}")
        elif not Path(config.ltspice_bode_ac_netlist_model).exists():
            errors.append(f"LTspice AC Bode netlist not found: {config.ltspice_bode_ac_netlist_model}")
        try:
            import_pyltspice()
        except ImportError as exc:
            errors.append(str(exc))
    elif backend == "simplis":
        if not config.simplis_exe or not Path(config.simplis_exe).exists():
            errors.append("SIMetrix/SIMPLIS executable not found. Set SIMPLIS_EXE or choose SIMetrix.exe.")
        for label, path in (
            ("SIMPLIS schematic", config.simplis_schematic_model),
            ("SIMPLIS transient netlist", config.simplis_netlist_model),
        ):
            if not Path(path).exists():
                errors.append(f"{label} not found: {path}")
    else:
        errors.append(f"Unsupported backend: {config.sim_backend}")

    if config.max_iterations < 1:
        errors.append("Max Iter must be at least 1.")
    if config.target_overshoot < 0 or config.target_undershoot < 0:
        errors.append("Overshoot and undershoot targets must be non-negative.")
    if config.target_settling_time <= 0:
        errors.append("Settling-time target must be positive.")
    if config.run_bode_analysis and backend != "simplis":
        if config.bode_freq_stop_hz <= config.bode_freq_start_hz:
            errors.append("Bode stop frequency must be greater than start frequency.")
        if config.bode_coarse_num_points < 5:
            errors.append("Bode point count must be at least 5.")
        if backend == "plecs":
            if config.bode_dense_num_points < 5:
                errors.append("Dense Bode point count must be at least 5.")
            if config.bode_extraction_cycles < 1:
                errors.append("Bode extraction cycles must be at least 1.")
    return errors


def config_from_payload(payload: Optional[Dict[str, Any]]) -> TuningConfig:
    """Create a TuningConfig from a partial API payload."""
    cfg = TuningConfig()
    for key, value in (payload or {}).items():
        if key == "backend":
            continue
        if hasattr(cfg, key):
            setattr(cfg, key, value)
    return cfg


def config_to_dict(config: TuningConfig) -> Dict[str, Any]:
    data = dataclasses.asdict(config)
    data["backend"] = config.backend
    return data


def result_to_dict(result: Optional[TuningResult]) -> Optional[Dict[str, Any]]:
    if result is None:
        return None
    return dataclasses.asdict(result)


def bode_to_dict(bode: Optional[BodeResult]) -> Optional[Dict[str, Any]]:
    if bode is None:
        return None
    return {
        "freq_hz": bode.freq_hz,
        "mag_db": bode.mag_db,
        "phase_deg": bode.phase_deg,
        "real_vals": bode.real_vals,
        "imag_vals": bode.imag_vals,
        "elapsed_s": bode.elapsed_s,
        "coarse_elapsed_s": bode.coarse_elapsed_s,
        "dense_elapsed_s": bode.dense_elapsed_s,
        "metrics": dataclasses.asdict(bode.metrics),
    }


def waveform_from_data(header: List[str], rows: List[List[float]]) -> Dict[str, List[float]]:
    vout_col = 2
    for i, name in enumerate(header):
        lower = name.lower()
        if "voltage" in lower or "vout" in lower:
            vout_col = i
            break
    time_vals: List[float] = []
    time_ms: List[float] = []
    vout_vals: List[float] = []
    il_vals: List[float] = []
    for row in rows:
        if not row:
            continue
        t = float(row[0])
        time_vals.append(t)
        time_ms.append(t * 1000.0)
        il_vals.append(float(row[1]) if len(row) > 1 else 0.0)
        vout_vals.append(float(row[vout_col]) if len(row) > vout_col else 0.0)
    return {"time": time_vals, "time_ms": time_ms, "vout": vout_vals, "il": il_vals}


def decimate_rows(rows: List[List[float]], max_points: int = 5000) -> List[List[float]]:
    if len(rows) <= max_points or max_points < 2:
        return rows
    last_idx = len(rows) - 1
    return [rows[round(i * last_idx / (max_points - 1))] for i in range(max_points)]


class EventBroker:
    def __init__(self) -> None:
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.clients: set[WebSocket] = set()

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.clients.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self.clients.discard(websocket)

    async def _broadcast(self, event: Dict[str, Any]) -> None:
        stale: List[WebSocket] = []
        for client in list(self.clients):
            try:
                await client.send_json(event)
            except Exception:
                stale.append(client)
        for client in stale:
            self.disconnect(client)

    def publish(self, event_type: str, payload: Dict[str, Any]) -> None:
        event = {"type": event_type, "timestamp": time.time(), "payload": payload}
        if self.loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(event), self.loop)


class TuningSession:
    def __init__(self, broker: EventBroker) -> None:
        self.broker = broker
        self.lock = threading.Lock()
        self.pause_event = threading.Event()
        self.pause_event.set()
        self.stop_flag = False
        self.worker: Optional[threading.Thread] = None
        self.auto_tuner: Optional[AutoTuner] = None
        self.config = TuningConfig()
        self.results: List[TuningResult] = []
        self.waveforms: Dict[int, Dict[str, Any]] = {}
        self.bodes: Dict[int, BodeResult] = {}
        self.mode = "Idle"
        self.phase = "-"
        self.running = False
        self.paused = False
        self.iteration = 0
        self.total_iterations = 0
        self.results_dir = str(self.config.results_dir)
        self.work_model_path = ""

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            best = select_best_result(self.results)
            return {
                "running": self.running,
                "paused": self.paused,
                "mode": self.mode,
                "backend": self.config.backend,
                "phase": self.phase,
                "iteration": self.iteration,
                "total_iterations": self.total_iterations,
                "results_dir": self.results_dir,
                "work_model_path": self.work_model_path,
                "current": result_to_dict(self.results[-1] if self.results else None),
                "best": result_to_dict(best),
                "history": [result_to_dict(result) for result in self.results],
            }

    def _publish_status(self) -> None:
        self.broker.publish("status", self.snapshot())

    def _log(self, message: str) -> None:
        self.broker.publish("log", {"message": message})

    def _set_running(self, running: bool, mode: str) -> None:
        with self.lock:
            self.running = running
            self.mode = mode
            self.paused = False if not running else self.paused
        self._publish_status()

    def _ensure_idle(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            raise HTTPException(status_code=409, detail="A tuning job is already running.")

    def _run_bode_for_iteration(
        self,
        config: TuningConfig,
        tuner: AutoTuner,
        iter_num: int,
        Kp: float,
        Ki: float,
        Kd: float,
        Kf: float,
    ) -> Optional[BodeResult]:
        if not getattr(config, "run_bode_analysis", False):
            return None
        if config.backend == "simplis":
            return None
        if config.backend == "ltspice":
            return run_ltspice_loop_gain_analysis(
                config,
                Kp,
                Ki,
                Kd,
                Kf,
                getattr(config, "bode_freq_start_hz", 1e3),
                getattr(config, "bode_freq_stop_hz", 1e5),
                int(getattr(config, "bode_coarse_num_points", 31)),
                int(getattr(config, "bode_dense_num_points", 51)),
                tuner.ltspice,
            )
        if tuner.plecs.server is None:
            return None
        return run_loop_gain_analysis(
            tuner.plecs.server,
            config.model_id,
            getattr(config, "bode_freq_start_hz", 1e3),
            getattr(config, "bode_freq_stop_hz", 1e5),
            int(getattr(config, "bode_coarse_num_points", 31)),
        )

    def _store_iteration(
        self,
        config: TuningConfig,
        tuner: AutoTuner,
        result: TuningResult,
        phase: str,
        bode: Optional[BodeResult],
        extra_payload: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        rows = decimate_rows(tuner.last_data)
        waveform = waveform_from_data(tuner.last_header, rows)
        self.waveforms[result.iter_num] = {
            "header": tuner.last_header,
            "data": rows,
            "phase": phase,
            "bode_metrics": dataclasses.asdict(bode.metrics) if bode is not None else {},
            "targets": {
                "target_os": config.target_overshoot,
                "target_us": config.target_undershoot,
                "max_osc": config.max_oscillations,
                "target_ts": config.target_settling_time,
            },
            "backend": config.backend,
            "results_dir": str(config.results_dir),
            "work_model_path": str(tuner.work_model_path or ""),
        }
        if bode is not None:
            self.bodes[result.iter_num] = bode

        figures_dir = Path(config.results_dir)
        figures_dir.mkdir(parents=True, exist_ok=True)
        save_iteration_frame(
            figures_dir / f"iter{result.iter_num + 1}.png",
            result,
            waveform["time"],
            waveform["vout"],
            bode,
            config.target_overshoot,
            config.target_undershoot,
        )

        payload = {
            "result": result_to_dict(result),
            "phase": phase,
            "backend": config.backend,
            "results_dir": str(config.results_dir),
            "work_model_path": str(tuner.work_model_path or ""),
            "waveform": waveform,
            "bode": bode_to_dict(bode),
        }
        if extra_payload:
            payload.update(extra_payload)
        self.broker.publish("iteration", payload)
        return payload

    def _write_exports(self, config: TuningConfig) -> None:
        results_dir = Path(config.results_dir)
        results_dir.mkdir(parents=True, exist_ok=True)
        write_time_workbook(results_dir / "data_time_iterations.xlsx", self.results, self.waveforms)
        if self.bodes:
            write_bode_workbook(results_dir / "data_bode_iterations.xlsx", self.bodes)

    def start_auto(self, config: TuningConfig) -> Dict[str, Any]:
        with self.lock:
            self._ensure_idle()
            self.stop_flag = False
            self.pause_event.set()
            self.paused = False
            self.config = config
            self.results = []
            self.waveforms = {}
            self.bodes = {}
            self.iteration = 0
            self.total_iterations = int(config.max_iterations)
            self.results_dir = str(config.results_dir)
            self.work_model_path = ""
            self.phase = "-"
            self.running = True
            self.mode = "Auto"
            self.worker = threading.Thread(target=self._auto_worker, args=(config,), daemon=True)
            self.worker.start()
        self._publish_status()
        return self.snapshot()

    def _auto_worker(self, config: TuningConfig) -> None:
        try:
            tuner = AutoTuner(config)
            self.auto_tuner = tuner
            label = {"ltspice": "LTspice", "simplis": "SIMPLIS"}.get(config.backend, "PLECS")
            self._log(f"Preparing {label}...")
            tuner.setup()
            with self.lock:
                self.work_model_path = str(tuner.work_model_path or "")
                self.results_dir = str(config.results_dir)
            self._log(f"{label} ready. Starting tuning loop.")
            self._publish_status()

            Kp, Ki, Kd, Kf = tuner.tuner.get_initial_params()
            self._log(f"Initial: Kp={Kp:.5f} Ki={Ki:.2f} Kd={Kd:.2e} Kf={Kf:.0f}")

            for i in range(config.max_iterations):
                self.pause_event.wait()
                if self.stop_flag:
                    self._log("Stopped by user.")
                    break
                phase = getattr(tuner.tuner, "phase", "unknown")
                with self.lock:
                    self.phase = phase
                    self.iteration = i + 1
                self._publish_status()

                result = tuner.run_iteration(i, Kp, Ki, Kd, Kf)
                tuner.results.append(result)
                self.results.append(result)
                bode = self._run_bode_for_iteration(config, tuner, i, result.Kp, result.Ki, result.Kd, result.Kf)
                self._store_iteration(config, tuner, result, phase, bode)
                self._log(
                    f"Iter {i}: {phase} | OS={result.overshoot:.1f}% "
                    f"US={result.undershoot:.1f}% Osc={result.osc_count} -> {result.status}"
                )
                self._publish_status()

                if i < config.max_iterations - 1:
                    Kp, Ki, Kd, Kf = tuner.tuner.adjust(
                        Kp,
                        Ki,
                        Kd,
                        Kf,
                        result.overshoot,
                        result.undershoot,
                        result.osc_count,
                        result.settling_time,
                    )

            self._write_exports(config)
            best = select_best_result(self.results)
            self.broker.publish("finished", {"pass": bool(best and best.status == "PASS"), "best": result_to_dict(best)})
        except Exception as exc:
            self.broker.publish("error", {"message": str(exc)})
        finally:
            with self.lock:
                self.running = False
                self.paused = False
                self.mode = "Idle"
            self._publish_status()

    def start_single(self, config: TuningConfig, params: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            self._ensure_idle()
            self.stop_flag = False
            self.pause_event.set()
            self.paused = False
            self.config = config
            self.total_iterations = max(1, int(config.max_iterations))
            self.results_dir = str(config.results_dir)
            iter_num = int(params.get("iter_num", len(self.results)))
            if iter_num == 0 and self.results:
                self.results = []
                self.waveforms = {}
                self.bodes = {}
                self.auto_tuner = None
                self.work_model_path = ""
            if all(key in params for key in ("Kp", "Ki", "Kd", "Kf")):
                pid = (float(params["Kp"]), float(params["Ki"]), float(params["Kd"]), float(params["Kf"]))
            else:
                pid = CompensatorDesign().compute(config.wc_initial, config.phi_m_initial)
            self.running = True
            self.mode = "Single"
            self.worker = threading.Thread(target=self._single_worker, args=(config, iter_num, pid, iter_num > 0), daemon=True)
            self.worker.start()
        self._publish_status()
        return self.snapshot()

    def _single_worker(
        self,
        config: TuningConfig,
        iter_num: int,
        pid: tuple[float, float, float, float],
        reuse_work_model: bool,
    ) -> None:
        try:
            existing = self.auto_tuner
            if reuse_work_model and existing is not None and existing.can_reuse_work_model(config):
                tuner = existing
                tuner.config = config
            else:
                tuner = AutoTuner(config)
            self.auto_tuner = tuner
            label = {"ltspice": "LTspice", "simplis": "SIMPLIS"}.get(config.backend, "PLECS")
            self._log(f"Preparing {label}...")
            tuner.setup(reuse_work_model=reuse_work_model)
            with self.lock:
                self.work_model_path = str(tuner.work_model_path or "")
                self.iteration = iter_num + 1
                self.phase = getattr(tuner.tuner, "phase", "unknown")
            self._publish_status()

            Kp, Ki, Kd, Kf = pid
            result = tuner.run_iteration(iter_num, Kp, Ki, Kd, Kf)
            tuner.results.append(result)
            self.results.append(result)
            bode = self._run_bode_for_iteration(config, tuner, iter_num, result.Kp, result.Ki, result.Kd, result.Kf)
            next_params = tuner.tuner.adjust(
                Kp,
                Ki,
                Kd,
                Kf,
                result.overshoot,
                result.undershoot,
                result.osc_count,
                result.settling_time,
            )
            self._store_iteration(
                config,
                tuner,
                result,
                self.phase,
                bode,
                {
                    "next_params": {
                        "Kp": next_params[0],
                        "Ki": next_params[1],
                        "Kd": next_params[2],
                        "Kf": next_params[3],
                    }
                },
            )
            self._write_exports(config)
            self.broker.publish("finished", {"pass": result.status == "PASS", "best": result_to_dict(result)})
        except Exception as exc:
            self.broker.publish("error", {"message": str(exc)})
        finally:
            with self.lock:
                self.running = False
                self.paused = False
                self.mode = "Idle"
            self._publish_status()

    def reset(self) -> Dict[str, Any]:
        with self.lock:
            if self.worker is not None and self.worker.is_alive():
                raise HTTPException(status_code=409, detail="A tuning job is already running.")
            self.stop_flag = False
            self.pause_event.set()
            self.paused = False
            self.auto_tuner = None
            self.results = []
            self.waveforms = {}
            self.bodes = {}
            self.mode = "Idle"
            self.phase = "-"
            self.running = False
            self.iteration = 0
            self.total_iterations = 0
            self.work_model_path = ""
        self._publish_status()
        return self.snapshot()

    def pause(self) -> Dict[str, Any]:
        with self.lock:
            self.paused = True
        self.pause_event.clear()
        self._publish_status()
        return self.snapshot()

    def resume(self) -> Dict[str, Any]:
        with self.lock:
            self.paused = False
        self.pause_event.set()
        self._publish_status()
        return self.snapshot()

    def stop(self) -> Dict[str, Any]:
        self.stop_flag = True
        self.pause_event.set()
        self._publish_status()
        return self.snapshot()

    def result_files(self) -> List[Dict[str, Any]]:
        root = Path(self.results_dir)
        if not root.exists():
            return []
        files = []
        for path in sorted(root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
            if path.is_file():
                files.append({"name": path.name, "path": str(path), "size": path.stat().st_size})
        return files


broker = EventBroker()
session = TuningSession(broker)
app = FastAPI(title="Buck PID Auto-Tuner API")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def on_startup() -> None:
    broker.set_loop(asyncio.get_running_loop())


@app.get("/api/health")
def health() -> Dict[str, Any]:
    return {"ok": True, "app": "BuckPidAutoTuner"}


@app.get("/api/config/defaults")
def config_defaults() -> Dict[str, Any]:
    cfg = TuningConfig()
    Kp, Ki, Kd, Kf = CompensatorDesign().compute(cfg.wc_initial, cfg.phi_m_initial)
    return {"config": config_to_dict(cfg), "pid": {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Kf": Kf}}


@app.post("/api/config/validate")
def config_validate(payload: Dict[str, Any]) -> Dict[str, Any]:
    cfg = config_from_payload(payload.get("config", payload))
    errors = validate_config_values(cfg)
    return {"valid": not errors, "errors": errors, "config": config_to_dict(cfg)}


@app.post("/api/pid/compute")
def pid_compute(payload: Dict[str, Any]) -> Dict[str, float]:
    cfg = config_from_payload(payload.get("config", payload))
    wc = float(payload.get("wc", cfg.wc_initial))
    phi_m = float(payload.get("phi_m", cfg.phi_m_initial))
    Kp, Ki, Kd, Kf = CompensatorDesign().compute(wc, phi_m)
    return {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Kf": Kf}


@app.post("/api/run/auto")
def run_auto(payload: Dict[str, Any]) -> Dict[str, Any]:
    cfg = config_from_payload(payload.get("config", payload))
    errors = validate_config_values(cfg)
    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})
    return session.start_auto(cfg)


@app.post("/api/run/single")
def run_single(payload: Dict[str, Any]) -> Dict[str, Any]:
    cfg = config_from_payload(payload.get("config", {}))
    errors = validate_config_values(cfg)
    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})
    return session.start_single(cfg, payload.get("pid", {}))


@app.post("/api/run/pause")
def run_pause() -> Dict[str, Any]:
    return session.pause()


@app.post("/api/run/resume")
def run_resume() -> Dict[str, Any]:
    return session.resume()


@app.post("/api/run/stop")
def run_stop() -> Dict[str, Any]:
    return session.stop()


@app.post("/api/run/reset")
def run_reset() -> Dict[str, Any]:
    return session.reset()


@app.get("/api/results/current")
def results_current() -> Dict[str, Any]:
    data = session.snapshot()
    data["files"] = session.result_files()
    return data


@app.post("/api/results/gif")
def results_gif() -> Dict[str, str]:
    out_path = Path(session.results_dir) / "animation.gif"
    plot_animation(str(out_path))
    return {"path": str(out_path)}


@app.websocket("/api/events")
async def events(websocket: WebSocket) -> None:
    await broker.connect(websocket)
    await websocket.send_json({"type": "status", "timestamp": time.time(), "payload": session.snapshot()})
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        broker.disconnect(websocket)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
