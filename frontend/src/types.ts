export type BackendName = 'plecs' | 'ltspice' | 'simplis';

export interface TuningConfig {
  sim_backend: BackendName | string;
  backend?: BackendName;
  plecs_exe: string;
  plecs_model: string;
  ltspice_exe: string;
  ltspice_asc_model: string;
  ltspice_netlist_model: string;
  ltspice_bode_netlist_model: string;
  ltspice_bode_ac_netlist_model: string;
  ltspice_bode_mode: string;
  simplis_exe: string;
  simplis_schematic_model: string;
  simplis_netlist_model: string;
  rpc_url: string;
  model_id: string;
  results_dir: string;
  work_dir: string;
  ltspice_work_dir: string;
  simplis_work_dir: string;
  sim_time_span: string;
  load_pulse_frequency: string;
  load_pulse_duty_cycle: string;
  load_pulse_delay: string;
  cout_v_init: string;
  inductor_i_init: string;
  target_overshoot: number;
  target_undershoot: number;
  max_oscillations: number;
  target_settling_time: number;
  max_iterations: number;
  bode_freq_start_hz: number;
  bode_freq_stop_hz: number;
  bode_extraction_cycles: number;
  bode_coarse_num_points: number;
  bode_dense_num_points: number;
  run_bode_analysis: boolean;
  wc_min: number;
  wc_max: number;
  wc_initial: number;
  phi_m_min: number;
  phi_m_max: number;
  phi_m_initial: number;
}

export interface PidValues {
  Kp: number;
  Ki: number;
  Kd: number;
  Kf: number;
}

export interface TuningResult {
  iter_num: number;
  Kp: number;
  Ki: number;
  Kd: number;
  Kf: number;
  overshoot: number;
  undershoot: number;
  osc_count: number;
  settling_time: number;
  status: string;
}

export interface BodeData {
  freq_hz: number[];
  mag_db: number[];
  phase_deg: number[];
  elapsed_s: number;
  metrics: {
    crossover_hz: number | null;
    phase_margin_deg: number | null;
    gain_margin_db: number | null;
    phase_crossover_hz: number | null;
  };
}

export interface IterationPayload {
  result: TuningResult;
  phase: string;
  backend: BackendName;
  results_dir: string;
  work_model_path: string;
  waveform: {
    time: number[];
    time_ms: number[];
    vout: number[];
    il: number[];
  };
  bode: BodeData | null;
  next_params?: PidValues;
}

export interface SessionStatus {
  running: boolean;
  paused: boolean;
  mode: string;
  backend: BackendName;
  phase: string;
  iteration: number;
  total_iterations: number;
  results_dir: string;
  work_model_path: string;
  current: TuningResult | null;
  best: TuningResult | null;
}

export interface ApiEvent {
  type: 'status' | 'log' | 'iteration' | 'finished' | 'error';
  timestamp: number;
  payload: any;
}
