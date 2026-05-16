import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { ReactNode, Ref } from 'react';
import ReactECharts from 'echarts-for-react';
import {
  Activity,
  BarChart3,
  Circle,
  FolderOpen,
  Gauge,
  Pause,
  Play,
  RefreshCw,
  RotateCcw,
  Save,
  Square,
  Zap
} from 'lucide-react';
import type { ApiEvent, IterationPayload, PidValues, SessionStatus, TuningConfig } from './types';

const DEFAULT_API = 'http://127.0.0.1:8765';
const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;
type FilePickerFilter = { name: string; extensions: string[] };
type FilePickerOptions = { title: string; currentPath?: string; filters?: FilePickerFilter[] };

const EXE_FILTERS: FilePickerFilter[] = [
  { name: 'Executables', extensions: ['exe'] },
  { name: 'All files', extensions: ['*'] }
];
const PLECS_MODEL_FILTERS: FilePickerFilter[] = [
  { name: 'PLECS models', extensions: ['plecs'] },
  { name: 'All files', extensions: ['*'] }
];
const LTSPICE_ASC_FILTERS: FilePickerFilter[] = [
  { name: 'LTspice schematics', extensions: ['asc'] },
  { name: 'All files', extensions: ['*'] }
];
const LTSPICE_NETLIST_FILTERS: FilePickerFilter[] = [
  { name: 'SPICE netlists', extensions: ['cir', 'net', 'sp'] },
  { name: 'All files', extensions: ['*'] }
];
const SIMPLIS_SCHEMATIC_FILTERS: FilePickerFilter[] = [
  { name: 'SIMPLIS schematics', extensions: ['sxsch'] },
  { name: 'All files', extensions: ['*'] }
];
const SIMPLIS_NETLIST_FILTERS: FilePickerFilter[] = [
  { name: 'SIMPLIS netlists', extensions: ['net', 'cir', 'sp'] },
  { name: 'All files', extensions: ['*'] }
];

declare global {
  interface Window {
    buckAutoTuner?: {
      openFile: (options: FilePickerOptions) => Promise<string | null>;
    };
  }
}

function apiFromLocation(): string {
  const param = new URLSearchParams(window.location.search).get('api');
  return param || DEFAULT_API;
}

function fmt(value: number | null | undefined, digits = 2): string {
  if (value === null || value === undefined || Number.isNaN(value)) return '-';
  if (Math.abs(value) >= 100000) return value.toExponential(2);
  if (Math.abs(value) > 1000) return value.toFixed(0);
  if (Math.abs(value) < 0.001 && value !== 0) return value.toExponential(2);
  return value.toFixed(digits);
}

function fmtHz(value: number | null | undefined): string {
  if (value === null || value === undefined) return '-';
  if (Math.abs(value) >= 1e6) return `${fmt(value / 1e6, 2)} MHz`;
  if (Math.abs(value) >= 1e3) return `${fmt(value / 1e3, 2)} kHz`;
  return `${fmt(value, 0)} Hz`;
}

function statusClass(status?: string | null): 'neutral' | 'pass' | 'fail' {
  if (!status) return 'neutral';
  return status.toLowerCase() === 'pass' ? 'pass' : 'fail';
}

function StatTile({
  label,
  value,
  tone = 'neutral'
}: {
  label: string;
  value: string;
  tone?: 'neutral' | 'pass' | 'fail' | 'warn';
}) {
  return (
    <div className={`stat-tile ${tone}`}>
      <div className="stat-label">{label}</div>
      <div className="stat-value">{value}</div>
    </div>
  );
}

function Field({
  label,
  value,
  onChange,
  step = 1,
  min,
  type = 'number',
  disabled = false
}: {
  label: string;
  value: string | number;
  onChange: (value: string) => void;
  step?: number | string;
  min?: number;
  type?: 'number' | 'text';
  disabled?: boolean;
}) {
  return (
    <label className="field">
      <span>{label}</span>
      <input
        type={type}
        value={value}
        step={step}
        min={min}
        disabled={disabled}
        onChange={(event) => onChange(event.target.value)}
      />
    </label>
  );
}

function PathField({
  label,
  value,
  onChange,
  filters,
  onError
}: {
  label: string;
  value: string;
  onChange: (value: string) => void;
  filters: FilePickerFilter[];
  onError: (message: string) => void;
}) {
  const canBrowse = Boolean(window.buckAutoTuner?.openFile);

  const browse = async () => {
    try {
      const selected = await window.buckAutoTuner?.openFile({
        title: `Select ${label}`,
        currentPath: value,
        filters
      });
      if (selected) {
        onChange(selected);
      }
    } catch (error) {
      onError(error instanceof Error ? error.message : String(error));
    }
  };

  return (
    <div className="path-field">
      <Field label={label} value={value} type="text" onChange={onChange} />
      <button className="icon-button" disabled={!canBrowse} title={`Browse for ${label}`} onClick={browse}>
        <FolderOpen size={16} />
      </button>
    </div>
  );
}

function Section({ title, children, sectionRef }: { title: string; children: ReactNode; sectionRef?: Ref<HTMLElement> }) {
  return (
    <section className="inspector-section" ref={sectionRef}>
      <h3>{title}</h3>
      {children}
    </section>
  );
}

type RailTarget = 'workbench' | 'results' | 'backend';

export default function App() {
  const apiBase = useMemo(apiFromLocation, []);
  const wsBase = useMemo(() => apiBase.replace(/^http/, 'ws'), [apiBase]);
  const workspaceRef = useRef<HTMLElement | null>(null);
  const lowerRef = useRef<HTMLElement | null>(null);
  const backendSectionRef = useRef<HTMLElement | null>(null);
  const [defaults, setDefaults] = useState<{ config: TuningConfig; pid: PidValues } | null>(null);
  const [config, setConfig] = useState<TuningConfig | null>(null);
  const [pid, setPid] = useState<PidValues>({ Kp: 0, Ki: 0, Kd: 0, Kf: 0 });
  const [status, setStatus] = useState<SessionStatus | null>(null);
  const [iterations, setIterations] = useState<IterationPayload[]>([]);
  const [selectedIter, setSelectedIter] = useState<number | null>(null);
  const [logs, setLogs] = useState<string[]>([]);
  const [connected, setConnected] = useState(false);
  const [banner, setBanner] = useState<string | null>(null);
  const [activeRail, setActiveRail] = useState<RailTarget>('workbench');

  const request = useCallback(
    async <T,>(path: string, init?: RequestInit): Promise<T> => {
      const res = await fetch(`${apiBase}${path}`, {
        headers: { 'Content-Type': 'application/json', ...(init?.headers || {}) },
        ...init
      });
      const text = await res.text();
      const data = text ? JSON.parse(text) : {};
      if (!res.ok) {
        const detail = data.detail?.errors?.join('\n') || data.detail?.message || data.detail || res.statusText;
        throw new Error(String(detail));
      }
      return data as T;
    },
    [apiBase]
  );

  useEffect(() => {
    let alive = true;
    request<{ config: TuningConfig; pid: PidValues }>('/api/config/defaults')
      .then((data) => {
        if (!alive) return;
        setDefaults(data);
        setConfig(data.config);
        setPid(data.pid);
      })
      .catch((error) => setBanner(error.message));
    request<SessionStatus>('/api/results/current')
      .then((data) => alive && setStatus(data))
      .catch(() => undefined);
    return () => {
      alive = false;
    };
  }, [request]);

  useEffect(() => {
    const ws = new WebSocket(`${wsBase}/api/events`);
    ws.onopen = () => setConnected(true);
    ws.onclose = () => setConnected(false);
    ws.onerror = () => setConnected(false);
    ws.onmessage = (message) => {
      const event = JSON.parse(message.data) as ApiEvent;
      if (event.type === 'status') {
        setStatus(event.payload as SessionStatus);
      } else if (event.type === 'log') {
        setLogs((prev) => [...prev.slice(-160), event.payload.message]);
      } else if (event.type === 'iteration') {
        const payload = event.payload as IterationPayload;
        setIterations((prev) => {
          const next = prev.filter((item) => item.result.iter_num !== payload.result.iter_num);
          next.push(payload);
          next.sort((a, b) => a.result.iter_num - b.result.iter_num);
          return next;
        });
        setSelectedIter(payload.result.iter_num);
        if (payload.next_params) {
          setPid(payload.next_params);
        }
      } else if (event.type === 'finished') {
        const best = event.payload.best;
        setLogs((prev) => [...prev.slice(-160), `Finished. Best: ${best ? `Iter ${best.iter_num} ${best.status}` : 'n/a'}`]);
      } else if (event.type === 'error') {
        setBanner(event.payload.message);
      }
    };
    return () => ws.close();
  }, [wsBase]);

  const selected = useMemo(() => {
    if (!iterations.length) return null;
    return iterations.find((item) => item.result.iter_num === selectedIter) || iterations[iterations.length - 1];
  }, [iterations, selectedIter]);

  const patchConfig = useCallback(<K extends keyof TuningConfig>(key: K, value: TuningConfig[K]) => {
    setConfig((prev) => (prev ? { ...prev, [key]: value } : prev));
  }, []);

  const setBackend = (backend: 'plecs' | 'ltspice' | 'simplis') => {
    if (!config) return;
    setConfig({ ...config, sim_backend: backend, backend, run_bode_analysis: backend === 'simplis' ? false : config.run_bode_analysis });
  };

  const runAction = async (action: 'auto' | 'single' | 'pause' | 'resume' | 'stop') => {
    try {
      setBanner(null);
      if (action === 'auto') {
        setIterations([]);
        setSelectedIter(null);
        setLogs([]);
        await request('/api/run/auto', { method: 'POST', body: JSON.stringify({ config }) });
      } else if (action === 'single') {
        await request('/api/run/single', {
          method: 'POST',
          body: JSON.stringify({ config, pid: { ...pid, iter_num: iterations.length } })
        });
      } else {
        await request(`/api/run/${action}`, { method: 'POST', body: '{}' });
      }
    } catch (error) {
      setBanner(error instanceof Error ? error.message : String(error));
    }
  };

  const computePid = async () => {
    if (!config) return;
    try {
      const next = await request<PidValues>('/api/pid/compute', {
        method: 'POST',
        body: JSON.stringify({ config, wc: config.wc_initial, phi_m: config.phi_m_initial })
      });
      setPid(next);
    } catch (error) {
      setBanner(error instanceof Error ? error.message : String(error));
    }
  };

  const saveGif = async () => {
    try {
      const result = await request<{ path: string }>('/api/results/gif', { method: 'POST', body: '{}' });
      setLogs((prev) => [...prev.slice(-160), `Saved animation: ${result.path}`]);
    } catch (error) {
      setBanner(error instanceof Error ? error.message : String(error));
    }
  };

  const resetDefaults = async () => {
    if (!defaults || status?.running) return;
    try {
      await request('/api/run/reset', { method: 'POST', body: '{}' });
      setConfig({ ...defaults.config });
      setPid({ ...defaults.pid });
      setIterations([]);
      setSelectedIter(null);
      setLogs([]);
      setBanner('Defaults restored. Ready for a fresh run.');
    } catch (error) {
      setBanner(error instanceof Error ? error.message : String(error));
    }
  };

  const useDefaultExampleModels = () => {
    if (!defaults || status?.running) return;
    setConfig((prev) =>
      prev
        ? {
            ...prev,
            model_id: defaults.config.model_id,
            plecs_model: defaults.config.plecs_model,
            ltspice_asc_model: defaults.config.ltspice_asc_model,
            ltspice_netlist_model: defaults.config.ltspice_netlist_model,
            ltspice_bode_netlist_model: defaults.config.ltspice_bode_netlist_model,
            ltspice_bode_ac_netlist_model: defaults.config.ltspice_bode_ac_netlist_model,
            simplis_schematic_model: defaults.config.simplis_schematic_model,
            simplis_netlist_model: defaults.config.simplis_netlist_model
          }
        : prev
    );
    setBanner('Default buck example models selected.');
  };

  const jumpToRail = (target: RailTarget) => {
    setActiveRail(target);
    window.requestAnimationFrame(() => {
      if (target === 'workbench') {
        workspaceRef.current?.scrollTo({ top: 0, behavior: 'smooth' });
      } else if (target === 'results') {
        lowerRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
      } else {
        backendSectionRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  };

  const waveformOption = useMemo(() => {
    const waveform = selected?.waveform;
    const result = selected?.result;
    const targetOs = config?.target_overshoot ?? 4;
    const targetUs = config?.target_undershoot ?? 4;
    const upper = 5 * (1 + targetOs / 100);
    const lower = 5 * (1 - targetUs / 100);
    return {
      color: [result?.status === 'PASS' ? '#0f9f6e' : '#2563eb', '#94a3b8'],
      tooltip: { trigger: 'axis' },
      legend: { top: 4, right: 8 },
      grid: { left: 54, right: 58, top: 42, bottom: 50 },
      xAxis: {
        type: 'value',
        name: 'Time (ms)',
        nameLocation: 'middle',
        nameGap: 30,
        axisLine: { lineStyle: { color: '#c7ced8' } }
      },
      yAxis: { type: 'value', name: 'Vout (V)', nameGap: 28, min: 4.4, max: 5.6 },
      dataZoom: [{ type: 'inside' }, { type: 'slider', height: 18, bottom: 8 }],
      series: [
        {
          name: result ? `Iter ${result.iter_num}` : 'Vout',
          type: 'line',
          showSymbol: false,
          smooth: false,
          lineStyle: { width: 2.4 },
          data: waveform ? waveform.time_ms.map((x, i) => [x, waveform.vout[i]]) : [],
          markLine: {
            symbol: 'none',
            silent: true,
            lineStyle: { type: 'dashed', width: 1 },
            data: [
              { yAxis: 5, label: { formatter: '5.0 V' } },
              { yAxis: upper, label: { formatter: `${targetOs}% OS` } },
              { yAxis: lower, label: { formatter: `${targetUs}% US` } }
            ]
          }
        }
      ]
    };
  }, [selected, config]);

  const bodeOption = useMemo(() => {
    const bode = selected?.bode;
    const startHz = Math.max(config?.bode_freq_start_hz ?? 10, 1e-6);
    const configuredStopHz = Math.max(config?.bode_freq_stop_hz ?? 100000, 1e-6);
    const stopHz = configuredStopHz > startHz ? configuredStopHz : startHz * 1.001;
    return {
      color: ['#2563eb', '#f97316'],
      tooltip: { trigger: 'axis' },
      legend: { top: 2, right: 12 },
      grid: { left: 64, right: 78, top: 58, bottom: 54 },
      xAxis: {
        type: 'log',
        name: 'Frequency (Hz)',
        nameLocation: 'middle',
        nameGap: 32,
        min: startHz,
        max: stopHz
      },
      yAxis: [
        { type: 'value', name: 'Mag (dB)', nameLocation: 'middle', nameGap: 42 },
        { type: 'value', name: 'Phase (deg)', nameLocation: 'middle', nameGap: 50 }
      ],
      dataZoom: [
        { type: 'inside', filterMode: 'none' },
        { type: 'slider', height: 18, bottom: 8, filterMode: 'none', startValue: startHz, endValue: stopHz }
      ],
      series: [
        {
          name: 'Magnitude',
          type: 'line',
          showSymbol: false,
          data: bode ? bode.freq_hz.map((x, i) => [x, bode.mag_db[i]]) : []
        },
        {
          name: 'Phase',
          type: 'line',
          yAxisIndex: 1,
          showSymbol: false,
          data: bode ? bode.freq_hz.map((x, i) => [x, bode.phase_deg[i]]) : []
        }
      ]
    };
  }, [selected, config]);

  const metricsOption = useMemo(
    () => ({
      color: ['#2563eb', '#14b8a6', '#f97316', '#64748b'],
      tooltip: { trigger: 'axis' },
      legend: { top: 2, right: 10 },
      grid: { left: 52, right: 58, top: 48, bottom: 36 },
      xAxis: { type: 'category', data: iterations.map((item) => `I${item.result.iter_num}`) },
      yAxis: [
        { type: 'value', name: '%', nameLocation: 'middle', nameGap: 34 },
        { type: 'value', name: 'Ts (ms)', nameLocation: 'middle', nameGap: 42 }
      ],
      series: [
        { name: 'OS', type: 'line', data: iterations.map((item) => item.result.overshoot) },
        { name: 'US', type: 'line', data: iterations.map((item) => item.result.undershoot) },
        { name: 'Osc', type: 'bar', data: iterations.map((item) => item.result.osc_count) },
        {
          name: 'Ts',
          type: 'line',
          yAxisIndex: 1,
          data: iterations.map((item) => item.result.settling_time * 1000)
        }
      ]
    }),
    [iterations]
  );

  if (!config) {
    return (
      <main className="boot">
        <RefreshCw className="spin" size={28} />
        <span>Starting workbench...</span>
      </main>
    );
  }

  const activeBackend = (config.sim_backend || config.backend || 'ltspice') as 'plecs' | 'ltspice' | 'simplis';
  const current = status?.current || selected?.result || null;
  const best = status?.best || null;
  const running = Boolean(status?.running);
  const usingDefaultModels = Boolean(
    defaults &&
      config.plecs_model === defaults.config.plecs_model &&
      config.ltspice_asc_model === defaults.config.ltspice_asc_model &&
      config.ltspice_netlist_model === defaults.config.ltspice_netlist_model &&
      config.ltspice_bode_netlist_model === defaults.config.ltspice_bode_netlist_model &&
      config.ltspice_bode_ac_netlist_model === defaults.config.ltspice_bode_ac_netlist_model &&
      config.simplis_schematic_model === defaults.config.simplis_schematic_model &&
      config.simplis_netlist_model === defaults.config.simplis_netlist_model
  );
  const simplisSelected = activeBackend === 'simplis';

  return (
    <div className="app-shell">
      <aside className="rail">
        <div className="brand-mark">
          <Zap size={20} />
        </div>
        <button className={`rail-button ${activeRail === 'workbench' ? 'active' : ''}`} title="Workbench" onClick={() => jumpToRail('workbench')}>
          <Activity size={20} />
        </button>
        <button className={`rail-button ${activeRail === 'results' ? 'active' : ''}`} title="Results" onClick={() => jumpToRail('results')}>
          <BarChart3 size={20} />
        </button>
        <button className={`rail-button ${activeRail === 'backend' ? 'active' : ''}`} title="Backend" onClick={() => jumpToRail('backend')}>
          <Gauge size={20} />
        </button>
      </aside>

      <main className="workspace" ref={workspaceRef}>
        <header className="topbar">
          <div>
            <div className="eyebrow">Buck PID Auto-Tuner</div>
            <h1>Control workbench</h1>
          </div>
          <div className="topbar-actions">
            <div className={`connection ${connected ? 'online' : ''}`}>
              <Circle size={10} fill="currentColor" />
              {connected ? 'Backend online' : 'Backend offline'}
            </div>
            <button className="button ghost" onClick={() => request('/api/results/current').then((data) => setStatus(data as SessionStatus))}>
              <RefreshCw size={16} />
              Refresh
            </button>
          </div>
        </header>

        {banner && (
          <div className="banner">
            <span>{banner}</span>
            <button onClick={() => setBanner(null)}>Dismiss</button>
          </div>
        )}

        <section className="summary-strip">
          <StatTile label="Backend" value={(status?.backend || activeBackend).toUpperCase()} />
          <StatTile label="Mode" value={status?.mode || 'Ready'} tone={running ? 'warn' : 'neutral'} />
          <StatTile label="Phase" value={status?.phase || '-'} />
          <StatTile label="Iteration" value={`${status?.iteration ?? 0} / ${status?.total_iterations || config.max_iterations}`} />
          <StatTile label="Current" value={current ? `${current.status} OS ${fmt(current.overshoot, 1)}%` : '-'} tone={statusClass(current?.status)} />
          <StatTile label="Best" value={best ? `I${best.iter_num} ${best.status}` : '-'} tone={statusClass(best?.status)} />
        </section>

        <section className="analysis-grid">
          <div className="panel waveform-panel">
            <div className="panel-heading">
              <div>
                <h2>Output waveform</h2>
                <p>{selected ? `Iteration ${selected.result.iter_num} · ${selected.result.status}` : 'Waiting for iteration data'}</p>
              </div>
              <span className={`pill ${statusClass(selected?.result.status)}`}>{selected?.result.status || 'Idle'}</span>
            </div>
            <ReactECharts option={waveformOption} className="chart tall" notMerge />
          </div>

          <div className="panel bode-panel">
            <div className="panel-heading">
              <div>
                <h2>Loop gain Bode</h2>
                <p>
                  fc {fmtHz(selected?.bode?.metrics.crossover_hz)} · PM{' '}
                  {fmt(selected?.bode?.metrics.phase_margin_deg, 1)} deg
                </p>
              </div>
              <span className="pill neutral">
                {simplisSelected ? 'SIMPLIS Bode off' : config.ltspice_bode_mode === 'ac' ? 'AC sweep' : 'Switching FRA'}
              </span>
            </div>
            <ReactECharts option={bodeOption} className="chart tall" notMerge />
          </div>
        </section>

        <section className="lower-grid" ref={lowerRef}>
          <div className="panel">
            <div className="panel-heading">
              <div>
                <h2>Metrics trend</h2>
                <p>Overshoot, undershoot, oscillation count, and settling time</p>
              </div>
            </div>
            <ReactECharts option={metricsOption} className="chart compact" notMerge />
          </div>

          <div className="panel history-panel">
            <div className="panel-heading">
              <div>
                <h2>Iteration history</h2>
                <p>Click a row to inspect waveform and Bode data</p>
              </div>
            </div>
            <div className="table-wrap">
              <table>
                <thead>
                  <tr>
                    <th>Iter</th>
                    <th>Phase</th>
                    <th>Status</th>
                    <th>OS%</th>
                    <th>US%</th>
                    <th>Osc</th>
                    <th>Ts ms</th>
                    <th>fc</th>
                    <th>PM</th>
                  </tr>
                </thead>
                <tbody>
                  {iterations.map((item) => (
                    <tr
                      key={item.result.iter_num}
                      className={item.result.iter_num === selected?.result.iter_num ? 'selected' : ''}
                      onClick={() => setSelectedIter(item.result.iter_num)}
                    >
                      <td>{item.result.iter_num}</td>
                      <td>{item.phase}</td>
                      <td>
                        <span className={`mini-status ${statusClass(item.result.status)}`}>{item.result.status}</span>
                      </td>
                      <td>{fmt(item.result.overshoot, 1)}</td>
                      <td>{fmt(item.result.undershoot, 1)}</td>
                      <td>{item.result.osc_count}</td>
                      <td>{fmt(item.result.settling_time * 1000, 3)}</td>
                      <td>{fmtHz(item.bode?.metrics.crossover_hz)}</td>
                      <td>{fmt(item.bode?.metrics.phase_margin_deg, 1)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        </section>
      </main>

      <aside className="inspector">
        <div className="control-stack">
          <div className="segmented">
            <button className={activeBackend === 'ltspice' ? 'active' : ''} onClick={() => setBackend('ltspice')}>
              LTspice
            </button>
            <button className={activeBackend === 'plecs' ? 'active' : ''} onClick={() => setBackend('plecs')}>
              PLECS
            </button>
            <button className={activeBackend === 'simplis' ? 'active' : ''} onClick={() => setBackend('simplis')}>
              SIMPLIS
            </button>
          </div>

          <div className="command-grid">
            <button className="button primary" disabled={running} onClick={() => runAction('auto')}>
              <Play size={16} />
              Auto Tune
            </button>
            <button className="button" disabled={running} onClick={() => runAction('single')}>
              <Zap size={16} />
              Single
            </button>
            <button className="button" disabled={!running || status?.paused} onClick={() => runAction('pause')}>
              <Pause size={16} />
              Pause
            </button>
            <button className="button" disabled={!running || !status?.paused} onClick={() => runAction('resume')}>
              <Play size={16} />
              Resume
            </button>
            <button className="button danger" disabled={!running} onClick={() => runAction('stop')}>
              <Square size={16} />
              Stop
            </button>
            <button className="button" disabled={!iterations.length} onClick={saveGif}>
              <Save size={16} />
              GIF
            </button>
            <button className="button" disabled={running || !defaults} onClick={resetDefaults}>
              <RotateCcw size={16} />
              Reset
            </button>
          </div>
        </div>

        <Section title="PID Parameters">
          <div className="field-grid">
            <Field label="Kp" value={pid.Kp} step={0.001} onChange={(v) => setPid({ ...pid, Kp: Number(v) })} />
            <Field label="Ki" value={pid.Ki} step={10} onChange={(v) => setPid({ ...pid, Ki: Number(v) })} />
            <Field label="Kd" value={pid.Kd} step={1e-7} onChange={(v) => setPid({ ...pid, Kd: Number(v) })} />
            <Field label="Kf" value={pid.Kf} step={1000} onChange={(v) => setPid({ ...pid, Kf: Number(v) })} />
          </div>
        </Section>

        <Section title="Design Variables">
          <div className="field-grid">
            <Field label="wc rad/s" value={config.wc_initial} step={1000} onChange={(v) => patchConfig('wc_initial', Number(v))} />
            <Field
              label="phi_m deg"
              value={fmt(config.phi_m_initial * RAD_TO_DEG, 1)}
              step={1}
              onChange={(v) => patchConfig('phi_m_initial', Number(v) * DEG_TO_RAD)}
            />
          </div>
          <button className="button wide" onClick={computePid}>
            <RotateCcw size={16} />
            Compute PID
          </button>
        </Section>

        <Section title="Targets">
          <div className="field-grid">
            <Field label="OS %" value={config.target_overshoot} step={0.5} onChange={(v) => patchConfig('target_overshoot', Number(v))} />
            <Field label="US %" value={config.target_undershoot} step={0.5} onChange={(v) => patchConfig('target_undershoot', Number(v))} />
            <Field label="Max Osc" value={config.max_oscillations} step={1} onChange={(v) => patchConfig('max_oscillations', Number(v))} />
            <Field
              label="Ts ms"
              value={config.target_settling_time * 1000}
              step={0.05}
              onChange={(v) => patchConfig('target_settling_time', Number(v) / 1000)}
            />
            <Field label="Max Iter" value={config.max_iterations} step={1} min={1} onChange={(v) => patchConfig('max_iterations', Number(v))} />
          </div>
        </Section>

        <Section title="Bode Analysis">
          <label className="check-row">
            <input
              type="checkbox"
              checked={!simplisSelected && config.run_bode_analysis}
              disabled={simplisSelected}
              onChange={(event) => patchConfig('run_bode_analysis', event.target.checked)}
            />
            <span>Run Bode analysis</span>
          </label>
          <div className="segmented small">
            <button disabled={simplisSelected} className={config.ltspice_bode_mode === 'ac' ? 'active' : ''} onClick={() => patchConfig('ltspice_bode_mode', 'ac')}>
              AC
            </button>
            <button
              disabled={simplisSelected}
              className={config.ltspice_bode_mode !== 'ac' ? 'active' : ''}
              onClick={() => patchConfig('ltspice_bode_mode', 'switching')}
            >
              FRA
            </button>
          </div>
          <div className="field-grid">
            <Field label="Start Hz" value={config.bode_freq_start_hz} step={100} disabled={simplisSelected} onChange={(v) => patchConfig('bode_freq_start_hz', Number(v))} />
            <Field label="Stop Hz" value={config.bode_freq_stop_hz} step={1000} disabled={simplisSelected} onChange={(v) => patchConfig('bode_freq_stop_hz', Number(v))} />
            <Field label="Coarse pts" value={config.bode_coarse_num_points} step={1} disabled={simplisSelected} onChange={(v) => patchConfig('bode_coarse_num_points', Number(v))} />
            <Field label="Dense pts" value={config.bode_dense_num_points} step={1} disabled={simplisSelected} onChange={(v) => patchConfig('bode_dense_num_points', Number(v))} />
          </div>
        </Section>

        <Section title="Backend Paths" sectionRef={backendSectionRef}>
          <PathField
            label="LTspice exe"
            value={config.ltspice_exe}
            filters={EXE_FILTERS}
            onChange={(v) => patchConfig('ltspice_exe', v)}
            onError={setBanner}
          />
          <PathField
            label="PLECS exe"
            value={config.plecs_exe}
            filters={EXE_FILTERS}
            onChange={(v) => patchConfig('plecs_exe', v)}
            onError={setBanner}
          />
          <PathField
            label="SIMetrix exe"
            value={config.simplis_exe}
            filters={EXE_FILTERS}
            onChange={(v) => patchConfig('simplis_exe', v)}
            onError={setBanner}
          />
          <div className="model-tools">
            <button className="button wide" disabled={running || !defaults} onClick={useDefaultExampleModels}>
              <RotateCcw size={16} />
              Use Default Buck Example
            </button>
            <span className={`model-source ${usingDefaultModels ? 'default' : ''}`}>
              {usingDefaultModels ? 'Using bundled example models' : 'Custom model paths active'}
            </span>
          </div>
          <div className="model-path-fields">
            <PathField
              label="PLECS model"
              value={config.plecs_model}
              filters={PLECS_MODEL_FILTERS}
              onChange={(v) => patchConfig('plecs_model', v)}
              onError={setBanner}
            />
            <PathField
              label="LTspice schematic"
              value={config.ltspice_asc_model}
              filters={LTSPICE_ASC_FILTERS}
              onChange={(v) => patchConfig('ltspice_asc_model', v)}
              onError={setBanner}
            />
            <PathField
              label="LTspice transient netlist"
              value={config.ltspice_netlist_model}
              filters={LTSPICE_NETLIST_FILTERS}
              onChange={(v) => patchConfig('ltspice_netlist_model', v)}
              onError={setBanner}
            />
            <PathField
              label="LTspice AC Bode netlist"
              value={config.ltspice_bode_ac_netlist_model}
              filters={LTSPICE_NETLIST_FILTERS}
              onChange={(v) => patchConfig('ltspice_bode_ac_netlist_model', v)}
              onError={setBanner}
            />
            <PathField
              label="LTspice FRA Bode netlist"
              value={config.ltspice_bode_netlist_model}
              filters={LTSPICE_NETLIST_FILTERS}
              onChange={(v) => patchConfig('ltspice_bode_netlist_model', v)}
              onError={setBanner}
            />
            <PathField
              label="SIMPLIS schematic"
              value={config.simplis_schematic_model}
              filters={SIMPLIS_SCHEMATIC_FILTERS}
              onChange={(v) => patchConfig('simplis_schematic_model', v)}
              onError={setBanner}
            />
            <PathField
              label="SIMPLIS transient netlist"
              value={config.simplis_netlist_model}
              filters={SIMPLIS_NETLIST_FILTERS}
              onChange={(v) => patchConfig('simplis_netlist_model', v)}
              onError={setBanner}
            />
          </div>
          <div className="path-line">
            <FolderOpen size={15} />
            <span>{status?.results_dir || config.results_dir}</span>
          </div>
        </Section>

        <Section title="Run Log">
          <div className="log-box">
            {logs.slice(-80).map((line, idx) => (
              <div key={`${idx}-${line}`}>{line}</div>
            ))}
          </div>
        </Section>
      </aside>
    </div>
  );
}
