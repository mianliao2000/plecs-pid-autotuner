import xmlrpc.client as x
from pathlib import Path
import time
import csv
import subprocess
import sys
import os

TASK = Path(r'C:\Users\liaom\.openclaw\workspace\matlab\task_2026-03-19_plecs_rpc_pipeline')
MODEL_PATH = TASK / 'buck.plecs'
MODEL_ID = 'buck'
RPC_URL = 'http://127.0.0.1:1080/RPC2'
PLECS_EXE = r'C:\Users\liaom\Documents\Plexim\PLECS 4.9 (64 bit)\PLECS.exe'
CSV_PATH = TASK / 'buck_scope_data_rpc.csv'
PNG_PATH = TASK / 'buck_scope_bitmap_rpc.png'
SUMMARY_PATH = TASK / 'run_summary.txt'


def decode_blob(obj):
    if hasattr(obj, 'data'):
        return obj.data
    if isinstance(obj, bytes):
        return obj
    if isinstance(obj, str):
        return obj.encode('utf-8')
    return str(obj).encode('utf-8')


def rpc_connect(retries=12, delay=1.0):
    last = None
    for _ in range(retries):
        try:
            s = x.ServerProxy(RPC_URL, allow_none=True)
            s.plecs.statistics()
            return s
        except Exception as e:
            last = e
            time.sleep(delay)
    raise last


def ensure_plecs_running():
    try:
        s = rpc_connect(retries=1, delay=0.2)
        return s, False
    except Exception:
        subprocess.Popen([PLECS_EXE, str(MODEL_PATH)])
        s = rpc_connect(retries=20, delay=1.0)
        return s, True


def ensure_model_loaded(server):
    stats = server.plecs.statistics()
    ids = [m['id'] for m in stats['models']]
    if MODEL_ID not in ids:
        server.plecs.load(str(MODEL_PATH))
        time.sleep(1)


def run_simulation(server):
    server.plecs.webserver('startSimulation', MODEL_ID, {})
    while True:
        state = server.plecs.webserver('getSimulationState', MODEL_ID, {})
        txt = state.get('state', '') if isinstance(state, dict) else str(state)
        if 'running' not in txt.lower():
            return state
        time.sleep(0.2)


def export_scope_csv(server):
    res = server.plecs.webserver('getScopeCsv', f'{MODEL_ID}/Scope', {})
    blob = res['csv'] if isinstance(res, dict) else res
    CSV_PATH.write_bytes(decode_blob(blob))


def export_scope_bitmap(server):
    res = server.plecs.webserver('getScopeBitmap', f'{MODEL_ID}/Scope', {})
    blob = None
    if isinstance(res, dict):
        blob = res.get('imageData') or res.get('bitmap') or next(iter(res.values()))
    else:
        blob = res
    PNG_PATH.write_bytes(decode_blob(blob))


def summarize_csv():
    with CSV_PATH.open('r', encoding='utf-8', newline='') as f:
        reader = csv.reader(f)
        rows = list(reader)
    header = rows[0]
    data = rows[1:]
    n = len(data)
    start = int(0.8 * n)
    seg = data[start:] if n else []
    def col(i):
        return [float(r[i]) for r in seg]
    summary = {
        'rows': n,
        'header': header,
        'avg_inductor_current': sum(col(1))/len(seg) if seg else None,
        'avg_output_voltage': sum(col(2))/len(seg) if seg else None,
        'min_output_voltage': min(col(2)) if seg else None,
        'max_output_voltage': max(col(2)) if seg else None,
    }
    SUMMARY_PATH.write_text('\n'.join(f'{k}: {v}' for k, v in summary.items()), encoding='utf-8')
    return summary


def main():
    server, launched = ensure_plecs_running()
    ensure_model_loaded(server)
    sim_state = run_simulation(server)
    export_scope_csv(server)
    export_scope_bitmap(server)
    summary = summarize_csv()
    print('launched_plecs:', launched)
    print('simulation_state:', sim_state)
    print('csv:', CSV_PATH)
    print('png:', PNG_PATH)
    print('summary:', SUMMARY_PATH)
    print('avg_output_voltage:', summary['avg_output_voltage'])


if __name__ == '__main__':
    sys.exit(main() or 0)
