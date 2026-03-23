---
name: plecs-rpc-scope-export
description: Use when a user wants PLECS Standalone scope data exported through RPC/webserver without relying on GUI scope windows. Handles enabling XmlRpc, loading a .plecs model, simulating, exporting scope CSV/bitmap, and producing a MATLAB plot from the exported CSV.
---

# PLECS RPC Scope Export

Use this skill for PLECS Standalone tasks where the user wants a reliable, non-GUI workflow to:
- load a `.plecs` model
- run simulation
- export scope CSV
- export scope bitmap
- create a MATLAB plot from the CSV

## Workflow

1. Create a **new dedicated task folder** for each run.
2. Copy the target `.plecs` model into that folder.
3. Ensure Windows registry key `HKCU\Software\Plexim\PLECS4.9\XmlRpcEnabled` is `true`.
4. Set `ReopenScopes=false` during validation when you need to prove export works without opening the Scope GUI.
5. Launch PLECS and connect to `http://127.0.0.1:1080/RPC2`.
6. Use RPC/webserver calls to:
   - inspect statistics
   - start simulation
   - poll simulation state until finished
   - export scope CSV with `getScopeCsv`
   - export scope bitmap with `getScopeBitmap`
7. Save outputs into the same task folder.
8. Run MATLAB on the CSV and export a plot PNG.

## Required RPC pattern

Use XML-RPC at:
- `http://127.0.0.1:1080/RPC2`

Useful calls:
- `plecs.statistics()`
- `plecs.webserver('startSimulation', modelName, {})`
- `plecs.webserver('getSimulationState', modelName, {})`
- `plecs.webserver('getScopeCsv', modelName + '/Scope', {})`
- `plecs.webserver('getScopeBitmap', modelName + '/Scope', {})`
- `plecs.webserver('getScopeInfo', modelName + '/Scope', {})`

`getScopeCsv` returns XML-RPC Binary payload inside `res['csv']`; decode `.data` and write bytes/text to file.

## Files to reuse

Read and adapt these scripts from `scripts/`:
- `plecs_rpc_pipeline.py` — load/simulate/export CSV + bitmap + summary
- `run_all.ps1` — one-click wrapper
- `plot_buck_scope_rpc.m` — MATLAB plotting template
- `run_matlab_plot.ps1` — MATLAB batch wrapper

## Notes

- Do not rely on opening the Scope window unless the user explicitly wants GUI automation.
- If RPC connection is refused, check `XmlRpcEnabled` and restart PLECS.
- Keep all outputs inside the task folder so runs do not mix.
