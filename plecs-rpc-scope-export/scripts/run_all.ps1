$ErrorActionPreference = 'Stop'

$task = 'C:\Users\liaom\.openclaw\workspace\matlab\task_2026-03-19_plecs_rpc_pipeline'
$pythonScript = Join-Path $task 'plecs_rpc_pipeline.py'
$matlabScript = Join-Path $task 'run_matlab_plot.ps1'

Write-Host 'Step 1: Run PLECS RPC pipeline...'
python $pythonScript

Write-Host 'Step 2: Run MATLAB plot...'
powershell -ExecutionPolicy Bypass -File $matlabScript

Write-Host 'Done.'
Write-Host "CSV:    $task\buck_scope_data_rpc.csv"
Write-Host "Bitmap: $task\buck_scope_bitmap_rpc.png"
Write-Host "Plot:   $task\buck_scope_plot_matlab.png"
Write-Host "Summary:$task\run_summary.txt"
