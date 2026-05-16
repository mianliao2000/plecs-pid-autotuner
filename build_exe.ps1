$ErrorActionPreference = "Stop"

Set-Location -LiteralPath $PSScriptRoot

python -m PyInstaller --noconfirm --clean buck_autotuner.spec

$exe = Join-Path $PSScriptRoot "dist\BuckPidAutoTuner.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    throw "Expected EXE was not created: $exe"
}

Write-Host "Built: $exe"
