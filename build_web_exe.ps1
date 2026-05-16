$ErrorActionPreference = "Stop"

Set-Location -LiteralPath $PSScriptRoot
Remove-Item Env:ELECTRON_RUN_AS_NODE -ErrorAction SilentlyContinue

function Invoke-Checked {
    param(
        [string]$FilePath,
        [string[]]$Arguments
    )
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $FilePath $($Arguments -join ' ')"
    }
}

Invoke-Checked python @("-m", "py_compile", "web_backend.py", "auto_tune.py", "analyze.py", "bode_plot.py", "iteration_export.py", "ltspice_backend.py", "simplis_backend.py")
Invoke-Checked python @("-m", "PyInstaller", "--noconfirm", "--clean", "buck_backend.spec")

$backendExe = Join-Path $PSScriptRoot "dist\buck_web_backend.exe"
if (-not (Test-Path -LiteralPath $backendExe)) {
    throw "Expected backend EXE was not created: $backendExe"
}

Push-Location -LiteralPath (Join-Path $PSScriptRoot "frontend")
try {
    if (-not (Test-Path -LiteralPath ".\node_modules")) {
        Invoke-Checked npm @("install")
    }
    $env:CSC_IDENTITY_AUTO_DISCOVERY = "false"
    Invoke-Checked npm @("run", "typecheck")
    Invoke-Checked npm @("run", "build")
    Invoke-Checked npm @("run", "dist")
}
finally {
    Pop-Location
}

$webExe = Join-Path $PSScriptRoot "dist-electron\BuckPidAutoTuner-Web.exe"
if (-not (Test-Path -LiteralPath $webExe)) {
    throw "Expected Electron EXE was not created: $webExe"
}

Write-Host "Built: $webExe"
