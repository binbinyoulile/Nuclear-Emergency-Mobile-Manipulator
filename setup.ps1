[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -m venv .venv
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python -m venv .venv
} else {
    throw "Python 3.10+ is required. Install Python and ensure 'py' or 'python' is on PATH."
}

$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
& $venvPython -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 'Python 3.10+ is required.')"
& $venvPython -m pip install --upgrade pip
& $venvPython -m pip install -r requirements.txt

Write-Host ""
Write-Host "Python environment is ready."
$modelProbe = Join-Path $PSScriptRoot "models\universal_robots_ur10e\assets\base_0.obj"
if (-not (Test-Path $modelProbe)) {
    Write-Host "Official MuJoCo mesh assets are missing. Download them with:"
    Write-Host "  .\.venv\Scripts\python.exe scripts\fetch_models.py"
}
Write-Host "Activate later with: .\.venv\Scripts\Activate.ps1"
