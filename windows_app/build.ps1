$ErrorActionPreference = "Stop"

$appDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoDir = Split-Path -Parent $appDir
$venvDir = Join-Path $appDir ".venv"

if (-not (Test-Path $venvDir)) {
    python -m venv $venvDir
}

$python = Join-Path $venvDir "Scripts\python.exe"
& $python -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw "pip upgrade failed with exit code $LASTEXITCODE" }
& $python -m pip install -r (Join-Path $appDir "requirements.txt")
if ($LASTEXITCODE -ne 0) { throw "dependency installation failed with exit code $LASTEXITCODE" }
& $python -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name "WaveCanController" `
    --distpath (Join-Path $appDir "dist") `
    --workpath (Join-Path $appDir "build") `
    --specpath (Join-Path $appDir "build") `
    --paths $repoDir `
    (Join-Path $appDir "wavecan_controller.py")
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed with exit code $LASTEXITCODE" }

Write-Host "Built: $appDir\dist\WaveCanController.exe"
