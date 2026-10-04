# NexArm one-command setup for Windows.
#
#   powershell -ExecutionPolicy Bypass -File setup.ps1
#
# Installs uv if it is missing, builds a Python 3.12 environment, installs
# lerobot, then finds the arms and cameras and saves the answers.

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot

Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host " NexArm setup" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""

# --- uv -------------------------------------------------------------------
$uv = Get-Command uv -ErrorAction SilentlyContinue
if (-not $uv) {
    $uv = Get-Command "$env:USERPROFILE\.local\bin\uv.exe" -ErrorAction SilentlyContinue
}
if (-not $uv) {
    Write-Host "[1/4] installing uv..." -ForegroundColor Yellow
    Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
    $env:Path = "$env:USERPROFILE\.local\bin;$env:Path"
    $uv = Get-Command uv -ErrorAction SilentlyContinue
}
if (-not $uv) {
    Write-Host "uv did not install. Open a new PowerShell window and run this again." -ForegroundColor Red
    exit 1
}
Write-Host "[1/4] uv ready" -ForegroundColor Green

# --- environment ----------------------------------------------------------
# lerobot needs Python 3.12 or newer. uv downloads it if this machine has none.
Write-Host "[2/4] building the Python 3.12 environment..." -ForegroundColor Yellow
& $uv.Source venv --python 3.12 .venv --allow-existing
$py = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

# Existing on disk is not the same as working. A uv venv is a trampoline holding
# the absolute path of one exact Python build, and a uv upgrade or a moved folder
# leaves a python.exe that cannot start:
#   uv trampoline failed to spawn Python child process ... entity not found
# Catch it here rather than three steps later, where the error names uv and not
# anything the person was doing.
& $py -c "pass" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "      the environment could not start; rebuilding it..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force (Join-Path $PSScriptRoot ".venv") -ErrorAction SilentlyContinue
    & $uv.Source venv --python 3.12 .venv
    & $py -c "pass" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Python still will not start from .venv. Run: python start.py --doctor" -ForegroundColor Red
        exit 1
    }
}

# --- dependencies ---------------------------------------------------------
Write-Host "[3/4] installing lerobot and dependencies, this takes a few minutes..." -ForegroundColor Yellow
& $uv.Source pip install -e . --python $py
& $uv.Source pip install pyserial opencv-python --python $py

# Recording needs lerobot's "dataset" extra, which `pip install -e .` does NOT
# pull in. Without it teleop works and recording dies at import with a message
# about `datasets` that gives no hint the extra was the problem.
#
# The versions are pinned to the ranges in pyproject.toml rather than installed
# as `.[dataset]`, because that form also drags in torchcodec and can re-resolve
# torch -- and a re-resolved torch has broken this environment before. Picking
# the latest of each instead is what breaks it: av 18 has no `av.option`, which
# lerobot imports, and pandas 3 is outside the supported range.
Write-Host "      adding the recording dependencies..." -ForegroundColor Yellow
& $uv.Source pip install --python $py `
    "av>=15.0.0,<16.0.0" `
    "datasets>=4.7.0,<5.0.0" `
    "pandas>=2.0.0,<3.0.0" `
    "pyarrow>=21.0.0,<30.0.0" `
    "jsonlines>=4.0.0,<5.0.0"

# Motor SDKs for the arms that are not a NexArm: Feetech (SO-100 / SO-101) and
# Dynamixel (Koch, OpenManipulator-X). Same ranges as lerobot's own extras.
Write-Host "      adding support for SO-101, Koch and OpenManipulator arms..." -ForegroundColor Yellow
& $uv.Source pip install --python $py `
    "feetech-servo-sdk>=1.0.0,<2.0.0" `
    "dynamixel-sdk>=3.7.31,<3.9.0" `
    "deepdiff>=7.0.1,<9.0.0"

& $py -c "import lerobot, serial, cv2; from lerobot.scripts import lerobot_record, lerobot_replay; print('imports OK')"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Install finished but the imports failed. Nothing below will work yet." -ForegroundColor Red
    exit 1
}
Write-Host "[3/4] dependencies installed" -ForegroundColor Green

# --- check ----------------------------------------------------------------
Write-Host "[4/4] checking the install..." -ForegroundColor Yellow
Write-Host ""
& $py station\selftest.py
Write-Host ""

Write-Host "=========================================" -ForegroundColor Green
Write-Host " Ready." -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Green
Write-Host ""
Write-Host "  Plug both arms in, switch them on, then run:"
Write-Host ""
Write-Host "  .\.venv\Scripts\python.exe start.py" -ForegroundColor Cyan
Write-Host ""
Write-Host "  That opens the robot page in your browser and does the rest."
Write-Host ""
