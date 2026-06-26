# Start the ROV UI using this project's venv (Flask + opencv/onnx for crab detection).
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py = Join-Path $Root ".venv\Scripts\python.exe"
$Req = Join-Path $Root "requirements.txt"
if (-not (Test-Path $Py)) {
    Write-Error "Missing .venv — run: python -m venv .venv && .\.venv\Scripts\pip install -r requirements.txt"
    exit 1
}
& $Py -c "import flask" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Installing dependencies into .venv (first run)..."
    & $Py -m pip install -r $Req
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
& $Py (Join-Path $Root "rov_ui.py") @args