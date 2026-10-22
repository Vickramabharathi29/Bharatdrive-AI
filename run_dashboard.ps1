# BharatDrive-AI: Telemetry & Risk Dashboard PowerShell Launcher
# Usage in PowerShell: .\run_dashboard.ps1 or run_dashboard

param(
    [ValidateSet("web", "matlab", "test", "eval")]
    [string]$Mode = "web"
)

Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "   BHARATDRIVE-AI: AUTONOMOUS TELEMETRY & RISK DASHBOARD" -ForegroundColor White
Write-Host "   Adaptive Path Planning & Collision Avoidance on Indian Roads" -ForegroundColor Gray
Write-Host "=========================================================================" -ForegroundColor Cyan

switch ($Mode) {
    "web" {
        Write-Host "`n[1/2] Starting local web server on port 8085..." -ForegroundColor Green
        Start-Process python -ArgumentList "-m http.server 8085" -WindowStyle Hidden
        Start-Sleep -Seconds 1
        Write-Host "[2/2] Opening browser at http://localhost:8085/dashboard/index.html..." -ForegroundColor Green
        Start-Process "http://localhost:8085/dashboard/index.html"
        Write-Host "Dashboard launched successfully!" -ForegroundColor Cyan
    }
    "matlab" {
        $matlabPath = Get-Command matlab -ErrorAction SilentlyContinue
        if ($null -eq $matlabPath) {
            Write-Warning "MATLAB executable not found on system PATH."
            Write-Host "Opening Interactive Web Dashboard as fallback..." -ForegroundColor Yellow
            Start-Process "http://localhost:8085/dashboard/index.html"
        } else {
            Write-Host "Launching MATLAB App Designer Interface..." -ForegroundColor Green
            Start-Process matlab -ArgumentList '-nosplash -r "startup; run_dashboard;"'
        }
    }
    "test" {
        Write-Host "Running automated pytest test suite..." -ForegroundColor Green
        python -m pytest python/tests -v
    }
    "eval" {
        Write-Host "Running evaluation benchmarks..." -ForegroundColor Green
        python python/scripts/run_evaluation_benchmarks.py
    }
}
