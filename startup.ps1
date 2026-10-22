# BharatDrive-AI: Environment Startup Script for PowerShell
Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "   BHARATDRIVE-AI: AUTONOMOUS DRIVING PLATFORM INITIALIZED" -ForegroundColor White
Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Available Launch Commands:" -ForegroundColor Yellow
Write-Host "  • Web Dashboard:         .\run_dashboard.ps1 -Mode web"
Write-Host "  • MATLAB Interface:      .\run_dashboard.ps1 -Mode matlab"
Write-Host "  • Run Unit Tests:        .\run_dashboard.ps1 -Mode test"
Write-Host "  • Run Benchmarks:        .\run_dashboard.ps1 -Mode eval"
Write-Host ""
Write-Host "Launching default Web Dashboard..." -ForegroundColor Green
.\run_dashboard.ps1 -Mode web
