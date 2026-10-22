@echo off
REM =========================================================================
REM BharatDrive-AI: Telemetry & Risk Dashboard Launcher
REM Launches the Interactive Web Dashboard and/or MATLAB App Designer
REM =========================================================================

echo =========================================================================
echo    BHARATDRIVE-AI: AUTONOMOUS TELEMETRY & RISK DASHBOARD
echo    Adaptive Path Planning & Collision Avoidance on Indian Roads
echo =========================================================================
echo.
echo Choose an interface to launch:
echo   [1] Interactive Web Dashboard (60 FPS Canvas BEV, SVG Gauge, Evaluation Modal) [Recommended]
echo   [2] MATLAB App Designer Interface (Native MATLAB uifigure)
echo   [3] Run Automated Python Test Suite (83 Tests)
echo   [4] Run Evaluation Metrics Benchmarks & Generate Graphs
echo   [5] Exit
echo.

set /p choice="Enter your choice (1-5) [Default: 1]: "
if "%choice%"=="" set choice=1

if "%choice%"=="1" (
    echo.
    echo Starting local web server on port 8085...
    start "" python -m http.server 8085
    timeout /t 2 /nobreak >nul
    echo Opening browser at http://localhost:8085/dashboard/index.html...
    start "" http://localhost:8085/dashboard/index.html
    echo Dashboard active! Press Ctrl+C in server window to stop.
    goto end
)

if "%choice%"=="2" (
    echo.
    echo Searching for MATLAB on system PATH...
    where matlab >nul 2>&1
    if %errorlevel% neq 0 (
        echo [WARNING] 'matlab' command not found on system PATH.
        echo Please ensure MATLAB R2024b/R2025b is installed and added to PATH,
        echo or open MATLAB manually, navigate to this folder, and run 'startup; run_dashboard'.
        echo.
        echo Fallback: Opening Interactive Web Dashboard instead...
        start "" http://localhost:8085/dashboard/index.html
        goto end
    )
    echo Launching MATLAB App Designer Dashboard...
    matlab -nosplash -r "startup; run_dashboard;"
    goto end
)

if "%choice%"=="3" (
    echo.
    echo Running automated test suite with pytest...
    python -m pytest python/tests -v
    pause
    goto end
)

if "%choice%"=="4" (
    echo.
    echo Running evaluation metrics benchmarks and regenerating graphs...
    python python/scripts/run_evaluation_benchmarks.py
    pause
    goto end
)

:end
