@echo off
echo =========================================================================
echo    BHARATDRIVE-AI: AUTONOMOUS DRIVING PLATFORM INITIALIZED
echo =========================================================================
echo.
echo For Web Dashboard:          run_dashboard.bat  (or python -m http.server 8085)
echo For MATLAB App Designer:    matlab -r "startup; run_dashboard;"
echo For Automated Unit Tests:   python -m pytest python/tests -v
echo For Evaluation Benchmarks:  python python/scripts/run_evaluation_benchmarks.py
echo.
echo Launching Interactive Dashboard now...
call run_dashboard.bat
