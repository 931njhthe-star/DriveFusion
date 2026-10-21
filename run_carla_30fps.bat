@echo off
setlocal

set "CARLA_EXE=C:\CARLA_0.9.16\CarlaUE4.exe"
set "CARLA_CONFIG=C:\CARLA_0.9.16\CarlaUE4\Config\DefaultEngine.ini"
if not exist "%CARLA_EXE%" (
    echo CARLA executable not found: %CARLA_EXE%
    pause
    exit /b 1
)
if not exist "%CARLA_CONFIG%" (
    echo CARLA frame settings not found: %CARLA_CONFIG%
    pause
    exit /b 1
)
findstr /C:"bSmoothFrameRate=true" "%CARLA_CONFIG%" >nul
if errorlevel 1 (
    echo CARLA frame smoothing is not enabled in %CARLA_CONFIG%.
    pause
    exit /b 1
)
findstr /C:"Value=30))" "%CARLA_CONFIG%" >nul
if errorlevel 1 (
    echo CARLA frame limit is not set to 30 in %CARLA_CONFIG%.
    pause
    exit /b 1
)

tasklist /FI "IMAGENAME eq CarlaUE4-Win64-Shipping.exe" /NH | find /I "CarlaUE4-Win64-Shipping.exe" >nul
if not errorlevel 1 (
    echo CARLA is already running. Close the CARLA window, then run this file again.
    pause
    exit /b 1
)

"%CARLA_EXE%" -ExecCmds="t.MaxFPS 30"
