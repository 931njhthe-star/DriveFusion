@echo off
setlocal
cd /d "%~dp0"

set "PYTHON=.venv\Scripts\python.exe"
set "YOLO_DLL=build\yolo11-win-trt10.16\Release\yolo11_detector.dll"
set "YOLO_ENGINE=build\yolo11-win-trt10.16\Release\yolo11n.engine"
set "TENSORRT_BIN=C:\ai\TensorRT-10.16.1.11.Windows.amd64.cuda-13.2\TensorRT-10.16.1.11\bin"
set "OPENCV_BIN=C:\opencv\build\x64\vc16\bin"

if not exist "%PYTHON%" (
    echo Missing project Python: %PYTHON%
    echo Create the .venv and install requirements first.
    pause
    exit /b 1
)
if not exist "%YOLO_DLL%" (
    echo Missing YOLO DLL: %YOLO_DLL%
    pause
    exit /b 1
)
if not exist "%YOLO_ENGINE%" (
    echo Missing TensorRT engine: %YOLO_ENGINE%
    pause
    exit /b 1
)
if not exist "%TENSORRT_BIN%\nvinfer_10.dll" (
    echo TensorRT DLLs not found under: %TENSORRT_BIN%
    pause
    exit /b 1
)
if not exist "%OPENCV_BIN%\opencv_world*.dll" (
    echo OpenCV DLL not found under: %OPENCV_BIN%
    pause
    exit /b 1
)

"%PYTHON%" -m monitor.qt.app ^
    --yolo-library ".\%YOLO_DLL%" ^
    --yolo-engine ".\%YOLO_ENGINE%" ^
    --tensorrt-bin "%TENSORRT_BIN%" ^
    --opencv-bin "%OPENCV_BIN%"

echo.
echo Monitor closed with exit code %ERRORLEVEL%.
pause
