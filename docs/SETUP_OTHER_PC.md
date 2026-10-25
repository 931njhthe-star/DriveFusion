# 다른 PC에서 실행하기

현재 구성 기준의 **Windows 대시보드 실행 준비 안내**입니다. 검출 모듈은 Windows·Linux 빌드가 가능하지만, 센서 수집·Qt 대시보드까지 포함한 전체 실행은 Windows 기준으로 준비합니다.

## 1. 먼저 확보할 파일

| 항목 | 준비할 내용 |
| --- | --- |
| 프로젝트 저장소 | GitHub에서 내려받습니다. |
| **수정한 TensorRTx 소스** | 기존 PC의 `C:\ai\tensorrtx\yolo11`을 별도로 확보합니다. 현재 이 저장소에는 포함되어 있지 않습니다. 외부 소스에서 수정한 파일과 빌드 설정도 보존합니다. |
| YOLO11n 가중치 | 기존 모델 가중치를 별도로 준비합니다. `*.wts`는 Git에서 제외됩니다. |
| 기존 실험 자료 | 기존 결과를 분석하려면 `data/recordings/`의 필요한 워크플로 폴더를 별도 복사합니다. 새로 수집할 때는 필요하지 않습니다. |

## 2. 실행·빌드 환경 설치

- NVIDIA GPU 드라이버, CUDA Toolkit, TensorRT C++ SDK, OpenCV
- Visual Studio의 C++ 빌드 도구와 CMake
- Python 및 해당 Python에서 사용할 수 있는 **CARLA 0.9.16 Python API**
- CARLA 0.9.16 시뮬레이터와 사용하려는 맵

기존 PC의 SDK 버전과 빌드 명령은 [통합 포트폴리오 4절](CARLA_RADARVISION_INTEGRATED_PORTFOLIO_20261024.md)과 [CMake 빌드 기록](PHASE_1_5_CMAKE.md)을 참고합니다. 설치 위치와 GPU에 맞춰 경로·CUDA 아키텍처 설정을 변경합니다.

프로젝트 루트의 PowerShell에서 가상환경을 만듭니다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`requirements.txt`에는 현재 PySide6만 있으므로, **CARLA 버전과 Python 버전에 맞는 Python API도 이 가상환경에 별도로 설치**합니다. 설치 후 아래 명령으로 확인합니다.

```powershell
.\.venv\Scripts\python.exe -c "import carla, PySide6; print('Imports OK')"
```

## 3. 검출기 준비와 경로 수정

1. 확보한 TensorRTx 소스를 해당 PC의 SDK 경로로 설정해 CMake로 빌드합니다.
2. 대상 PC 환경에 맞는 TensorRT 엔진을 생성하고 검출 동작을 확인합니다. 기존 `.engine` 파일의 호환성을 가정하지 않습니다.
3. [run_yolo11n.bat](../run_yolo11n.bat)의 다음 값을 실제 위치에 맞게 수정합니다.
   - `YOLO_DLL`: `yolo11_detector.dll` 경로
   - `YOLO_ENGINE`: `yolo11n.engine` 경로
   - `TENSORRT_BIN`: TensorRT 실행 DLL 폴더
   - `OPENCV_BIN`: OpenCV 실행 DLL 폴더
4. [run_carla_30fps.bat](../run_carla_30fps.bat)의 `CARLA_EXE`, `CARLA_CONFIG`를 수정합니다. 이 스크립트는 CARLA 설정 파일의 프레임 제한·스무딩도 검사하므로, 검사 조건에 맞춰 준비하거나 실행 방식을 조정합니다.

`.venv/`, `build/`, `*.engine`, `*.wts`는 Git에서 제외됩니다. **GitHub에서 받은 파일만으로 검출 DLL과 엔진이 준비되지는 않습니다.**

## 4. 실행 순서와 확인

1. CARLA 서버를 실행합니다. 기본 연결 주소는 `127.0.0.1:2000`입니다.
2. CARLA 예제의 `manual_control.py` 등으로 `role_name=hero`인 차량을 생성합니다. 필요하면 맵·주변 교통도 설정합니다.
3. 프로젝트 루트에서 `run_yolo11n.bat`를 실행합니다.
4. 대시보드에서 연결 상태, 카메라 영상, YOLO 검출, 트래킹·거리 표시를 확인합니다.
5. 짧게 녹화한 뒤 세션 저장과 보고서 생성까지 확인합니다. 레이더 예측 위치 보정을 시험하려면 대시보드의 보정 설정도 확인합니다.

실행 문제를 찾을 때는 **Python import → CARLA 연결 → DLL 로딩 → 엔진 추론 → 기록·평가** 순서로 확인합니다.

## 관련 문서

- [주요 문서 안내](DOCUMENTATION_INDEX.md)
- [검출·트래킹·평가 기술 문서](DETECTION_TRACKING_EVALUATION_TECHNICAL.md)
- [레이더 트랙 보정 조건](RADAR_TRACK_CORRECTION.md)

Linux 검출기를 별도로 사용할 때는 `.so`·서버·엔진을 Linux 환경에서 준비합니다. 현재 Windows Qt 작업자에는 DLL/TCP 자동 선택이 연결되어 있지 않으므로, Linux 검출 서버를 실행하는 것만으로 대시보드의 검출 경로가 바뀌지는 않습니다.
