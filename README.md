# CARLA RadarVision · Phase 1

CARLA 0.9.16의 실행 중인 `hero` 차량에 프로젝트 전용 센서를 붙이고, RGB 카메라·레이더·차량 상태·IMU·GNSS·충돌·차선 이탈 데이터를 Qt 화면에 표시합니다. 측정값은 실행별 세션에 기록합니다. CARLA 설치본은 이 프로젝트 폴더 밖에 둡니다.

<img width="751" height="466" alt="화면 캡처 2026-10-20 152144" src="https://github.com/user-attachments/assets/de7aeb32-e1fe-46f4-8cde-6faef600e208" />
장기 목표는 CARLA와 분리된 C++17 Core에서 영상처리·추적·레이더 처리·센서 융합을 수행하고 Windows와 Linux에서 같은 코드로 검증하는 것입니다. 전체 폴더 구조, 현재 구현, 단계별 기술 기획은 [프로젝트 구조와 기술 기획](docs/PROJECT_OVERVIEW.md)에 정리했습니다.

## Windows 실행

CARLA와 `hero` 차량을 먼저 실행하세요. 이 프로젝트는 시뮬레이션 맵이나 동기화 설정을 변경하거나 서버를 직접 tick하지 않습니다.

```powershell
cd C:\programming\carla
uv venv --python 3.12 .venv
uv pip install --python .venv\Scripts\python.exe -r requirements.txt
uv pip install --python .venv\Scripts\python.exe "C:\CARLA_0.9.16\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
.\.venv\Scripts\python.exe -m monitor.qt.app
```

다른 CARLA 서버나 차량 역할을 사용할 경우 `--host`, `--port`, `--role`을 지정할 수 있습니다. 예: `--role hero --port 2000`.

## 화면과 기록

- RGB 영상: 640×480, 수집 FPS, 표시 FPS, 프레임 번호, 시뮬레이션 시간
- 레이더: 포인트 위치, 거리, 방위각, 고도각, 상대속도, 포인트 수, FPS
- 차량: 위치, 방향, 이동 속도, 속도 벡터
- IMU: 가속도, 각속도, 나침반 / GNSS: 위도, 경도, 고도
- 충돌과 차선 이탈: 감지된 이벤트 목록

실행 즉시 `data/recordings/YYYYMMDD_HHMMSS_ffffff/`에 기록합니다. `camera/`에는 JPEG 프레임, `measurements.jsonl`에는 각 센서 측정값과 프레임 번호·시뮬레이션 타임스탬프·수신 시각을 저장합니다. 화면의 기록 버튼으로 일시 중지하거나 다시 시작할 수 있습니다. 저장 대기열이 밀리면 누락 수를 화면에 표시합니다. 이 기록 디렉터리는 Git에서 제외됩니다.

Qt 앱이 만든 센서만 종료 시 제거하며, 기존 차량과 기존 센서는 유지합니다.
