# CARLA RadarVision — 프로젝트 개요와 진행 현황

> 기준: 2026-10-21 작업 트리. **현재**는 Python/Qt 프로토타입에서 확인한 상태, **목표**는 C++ Core를 포함한 원 기획, **계획**은 아직 구현되지 않은 작업이다.

## 1. 프로젝트 개요

CARLA의 카메라·레이더 데이터를 외부 프로그램으로 수집하고, **CARLA와 분리된 C++17 Core**에서 영상처리·객체 추적·레이더 처리·센서 융합·이벤트 검출을 수행한다. 원본과 처리 결과를 Qt 검증 도구와 웹 대시보드에 표시하고, 알고리즘 정확도와 실시간 처리 성능을 독립적으로 측정한다. 이 프로젝트는 C++ 응용 개발, Windows/Linux 빌드, 신호처리, OpenCV 비전, 센서 융합 역량을 재현 가능한 코드와 측정 결과로 보여주는 포트폴리오를 목표로 한다.

현재는 Windows용 Python/PySide6 프로토타입에서 Phase 4의 카메라·레이더 거리 융합과 저장 세션 평가까지 구현했다. YOLO 추론에는 별도로 빌드한 TensorRTx C++/CUDA DLL을 사용한다. CARLA 서버와 `hero` 역할의 차량은 별도로 실행해야 한다. 프로젝트 자체의 독립 C++ Core와 DSP 모듈, 융합 기반 이벤트 검출은 남은 작업이다.

### 목표

- 첫 처리 대상은 RGB 카메라 1대와 레이더 1대다. 기존 IMU·GNSS·이벤트 수집은 유지하되 Core의 필수 입력으로 묶지 않는다.
- C++ Core의 공개 데이터 모델과 알고리즘은 CARLA Python API, Qt, FastAPI에 의존하지 않는다.
- 동일한 C++ 소스를 MSVC와 GCC로 빌드하고, 고정 샘플의 Replay·단위 테스트를 양쪽에서 실행한다.
- 원본과 처리 결과를 분리해 저장하고 프레임·시각·좌표계·모델 버전으로 추적할 수 있게 한다.
- 정확도, 처리량, 지연, 누락률을 측정해 검출·추적·융합 기능을 검증한다.

### 목표 실행 환경

| 항목 | 목표 | 현재 확인된 상태 |
| --- | --- | --- |
| 시뮬레이터 | CARLA 0.9.16, Town02, 차량 5~10대 | CARLA 0.9.16 연결 코드와 `hero` 차량 검색 구현. Town02·차량 수는 검증 조건 |
| 개발·실행 OS | Windows 11, WSL2 Ubuntu 24.04 | Python Qt 앱은 Windows에서 구현. Linux Core 빌드는 미구현 |
| GPU·입력 | RTX 4070 SUPER 12GB, RGB 640×480, 레이더 1대 | GPU 확인. 카메라·레이더 부착 구현 |
| 처리 | C++17, OpenCV, CMake, MSVC/GCC | 외부 TensorRTx YOLO DLL과 Python 추적·레이더 처리·거리 융합·평가 구현. 독립 C++ Core와 루트 빌드 파일 미구현 |
| 표시·저장 | Qt 6, FastAPI/WebSocket/React, 파일 후 SQLite 검토 | PySide6 Qt 비교 화면, JPEG/JSONL 기록, MP4 비교 영상, 3~15m 실험 보고서 구현. 웹·DB 미구현 |
| 선택 기능 | LangGraph/LLM API | 필수 완료 기준에 포함하지 않음 |

### 현재 범위와 운영 원칙

- 실행 중인 CARLA 월드에서 지정한 `role_name`의 차량을 찾아 센서를 부착한다. 차량이 없거나 연결이 끊기면 재연결을 시도한다.
- 시뮬레이션 맵과 동기화 설정을 바꾸거나 서버를 직접 tick하지 않는다.
- 앱 종료 시 이 앱이 만든 센서만 제거한다. 기존 차량과 다른 센서는 유지한다.
- 현재 분석 경로는 `core/yolo11_detector.py`와 `monitor/qt/`의 작업자·추적·위치 추정·실험 평가 모듈에 구현되어 있다. 별도 C++ Core, 웹 화면, 전송 계층은 아직 구현되지 않았다.

## 2. 전체 폴더 구조

아래 트리는 주요 소스와 문서, 계획된 디렉터리를 나타낸다. `[비어 있음]`은 현재 구현 파일이 없는 디렉터리다. 환경·생성 파일과 개별 기록 세션은 생략했다.

```text
carla/
├─ benchmarks/                   [비어 있음] 성능 측정용 자리
├─ collector/
│  └─ live.py                     CARLA 센서 수집, 프레임 정렬, 세션 기록
├─ configs/                      [비어 있음] 실행 설정용 자리
├─ core/
│  ├─ yolo11_detector.py          TensorRTx C++/CUDA DLL의 Python 연결
│  ├─ include/                   [비어 있음]
│  ├─ src/                       [비어 있음]
│  └─ tests/                     [비어 있음]
├─ data/
│  └─ recordings/                실행별 JPEG/JSONL 세션과 평가 결과 (Git 제외)
├─ docs/
│  ├─ PROJECT_OVERVIEW.md         이 문서
│  ├─ DISTANCE_ESTIMATION.md     차체 간 거리 정의·융합 규칙·평가 결과
│  ├─ PORTFOLIO_PLAN.md          포트폴리오 방향·설계 결정
│  ├─ PATENT_REFERENCES.md       특허 참고 자료
│  └─ PHASE_1_5_CMAKE.md         TensorRTx 빌드·검증 기록
├─ monitor/
│  ├─ backend/                   [비어 있음]
│  ├─ qt/
│  │  ├─ app.py                  PySide6 모니터와 실행 진입점
│  │  ├─ yolo_worker.py          실시간 YOLO 작업자
│  │  ├─ tracker.py              영상·레이더 보조 추적
│  │  ├─ positioning.py          카메라·레이더 위치 및 거리 추정
│  │  ├─ experiment_evaluator.py 저장 세션의 3~15m 평가
│  │  └─ yolo_overlay_generator.py 저장 세션 비교 영상 생성
│  └─ web/                       [비어 있음]
├─ radar_dsp/
│  ├─ include/                   [비어 있음]
│  ├─ src/                       [비어 있음]
│  └─ tests/                     [비어 있음]
├─ samples/
│  └─ qt_preview.png             README 화면 미리보기
├─ scripts/                      [비어 있음] 실행·유지보수 스크립트용 자리
├─ tests/                        Python 거리 추정·평가·기록 검사
├─ transport/                    [비어 있음] 프로세스 간 데이터 전송용 자리
├─ .gitignore
├─ README.md                     소개와 Windows 실행 안내
├─ requirements.txt              Python 의존성
├─ run_carla_30fps.bat           CARLA 실행 보조 스크립트
└─ run_yolo11n.bat               YOLO 연결 Qt 실행 스크립트
```

`benchmarks/`, `configs/`, `core/include/`, `core/src/`, `core/tests/`, `monitor/backend/`, `monitor/web/`, `radar_dsp/`, `scripts/`, `transport/`의 비어 있는 부분은 **예정된 용도**다. 빈 디렉터리는 Git에서 추적되지 않으므로 새 체크아웃에는 보이지 않을 수 있다. 프로젝트 자체의 C++ 영상처리·센서 융합 모듈과 루트 `CMakeLists.txt`는 아직 없다.

## 3. 현재 구현 구조

```text
CARLA 서버 + 지정 차량
  ├─ RGB / 레이더 / IMU / GNSS / 충돌 / 차선 이탈 센서
  └─ 차량 월드 스냅샷
            ↓
      CarlaCollector (백그라운드 스레드)
            ├─ LiveState (프레임별 센서 값·수집률)
            │    ├─ YOLO 작업자 → 영상 추적 / 레이더 보조 추적
            │    ├─ 카메라·레이더 위치 및 거리 추정
            │    └─ MonitorWindow (Qt 비교 화면)
            └─ SessionRecorder (별도 기록 스레드) → JPEG + JSONL
                         ↓
              저장 세션 평가 → 3~15m 보고서 / 비교 영상
```

| 구성 요소 | 위치 | 책임 |
| --- | --- | --- |
| `CarlaCollector` | `collector/live.py` | CARLA 접속, 차량 검색, 센서 생성·제거, 콜백 변환, 차량 스냅샷 수집, 재연결 |
| `LiveState` | `collector/live.py` | 잠금으로 보호되는 최신 센서 값, 최근 이벤트, 최근 2초 수신률 제공 |
| `SessionRecorder` | `collector/live.py` | 별도 대기열과 스레드에서 측정값 JSONL 및 카메라 JPEG 저장, 누락 수 집계 |
| `YOLO11Detector`·작업자 | `core/yolo11_detector.py`, `monitor/qt/yolo_worker.py` | 별도 TensorRTx DLL로 검출하고 카메라 프레임·시각과 함께 결과 전달 |
| 추적기 | `monitor/qt/tracker.py` | 영상 전용 및 레이더 보조 트랙 연결·ID 관리 |
| 거리·위치 추정 | `monitor/qt/positioning.py` | 같은 프레임의 레이더 점 투영·군집화, 카메라 기본 거리와 조건부 레이더 거리 선택 |
| 저장 세션 평가 | `monitor/qt/experiment_evaluator.py` | 세 방법의 3~15m 차체 간격을 CARLA GT와 비교하고 보고서 생성 |
| 비교 영상 생성 | `monitor/qt/yolo_overlay_generator.py` | 저장 세션에서 원본·검출·추적·융합 2×2 영상 생성 |
| `MonitorWindow` | `monitor/qt/app.py` | RGB 비교 화면, 레이더 플롯·표, 차량·IMU·GNSS 값, 이벤트·저장·실험 버튼 제공 |
| `RadarPlot` | `monitor/qt/app.py` | 레이더 거리·방위각·상대속도를 2D로 표시 |

센서 설정은 현재 `CarlaCollector._attach_sensors()`에 고정되어 있다. RGB는 640×480, 시야각 90°, 설정상 `sensor_tick=0.066666`초(30FPS 월드 기준 약 15FPS)다. 레이더는 최대 거리 100 m, 수평 시야각 35°, 수직 시야각 20°, 초당 포인트 1,500개다. 카메라·레이더·IMU·GNSS를 같은 배치에서 생성하고 모두 동일한 `sensor_tick=0.066666`초로 수집한다. 융합에는 카메라와 CARLA 프레임 번호가 같은 레이더 측정만 사용한다. 실제 수집·표시 FPS는 실행 환경에 따라 달라지며 화면에 측정값으로 표시된다.

카메라 기반 거리와 레이더 기반 거리는 각각 관측한 상대 차 지점부터 내 차 bounding box 외곽까지의 평면 간격이다. 유효한 가까운 레이더 점 묶음이 카메라 거리와 1m 이내로 일치할 때 레이더를 선택하고, 그렇지 않으면 카메라 값을 쓴다. 2차원 위치의 가중 융합과 최종 거리 선택은 별도 계산이다. 자세한 조건과 GT 정의는 [거리 추정 기록](DISTANCE_ESTIMATION.md)에 있다.

## 4. 현재 실행과 기록 형식

Windows에서 CARLA 0.9.16과 대상 차량을 먼저 실행한 뒤, 프로젝트 루트에서 다음 명령을 실행한다. 초기 환경 구축과 CARLA Python wheel 설치 명령은 [README](../README.md#windows-실행)를 따른다.

```powershell
.\.venv\Scripts\python.exe -m monitor.qt.app --host 127.0.0.1 --port 2000 --role hero --output data/recordings
```

`--host`, `--port`, `--role`, `--output`은 선택 인자이며 위 값이 기본값이다. CARLA Python 패키지는 별도 wheel로 설치하며 `requirements.txt`에는 PySide6만 선언되어 있다.

앱 시작만으로 기록하지 않는다. 화면에서 일반 기록, YOLO 검출 기록, YOLO+트래킹 기록, YOLO+레이더 융합 기록 중 하나를 직접 시작한다. 세션 폴더에는 다음 파일이 들어간다.

| 경로 | 내용 |
| --- | --- |
| `session.json` | 생성 시각, CARLA 호스트·포트, 차량 역할, 저장 형식 |
| `measurements.jsonl` | 측정값 한 건당 JSON 한 줄. `kind`로 `camera`, `radar`, `vehicle`, `imu`, `gnss`, `collision`, `lane_invasion` 구분 |
| `camera/*.jpg` | 센서 ID와 프레임 번호로 이름 붙인 카메라 이미지. JSONL의 `image` 상대 경로로 참조 |
| `detections.jsonl` | YOLO 및 두 트래킹 세션의 검출 박스와 원본 프레임 정보 |
| `tracks.jsonl` | 일반 트래킹 세션의 영상 전용 트랙 또는 융합 세션의 레이더 보조 트랙 |
| `vision_tracks.jsonl` | 레이더 융합 세션에서 함께 저장하는 영상 전용 트랙 기준선 |
| `positions.jsonl` | 두 트래킹 세션의 카메라 단독·레이더 보조 위치 추정값과 CARLA 기준 위치 |
| `radar_fusion.jsonl` | 레이더 융합 기록에서 추출한 위치 추정값 전용 파일 |
| `ground_truth.jsonl` | 위치 오차 평가와 트랙 확인용 CARLA 차량 위치 |
| `accuracy_summary.json` | 기록 종료 시 생성되는 위치 정확도·센서 프레임 정렬 요약 |
| `experiment_results/report_3m_15m.md` | 저장 세션의 세 방법별 차체 간격 MAE·RMSE와 표본 수 |
| `experiment_results/summary_3m_15m.json` | 같은 실험의 기계 판독용 집계 |
| `experiment_results/position_errors_3m_15m.csv` | 차량·프레임별 GT/추정 거리와 레이더 선택 근거 |

측정 레코드는 공통적으로 `frame`, `simulation_timestamp`, `receive_timestamp`를 가진다. 센서 레코드에는 `sensor_id`가 있고, 카메라 이외의 센서 레코드에는 `sensor_transform`도 있다. 차량 레코드는 `vehicle_id`, 위치·회전·속도 벡터·km/h 속도를 포함한다. 레이더의 각 포인트는 `distance_m`, `azimuth_rad`, `altitude_rad`, `relative_velocity_mps`를 가진다. 두 각도는 저장 시 라디안이며 화면 표에서 도 단위로 변환한다.

기록 대기열이 가득 차면 해당 데이터는 누락되고 화면에 누적 누락 수가 표시된다. 따라서 저장 파일이 모든 시뮬레이션 프레임을 포함한다고 가정하면 안 된다. 여러 센서 레코드의 대응 관계는 JSONL 줄 순서보다 `frame`과 시뮬레이션 시각으로 판단한다. 레이더 융합 기록은 같은 프레임에 대한 영상 전용 트랙과 레이더 보조 트랙을 모두 저장한다. 화면의 **저장 세션 전체 비교 영상 생성** 버튼은 `*_radar_fusion` 세션에서 원본·YOLO·영상 트래킹·레이더 융합 화면을 2×2 MP4로 만들며, `ffmpeg`가 PATH에 있어야 한다. 과거 융합 세션에 `vision_tracks.jsonl`이 없으면 저장한 검출 기록을 재생해 영상 전용 트래킹 기준선을 생성한다. 추가 오버레이는 `annotated/`, `tracked/`, `radar_fusion/` 폴더에 기록한다.

**저장 세션 3~15m 실험 결과 생성** 버튼은 융합 세션의 카메라 트래킹·카메라/레이더 거리 선택·레이더 보조 트래킹을 같은 CARLA 프레임의 GT와 비교한다. GT는 두 차량의 회전 bounding box 외곽 사이 최단 평면 간격이며, GT 3m 이상 15m 이하인 표본만 평가한다. `accuracy_summary.json`의 기존 2차원 위치 오차와 이 실험의 **차체 간 거리 오차**는 서로 다른 지표다.

## 5. 단계별 진행 현황

### Phase 1 · 실시간 수집과 모니터링 — 구현됨

- 실행 중인 차량에 6종 센서 부착, 차량 스냅샷 수집, 연결 대기·재연결.
- Qt 단일 창에서 영상, 레이더, 주행 상태, 이벤트, 수집률, 기록 현황 표시.
- 세션별 JPEG·JSONL 기록 및 기록 일시 중지·재개.

### Phase 1.5 · TensorRTx YOLO11n 빌드와 저장 영상 검출 — Windows 검증 완료

- TensorRTx의 `yolo11/` CMake로 `yolo11_det`와 `yolo11_detector.dll`을 Windows에서 별도 빌드했다. CUDA 13.4, TensorRT 10.16.1, OpenCV 4.14를 사용했다. 기존 Python 수집기·PySide6 화면은 CMake에 넣지 않는다.
- YOLO11n `.pt`를 `.wts`로 변환하고 TensorRT 엔진을 만들었다. 저장된 CARLA JPEG에서 차량 1대의 클래스·박스·점수가 공식 Ultralytics 결과와 일치하는지 확인했다. `.cu`는 검출 헤드 플러그인과 전후처리를 담당하고 신경망 본체는 TensorRT가 실행한다.
- **남은 검증:** Ubuntu 빌드와 더 많은 장면의 정확도 비교.

현재 로컬 경로와 검증 결과는 [Phase 1.5 TensorRTx 기록](PHASE_1_5_CMAKE.md)을 따른다. CARLA Live 추론·결과 기록·Qt 표시를 연결했고, 기본 클래스 인식 IoU/중심 이동 트래커를 추가했다. 트래커 안정성과 ID 전환율은 다양한 CARLA 장면에서 아직 평가해야 한다.

### Phase 2 · YOLO11n CUDA 영상처리·탐지·트래킹 — Python/Qt 경로 구현

- Phase 1.5에서 검증한 TensorRTx YOLO11n 소스·가중치 변환 결과의 버전을 고정한다. 소스 커밋, 원본·결과 해시, 클래스 목록, 입력 크기·색상·정밀도와 출력 형식을 기록한다.
- 엔진을 한 번 로드하는 C++ DLL과 Python 작업자 스레드를 연결했다. 작업자는 기존 수집기의 최신 카메라 BGRA 프레임을 `ctypes`로 전달한다. 수집 콜백과 Qt GUI 스레드는 추론을 기다리지 않는다. 별도 프로세스/IPC는 현재 필요하지 않아 사용하지 않는다.
- 결과의 원본 `sensor_id`·`frame`·시각을 보존한다. Qt 좌우 패널은 각각 `원본`, `YOLO 검출`, `트래킹`, `레이더 융합 트래킹` 화면을 선택한다. 탐지와 트래킹은 별도 버튼으로 제어하며 트래킹은 탐지가 켜져 있을 때만 동작한다. 클래스·박스 IoU·중심 이동에 기반한 추적기가 ID를 유지하고 최대 10개 처리 프레임 동안 예측 박스를 남긴다. 레이더 포인트를 카메라 이미지로 투영해 검출 박스와 연결하고, 연결된 거리·상대속도를 트랙 연관 점수와 위치 추정에 사용한다. 가림·교차 상황의 ID 전환 평가와 외형 재식별은 남았다.
- 일반 기록은 원본 영상·센서 데이터를, YOLO 기록은 검출 데이터를 저장한다. YOLO+트래킹 기록은 레이더를 트랙 연결에 사용하지 않는 기준선이다. YOLO+레이더 융합 기록은 레이더 거리·속도를 트랙 연결과 위치 추정에 반영하면서 영상 전용 트랙도 함께 저장한다. 두 트래킹 모드는 `detections.jsonl`, `tracks.jsonl`, 평가용 CARLA 차량 위치 `ground_truth.jsonl`, 위치 추정 `positions.jsonl`을 저장한다. 전용 융합 기록에는 영상 전용 결과 `vision_tracks.jsonl`과 레이더 보조 결과 `tracks.jsonl`이 함께 들어가며, 융합 추정 레코드는 `radar_fusion.jsonl`에도 기록한다. 두 트래킹 기록은 종료 시 2차원 위치 정확도의 `accuracy_summary.json`을 만든다. 좌표 변환에는 카메라 FOV·월드 변환과 레이더 월드 변환을 사용한다. 2차원 위치에는 평평한 지면 가정과 고정 가중치(전방: 카메라 0.2/레이더 0.8, 좌우: 카메라 0.65/레이더 0.35)가 적용된다. 차체 간 최종 거리는 Phase 4에서 별도로 선택한다. CARLA 대상 차량 위치는 오차 평가·트랙 ID 연결에만 쓴다. 네 기록 모드는 동시에 사용할 수 없다.
- 초기 위치 기준선과 별도로 차량 간 거리 GT와 세 방법 비교를 구현했다. 같은 기록 세션에서 규칙을 탐색하고 평가한 결과는 [거리 추정 기록](DISTANCE_ESTIMATION.md)에 남겼다. 도로 경사·가림·다른 차량 모델과 경로에서 독립 검증해야 한다.
- 완료 기준은 ID 전환율과 누락 구간을 CARLA 장면에서 평가하고, 추론·추적이 센서 콜백과 파일 기록을 막지 않는지 확인하는 것이다. 전처리·GPU 업로드·모델·후처리·추적 지연과 대기열 누락을 구분해 측정한다. 후보 저장소가 제시한 성능 수치는 이 환경의 측정값으로 사용하지 않는다.

### Phase 3 · 레이더 검출점 처리 — Python 경로 구현, C++/DSP 계획

- Python `monitor/qt/positioning.py`에서 같은 프레임 레이더 점의 영상 투영, 거리·공간 후보 검사, 가까운 점 군집, 시간적 이동 검사와 카메라 관측과의 연관을 구현했다. 이를 독립 C++ `radar_dsp/` 모듈로 이관하고 단위 테스트를 확장하는 일은 남아 있다.
- **별도 I/Q 입력 경로**에서 FFT와 디지털 필터를 구현하고, 합성 신호의 알려진 주파수·잡음 조건으로 스펙트럼과 필터 결과를 검증한다. CARLA 검출점 테스트와 I/Q DSP 테스트의 입력 출처·성능 수치를 분리한다.
- CARLA 레이더는 검출점 배열을 제공하며 원시 I/Q 신호를 제공하지 않는다. 따라서 현재 검출점 처리 결과를 FFT·위상 신호 처리 성능으로 해석하지 않는다.
- 완료 기준은 두 입력 경로의 C++ 단위 테스트와 재현 가능한 결과 그래프·오차 수치다.

### Phase 4 · 카메라·레이더 융합과 거리 평가 — Python/Qt 프로토타입 구현

- Python/Qt에서는 카메라와 레이더를 CARLA 프레임 번호로 맞추고 센서 변환을 적용해 레이더 점을 YOLO 차량 상자에 연결한다. 영상 전용과 레이더 보조 트랙, 2차원 위치, 차체 간 거리를 기록하고 Live 화면에서 비교한다.
- **거리 기준:** GT는 같은 프레임의 내 차·상대 차 회전 bounding box 사이 최단 평면 간격이다. 카메라 관측 지점부터 내 차 외곽까지의 간격을 기본으로 하고, 가까운 레이더 점 3개 이상이 모이며 편차가 0.5m 이하이고 카메라 거리와 1m 이내로 일치하며 시간적 이동 검사를 통과할 때 레이더 거리를 선택한다. 레이더를 쓸 수 없거나 불일치하면 카메라 값을 사용한다. 2차원 위치 가중 융합과 거리 선택은 별개다.
- **평가 경로:** 저장 세션을 한 번에 재생해 영상 트래킹+카메라, 영상 트래킹+거리 융합, 레이더 보조 트래킹+거리 융합을 GT 3~15m 구간에서 비교한다. Markdown 보고서·JSON 집계·차량별 CSV와 2×2 비교 영상을 생성한다. CARLA 대상 차량 위치·크기·자세는 GT와 트랙 확인에만 쓰며 센서 기반 거리 추정에는 넣지 않는다.
- **현재 결과:** 2026-10-21 Town02 융합 세션의 공통 표본 254건에서 카메라 거리 MAE/RMSE는 0.952/1.210m, 조건부 레이더 선택은 0.917/1.199m였다. 레이더 최종 선택은 47건이었다. 두 융합 트래킹 방식의 거리 수치는 이 세션에서 같았다. 규칙 탐색에 쓴 세션을 다시 평가한 값이므로 독립 검증이나 추적 개선의 증거로 해석하지 않는다. 상세 표와 한계는 [차량 간 거리 추정과 평가 기록](DISTANCE_ESTIMATION.md)에 있다.
- **남은 원 기획:** 센서 융합을 독립 C++ Core로 이관하고, 근접·접근 이벤트 검출과 속도 오차·ID 전환·지연 평가를 추가한다. CARLA 충돌·차선 이탈 센서 이벤트 표시는 현재 구현돼 있지만 융합 결과에서 새 위험 이벤트를 판단하지는 않는다. 별도 주행 세션의 독립 검증도 필요하다.

### Phase 5 · Live 전송과 다중 화면 — 계획

- 현재 Python 수집기와 같은 프로세스의 YOLO DLL 연결을 별도 C++ Core·Live 전송 경로로 확장한다. `transport/`에 메시지 버전, 길이, 순서, 오류·재연결 규칙을 정의하고 영상 바이너리와 작은 메타데이터를 구분해 전송 지연·누락률을 측정한다.
- 기존 PySide6 Qt 화면을 검증 도구로 활용하고 `monitor/backend/`의 FastAPI/WebSocket, `monitor/web/`의 React 대시보드를 추가한다. 두 화면이 같은 처리 결과를 소비한다.
- 결과를 원본과 별도 파일에 저장하고 세션 재생을 완성한다. SQLite는 파일 기반 기록의 검색·조회 한계가 확인되면 도입한다.
- 완료 기준은 동일 세션의 Live·Replay 결과를 Qt와 웹에서 확인하고, 표시 지연·데이터 누락을 수치로 보고하는 것이다.

### Phase 6 · 크로스 플랫폼 검증과 공개 산출물 — 계획

- `benchmarks/`에 수집 FPS, C++ 처리 FPS, End-to-End 지연 평균·P95·P99, 통신 지연·누락률, CPU/GPU/VRAM, 멀티스레드 처리량, 거리·속도 오차, ID 유지·전환 횟수를 남긴다.
- Town02의 차량 5~10대, RGB 640×480, 레이더 1대라는 고정 조건에서 반복 측정한다. 목표 수치는 기준 실행 결과를 얻은 뒤 정한다.
- MSVC와 GCC의 CMake 빌드·CTest 결과, Windows 11·WSL2 Ubuntu 24.04 설치 절차, 샘플 세션·Replay 명령, 데모 영상, 외부 라이브러리 목록을 공개 포트폴리오에 정리한다.
- 완료 기준은 제3자가 샘플 데이터로 핵심 C++ 처리 결과와 벤치마크 절차를 재현할 수 있는 것이다.
