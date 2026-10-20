# CARLA RadarVision — 프로젝트 구조와 기술 기획

> 기준: 2026-10-20 작업 트리. **현재**는 코드에서 확인한 상태, **목표**는 사용자가 제시한 원 기획, **계획**은 아직 구현되지 않은 작업이다.

## 1. 프로젝트 개요

CARLA의 카메라·레이더 데이터를 외부 프로그램으로 수집하고, **CARLA와 분리된 C++17 Core**에서 영상처리·객체 추적·레이더 처리·센서 융합·이벤트 검출을 수행한다. 원본과 처리 결과를 Qt 검증 도구와 웹 대시보드에 표시하고, 알고리즘 정확도와 실시간 처리 성능을 독립적으로 측정한다. 이 프로젝트는 C++ 응용 개발, Windows/Linux 빌드, 신호처리, OpenCV 비전, 센서 융합 역량을 재현 가능한 코드와 측정 결과로 보여주는 포트폴리오를 목표로 한다.

현재 구현은 Windows용 Python/PySide6 수집·표시·기록 프로토타입이다. CARLA 서버와 `hero` 역할의 차량은 별도로 실행해야 한다.

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
| 처리 | C++17, OpenCV, CMake, MSVC/GCC | Core·빌드 파일 미구현 |
| 표시·저장 | Qt 6, FastAPI/WebSocket/React, 파일 후 SQLite 검토 | PySide6 Qt 화면과 JPEG/JSONL 기록 구현. 웹·DB 미구현 |
| 선택 기능 | LangGraph/LLM API | 필수 완료 기준에 포함하지 않음 |

### 현재 범위와 운영 원칙

- 실행 중인 CARLA 월드에서 지정한 `role_name`의 차량을 찾아 센서를 부착한다. 차량이 없거나 연결이 끊기면 재연결을 시도한다.
- 시뮬레이션 맵과 동기화 설정을 바꾸거나 서버를 직접 tick하지 않는다.
- 앱 종료 시 이 앱이 만든 센서만 제거한다. 기존 차량과 다른 센서는 유지한다.
- 현 단계에서 실제 구현은 `collector/live.py`와 `monitor/qt/app.py`에 집중되어 있다. 분석 엔진, 웹 화면, 전송 계층은 아직 구현되지 않았다.

## 2. 전체 폴더 구조

아래 트리는 프로젝트 루트의 실제 폴더와 주요 파일을 나타낸다. `[비어 있음]`은 현재 파일이 없는 디렉터리다. `.git/`, `.venv/`, `.vs/`, `data/recordings/`에는 생성 파일이 많아 내부 파일을 나열하지 않았다.

```text
carla/
├─ .git/                         Git 내부 데이터
├─ .venv/                        로컬 Python 가상환경 (Git 제외)
├─ .vs/                          IDE 생성 파일 (Git 제외)
├─ benchmarks/                   [비어 있음] 성능 측정용 자리
├─ collector/
│  ├─ live.py                     CARLA 센서 수집, 공유 상태, 세션 기록
│  └─ __pycache__/                Python 생성 파일 (Git 제외)
├─ configs/                      [비어 있음] 실행 설정용 자리
├─ core/
│  ├─ include/                   [비어 있음]
│  ├─ src/                       [비어 있음]
│  └─ tests/                     [비어 있음]
├─ data/
│  ├─ recordings/                실행별 세션 데이터 (Git 제외)
│  │  └─ YYYYMMDD_HHMMSS_ffffff/
│  │     ├─ session.json
│  │     ├─ measurements.jsonl
│  │     └─ camera/
│  │        └─ <sensor_id>_<frame>.jpg
│  ├─ 화면 캡처 2026-10-20 151919.png
│  └─ 화면 캡처 2026-10-20 152144.png
├─ docs/
│  └─ PROJECT_OVERVIEW.md         이 문서
├─ monitor/
│  ├─ backend/                   [비어 있음]
│  ├─ qt/
│  │  └─ app.py                  PySide6 실시간 모니터와 실행 진입점
│  └─ web/                       [비어 있음]
├─ radar_dsp/
│  ├─ include/                   [비어 있음]
│  ├─ src/                       [비어 있음]
│  └─ tests/                     [비어 있음]
├─ samples/
│  └─ qt_preview.png             README 화면 미리보기
├─ scripts/                      [비어 있음] 실행·유지보수 스크립트용 자리
├─ transport/                    [비어 있음] 프로세스 간 데이터 전송용 자리
├─ .gitignore
├─ README.md                     소개와 Windows 실행 안내
└─ requirements.txt              PySide6 의존성
```

`benchmarks/`, `configs/`, `core/`, `monitor/backend/`, `monitor/web/`, `radar_dsp/`, `scripts/`, `transport/`의 설명은 **예정된 용도**다. 빈 디렉터리는 Git에서 추적되지 않으므로 새 체크아웃에는 보이지 않을 수 있다. C++ 영상처리용 `vision/`, 센서 융합용 `fusion/`, 루트 `CMakeLists.txt`는 이후 단계에서 추가하며 현재 트리에는 없다.

## 3. 현재 구현 구조

```text
CARLA 서버 + 지정 차량
  ├─ RGB / 레이더 / IMU / GNSS / 충돌 / 차선 이탈 센서
  └─ 차량 월드 스냅샷
            ↓
      CarlaCollector (백그라운드 스레드)
            ├─ LiveState (최근 값과 수집률) → MonitorWindow (Qt, 50 ms 갱신)
            └─ SessionRecorder (별도 기록 스레드) → JPEG + JSONL
```

| 구성 요소 | 위치 | 책임 |
| --- | --- | --- |
| `CarlaCollector` | `collector/live.py` | CARLA 접속, 차량 검색, 센서 생성·제거, 콜백 변환, 차량 스냅샷 수집, 재연결 |
| `LiveState` | `collector/live.py` | 잠금으로 보호되는 최신 센서 값, 최근 이벤트, 최근 2초 수신률 제공 |
| `SessionRecorder` | `collector/live.py` | 별도 대기열과 스레드에서 측정값 JSONL 및 카메라 JPEG 저장, 누락 수 집계 |
| `MonitorWindow` | `monitor/qt/app.py` | RGB 영상, 레이더 플롯·표, 차량·IMU·GNSS 값, 이벤트·저장 상태 표시와 기록 토글 |
| `RadarPlot` | `monitor/qt/app.py` | 레이더 거리·방위각·상대속도를 2D로 표시 |

센서 설정은 현재 `CarlaCollector._attach_sensors()`에 고정되어 있다. RGB는 640×480, 시야각 90°, 설정상 `sensor_tick=0.05`초다. 레이더는 최대 거리 100 m, 수평 시야각 35°, 수직 시야각 20°, 초당 포인트 1,500개, `sensor_tick=0.05`초다. IMU와 GNSS도 `sensor_tick=0.05`초다. 실제 수집·표시 FPS는 실행 환경에 따라 달라지며 화면에 측정값으로 표시된다.

## 4. 현재 실행과 기록 형식

Windows에서 CARLA 0.9.16과 대상 차량을 먼저 실행한 뒤, 프로젝트 루트에서 다음 명령을 실행한다. 초기 환경 구축과 CARLA Python wheel 설치 명령은 [README](../README.md#windows-실행)를 따른다.

```powershell
.\.venv\Scripts\python.exe -m monitor.qt.app --host 127.0.0.1 --port 2000 --role hero --output data/recordings
```

`--host`, `--port`, `--role`, `--output`은 선택 인자이며 위 값이 기본값이다. CARLA Python 패키지는 별도 wheel로 설치하며 `requirements.txt`에는 PySide6만 선언되어 있다.

앱을 시작하면 세션 폴더가 즉시 생성되고 기록이 켜진다. 화면 버튼으로 기록을 일시 중지하거나 재개할 수 있다. 세션 폴더에는 다음 파일이 들어간다.

| 경로 | 내용 |
| --- | --- |
| `session.json` | 생성 시각, CARLA 호스트·포트, 차량 역할, 저장 형식 |
| `measurements.jsonl` | 측정값 한 건당 JSON 한 줄. `kind`로 `camera`, `radar`, `vehicle`, `imu`, `gnss`, `collision`, `lane_invasion` 구분 |
| `camera/*.jpg` | 센서 ID와 프레임 번호로 이름 붙인 카메라 이미지. JSONL의 `image` 상대 경로로 참조 |

측정 레코드는 공통적으로 `frame`, `simulation_timestamp`, `receive_timestamp`를 가진다. 센서 레코드에는 `sensor_id`가 있고, 카메라 이외의 센서 레코드에는 `sensor_transform`도 있다. 차량 레코드는 `vehicle_id`, 위치·회전·속도 벡터·km/h 속도를 포함한다. 레이더의 각 포인트는 `distance_m`, `azimuth_rad`, `altitude_rad`, `relative_velocity_mps`를 가진다. 두 각도는 저장 시 라디안이며 화면 표에서 도 단위로 변환한다.

기록 대기열이 가득 차면 해당 데이터는 누락되고 화면에 누적 누락 수가 표시된다. 따라서 저장 파일이 모든 시뮬레이션 프레임을 포함한다고 가정하면 안 된다. 여러 센서 레코드의 대응 관계는 JSONL 줄 순서보다 `frame`과 시뮬레이션 시각으로 판단한다.

## 5. 목표 아키텍처와 데이터 계약

```text
CARLA 서버 (Windows)
    ↓
Python collector ── 원본 카메라·레이더·평가용 GT 기록
    ↓ 버전 있는 SensorFrame + 바이너리 영상
transport adapter ── 파일 Replay 또는 Live 입력
    ↓
C++17 Core (CARLA·Python·Qt 비종속)
    ├─ vision: OpenCV 전처리 → YOLO11n ONNX 추론 → 검출 후처리 → 추적
    ├─ radar_dsp: CARLA 검출점 처리 / 별도 I/Q 입력의 FFT·필터 검증
    └─ fusion: 시간·좌표 변환 → 연관 → 상태 추정 → 이벤트 검출
    ↓ 버전 있는 ProcessedResult
파일 기록 ── Qt 검증 도구 ── FastAPI/WebSocket ── React 대시보드
```

CMake 타깃은 `core`, `vision`, `radar_dsp`, `fusion` 등 역할별 라이브러리로 나누고 CARLA 수집 어댑터와 UI를 바깥에 둔다. 같은 Core를 Windows/MSVC와 WSL2 Ubuntu/GCC에서 빌드·테스트한다. Live와 Replay는 **동일한 처리 인터페이스**에 `SensorFrame`을 공급한다. 수집, C++ 처리, 파일 기록, UI는 제한된 크기의 대기열과 별도 작업자로 연결하고 누락·지연을 계측한다. Python↔C++ 연결 방식은 파일 기반 Replay로 먼저 검증하고, Live 전송은 버전 있는 메시지와 영상 바이너리 페이로드로 구현해 지연·누락을 측정한다. 이 구조는 설계 목표이며 현재 구현 상태를 뜻하지 않는다.

| 공통 메시지 | 필수 정보 | 계산 가능할 때 추가할 정보 |
| --- | --- | --- |
| `SensorFrame` | `schema_version`, `sensor_id`, `frame_id`, `simulation_timestamp`, `receive_timestamp`, `sensor_transform`, `payload_type`, `payload` | 영상 크기·픽셀 형식, 레이더 단위·좌표계, 추적용 단계별 시각 |
| `ProcessedResult` | `schema_version`, `source_frame_id`, `processing_timestamp`, `processing_latency`, `object_id`, `object_class`, `bounding_box`, `confidence` | 융합된 `position`, `radial_velocity`, `estimated_velocity`, `event_type` |

`position`·`estimated_velocity`처럼 아직 계산하지 않은 값은 비워 두고, 카메라 2D 좌표를 3D 위치로 가장하지 않는다. `simulation_timestamp`는 시뮬레이션 시간이고 수신·처리 지연을 직접 계산할 벽시계가 아니다. End-to-End 지연에는 수집 입구, 전송, 처리 완료, 표시 시점의 측정값과 각 시계의 기준을 별도로 기록한다. Windows↔WSL2 경계를 지나는 측정은 시계 차이를 고려한다.

### 레이더 데이터의 기술적 한계

현재 CARLA 레이더의 `raw_data`는 이름과 달리 **반사파의 원시 I/Q 샘플이 아닌 검출점 배열**이다. 검출점에는 거리, 방위각, 고도각, 상대속도가 있다. 따라서 CARLA 입력만으로 FFT·위상 신호 필터 설계나 실제 레이더 수신 신호 처리 성능을 주장할 수 없다. CARLA 경로에서는 검출점 필터링·군집화·추적·카메라 연계를 검증한다. FFT와 디지털 필터는 `radar_dsp/`의 별도 I/Q 입력 인터페이스와 합성 신호 또는 출처가 명확한 샘플로 구현·테스트하고, 두 결과의 근거를 구분해 보고한다. [CARLA 레이더 센서 문서](https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor)

## 6. 단계별 기획

### Phase 1 · 실시간 수집과 모니터링 — 구현됨

- 실행 중인 차량에 6종 센서 부착, 차량 스냅샷 수집, 연결 대기·재연결.
- Qt 단일 창에서 영상, 레이더, 주행 상태, 이벤트, 수집률, 기록 현황 표시.
- 세션별 JPEG·JSONL 기록 및 기록 일시 중지·재개.

### Phase 1.5 · 크로스 플랫폼 C++ Core의 뼈대 — 다음 작업

- 루트 `CMakeLists.txt`와 `CMakePresets.json`을 만들고 C++17 `core` 라이브러리, 샘플 `replay` 실행 파일, CTest를 구성한다. `core`의 공개 헤더는 CARLA·Python·Qt 타입을 포함하지 않는다.
- `SensorFrame`·`ProcessedResult`의 필드·단위·좌표계와 원본/처리 결과의 구분을 정의한다. 기존 JSONL/JPEG 세션을 읽는 최소 파일 어댑터로 CARLA 없이 같은 입력을 공급한다.
- Windows/MSVC와 WSL2 Ubuntu 24.04/GCC에서 동일한 소스를 빌드하고, 고정 샘플의 파싱·시각/좌표 변환·데이터 무결성 테스트를 통과시킨다. 빌드·테스트 명령과 결과를 문서화한다.
- 완료 기준은 양 플랫폼에서 `cmake` 빌드와 `ctest`가 통과하고, 같은 저장 세션을 Core 입력으로 읽는 것이다. [CMake의 CTest 안내](https://cmake.org/cmake/help/latest/guide/tutorial/Testing%20and%20CTest.html)를 따른다.

이 뼈대를 먼저 만들면 이후 YOLO·레이더·융합 코드가 Python Qt 앱의 내부 구현에 묶이지 않는다.

### Phase 2 · C++ 영상처리·YOLO11n 탐지·트래킹 — 계획

- `C:\ai\carla-yolo11n\yolo11n.pt`를 기존 Python 3.13 YOLO 환경에서 ONNX로 한 번 내보낸다. C++ 런타임은 ONNX 파일을 읽으며 학습 폴더나 Python 패키지에 실행 시 의존하지 않는다. 모델 해시, 클래스 목록, 입력·출력 형식을 기록한다.
- `vision/`의 C++17/OpenCV 코드에서 영상 전처리, ONNX Runtime 추론, 검출 후처리, 다중 객체 트래킹, 주석 그리기, JPEG 인코딩을 구현한다. 트래커는 모델 변환과 별도 코드다. [Ultralytics 내보내기](https://docs.ultralytics.com/modes/export), [ONNX Runtime C++](https://onnxruntime.ai/docs/get-started/with-cpp)
- 먼저 Phase 1.5의 Replay 입력으로 `.pt` 기준 탐지와 C++ 결과를 비교한다. 그다음 Python 수집기의 카메라 프레임을 C++ 작업자에 연결한다. 원본 기록과 처리 결과를 분리하고, Qt는 C++에서 반환한 영상과 검출 결과를 표시한다.
- 완료 기준은 양 플랫폼의 CPU 실행에서 동일한 샘플의 검출·추적 테스트가 통과하고, 실제 CARLA 영상에서 ID가 표시되며, 추론 지연이 센서 콜백과 파일 기록을 막지 않는 것이다. GPU 가속은 지원 환경을 명시하고 별도로 측정한다.

### Phase 3 · C++ 레이더 처리와 DSP 검증 — 계획

- CARLA 레이더 검출점에 대한 이상치 제거, 시간적 연관, 군집화, 거리·상대속도 추정과 테스트를 `radar_dsp/`에 구현한다.
- **별도 I/Q 입력 경로**에서 FFT와 디지털 필터를 구현하고, 합성 신호의 알려진 주파수·잡음 조건으로 스펙트럼과 필터 결과를 검증한다. CARLA 검출점 테스트와 I/Q DSP 테스트의 입력 출처·성능 수치를 분리한다.
- [WO2023048459A1](#8-특허-참고-자료)의 구간별 신뢰도 판단과 [US11771954B2](#8-특허-참고-자료)의 다중 이동 신호 구분은 아이디어 검토 자료로 둔다. 원시 위상 신호가 없는 CARLA 경로에 특허의 절차를 그대로 적용했다고 주장하지 않는다.
- 완료 기준은 두 입력 경로의 C++ 단위 테스트와 재현 가능한 결과 그래프·오차 수치다.

### Phase 4 · C++ 센서 융합·이벤트 검출 — 계획

- 카메라 추적 객체와 레이더 객체의 프레임·시각을 맞추고 센서 외부 파라미터와 좌표 변환을 적용해 연관한다. 누락·불일치 측정값의 처리 규칙과 신뢰도를 남긴다.
- 융합 객체의 위치·속도 추정과 근접/접근 이벤트를 구현한다. [WO2024232644A1](#8-특허-참고-자료)의 영상 위치와 레이더 궤적 표시 개념은 비교 대상으로 사용한다.
- CARLA 액터 위치·속도는 **평가용 정답 데이터**로 별도 수집하고 Core 입력에서 제외한다. 거리·속도 오차와 추적 ID 전환을 재현 가능한 시나리오에서 측정한다.
- 완료 기준은 원본 카메라/레이더, 각 처리 결과, 융합 결과를 구별해 Replay·Live에서 확인하고 오차를 보고하는 것이다.

### Phase 5 · Live 전송과 다중 화면 — 계획

- `transport/`에 메시지 버전, 길이, 순서, 오류·재연결 규칙을 명시하고 Python 수집기↔C++ 처리기 간 Live 경로를 다듬는다. 영상 바이너리와 작은 메타데이터를 구분하고 전송 지연·누락률을 측정한다.
- 기존 PySide6 Qt 화면을 검증 도구로 활용하고 `monitor/backend/`의 FastAPI/WebSocket, `monitor/web/`의 React 대시보드를 추가한다. 두 화면이 같은 처리 결과를 소비한다.
- 결과를 원본과 별도 파일에 저장하고 세션 재생을 완성한다. SQLite는 파일 기반 기록의 검색·조회 한계가 확인되면 도입한다.
- 완료 기준은 동일 세션의 Live·Replay 결과를 Qt와 웹에서 확인하고, 표시 지연·데이터 누락을 수치로 보고하는 것이다.

### Phase 6 · 크로스 플랫폼 검증과 공개 산출물 — 계획

- `benchmarks/`에 수집 FPS, C++ 처리 FPS, End-to-End 지연 평균·P95·P99, 통신 지연·누락률, CPU/GPU/VRAM, 멀티스레드 처리량, 거리·속도 오차, ID 유지·전환 횟수를 남긴다.
- Town02의 차량 5~10대, RGB 640×480, 레이더 1대라는 고정 조건에서 반복 측정한다. 목표 수치는 기준 실행 결과를 얻은 뒤 정한다.
- MSVC와 GCC의 CMake 빌드·CTest 결과, Windows 11·WSL2 Ubuntu 24.04 설치 절차, 샘플 세션·Replay 명령, 데모 영상, 외부 라이브러리 목록을 공개 포트폴리오에 정리한다.
- 완료 기준은 제3자가 샘플 데이터로 핵심 C++ 처리 결과와 벤치마크 절차를 재현할 수 있는 것이다.

## 7. 채용 공고 대응과 남은 설계 결정

| 공고의 역량 | 이 프로젝트에서 보여줄 근거 |
| --- | --- |
| C++ 응용 개발·Windows/Linux·CMake | CARLA 비종속 Core, MSVC/GCC 빌드, CTest, 설치·Replay 문서 |
| FFT·필터 등 DSP | 별도 I/Q 입력에서 주파수·잡음 조건을 통제한 C++ 알고리즘과 정량 테스트 |
| 레이더 센서 처리 | CARLA 검출점의 필터링·연관·융합과 입력 한계를 명시한 평가 |
| OpenCV 객체 검출·트래킹 | C++ YOLO11n 추론 전후처리, 객체 ID 추적, 오탐·ID 전환 측정 |
| 카메라 계측·센서 융합 | 좌표·시각 보정, 거리·속도 추정, 평가용 CARLA 정답과의 오차 |
| 멀티스레드·통신·Qt | bounded queue, 버전 있는 메시지, 지연·누락 측정, Qt 검증 화면 |

이 포트폴리오는 구현·설계 역량을 보여주지만 공고의 **경력 5년이라는 기간 요건 자체를 대체하지는 않는다**. 임베디드 MCU/FPGA 연동과 실제 레이더 RF 신호 처리는 별도 장비·데이터 없이 수행한 것으로 적지 않는다.

착수 전 결정할 항목은 Live 전송 형식과 Python↔C++ 경계, 샘플 I/Q 데이터의 출처, YOLO11n 클래스·트래커와 ONNX Runtime 실행 장치, 센서 캘리브레이션 방식, 성능 목표의 기준 장면이다. 성능·정확도 수치는 구현 후 측정한다. 선택 기능인 LLM은 이 기준 기능과 검증이 완료된 뒤 검토한다.

## 8. 특허 참고 자료

아래 세 문헌은 사용자가 지정한 설계 참고 자료다. 모두 **골프공 또는 골프클럽**을 대상으로 하므로, 현재 **CARLA 차량·보행자** 모니터에 그대로 적용되는 구현 사양으로 취급하지 않는다. 특히 이 문헌들이 YOLO 모델이나 현재 프로젝트의 트래커를 지정하는 것은 아니다.

| 문헌 | 문헌의 핵심 내용 | 이 프로젝트에서 검토할 지점 |
| --- | --- | --- |
| [WO2023048459A1 — 타격된 볼의 레이더 데이터로 궤적 산출](https://patents.google.com/patent/WO2023048459A1/ko) | 초기 운동 구간의 레이더 데이터 유효성을 판단하고, 결과에 따라 초기 구간 또는 이후 구간의 데이터로 볼 궤적을 산출한다. | **Phase 3:** 입력 신뢰도에 따라 사용할 측정 구간을 평가하는 개념 참고. |
| [US11771954B2 — 레이더 데이터로 골프클럽 스윙 궤적 산출](https://patents.google.com/patent/US11771954B2/en) | 볼과 클럽 헤드의 레이더 신호를 구분하고 타격 시점을 기준으로 클럽 헤드의 궤적을 산출한다. | **Phase 3:** 여러 이동 신호와 이벤트 시각의 관계를 검토할 때 참고. |
| [WO2024232644A1 — 골프 샷 영상에 레이더 기반 볼 궤적 표시](https://patents.google.com/patent/WO2024232644A1/en) | 카메라 영상에서 볼의 초기 위치를 찾고, 레이더로 얻은 이동 궤적을 영상 위에 표시한다. | **Phase 3→4:** 영상 추적 객체와 레이더 결과의 시간·좌표 연계 및 화면 표시 개념 참고. |

현재 `collector/live.py`가 저장하는 CARLA 레이더 값은 포인트별 거리·방위각·고도각·상대속도다. WO2023048459A1에서 다루는 **원시 레이더 위상 신호**나 US11771954B2의 **볼·클럽 헤드 반사 신호**는 현재 입력에 없다. 따라서 두 문헌의 신호 처리 절차를 현 데이터에 직접 적용할 수 있다고 가정하지 않는다. Phase 3·4에 착수할 때는 입력과 보정 방법, 적용하려는 청구항과 권리 상태를 별도로 확인한다.
