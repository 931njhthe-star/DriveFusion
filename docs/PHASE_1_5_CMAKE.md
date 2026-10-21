# Phase 1.5 — TensorRTx YOLO11n 빌드와 저장 영상 검출

> 상태: 2026-10-21 Windows 빌드·엔진 생성·저장 JPEG 검출·공식 모델 비교 완료. Ubuntu 빌드는 아직 검증하지 않았다. DriveFusion 작업 루트는 `C:\programming\carla`, TensorRTx 체크아웃은 `C:\ai\tensorrtx`다. 전체 단계는 [프로젝트 오버뷰](PROJECT_OVERVIEW.md#5-단계별-진행-현황)를 따른다.

## 빌드 경계

기존 `collector/live.py`는 CARLA 카메라의 BGRA 프레임을 받고, `monitor/qt/app.py`는 같은 Python 프로세스에서 영상을 표시한다. `SessionRecorder`는 별도 스레드로 원본 JPEG와 센서 JSONL을 기록한다. 이 코드를 CMake에 넣지 않는다.

YOLO11n C++ 추론은 [TensorRTx의 `yolo11/` 프로젝트](https://github.com/wang-xinyu/tensorrtx/tree/main/yolo11)에서 **그 프로젝트의 CMake로 별도 빌드**한다. Phase 1.5를 위해 DriveFusion 루트 `CMakeLists.txt`를 만들거나 TensorRTx 소스를 복사하지 않는다. TensorRTx 빌드에는 **C++ 컴파일러, CUDA Toolkit, TensorRT C++ SDK, OpenCV**가 필요하다. 이전의 “C++·CUDA·OpenCV 세 가지만” 조건은 TensorRTx 추론 실행에는 적용할 수 없다.

```text
Phase 1.5: 기존 세션 JPEG → TensorRTx yolo11_det → YOLO11n 검출 결과
Phase 2:   CARLA → 기존 Python 수집기 → Python 작업자 스레드 → 상주 C++ DLL → 기존 PySide6 화면·결과 기록
                           └→ 기존 원본 JPEG·센서 JSONL 기록
```

TensorRTx의 `.cu`에는 검출 헤드 플러그인과 전처리·후처리 커널이 있다. 합성곱 등 신경망 본체는 TensorRT가 실행한다. 따라서 “YOLO11n 신경망 본체 전체를 직접 작성한 `.cu`”로 설명하지 않는다.

## 로컬 환경과 검증 결과

| 항목 | 현재 확인된 상태 |
| --- | --- |
| 소스 | `C:\ai\tensorrtx\yolo11`에 체크아웃됨. `yolo11_det.cpp`, `plugin/yololayer.cu`, `src/*.cu`, CMake 파일 확인 |
| 모델 | `C:\ai\tensorrtx\yolo11\models\yolo11n.pt`와 변환된 `yolo11n.wts` 확인 |
| GPU·CUDA·OpenCV | RTX 4070 SUPER, CUDA 13.4.92, OpenCV 4.14 (`C:\opencv\build`) |
| TensorRT C++ SDK | 10.16.1.11 Windows x64, CUDA 13.2용 패키지. 실제 루트는 `C:\ai\TensorRT-10.16.1.11.Windows.amd64.cuda-13.2\TensorRT-10.16.1.11` |
| Windows 빌드 | MSVC 2022, CUDA 아키텍처 89로 `yolo11_detector.dll`과 `yolo11_det.exe` 빌드 성공. CMake 경로를 인자로 받게 수정하고 `utils.h`는 `std::filesystem` 사용. `config.h`에 중복 포함 방지 추가 |

[저장소 안내](https://github.com/wang-xinyu/tensorrtx/blob/main/yolo11/readme.md)의 예시 환경은 CUDA 12.9·TensorRT 10.10·OpenCV 4.8이다. 이 PC에서는 CUDA 13.4·TensorRT 10.16.1·OpenCV 4.14로 빌드와 검출을 확인했다. TensorRT 11은 아직 시험하지 않았다. 개인 PC 경로는 공용 CMake에 고정하지 않고 설치 위치를 인자로 전달한다.

Windows 재빌드 명령 (`C:\programming\carla`에서 실행):

```powershell
$cmake = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$build = 'C:\programming\carla\build\yolo11-win-trt10.16'
$trt = 'C:\ai\TensorRT-10.16.1.11.Windows.amd64.cuda-13.2\TensorRT-10.16.1.11'
& $cmake -S 'C:\ai\tensorrtx\yolo11' -B $build -G 'Visual Studio 17 2022' -A x64 "-DTENSORRT_ROOT=$trt" '-DOpenCV_DIR=C:/opencv/build' '-DCMAKE_CUDA_ARCHITECTURES=89'
& $cmake --build $build --config Release --target yolo11_det
$bin = Join-Path $build 'Release'
$env:PATH = "$trt\bin;C:\opencv\build\x64\vc16\bin;$env:PATH"
& (Join-Path $bin 'yolo11_det.exe') -s 'C:\ai\tensorrtx\yolo11\models\yolo11n.wts' (Join-Path $bin 'yolo11n.engine') n
```

생성된 엔진은 `build/yolo11-win-trt10.16/Release/yolo11n.engine`다. 엔진은 이 Windows·TensorRT·GPU 조합에서 생성한 것이므로 Ubuntu에서는 별도로 생성한다.

## 작업 순서와 완료 기준

1. TensorRT C++ SDK 버전과 설치 위치를 확정하고, TensorRTx의 YOLO11 CMake를 현재 Windows 환경에 맞게 고친다. `dirent.h` 의존성도 처리한다. 기존 `yolo11_det` 타깃부터 빌드한다.
2. 기존 YOLO11n `.pt`를 `.wts`로 변환하고, TensorRTx 실행 파일로 `.engine`을 생성한다. 변환 도구·소스 커밋·모델 해시를 기록한다. Windows와 Linux의 엔진 파일은 각각 생성한다.
3. 기존 세션에서 JPEG 한 장을 골라 TensorRTx로 검출한다. 현재 예제의 `-d`는 **이미지 파일이 아니라 폴더**를 받으므로 한 장만 담은 입력 폴더를 사용한다.
4. 같은 JPEG의 공식 Ultralytics YOLO11n 결과와 클래스·박스·점수를 비교하고 입력 크기·정밀도·임곗값을 기록한다.

**Windows 검증:** `yolo11_det.exe -s`로 엔진을 생성했고 `-d ... g`로 CUDA 후처리를 실행했다. CARLA 세션의 `414_01642008.jpg`에서 차량 1대(class 2)를 찾았다. C++ DLL 결과는 `(x=298, y=257, w=41, h=34, confidence=0.775)`였고, 같은 이미지의 공식 Ultralytics YOLO11n은 `(298.4, 257.4)-(339.1, 291.5), confidence=0.771`이었다. 한 장에 대한 일치 확인이며 전체 데이터셋 정확도 검증은 아니다. 이후 실제 CARLA 프레임을 C++ DLL로 보내 `detections.jsonl` 기록을 확인했고, Qt offscreen 실행에서 YOLO11n 탭의 검출 이미지 표시도 확인했다. 이후 Phase 2 작업으로 실시간 탐지 제어, 저장 세션의 박스 이미지 생성, 기본 IoU/중심 이동 추적 ID를 추가했지만, ID 전환율·가림 상황은 아직 평가하지 않았다. Ubuntu 빌드는 남았다.

검증에 사용한 TensorRTx 체크아웃은 커밋 `89db562`에 Windows CMake·C++ DLL·`config.h` 중복 포함 방지 등의 로컬 변경을 더한 작업 트리다. 파일 SHA-256은 다음과 같다.

| 파일 | SHA-256 |
| --- | --- |
| `models/yolo11n.pt` | `0EBBC80D4A7680D14987A577CD21342B65ECFD94632BD9A8DA63AE6417644EE1` |
| `models/yolo11n.wts` | `E8C8872F00A63669F4F2478135EA6F9A5DD99707D7E5426D3464C24486CE7B27` |
| `build/yolo11-win-trt10.16/Release/yolo11n.engine` | `12C08A79B4F17421FAAE2C8BB674412B00875BBEE0954431BB773145327D6928` |

## Phase 2 연결 방식

TensorRTx의 `yolo11_det.cpp`는 폴더를 순회하고 결과 JPEG를 저장하는 예제다. 실시간 경로에서는 별도 `yolo11_detector.dll`이 엔진을 한 번 로드한다. DriveFusion의 Python 작업자 스레드는 `LiveState`의 최신 BGRA 프레임을 `ctypes`로 DLL에 전달하고 박스·클래스·점수를 받는다. 카메라 콜백과 Qt GUI 스레드는 추론 완료를 기다리지 않는다. 현재 구현은 같은 프로세스 내 DLL 호출이며 별도 IPC는 사용하지 않는다.

검출 결과에는 반드시 원본 `frame`과 `sensor_id`를 붙인다. PySide6 화면은 같은 프레임의 영상에만 박스를 그린다. 기존 `SessionRecorder`의 원본 JPEG·센서 JSONL은 유지하고, 추론 결과는 같은 세션의 `detections.jsonl`에 별도로 기록한다. 필요한 경우에만 주석이 그려진 JPEG를 추가한다. 레이더·IMU·GNSS 기록은 TensorRTx로 보내지 않는다.
