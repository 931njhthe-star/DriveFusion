# CARLA RadarVision 주요 문서 안내

작성일: 2026-10-24 · 프로젝트 루트: `C:\programming\carla`

프로젝트를 다시 실행하거나, 구현을 확인하거나, 실험 결과와 논문을 이어서 작성할 때 필요한 문서와 자료의 위치를 정리했습니다. 링크는 이 문서가 있는 `docs/` 폴더를 기준으로 작성했습니다.

## 1. 목적에 따라 바로 찾기

| 하고 싶은 일 | 먼저 열 문서 | 확인할 내용 |
| --- | --- | --- |
| 프로젝트를 짧게 소개하기 | [README](../README.md) | 시스템 소개, 대시보드, 센서·검출·트래킹, 실험 결과 요약 |
| 무엇을 만들고 검증했는지 전체적으로 보기 | [통합 기술 포트폴리오](CARLA_RADARVISION_INTEGRATED_PORTFOLIO_20261024.md) | 실행 구조, 플랫폼별 경계, 구현 기술, 전체 데이터와 보정 구간의 결과, 골프존 업무와의 연결 |
| 다시 실행하기 | [실행법](../실행법.txt), [프로젝트 개요](PROJECT_OVERVIEW.md) | CARLA·차량·교통 생성, 검출기 실행, 현재 구조와 진행 현황 |
| 검출부터 보고서까지 구현을 확인하기 | [검출·트래킹·평가 기술 문서](DETECTION_TRACKING_EVALUATION_TECHNICAL.md) | 데이터 동기화, YOLO 호출, 트래킹, 거리 산출, GT·가시성·지표와 보고서 생성 |
| 레이더가 트래킹을 어떻게 보정하는지 보기 | [레이더 트랙 보정 문서](RADAR_TRACK_CORRECTION.md) | 검출 공백의 예측 박스 보정, 연관·품질 조건, 기록 필드와 제약 |
| 차체 간 거리와 레이더 거리 보정 조건 확인하기 | [거리 추정 기록](DISTANCE_ESTIMATION.md) | GT 정의, 카메라 거리, 레이더 군집, 조건부 거리 선택과 평가 이력 |
| 9개 세션의 전체 결과 확인하기 | [전체 통합 보고서][COMBINED], [날씨별 상세 보고서][DETAILED] | 전체·날씨별·거리별 거리 오차, 관측·예측 표본, 가시 GT 기준 검출률 |
| 보정 437건과 레이더 242건·카메라 195건 비교하기 | [보정 구간 통합 짝비교 분석][INTEGRATED] | 세 방법의 모든 쌍별 비교, 날씨·거리별 결과와 포함·제외 조건 |
| 작성해 둔 논문 열기 | [논문 원고][MANUSCRIPT] | 검출 공백 중 레이더 보조 트래킹 효과를 다룬 논문 초안 |
| 논문·블로그용 그림 찾기 | [그림 캡션과 해석 안내][CAPTIONS] | 그림별 용도·주장·한계·출처와 생성 절차 |

## 2. 설계·구현 문서

모두 `docs/`에 있습니다.

| 문서 | 용도와 읽을 부분 |
| --- | --- |
| [CARLA_RADARVISION_INTEGRATED_PORTFOLIO_20261024.md](CARLA_RADARVISION_INTEGRATED_PORTFOLIO_20261024.md) | 시스템 개발과 검증을 함께 설명하는 종합 문서입니다. 3~6절은 실행 구조·CMake·프레임 처리, 7~11절은 트래킹·레이더·기록·평가, 12~15절은 전체 및 보정 구간 결과, 17절은 지원 업무와의 연결입니다. |
| [DETECTION_TRACKING_EVALUATION_TECHNICAL.md](DETECTION_TRACKING_EVALUATION_TECHNICAL.md) | 구현을 조사한 기술 문서입니다. 4절은 영상 트래킹, 5절은 레이더 연관·거리 선택, 6~7절은 GT·가시성과 평가 지표, 8절은 보고서 생성입니다. |
| [RADAR_TRACK_CORRECTION.md](RADAR_TRACK_CORRECTION.md) | YOLO 검출 공백 중 기존 트랙을 레이더로 보정하는 원리와 조건을 설명합니다. 트랙 위치 보정과 최종 거리의 레이더 사용을 구분할 때 참고합니다. |
| [DISTANCE_ESTIMATION.md](DISTANCE_ESTIMATION.md) | 거리 정의와 조건부 레이더 거리 선택의 개발·평가 기록입니다. 이전 세션의 수치가 포함되므로 해당 세션과 평가 모집단을 확인합니다. |
| [PHASE_1_5_CMAKE.md](PHASE_1_5_CMAKE.md) | TensorRTx의 별도 CMake 빌드, Windows 환경·엔진 생성·저장 이미지 검출을 기록했습니다. 문서 상단은 2026-10-21 당시 검증 상태입니다. |
| [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md) | 폴더 구조, 현재 실행·기록 형식, 초기 목표와 단계별 진행 상황을 확인합니다. 목표·계획과 구현 완료 내용을 구분해 읽습니다. |
| [PORTFOLIO_PLAN.md](PORTFOLIO_PLAN.md) | 채용공고 역량에 대응하는 근거와 남은 설계 방향입니다. 향후 계획을 포함하므로 완료 기능의 목록으로 사용하지 않습니다. |
| [PATENT_REFERENCES.md](PATENT_REFERENCES.md) | 골프공·골프클럽의 레이더·영상 연계 관련 설계 참고 문헌입니다. 현재 차량 시스템의 구현 사양과 구분합니다. |

**현재 동작을 수정할 때는 소스와 해당 세션 설정을 함께 확인합니다.** 문서별 작성 시점이 다릅니다. 예를 들어 초기 CMake 문서의 Ubuntu 검증 상태와 이후 통합 포트폴리오의 Windows·Linux 검증 범위가 다를 수 있습니다.

## 3. 기준 실험 데이터와 전체 결과

현재 README와 포트폴리오에서 주로 인용하는 워크플로는 다음입니다.

```text
data/recordings/workflow_20261023_203600_8af7ab2a/
├─ workflow.json                       수집 워크플로 설정·기록
├─ weather_<날씨>/
│  └─ <세션 ID>_radar_fusion/
│     ├─ session.json                   세션 설정과 메타데이터
│     ├─ camera/                        원본 카메라 이미지
│     ├─ visibility_masks/              인스턴스 분할 마스크 이미지
│     ├─ measurements.jsonl             센서 측정 기록
│     ├─ ground_truth.jsonl             GT 차량·자세·차체 정보
│     ├─ detections.jsonl               YOLO 검출 결과
│     ├─ vision_tracks.jsonl            영상 기준 트래킹 결과
│     ├─ tracks.jsonl                   트래킹 결과
│     ├─ positions.jsonl                위치·거리 처리 결과
│     ├─ radar_fusion.jsonl              레이더 융합·보정 결과
│     ├─ visibility_masks.jsonl          마스크 이미지와 프레임의 연결 정보
│     └─ experiment_results/            개별 세션 분석 결과
├─ experiment_results/                 9개 조건 통합 결과·추가 분석·논문
├─ figures/                            출판용 그림·생성 스크립트·캡션
└─ weather_sample_images/               날씨별 대표 원본 이미지 9장
```

| 자료 | 용도 |
| --- | --- |
| [workflow.json][WORKFLOW] | 9개 조건의 수집 구성과 워크플로 기록 확인 |
| [report_weather_combined_all.md][COMBINED] | 9개 조건 전체·날씨별·거리별 결과를 빠르게 확인 |
| [report_weather_detailed_all.md][DETAILED] | 통합 결과와 각 날씨의 상세 분석을 한 문서에서 확인 |
| [summary_weather_combined_all.json][JSON] | 프로그램으로 집계값·표본 수·지표를 확인 |
| [position_errors_weather_combined_all.csv][CSV] | 차량·프레임·방법별 행을 직접 필터링하고 같은 표본으로 재계산 |

보고서의 값은 **방법별 유효 표본**인지 **세 방법의 공통 유효 표본**인지 먼저 확인합니다. 전체 공통 거리 표본 16,491건은 평가 프레임 수 16,213이나 투영 GT 차량·프레임 수 161,061과 다른 단위·모집단입니다.

## 4. 레이더 보정 구간의 심층 분석

세 분석 보고서는 같은 워크플로를 서로 다른 범위로 설명합니다.

| 보고서 | 중심 모집단 | 다시 읽을 이유 |
| --- | --- | --- |
| [radar_corrected_prediction_paired_analysis_20261023.md][PAIRED] | 레이더 보정 예측 437건 | 최초 짝비교 분석, 구현·GT·모집단 정의와 논문 Methods·Results 초안 확인 |
| [radar_selected_prediction_pairwise_analysis_20261024.md][SELECTED] | 최종 레이더 거리 사용 242건 | 레이더 거리 사용 구간의 세 방법 쌍별 개선율을 자세히 확인 |
| [radar_corrected_all_groups_integrated_analysis_20261024.md][INTEGRATED] | 보정 전체 437건 및 레이더 242건·카메라 195건 | 전체·9개 조건·거리 구간별 세 방법 비교를 같은 수준으로 확인하는 통합 분석 |

**전체 시스템의 효과는 3절의 전체 결과와 함께 확인합니다.** 437건은 전체 공통 거리 표본의 2.65%이며, 최종 거리 출처에 따라 242건과 195건으로 나뉩니다.

### 집단 이름이 문서마다 다른 점

| 같은 실제 모집단 | 분석 보고서·논문에서의 표기 | 최근 README 요약에서의 표기 |
| --- | --- | --- |
| 레이더 보정 예측 전체 437건 | A | A+B |
| 최종 레이더 거리 사용 242건 | B | A |
| 최종 카메라 거리 사용 195건 | C | B |

따라서 표를 옮길 때는 **문자보다 표본 수와 선정 조건**을 확인합니다. 272회는 별도 지표인 전체 트래킹 ID 변경 횟수이며, 437건의 분할에 포함되지 않습니다.

## 5. 작성해 둔 논문

위치: `data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/`

| 파일 | 용도 |
| --- | --- |
| [manuscript.md][MANUSCRIPT] | 수정·확장할 때 사용하는 Markdown 논문 원고 |
| [manuscript.pdf][PAPERPDF] | 이미 생성되어 있는 열람용 PDF |
| [manuscript.html][PAPERHTML] | 이미 생성되어 있는 HTML 열람본 |
| [data_snapshot.json][SNAPSHOT] | 논문 생성에 사용한 데이터 스냅샷 확인 |
| [build_paper.py][BUILDPAPER] | 원고·그림·출력물 생성 과정 확인 |
| [assets/][PAPERASSETS] | 논문에 삽입한 데이터 흐름도·오차 비교·하위집단 그림 |

논문 제목은 **「CARLA 기반 영상 검출 공백에서 레이더 보조 추적과 조건부 거리 선택의 효과 분석」**입니다. 현재 초안의 주 분석은 437건이며, 프로젝트 전체 결과를 설명하려면 전체 통합 보고서와 포트폴리오의 12~13절을 함께 참고합니다. 생성 스크립트로 다시 출력할 때는 기존 산출물의 변경 범위를 먼저 확인합니다.

## 6. 그림과 README용 이미지

| 위치 | 내용 |
| --- | --- |
| [figures/captions_and_notes.md][CAPTIONS] | 그림의 본문·보충자료 구분, 캡션, 원본 대조와 해석 한계 |
| [figures/generate_figures.py][FIGSCRIPT] | 그림을 재현하는 Python 스크립트 |
| [figures/][FIGURES] | PNG·SVG 그림. 검출률 히트맵, 거리별 검출률, 표본 구성, 공통 거리 오차, 보정 예측 짝비교, ID 변경·레이더 사용 비율 등 |
| [weather_sample_images/][SAMPLES] | 각 날씨 세션의 중간 시점에서 복사한 원본 이미지 1장씩, 총 9장 |
| [docs/images/weather/](images/weather/) | GitHub README의 3×3 날씨 표에 사용할 이미지 9장 |

## 7. 예전에 분리해서 만든 제출용 문서

`output/pdf/golfzon_portfolio_20261024/`에는 통합 포트폴리오 이전에 만든 문서가 있습니다.

| 문서 | 위치 |
| --- | --- |
| 센서 융합 연구보고서 | [01_sensor_fusion_research.md](../output/pdf/golfzon_portfolio_20261024/01_sensor_fusion_research.md) |
| 실시간 아키텍처 설계서 | [02_realtime_architecture.md](../output/pdf/golfzon_portfolio_20261024/02_realtime_architecture.md) |
| 연구개발 성과 요약서 | [03_rd_achievement_summary.md](../output/pdf/golfzon_portfolio_20261024/03_rd_achievement_summary.md) |
| 근거 목록 | [evidence_index.md](../output/pdf/golfzon_portfolio_20261024/evidence_index.md) |

같은 폴더에 기존 PDF·HTML도 있습니다. 현재 제출용 내용을 이어서 정리할 때는 **통합 기술 포트폴리오를 먼저 확인**하고, 이 문서들은 이전 구성과 표현을 참고하는 용도로 사용합니다.

## 8. 문서에서 실제 코드로 이동하기

| 확인할 기능 | 주요 소스 |
| --- | --- |
| 센서 생성·콜백·세션 기록 | [collector/live.py](../collector/live.py) |
| Python에서 C++ 검출 라이브러리 호출 | [core/yolo11_detector.py](../core/yolo11_detector.py) |
| TCP 검출기 클라이언트 | [core/yolo11_tcp_detector.py](../core/yolo11_tcp_detector.py) |
| 백그라운드 검출·트래킹·거리 처리 연결 | [monitor/qt/yolo_worker.py](../monitor/qt/yolo_worker.py) |
| 영상 및 레이더 보조 트래킹 | [monitor/qt/tracker.py](../monitor/qt/tracker.py) |
| 레이더 트랙 보정용 군집 처리 | [monitor/qt/radar_track_correction.py](../monitor/qt/radar_track_correction.py) |
| 좌표 변환·레이더 연관·거리 산출 | [monitor/qt/positioning.py](../monitor/qt/positioning.py) |
| 가시성 마스크와 YOLO 연결 평가 | [monitor/qt/visibility.py](../monitor/qt/visibility.py) |
| 개별 세션 재생 평가·보고서 | [monitor/qt/experiment_evaluator.py](../monitor/qt/experiment_evaluator.py) |
| 날씨별·통합 보고서 | [monitor/qt/weather_workflow_evaluator.py](../monitor/qt/weather_workflow_evaluator.py) |
| 날씨 수집 스케줄 | [monitor/qt/weather_workflow.py](../monitor/qt/weather_workflow.py) |
| 신호등 제어 | [monitor/qt/traffic_lights.py](../monitor/qt/traffic_lights.py) |
| Qt 대시보드 | [monitor/qt/app.py](../monitor/qt/app.py) |

검출기 CMake 빌드의 외부 작업 경로와 환경은 [CMake 기록](PHASE_1_5_CMAKE.md) 및 [통합 포트폴리오](CARLA_RADARVISION_INTEGRATED_PORTFOLIO_20261024.md)의 4절에서 확인합니다. 프로젝트 전체가 C++로 구현된 것은 아니며, 트래킹·레이더 처리·거리 산출·평가는 현재 Python 구현입니다.

## 9. 나중에 다시 사용할 때 확인할 사항

- **전체 결과와 보정 하위집단 결과를 함께 읽습니다.** 437건·242건의 개선율을 전체 16,491건의 개선율로 옮기지 않습니다.
- **재계산은 같은 키와 분모로 수행합니다.** 방법 비교 키는 `(session, frame, gt_actor_id)`이며, 관측·예측 여부와 유효 거리 조건을 확인합니다.
- **GT 투영과 가시성을 구분합니다.** 투영 GT 수를 그대로 YOLO 검출률의 분모로 사용하지 않습니다. 정의는 기술 문서의 6~7절에서 확인합니다.
- **세션 당시 설정은 각 `session.json`으로 확인합니다.** 현재 소스의 기본값이 과거 실험에도 적용됐다고 가정하지 않습니다.
- **로컬 데이터는 별도로 보관합니다.** 현재 [.gitignore](../.gitignore)는 `/data/`와 `/실행법.txt`를 제외합니다. 이 안내서만 GitHub에 올려서는 해당 데이터·보고서·논문이 함께 보존되지 않습니다. 실험을 다시 분석하려면 원본과 분석 CSV·JSON을 포함한 워크플로 폴더를 별도로 보관합니다.
- **제출·공유 시 연결 파일도 함께 준비합니다.** 논문의 그림은 `assets/`, README 날씨 이미지는 `docs/images/weather/`에 있습니다. 기준 실험이 바뀌면 이 안내서의 워크플로 경로와 보고서 링크도 함께 갱신합니다.

[WORKFLOW]: ../data/recordings/workflow_20261023_203600_8af7ab2a/workflow.json
[COMBINED]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/report_weather_combined_all.md
[DETAILED]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/report_weather_detailed_all.md
[JSON]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/summary_weather_combined_all.json
[CSV]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/position_errors_weather_combined_all.csv
[PAIRED]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/radar_corrected_prediction_paired_analysis_20261023.md
[SELECTED]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/radar_selected_prediction_pairwise_analysis_20261024.md
[INTEGRATED]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/radar_corrected_all_groups_integrated_analysis_20261024.md
[MANUSCRIPT]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/manuscript.md
[PAPERPDF]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/manuscript.pdf
[PAPERHTML]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/manuscript.html
[SNAPSHOT]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/data_snapshot.json
[BUILDPAPER]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/build_paper.py
[PAPERASSETS]: ../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/assets/
[CAPTIONS]: ../data/recordings/workflow_20261023_203600_8af7ab2a/figures/captions_and_notes.md
[FIGSCRIPT]: ../data/recordings/workflow_20261023_203600_8af7ab2a/figures/generate_figures.py
[FIGURES]: ../data/recordings/workflow_20261023_203600_8af7ab2a/figures/
[SAMPLES]: ../data/recordings/workflow_20261023_203600_8af7ab2a/weather_sample_images/
