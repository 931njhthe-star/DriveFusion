# 포트폴리오 근거 색인



작성 기준 2026-10-24. 채용공고는 사용자가 제공한 내용으로 매핑했으며 회사 매출·인원·설립일 등은 문서에서 주장하지 않았다.



## 자료 우선순위



현재 소스와 실제 실험 기록을 우선했다. PROJECT_OVERVIEW의 기준일은 2026-10-21이므로 이후 기능과 차이가 있다. PORTFOLIO_PLAN의 목표 구조를 완료 성과로 전환하지 않았다. 외부 TensorRTx는 제3자 기반이며 프로젝트 전체를 직접 작성한 C++/CUDA 신경망으로 표현하지 않았다.



## 주요 자료



### S01 보정 예측 437건의 원 분석

[radar_corrected_prediction_paired_analysis_20261023.md](../../../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/radar_corrected_prediction_paired_analysis_20261023.md)



### S02 437·242·195건 통합 분석

[radar_corrected_all_groups_integrated_analysis_20261024.md](../../../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/radar_corrected_all_groups_integrated_analysis_20261024.md)



### S03 통합 평가 CSV

[position_errors_weather_combined_all.csv](../../../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/position_errors_weather_combined_all.csv)



### S04 논문 초안과 시뮬레이터 근거

[manuscript.md](../../../data/recordings/workflow_20261023_203600_8af7ab2a/experiment_results/paper_radar_assisted_distance_20261024/manuscript.md)



### S05 현재 구현 기술 문서

[DETECTION_TRACKING_EVALUATION_TECHNICAL.md](../../../docs/DETECTION_TRACKING_EVALUATION_TECHNICAL.md)



### S06 수집·상태·기록 코드

[live.py](../../../collector/live.py)



### S07 작업자·C ABI Python 브리지

[yolo_worker.py](../../../monitor/qt/yolo_worker.py)



### S08 클래스 기반 탐욕 매칭 추적기

[tracker.py](../../../monitor/qt/tracker.py)



### S09 좌표·거리·품질 게이트

[positioning.py](../../../monitor/qt/positioning.py)



### S10 세션 평가와 가시성

[experiment_evaluator.py](../../../monitor/qt/experiment_evaluator.py)



### S11 Qt UI와 워크플로

[app.py](../../../monitor/qt/app.py)



### S12 외부 TensorRTx CMake 빌드 기록

[PHASE_1_5_CMAKE.md](../../../docs/PHASE_1_5_CMAKE.md)



### S13 외부 C++ 추론 구현

[detector.cpp](../../../../../ai/tensorrtx/yolo11/src/detector.cpp)



### S14 레이더 보정·검증 테스트

[test_radar_track_correction.py](../../../tests/test_radar_track_correction.py)



### S15 WSL TCP 클라이언트

[yolo11_tcp_detector.py](../../../core/yolo11_tcp_detector.py)



## 추가 코드와 기록



[PROJECT_OVERVIEW.md](../../../docs/PROJECT_OVERVIEW.md)


[PORTFOLIO_PLAN.md](../../../docs/PORTFOLIO_PLAN.md)


[RADAR_TRACK_CORRECTION.md](../../../docs/RADAR_TRACK_CORRECTION.md)


[yolo11_detector.py](../../../core/yolo11_detector.py)


[visibility.py](../../../monitor/qt/visibility.py)


[radar_track_correction.py](../../../monitor/qt/radar_track_correction.py)


[weather_workflow.py](../../../monitor/qt/weather_workflow.py)


[weather_workflow_evaluator.py](../../../monitor/qt/weather_workflow_evaluator.py)


[test_yolo11_tcp_detector.py](../../../tests/test_yolo11_tcp_detector.py)


[CMakeLists.txt](../../../../../ai/tensorrtx/yolo11/CMakeLists.txt)


[detector_c.h](../../../../../ai/tensorrtx/yolo11/include/detector_c.h)


[detector_c.cpp](../../../../../ai/tensorrtx/yolo11/src/detector_c.cpp)


## 재현 절차



`C:\programming\carla\.venv\Scripts\python.exe output/pdf/golfzon_portfolio_20261024/build_portfolio.py`



기존 CSV를 두 번 읽어 M3 보정 예측을 선정한 뒤 M1·M2를 동일 키로 연결한다. 중복·유효 오차·GT 일치·오차 재계산·242/195 분할·메타데이터를 검사한다. YOLO 재추론 또는 CARLA 재생 실험을 새로 실행하지 않는다. 출력 대상은 이 포트폴리오 폴더와 tmp/pdfs/golfzon_portfolio_20261024의 미리보기뿐이다.



## 제출 전 개인 정보



성명·연락처·수행 기간·본인 담당 범위는 확인되지 않아 새로 만들지 않았다. 개인 이력과 맞는 정보를 별도 이력에 기재한다. 경력 5년, 원시 RF DSP, MCU/FPGA 경험을 이 프로젝트만으로 입증하지 않는다.