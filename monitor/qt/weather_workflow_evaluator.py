"""Evaluate weather workflow sessions and pool observed or predicted track errors."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from monitor.qt.experiment_evaluator import (
    DISTANCE_BANDS,
    LEGACY_MAX_DISTANCE_M,
    LEGACY_MIN_DISTANCE_M,
    METHODS,
    _selected_rows,
    _summary_for_rows,
    _visibility_report_lines,
    evaluate_session,
)


def is_weather_workflow(path: Path) -> bool:
    return (Path(path) / "workflow.json").is_file()


def _suffix(low: float, high: float | None, include_max: bool) -> str:
    if low == 0 and high is None:
        return "all"
    if high is None:
        return f"{low:g}m_plus"
    return f"{low:g}m_{high:g}m" + ("_exclusive" if not include_max else "")


def _pooled_summary(rows: list[dict[str, Any]], session_summaries: list[dict[str, Any]],
                    low: float, high: float | None, include_max: bool) -> dict[str, Any]:
    selected = _selected_rows(rows, low, high, include_max)
    bands = []
    for label, band_low, band_high in DISTANCE_BANDS:
        if band_high is not None and band_high <= low:
            continue
        if high is not None and (band_low > high or (band_low == high and not include_max)):
            continue
        band_rows = [row for row in selected if band_low <= row["gt_distance_m"]
                     and (band_high is None or row["gt_distance_m"] < band_high)]
        bands.append({"label": label, "min_inclusive": band_low,
                      "max_exclusive": band_high, **_summary_for_rows(band_rows)})
    return {
        "session_count": len(session_summaries),
        "sessions": [summary["session"] for summary in session_summaries],
        "tracking_max_lost_frames_by_session": {
            summary["session"]: summary.get("tracking_max_lost_frames", 10)
            for summary in session_summaries
        },
        "radar_track_correction_by_session": {
            summary["session"]: summary.get("radar_track_correction_enabled", False)
            for summary in session_summaries
        },
        "frames_scored": sum(summary["frames_scored"] for summary in session_summaries),
        "visibility_mask_frames_recorded": sum(
            summary.get("visibility_mask_frames_recorded", 0) for summary in session_summaries),
        "visibility_mask_frames_aligned": sum(
            summary.get("visibility_mask_frames_aligned", 0) for summary in session_summaries),
        "visibility_mask_frames_invalid": sum(
            summary.get("visibility_mask_frames_invalid", 0) for summary in session_summaries),
        "visibility_sessions": sum(
            summary.get("visibility_mask_frames_aligned", 0) > 0
            for summary in session_summaries),
        "distance_filter_m": {"min_inclusive": low, "max_m": high, "include_max": include_max},
        **_summary_for_rows(selected),
        "distance_bands": bands,
        "legacy_3m_15m": _summary_for_rows(
            _selected_rows(rows, LEGACY_MIN_DISTANCE_M, LEGACY_MAX_DISTANCE_M)),
    }


def _fmt(value: float | None) -> str:
    return f"{value:.3f}" if value is not None else "-"


def _method_table(summary: dict[str, Any]) -> list[str]:
    lines = [
        "| 방법 | 관측 매칭 | 거리 표본 관측+예측=전체 | 최종 MAE (m) | 최종 RMSE (m) | 공통 표본 MAE (m) | 관측 MAE (m) | 예측 MAE (m) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for method, label in METHODS.items():
        stats = summary["methods"][method]
        lines.append(
            f"| {label} | {stats['observed_matches']} | "
            f"{stats['observed_distance_samples']}+{stats['predicted_distance_samples']}="
            f"{stats['distance_samples']} | "
            f"{_fmt(stats['mae_m'])} | {_fmt(stats['rmse_m'])} | "
            f"{_fmt(summary['common_sample_mae_m'][method])} | "
            f"{_fmt(stats['observed_mae_m'])} | {_fmt(stats['predicted_mae_m'])} |")
    return lines


def _write_result(destination: Path, rows: list[dict[str, Any]],
                  summary: dict[str, Any], suffix: str, title: str,
                  weather_table: list[tuple[str, dict[str, Any]]] | None = None,
                  skipped: list[dict[str, str]] | None = None) -> dict[str, Any]:
    destination.mkdir(parents=True, exist_ok=True)
    prefix = "weather_combined_" if weather_table is not None else "distance_profile_"
    report_path = destination / f"report_{prefix}{suffix}.md"
    summary_path = destination / f"summary_{prefix}{suffix}.json"
    csv_path = destination / f"position_errors_{prefix}{suffix}.csv"
    fieldnames = list(rows[0]) if rows else []
    with csv_path.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    summary = {**summary, "report_path": str(report_path),
               "summary_path": str(summary_path), "csv_path": str(csv_path)}
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    distance = summary["distance_filter_m"]
    high = distance["max_m"]
    range_text = f"{distance['min_inclusive']:g}m 이상"
    if high is not None:
        range_text += f" {high:g}m {'이하' if distance['include_max'] else '미만'}"
    report = [
        f"# {title}", "",
        f"- 평가 거리: {range_text}",
        f"- 세션: {summary['session_count']}개",
        f"- YOLO 공백 중 레이더 보정 적용 세션: "
        f"{sum(summary['radar_track_correction_by_session'].values())}개",
        f"- 평가 프레임: {summary['frames_scored']}",
        f"- 투영 GT(가림 미판정): {summary['eligible_gt_opportunities']}건",
        f"- 세 방법 공통 관측 또는 예측 거리 표본: {summary['common_samples']}건",
        "- 최종 MAE/RMSE는 현재 YOLO 관측 거리 OR 유지 트랙 예측 거리의 차량·프레임 표본을 합쳐 계산합니다.",
        "- MAE/RMSE는 세션 평균의 평균이 아니라 모든 거리 표본을 합쳐 계산한 값입니다.",
        "- 프레임·차량 ID는 세션별로 구분해 집계합니다.",
        "", "## 전체 추적 거리 결과", "", *_method_table(summary),
    ]
    report.extend([
        "", "## 예측 거리 품질 검사", "",
        "GT 오차를 사용하지 않고 예측 위치의 급변·불안정한 지면 투영을 거부합니다. 트랙은 유지하며 해당 거리만 최종 MAE/RMSE에서 제외합니다.",
        "", "| 방법 | GT 연결 예측 트랙 | 거리 산출 | 거리 미산출 | 품질 검사 거부 | 미산출 이유별 건수 |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ])
    for method, label in METHODS.items():
        stats = summary["methods"][method]
        reasons = ", ".join(
            f"{reason}: {count}"
            for reason, count in sorted(stats["prediction_distance_rejection_reasons"].items())
        ) or "없음"
        report.append(
            f"| {label} | {stats['predicted_track_matches']} | "
            f"{stats['predicted_distance_samples']} | "
            f"{stats['prediction_distance_unavailable']} | "
            f"{stats['prediction_distance_rejected']} | {reasons} |"
        )
    radar_prediction = summary["methods"]["radar_tracking_fusion"]
    report.extend([
        "", "## YOLO 미검출 중 레이더 보정", "",
        "레이더 보정 예측은 현재 프레임의 YOLO 관측과 구분하며 최종 거리 MAE/RMSE에도 포함합니다.",
        f"- 레이더 보정: {radar_prediction['radar_corrected_predicted_frames']}프레임, "
        f"거리 표본 {radar_prediction['radar_corrected_predicted_samples']}건, "
        f"MAE {_fmt(radar_prediction['radar_corrected_predicted_mae_m'])}m",
        f"- 레이더 보정 예측 평면 위치: "
        f"{radar_prediction['radar_corrected_predicted_position_samples']}건, "
        f"MAE {_fmt(radar_prediction['radar_corrected_predicted_position_mae_m'])}m",
        f"- 레이더 보정 예측 박스와 투영 GT 중심 차이: "
        f"{_fmt(radar_prediction['radar_corrected_predicted_bbox_center_mae_px'])}px "
        "(가림 미판정 기하학적 기준)",
        f"- 보정 없는 예측: 거리 표본 "
        f"{radar_prediction['uncorrected_predicted_samples']}건, "
        f"MAE {_fmt(radar_prediction['uncorrected_predicted_mae_m'])}m",
    ])
    report.extend(_visibility_report_lines(summary))
    if weather_table is not None:
        report.extend(["", "## 날씨별 결과", "",
                       "| 날씨 | 세션 | 프레임 | 방법 | 관측+예측 거리 표본 | 최종 MAE (m) | 최종 RMSE (m) |",
                       "| --- | ---: | ---: | --- | ---: | ---: | ---: |"])
        for preset, weather_summary in weather_table:
            for method, label in METHODS.items():
                stats = weather_summary["methods"][method]
                report.append(
                    f"| {preset} | {weather_summary['session_count']} | "
                    f"{weather_summary['frames_scored']} | {label} | "
                    f"{stats['distance_samples']} | {_fmt(stats['mae_m'])} | "
                    f"{_fmt(stats['rmse_m'])} |")
        report.extend([
            "", "## 날씨별 가시성 마스크 기준 YOLO 검출률", "",
            "마스크가 있는 프레임의 가시 GT만 분모로 사용합니다. 마스크가 없는 이전 세션은 `측정 불가`입니다.",
            "", "| 날씨 | 마스크 정렬 프레임/평가 프레임 | 가시 GT | YOLO 검출/가시 GT |",
            "| --- | ---: | ---: | ---: |",
        ])
        for preset, weather_summary in weather_table:
            item = weather_summary["visibility"]
            visible = item["visible_gt_opportunities"]
            detected = item["yolo_detected_visible_gt"]
            rate = (f"{detected}/{visible} ({detected / visible * 100:.1f}%)"
                    if visible else "측정 불가")
            report.append(
                f"| {preset} | {weather_summary['visibility_mask_frames_aligned']}"
                f"/{weather_summary['frames_scored']} | {visible} | {rate} |")
    report.extend(["", "## 거리 구간별 결과", "",
                   "| GT 거리 | 방법 | 관측+예측 거리 표본 | 최종 MAE (m) | 최종 RMSE (m) |",
                   "| --- | --- | ---: | ---: | ---: |"])
    for band in summary["distance_bands"]:
        for method, label in METHODS.items():
            stats = band["methods"][method]
            report.append(f"| {band['label']} | {label} | {stats['distance_samples']} | "
                          f"{_fmt(stats['mae_m'])} | {_fmt(stats['rmse_m'])} |")
    report.extend(["", "## 기존 3~15m 거리 범위의 추적 결과", "", *_method_table(summary["legacy_3m_15m"]),
                   "", "## 포함 세션", ""])
    report.extend(f"- `{session}`" for session in summary["sessions"])
    if skipped:
        report.extend(["", "## 제외한 세션", ""])
        report.extend(f"- `{item['session']}`: {item['reason']}" for item in skipped)
    report.append("")
    report_path.write_text("\n".join(report), encoding="utf-8")
    return summary


def write_detailed_weather_report(summary: dict[str, Any]) -> Path:
    """Keep the compact report and add a standalone report with full weather sections."""
    short_path = Path(summary["report_path"])
    detailed_path = short_path.with_name(short_path.name.replace(
        "report_weather_combined_", "report_weather_detailed_", 1))
    if detailed_path == short_path:
        raise ValueError("통합 날씨 보고서 경로가 아닙니다.")
    overview = short_path.read_text(encoding="utf-8").splitlines()
    overview[0] = "# 날씨별 상세 통합 거리 추정 결과"
    lines = [*overview, "", "---", "", "## 날씨별 상세 분석", ""]
    for preset, weather in summary["weather_results"].items():
        weather_path = Path(weather["report_path"])
        lines.append(f"별도 보고서: `{weather_path}`")
        lines.append("")
        for line in weather_path.read_text(encoding="utf-8").splitlines():
            lines.append("##" + line if line.startswith("#") else line)
        lines.append("")
    detailed_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    summary["detailed_report_path"] = str(detailed_path)
    Path(summary["summary_path"]).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return detailed_path


def evaluate_weather_workflow(workflow_dir: Path, min_distance_m: float = 0.0,
                              max_distance_m: float | None = None,
                              include_max: bool = True) -> dict[str, Any]:
    workflow_dir = Path(workflow_dir).resolve()
    if not is_weather_workflow(workflow_dir):
        raise ValueError("날씨 워크플로 상위 폴더를 선택하세요.")
    manifest = json.loads((workflow_dir / "workflow.json").read_text(encoding="utf-8"))
    workflow_id = manifest["workflow_id"]
    by_weather: dict[str, dict[str, Any]] = {}
    skipped: list[dict[str, str]] = []
    for weather_dir in sorted(workflow_dir.glob("weather_*")):
        if not weather_dir.is_dir():
            continue
        preset = weather_dir.name.removeprefix("weather_")
        for session_dir in sorted(weather_dir.iterdir()):
            if not session_dir.is_dir() or not (session_dir / "session.json").is_file():
                continue
            try:
                session_metadata = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
                context = session_metadata.get("weather_workflow") or {}
                if (context.get("workflow_id") != workflow_id
                        or context.get("weather_preset") != preset):
                    raise ValueError("폴더의 날씨·워크플로 ID와 세션 정보가 다릅니다")
                raw_rows: list[dict[str, Any]] = []
                session_summary = evaluate_session(
                    session_dir, min_distance_m=min_distance_m,
                    max_distance_m=max_distance_m, include_max=include_max,
                    rows_out=raw_rows)
            except (ValueError, KeyError, OSError, json.JSONDecodeError) as exc:
                skipped.append({"session": str(session_dir), "reason": str(exc)})
                continue
            weather = by_weather.setdefault(preset, {"rows": [], "summaries": []})
            weather["rows"].extend({**row, "session": str(session_dir),
                                    "weather_preset": preset} for row in raw_rows)
            weather["summaries"].append(session_summary)
    if not by_weather:
        raise ValueError("분석할 트래킹·레이더 융합 세션이 없습니다. "
                         "기록이 완료된 세션과 제외 사유를 확인하세요: "
                         + "; ".join(item["reason"] for item in skipped[:3]))
    destination = workflow_dir / "experiment_results"
    suffix = _suffix(min_distance_m, max_distance_m, include_max)
    weather_results: dict[str, dict[str, Any]] = {}
    all_rows: list[dict[str, Any]] = []
    all_summaries: list[dict[str, Any]] = []
    for preset, data in by_weather.items():
        rows = data["rows"]
        summaries = data["summaries"]
        pooled = _pooled_summary(rows, summaries, min_distance_m, max_distance_m, include_max)
        pooled.update({"weather_preset": preset, "workflow_id": workflow_id})
        selected = _selected_rows(rows, min_distance_m, max_distance_m, include_max)
        weather_results[preset] = _write_result(
            destination / f"weather_{preset}", selected, pooled, suffix,
            f"{preset} 거리 추정 결과")
        all_rows.extend(rows)
        all_summaries.extend(summaries)
    combined = _pooled_summary(all_rows, all_summaries,
                               min_distance_m, max_distance_m, include_max)
    combined.update({"kind": "weather_workflow", "workflow_id": workflow_id,
                     "workflow_dir": str(workflow_dir), "weather_count": len(weather_results),
                     "weather_results": weather_results, "skipped_sessions": skipped})
    selected = _selected_rows(all_rows, min_distance_m, max_distance_m, include_max)
    result = _write_result(
        destination, selected, combined, suffix, "날씨별·통합 거리 추정 결과",
        list(weather_results.items()), skipped)
    write_detailed_weather_report(result)
    return result
