"""Generate a report-ready PNG from a POCO pose evaluation summary.json."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch


KOREAN_FONT_CANDIDATES = (
    Path("/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf"),
    Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
)

LABEL_KO = {
    "Optimal": "바른 자세",
    "Asymmetric": "비대칭",
    "Forward Head": "거북목",
    "Chin Propping": "턱 괴기",
}


def _font_properties():
    for path in KOREAN_FONT_CANDIDATES:
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            return font_manager.FontProperties(fname=str(path))
    return font_manager.FontProperties()


def _pct(value):
    return f"{float(value) * 100:.2f}%"


def _card(fig, x, title, value, subtitle, accent, font):
    y, width, height = 0.718, 0.215, 0.115
    shadow = FancyBboxPatch(
        (x + 0.003, y - 0.004),
        width,
        height,
        boxstyle="round,pad=0.008,rounding_size=0.012",
        transform=fig.transFigure,
        facecolor="#CBD5E1",
        edgecolor="none",
        alpha=0.45,
        zorder=1,
    )
    card = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.008,rounding_size=0.012",
        transform=fig.transFigure,
        facecolor="white",
        edgecolor="#E2E8F0",
        linewidth=1.0,
        zorder=2,
    )
    accent_bar = FancyBboxPatch(
        (x, y),
        0.006,
        height,
        boxstyle="round,pad=0,rounding_size=0.006",
        transform=fig.transFigure,
        facecolor=accent,
        edgecolor="none",
        zorder=3,
    )
    fig.patches.extend([shadow, card, accent_bar])
    fig.text(
        x + 0.018,
        y + 0.084,
        title,
        color="#475569",
        fontsize=10,
        fontproperties=font,
        zorder=4,
    )
    fig.text(
        x + 0.018,
        y + 0.041,
        value,
        color="#0F172A",
        fontsize=22,
        fontweight="bold",
        fontproperties=font,
        zorder=4,
    )
    fig.text(
        x + 0.018,
        y + 0.014,
        subtitle,
        color=accent,
        fontsize=8.5,
        fontproperties=font,
        zorder=4,
    )


def render(summary_path: Path, output_path: Path):
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    labels = list(summary["labels"])
    matrix = np.asarray(summary["confusion_matrix"]["values"], dtype=float)
    row_totals = matrix.sum(axis=1, keepdims=True)
    normalized = np.divide(
        matrix,
        row_totals,
        out=np.zeros_like(matrix),
        where=row_totals > 0,
    )
    font = _font_properties()
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(12.8, 7.2), dpi=150, facecolor="#F4F7FB")

    header = FancyBboxPatch(
        (0, 0.86),
        1,
        0.14,
        boxstyle="square,pad=0",
        transform=fig.transFigure,
        facecolor="#0B1F3A",
        edgecolor="none",
        zorder=0,
    )
    fig.patches.append(header)
    fig.text(
        0.04,
        0.935,
        "POCO 포즈 모델 성능평가 결과",
        color="white",
        fontsize=23,
        fontweight="bold",
        fontproperties=font,
    )
    generated = str(summary.get("generated_at", "")).replace("T", " ")
    fig.text(
        0.04,
        0.89,
        f"POSE-ONLY · 4-class GRU · 평가 완료 {generated}",
        color="#BFDBFE",
        fontsize=10,
        fontproperties=font,
    )
    fig.text(
        0.96,
        0.913,
        "EMBEDDED AI POSTURE MONITORING",
        color="#93C5FD",
        fontsize=9,
        ha="right",
        fontweight="bold",
        fontproperties=font,
    )

    latency = summary["latency_ms"]
    landmark = summary["landmark"]
    _card(
        fig,
        0.04,
        "Macro F1",
        _pct(summary["macro_f1"]),
        "4개 자세의 F1을 동일 가중 평균",
        "#2563EB",
        font,
    )
    _card(
        fig,
        0.275,
        "Accuracy",
        _pct(summary["accuracy"]),
        f"예측 윈도우 {summary['prediction_count']:,}개",
        "#0F766E",
        font,
    )
    _card(
        fig,
        0.51,
        "Landmark 유효 검출률",
        _pct(landmark["control_valid_rate"]),
        f"유효 {landmark['control_valid_frames']:,} / {landmark['total_recorded_frames']:,} frames",
        "#7C3AED",
        font,
    )
    _card(
        fig,
        0.745,
        "Raspberry Pi P95 latency",
        f"{latency['p95']:.2f} ms",
        f"평균 {latency['mean']:.2f} ms · 최대 {latency['max']:.2f} ms",
        "#EA580C",
        font,
    )

    recall_ax = fig.add_axes([0.055, 0.37, 0.405, 0.285], facecolor="white")
    recalls = [float(summary["per_label"][label]["recall"]) * 100 for label in labels]
    korean_labels = [LABEL_KO.get(label, label) for label in labels]
    colors = ["#2563EB", "#0F766E", "#DC2626", "#7C3AED"]
    y_pos = np.arange(len(labels))
    bars = recall_ax.barh(y_pos, recalls, color=colors, height=0.58)
    recall_ax.set_yticks(y_pos, korean_labels, fontproperties=font, fontsize=10)
    recall_ax.invert_yaxis()
    recall_ax.set_xlim(0, 105)
    recall_ax.set_xticks([0, 25, 50, 75, 100])
    recall_ax.set_xticklabels(
        ["0", "25", "50", "75", "100%"], fontproperties=font, fontsize=8
    )
    recall_ax.grid(axis="x", color="#E2E8F0", linewidth=0.8)
    recall_ax.set_axisbelow(True)
    recall_ax.set_title(
        "자세별 Recall",
        loc="left",
        pad=11,
        fontsize=13,
        fontweight="bold",
        fontproperties=font,
        color="#0F172A",
    )
    for bar, value in zip(bars, recalls):
        recall_ax.text(
            min(value + 1.3, 101.5),
            bar.get_y() + bar.get_height() / 2,
            f"{value:.2f}%",
            va="center",
            ha="left" if value < 98 else "right",
            color="#0F172A" if value < 98 else "white",
            fontsize=9,
            fontweight="bold",
            fontproperties=font,
        )
    for spine in recall_ax.spines.values():
        spine.set_color("#E2E8F0")

    cm_ax = fig.add_axes([0.545, 0.37, 0.405, 0.285], facecolor="white")
    image = cm_ax.imshow(normalized, cmap="Blues", vmin=0, vmax=1)
    cm_ax.set_xticks(range(len(labels)), korean_labels, fontproperties=font, fontsize=9)
    cm_ax.set_yticks(range(len(labels)), korean_labels, fontproperties=font, fontsize=9)
    cm_ax.set_xlabel("예측 자세", fontproperties=font, fontsize=9, color="#475569")
    cm_ax.set_ylabel("실제 자세", fontproperties=font, fontsize=9, color="#475569")
    cm_ax.set_title(
        "Confusion Matrix  (개수 / 실제 자세 내 비율)",
        loc="left",
        pad=11,
        fontsize=13,
        fontweight="bold",
        fontproperties=font,
        color="#0F172A",
    )
    for row in range(len(labels)):
        for col in range(len(labels)):
            ratio = normalized[row, col]
            cm_ax.text(
                col,
                row,
                f"{int(matrix[row, col])}\n{ratio * 100:.1f}%",
                ha="center",
                va="center",
                fontsize=8.5,
                color="white" if ratio >= 0.55 else "#0F172A",
                fontweight="bold" if row == col else "normal",
                fontproperties=font,
            )
    cm_ax.tick_params(length=0)
    for spine in cm_ax.spines.values():
        spine.set_color("#E2E8F0")
    colorbar = fig.colorbar(image, ax=cm_ax, fraction=0.042, pad=0.035)
    colorbar.ax.tick_params(labelsize=7)
    colorbar.outline.set_edgecolor("#CBD5E1")

    insight_box = FancyBboxPatch(
        (0.04, 0.065),
        0.92,
        0.235,
        boxstyle="round,pad=0.012,rounding_size=0.012",
        transform=fig.transFigure,
        facecolor="white",
        edgecolor="#E2E8F0",
        linewidth=1.0,
    )
    fig.patches.append(insight_box)
    fig.text(
        0.06,
        0.262,
        "결과 해석",
        fontsize=13,
        fontweight="bold",
        color="#0F172A",
        fontproperties=font,
    )
    forward_index = labels.index("Forward Head")
    chin_index = labels.index("Chin Propping")
    forward_total = int(matrix[forward_index].sum())
    forward_to_chin = int(matrix[forward_index, chin_index])
    confusion_rate = 100.0 * forward_to_chin / max(1, forward_total)
    clip_counts = summary.get("clip_count_by_label", {})
    clip_text = " / ".join(
        f"{LABEL_KO.get(label, label)} {int(clip_counts.get(label, 0))}개"
        for label in labels
    )
    insights = [
        ("강점", "바른 자세 Recall 100.00%, 비대칭 Recall 99.54%, Landmark 유효 검출률 100.00%"),
        (
            "개선점",
            f"거북목 Recall 73.09% — 거북목 {forward_total}개 중 {forward_to_chin}개({confusion_rate:.1f}%)를 턱 괴기로 오분류",
        ),
        ("시험 규모", f"{summary['clip_count']} clips · {landmark['total_recorded_frames']:,} frames · {summary['prediction_count']:,} prediction windows"),
        ("클립 구성", clip_text),
    ]
    y_positions = [0.222, 0.181, 0.140, 0.099]
    tag_colors = ["#0F766E", "#DC2626", "#2563EB", "#7C3AED"]
    for (tag, text), y, color in zip(insights, y_positions, tag_colors):
        fig.text(
            0.06,
            y,
            tag,
            color=color,
            fontsize=9.5,
            fontweight="bold",
            fontproperties=font,
        )
        fig.text(
            0.116,
            y,
            text,
            color="#334155",
            fontsize=9.2,
            fontproperties=font,
        )
    fig.text(
        0.06,
        0.074,
        "※ 단일 평가 세션의 중첩 시계열 윈도우 기준 결과이며, 다수 사용자 일반화 성능과는 구분해야 합니다.",
        color="#64748B",
        fontsize=8,
        fontproperties=font,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("summary", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or args.summary.with_name("pose_evaluation_result.png")
    render(args.summary, output)
    print(output.resolve())


if __name__ == "__main__":
    main()
