import os
import sys
import time
import platform
import threading
import subprocess
from pathlib import Path
from html import escape
from textwrap import dedent

import pandas as pd
import plotly.graph_objects as go
import streamlit as st




# ------------------------------------------------------------
# 경로 설정
# ------------------------------------------------------------

BASE_DIR = Path(__file__).resolve().parent.parent
PREPROCESS_DIR = BASE_DIR / "streamlit" / "preprocess"
SESSION_LOG_DIR = BASE_DIR / "data" / "session_log"
REPORT_DIR = BASE_DIR / "data" / "reports"

sys.path.append(str(BASE_DIR))
sys.path.append(str(PREPROCESS_DIR))



with st.expander("경로 확인"):
    st.write("BASE_DIR:", BASE_DIR)
    st.write("SESSION_LOG_DIR:", SESSION_LOG_DIR)
    st.write("CSV 파일:", [p.name for p in SESSION_LOG_DIR.glob("*.csv")])

# ------------------------------------------------------------
# 전처리 모듈 import
# ------------------------------------------------------------

try:
    from workspace.streamlit.preprocess.report_writer import load_report_summary, generate_report_from_csv
    from workspace.streamlit.preprocess.data_loader import load_posture_log
    from workspace.streamlit.preprocess.summary_builder import build_report_summary
    from workspace.streamlit.preprocess.domain_schema import (
        POSTURE_LABELS,
        BAD_POSTURE_LABELS,
        FATIGUE_LABELS,
        POSTURE_DISPLAY_NAME,
        FATIGUE_DISPLAY_NAME,
        POSTURE_COLOR,
        FATIGUE_COLOR,
        POSTURE_TIP,
        DEFAULT_POSTURE_TIP,
        POSTURE_OPTIMAL,
        POSTURE_FORWARD_HEAD,
        POSTURE_CHIN_PROPPING,
        POSTURE_ASYMMETRIC,
    )
except ModuleNotFoundError:
    from preprocess.report_writer import load_report_summary, generate_report_from_csv
    from preprocess.data_loader import load_posture_log
    from preprocess.summary_builder import build_report_summary
    from preprocess.domain_schema import (
        POSTURE_LABELS,
        BAD_POSTURE_LABELS,
        FATIGUE_LABELS,
        POSTURE_DISPLAY_NAME,
        FATIGUE_DISPLAY_NAME,
        POSTURE_COLOR,
        FATIGUE_COLOR,
        POSTURE_TIP,
        DEFAULT_POSTURE_TIP,
        POSTURE_OPTIMAL,
        POSTURE_FORWARD_HEAD,
        POSTURE_CHIN_PROPPING,
        POSTURE_ASYMMETRIC,
    )


# ------------------------------------------------------------
# 페이지 설정
# ------------------------------------------------------------

st.set_page_config(
    page_title="VisionPoseCoach",
    page_icon="🧘",
    layout="wide"
)


# ------------------------------------------------------------
# 표시명 / 색상 설정
# ------------------------------------------------------------
# 라벨/색상/문구 기준은 log_schema.json -> domain_schema.py를 통해 한 곳에서 관리한다.

POSTURE_LABEL_MAP = POSTURE_DISPLAY_NAME
FATIGUE_LABEL_MAP = FATIGUE_DISPLAY_NAME
POSTURE_COLOR_MAP = POSTURE_COLOR
FATIGUE_COLOR_MAP = FATIGUE_COLOR
POSTURE_TIP_MAP = POSTURE_TIP


# ------------------------------------------------------------
# CSS
# ------------------------------------------------------------

st.markdown(
    """
    <style>
    .stApp {
        background-color: #F7F7F6;
    }

    /* Raspberry Pi 800x480 대응 */
    @media (max-width: 900px) {
        .block-container {
            padding-top: 0.35rem;
            padding-bottom: 0.5rem;
            padding-left: 0.45rem;
            padding-right: 0.45rem;
            max-width: 100%;
        }

        .main-header-wrap {
            padding: 12px 14px;
            margin-bottom: 10px;
            border-radius: 12px;
        }

        .main-title {
            font-size: 24px;
        }

        .main-sub {
            font-size: 11px;
            line-height: 1.45;
        }

        div[data-testid="stSelectbox"] label {
            font-size: 12px;
            margin-bottom: 2px;
        }

        div[data-testid="stSelectbox"] [data-baseweb="select"] > div {
            min-height: 38px;
            height: 38px;
            font-size: 13px;
        }

        div[data-testid="stButton"] > button {
            width: 100%;
            min-height: 38px;
            padding: 0.25rem 0.35rem;
            font-size: 13px;
            white-space: nowrap;
        }

        .refresh-button-spacer {
            height: 26px;
        }

        .exit-button-spacer {
            height: 26px;
        }
    }

    .block-container {
        padding-top: 0.4rem;
        padding-bottom: 0.6rem;
        padding-left: 0.6rem;
        padding-right: 0.6rem;
        max-width: 100%;
    }

    .main-header-wrap {
        background: #ffffff;
        border: 1px solid #ececec;
        border-radius: 18px;
        padding: 22px 28px;
        margin-bottom: 18px;
        box-shadow: 0 3px 12px rgba(0,0,0,0.03);
    }

    .main-title {
        font-size: 34px;
        font-weight: 800;
        color: #222222;
        margin-bottom: 2px;
    }

    .main-sub {
        font-size: 14px;
        color: #8d8d8d;
    }

    .section-title {
        font-size: 20px;
        font-weight: 800;
        color: #222222;
        margin-top: 8px;
        margin-bottom: 10px;
    }

    .metric-card {
        background: #ffffff;
        border: 1px solid #ececec;
        border-radius: 18px;
        padding: 22px 24px;
        min-height: 154px;
        box-shadow: 0 3px 12px rgba(0,0,0,0.03);
    }

    .metric-top {
        display: flex;
        align-items: center;
        justify-content: space-between;
        margin-bottom: 14px;
    }

    .metric-title {
        font-size: 15px;
        font-weight: 700;
        color: #5a5a5a;
    }

    .metric-icon-box {
        width: 40px;
        height: 40px;
        border-radius: 12px;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 18px;
        font-weight: 700;
    }

    .metric-value {
        font-size: 40px;
        font-weight: 800;
        line-height: 1.1;
        margin-bottom: 8px;
    }

    .metric-sub {
        font-size: 13px;
        color: #9a9a9a;
        line-height: 1.4;
    }

    /* 3개짜리 카드 행 - 살짝 축소 */
    .metric-card-small {
        padding: 18px 20px;
        min-height: 138px;
    }

    .metric-card-small .metric-top {
        margin-bottom: 10px;
    }

    .metric-card-small .metric-title {
        font-size: 14px;
    }

    .metric-card-small .metric-value {
        font-size: 34px;
        margin-bottom: 6px;
    }

    .metric-card-small .metric-sub {
        font-size: 12px;
    }

    .metric-card-small .metric-icon-box {
        width: 36px;
        height: 36px;
        border-radius: 11px;
        font-size: 16px;
    }

    /* 4개짜리 카드 행 - 더 많이 축소 */
    .metric-card-tiny {
        padding: 14px 12px;
        min-height: 118px;
        border-radius: 14px;
    }

    .metric-card-tiny .metric-top {
        margin-bottom: 8px;
    }

    .metric-card-tiny .metric-title {
        font-size: 12px;
        line-height: 1.25;
        letter-spacing: -0.4px;
    }

    .metric-card-tiny .metric-value {
        font-size: 24px;
        line-height: 1.15;
        margin-bottom: 4px;
        letter-spacing: -0.5px;
    }

    .metric-card-tiny .metric-sub {
        font-size: 11px;
        line-height: 1.3;
    }

    .metric-card-tiny .metric-icon-box {
        width: 30px;
        height: 30px;
        border-radius: 9px;
        font-size: 13px;
    }

    .table-card {
        background: #ffffff;
        border: 1px solid #ececec;
        border-radius: 18px;
        padding: 18px 20px;
        box-shadow: 0 3px 12px rgba(0,0,0,0.03);
        margin-top: 8px;
    }

    .table-title {
        font-size: 18px;
        font-weight: 800;
        color: #222222;
        margin-bottom: 14px;
    }

    .pretty-table {
        width: 100%;
        border-collapse: collapse;
    }

    .pretty-table thead tr {
        background: #f7f7fb;
    }

    .pretty-table th {
        text-align: left;
        padding: 12px 14px;
        font-size: 14px;
        font-weight: 700;
        color: #555555;
        border-bottom: 1px solid #ededed;
    }

    .pretty-table td {
        padding: 12px 14px;
        font-size: 14px;
        color: #333333;
        border-bottom: 1px solid #f2f2f2;
    }

    .pretty-table tbody tr:last-child td {
        border-bottom: none;
    }

    .issue-card {
        background: #ffffff;
        border: 1px solid #ececec;
        border-radius: 18px;
        padding: 18px 20px;
        min-height: 220px;
        box-shadow: 0 3px 12px rgba(0,0,0,0.03);
    }

    .issue-head {
        display: flex;
        align-items: center;
        justify-content: space-between;
        margin-bottom: 10px;
    }

    .issue-rank {
        width: 42px;
        height: 42px;
        border-radius: 999px;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 16px;
        font-weight: 800;
    }

    .issue-label {
        font-size: 15px;
        color: #7a7a7a;
        font-weight: 700;
        margin-bottom: 6px;
    }

    .issue-name {
        font-size: 24px;
        font-weight: 800;
        color: #222222;
        margin-bottom: 14px;
    }

    .issue-value-row {
        display: flex;
        align-items: baseline;
        gap: 10px;
        margin-bottom: 18px;
    }

    .issue-ratio {
        font-size: 24px;
        font-weight: 800;
    }

    .issue-time {
        font-size: 16px;
        color: #666666;
        font-weight: 600;
    }

    .issue-divider {
        width: 100%;
        height: 1px;
        background: #f0f0f0;
        margin: 14px 0 16px 0;
    }

    .issue-tip {
        font-size: 14px;
        line-height: 1.65;
        color: #5c5c5c;
    }

    .info-box {
        background: #ffffff;
        border: 1px solid #ececec;
        border-radius: 18px;
        padding: 18px 20px;
        box-shadow: 0 3px 12px rgba(0,0,0,0.03);
        font-size: 14px;
        color: #555555;
        line-height: 1.75;
    }

    div[data-testid="stTabs"] button {
        font-weight: 700;
    }

    .session-help-text {
        font-size: 13px;
        color: #8d8d8d;
        margin-top: -4px;
        margin-bottom: 8px;
    }


    /* Raspberry Pi 800x480 최종 보정: 이 블록은 CSS 맨 아래에 있어야 한다. */
    @media (max-width: 900px) {
        .block-container {
            padding-top: 0.35rem;
            padding-bottom: 0.5rem;
            padding-left: 0.45rem;
            padding-right: 0.45rem;
            max-width: 100%;
        }

        .main-header-wrap {
            padding: 12px 14px;
            margin-bottom: 10px;
            border-radius: 12px;
        }

        .main-title {
            font-size: 24px;
        }

        .main-sub {
            font-size: 11px;
            line-height: 1.45;
        }

        .section-title {
            font-size: 16px;
            margin-top: 6px;
            margin-bottom: 8px;
        }

        div[data-testid="stSelectbox"] label {
            font-size: 12px;
            margin-bottom: 2px;
        }

        div[data-testid="stSelectbox"] [data-baseweb="select"] > div {
            min-height: 38px;
            height: 38px;
            font-size: 13px;
        }

        div[data-testid="stButton"] > button {
            width: 100%;
            min-height: 38px;
            padding: 0.25rem 0.35rem;
            font-size: 13px;
            white-space: nowrap;
        }

        .refresh-button-spacer {
            height: 26px;
        }

        .exit-button-spacer {
            height: 26px;
        }

        .metric-card-small {
            padding: 14px 14px;
            min-height: 118px;
            border-radius: 14px;
        }

        .metric-card-small .metric-top {
            margin-bottom: 8px;
        }

        .metric-card-small .metric-title {
            font-size: 12px;
            line-height: 1.25;
            letter-spacing: -0.3px;
        }

        .metric-card-small .metric-value {
            font-size: 26px;
            line-height: 1.15;
            margin-bottom: 4px;
            letter-spacing: -0.4px;
        }

        .metric-card-small .metric-sub {
            font-size: 11px;
        }

        .metric-card-small .metric-icon-box {
            width: 30px;
            height: 30px;
            border-radius: 9px;
            font-size: 13px;
        }

        .metric-card-tiny {
            padding: 10px 8px;
            min-height: 96px;
            border-radius: 12px;
        }

        .metric-card-tiny .metric-top {
            margin-bottom: 6px;
        }

        .metric-card-tiny .metric-title {
            font-size: 10px;
            line-height: 1.2;
            letter-spacing: -0.5px;
        }

        .metric-card-tiny .metric-value {
            font-size: 18px;
            line-height: 1.15;
            margin-bottom: 2px;
            letter-spacing: -0.6px;
        }

        .metric-card-tiny .metric-sub {
            font-size: 10px;
            line-height: 1.2;
        }

        .metric-card-tiny .metric-icon-box {
            width: 24px;
            height: 24px;
            border-radius: 8px;
            font-size: 11px;
        }
    }

    </style>
    """,
    unsafe_allow_html=True
)


# ------------------------------------------------------------
# 유틸
# ------------------------------------------------------------

def seconds_to_text(seconds: int) -> str:
    seconds = int(seconds)
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    remain_seconds = seconds % 60

    if hours > 0:
        return f"{hours}시간 {minutes}분 {remain_seconds}초"
    if minutes > 0:
        return f"{minutes}분 {remain_seconds}초"
    return f"{remain_seconds}초"


def format_percent_from_ratio(value: float) -> str:
    try:
        return f"{float(value) * 100:.1f}%"
    except Exception:
        return "0.0%"


def format_number(value, digit: int = 2) -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.{digit}f}"
    except Exception:
        return str(value)


def get_posture_name(label: str) -> str:
    return POSTURE_LABEL_MAP.get(label, label)


def get_fatigue_name(label: str) -> str:
    return FATIGUE_LABEL_MAP.get(label, label)


def get_posture_color(label: str) -> str:
    return POSTURE_COLOR_MAP.get(label, "#9aa3af")


def get_fatigue_color(label: str) -> str:
    return FATIGUE_COLOR_MAP.get(label, "#9aa3af")


def render_metric_card(
    title: str,
    value: str,
    sub_text: str,
    color: str,
    icon: str,
    card_size: str = "default"
) -> None:
    """
    Metric 카드 렌더링 함수.

    card_size:
    - default: 기본 카드
    - small  : 3개짜리 행 카드용
    - tiny   : 4개짜리 행 카드용
    """
    icon_bg = f"{color}15"

    card_class = "metric-card"

    if card_size == "small":
        card_class += " metric-card-small"

    elif card_size == "tiny":
        card_class += " metric-card-tiny"

    st.markdown(
        f"""
        <div class="{card_class}">
            <div class="metric-top">
                <div class="metric-title">{escape(title)}</div>
                <div class="metric-icon-box" style="background:{icon_bg}; color:{color};">{escape(icon)}</div>
            </div>
            <div class="metric-value" style="color:{color};">{escape(value)}</div>
            <div class="metric-sub">{escape(sub_text)}</div>
        </div>
        """,
        unsafe_allow_html=True
    )


def render_pretty_table(title: str, columns: list[str], rows: list[list[str]]) -> None:
    header_html = "".join([f"<th>{escape(str(col))}</th>" for col in columns])

    body_html = ""
    for row in rows:
        row_html = "".join([f"<td>{escape(str(cell))}</td>" for cell in row])
        body_html += f"<tr>{row_html}</tr>"

    html = f"""
    <div class="table-card">
        <div class="table-title">{escape(title)}</div>
        <table class="pretty-table">
            <thead>
                <tr>{header_html}</tr>
            </thead>
            <tbody>
                {body_html}
            </tbody>
        </table>
    </div>
    """
    st.markdown(html, unsafe_allow_html=True)


def render_issue_card(rank: int, label: str, ratio_text: str, time_text: str, tip: str, color: str) -> None:
    badge_bg = f"{color}18"

    html = dedent(f"""
    <div class="issue-card" style="border-left:6px solid {color};">
        <div class="issue-head">
            <div></div>
            <div class="issue-rank" style="background:{badge_bg}; color:{color};">{rank}위</div>
        </div>
        <div class="issue-label">문제 유형</div>
        <div class="issue-name">{escape(label)}</div>
        <div class="issue-value-row">
            <div class="issue-ratio" style="color:{color};">{escape(ratio_text)}</div>
            <div class="issue-time">{escape(time_text)}</div>
        </div>
        <div class="issue-divider"></div>
        <div class="issue-tip">💡 {escape(tip)}</div>
    </div>
    """)

    st.markdown(html, unsafe_allow_html=True)


def make_posture_distribution_df(posture_type_seconds: dict) -> pd.DataFrame:
    display_order = POSTURE_LABELS
    rows = []

    for label in display_order:
        seconds = int(posture_type_seconds.get(label, 0))
        if seconds <= 0:
            continue

        rows.append({
            "label": label,
            "name": get_posture_name(label),
            "seconds": seconds,
            "color": get_posture_color(label)
        })

    return pd.DataFrame(rows)


def make_bad_posture_ratio_df(posture_type_seconds: dict, total_logged_sec: int) -> pd.DataFrame:
    display_order = BAD_POSTURE_LABELS
    rows = []

    for label in display_order:
        seconds = int(posture_type_seconds.get(label, 0))
        ratio = (seconds / total_logged_sec * 100) if total_logged_sec > 0 else 0

        rows.append({
            "label": label,
            "name": get_posture_name(label),
            "seconds": seconds,
            "ratio_percent": round(ratio, 1),
            "color": get_posture_color(label)
        })

    return pd.DataFrame(rows)


def make_fatigue_distribution_df(fatigue_label_seconds: dict) -> pd.DataFrame:
    display_order = FATIGUE_LABELS
    rows = []

    for label in display_order:
        seconds = int(fatigue_label_seconds.get(label, 0))
        if seconds <= 0:
            continue

        rows.append({
            "label": label,
            "name": get_fatigue_name(label),
            "seconds": seconds,
            "color": get_fatigue_color(label)
        })

    return pd.DataFrame(rows)

def clamp_score(score: float) -> float:
    if score is None:
        return 0.0

    return max(0.0, min(100.0, float(score)))


def make_score_bar_df(posture_score: float, fatigue_score: float, posture_grade: str = "-", fatigue_grade: str = "-") -> pd.DataFrame:
    return pd.DataFrame([
        {
            "name": "자세 점수",
            "score": clamp_score(posture_score),
            "grade": posture_grade,
            "color": "#37F731"
        },
        {
            "name": "피로도 점수",
            "score": clamp_score(fatigue_score),
            "grade": fatigue_grade,
            "color": "#FFB547"
        }
    ])


def render_score_bar_chart(df: pd.DataFrame, title: str) -> None:
    fig = go.Figure()

    fig.add_trace(
        go.Bar(
            x=df["name"],
            y=df["score"],
            marker_color=df["color"],
            width=0.18,
            text=[f"{v:.1f}점" for v in df["score"]],
            textposition="outside",
            textfont=dict(
                color="#000000",
                size=12
            ),
            cliponaxis=False,

            # 추가
            customdata=df[["grade"]],
            hovertemplate=(
                "<b>%{x}</b><br>"
                "점수: %{y:.1f}점<br>"
                "등급: %{customdata[0]}"
                "<extra></extra>"
            )
        )
    )

    fig.update_layout(
        title=dict(
            text=title,
            x=0.02,
            xanchor="left",
            font=dict(size=20, color="#000000")
        ),
        paper_bgcolor="#ffffff",
        plot_bgcolor="#ffffff",
        height=430,
        margin=dict(l=40, r=25, t=65, b=40),
        showlegend=False,
        xaxis=dict(
            title="",
            showgrid=False,
            tickfont=dict(size=15, color="#000000")
        ),
        yaxis=dict(
            title="",
            range=[0, 105],
            ticksuffix="점",
            gridcolor="#ebebeb",
            zeroline=False,
            tickfont=dict(size=14, color="#000000")
        )
    )

    st.plotly_chart(fig, width="stretch")

def render_bad_posture_bar_chart(df: pd.DataFrame, title: str) -> None:
    fig = go.Figure()

    fig.add_trace(
        go.Bar(
            x=df["name"],
            y=df["ratio_percent"],
            marker_color=df["color"],
            width=0.35,
            text=[f"{v:.1f}%" for v in df["ratio_percent"]],
            textfont=dict(
                color="#000000",  # 막대 위 텍스트 색상
                size=15),
            textposition="outside"
        )
    )

    fig.update_layout(
        title=dict(text=title, x=0.02, xanchor="left", font=dict(size=20, color="#222")),
        paper_bgcolor="#ffffff",
        plot_bgcolor="#ffffff",
        height=430,
        margin=dict(l=40, r=25, t=65, b=40),
        showlegend=False,
        xaxis=dict(
            title="",
            showgrid=False,
            tickfont=dict(size=15, color="#666666")
        ),
        yaxis=dict(
            title="",
            range=[0, max(20, float(df["ratio_percent"].max()) + 3)],
            ticksuffix="%",
            gridcolor="#ebebeb",
            zeroline=False,
            tickfont=dict(size=14, color="#666666")
        )
    )

    st.plotly_chart(fig, width="stretch")


def render_donut_chart(df: pd.DataFrame, title: str, center_text: str = "") -> None:
    required_columns = {"name", "seconds", "color"}

    if df.empty or not required_columns.issubset(df.columns):
        st.info(f"{title}에 표시할 데이터가 없습니다.")
        return

    fig = go.Figure(
        data=[
            go.Pie(
                labels=df["name"],
                values=df["seconds"],
                hole=0.62,
                sort=False,
                marker=dict(
                    colors=df["color"],
                    line=dict(color="#ffffff", width=4)
                ),
                texttemplate="%{label} %{percent}",
                textposition="outside",
                textfont=dict(size=15),
                hovertemplate="%{label}<br>%{value}초<extra></extra>"
            )
        ]
    )

    fig.update_layout(
        title=dict(text=title, x=0.02, xanchor="left", font=dict(size=20, color="#222")),
        paper_bgcolor="#ffffff",
        plot_bgcolor="#ffffff",
        height=430,
        margin=dict(l=20, r=20, t=65, b=10),
        showlegend=False,
        annotations=[
            dict(
                text=center_text,
                x=0.5,
                y=0.5,
                font=dict(size=18, color="#666"),
                showarrow=False
            )
        ]
    )

    st.plotly_chart(fig, width="stretch")


def get_session_time_range(
    raw_df: pd.DataFrame,
    generated_at: str | None = None,
    total_logged_sec: int | float | None = None
) -> tuple[pd.Timestamp, pd.Timestamp]:
    """
    그래프 x축을 측정 시작 시간부터 측정 마지막 시간까지 잡기 위한 시간 범위를 구한다.

    우선순위:
    1. 원본 로그 timestamp의 최소/최대값
    2. generated_at - total_logged_sec ~ generated_at
    3. 실행 날짜 00:00 ~ 00:01
    """
    if raw_df is not None and not raw_df.empty and "timestamp" in raw_df.columns:
        timestamps = pd.to_datetime(raw_df["timestamp"], errors="coerce").dropna()

        if not timestamps.empty:
            start_time = timestamps.min().floor("min")
            end_time = timestamps.max().ceil("min")

            if end_time <= start_time:
                end_time = start_time + pd.Timedelta(minutes=1)

            return start_time, end_time

    generated_time = pd.to_datetime(generated_at, errors="coerce")
    if not pd.isna(generated_time):
        end_time = generated_time.ceil("min")

        try:
            total_sec = int(float(total_logged_sec)) if total_logged_sec is not None else 0
        except Exception:
            total_sec = 0

        if total_sec > 0:
            start_time = end_time - pd.to_timedelta(total_sec, unit="s")
            start_time = start_time.floor("min")
        else:
            start_time = end_time.normalize()

        if end_time <= start_time:
            end_time = start_time + pd.Timedelta(minutes=1)

        return start_time, end_time

    start_time = pd.Timestamp.today().normalize()
    end_time = start_time + pd.Timedelta(minutes=1)
    return start_time, end_time


def attach_real_time_to_minute_df(
    minute_df: pd.DataFrame,
    raw_df: pd.DataFrame,
    generated_at: str | None = None,
    total_logged_sec: int | float | None = None
) -> pd.DataFrame:
    """
    minute_index 기반 요약 데이터에 실제 시간 컬럼을 붙인다.

    기존 minute_index는 세션 시작 후 몇 분이 지났는지에 가까운 값이므로,
    원본 로그의 timestamp 시작 시간에 minute_index를 더해서 실제 시간을 만든다.
    또한 x축 범위를 측정 시작~마지막 값으로 제한하기 위해 세션 시작/끝 시간을 같이 보관한다.
    """
    if minute_df.empty:
        return minute_df

    result = minute_df.copy()

    if "minute_index" not in result.columns:
        return result

    result["minute_index"] = pd.to_numeric(result["minute_index"], errors="coerce")
    result = result.dropna(subset=["minute_index"])

    session_start_time, session_end_time = get_session_time_range(
        raw_df=raw_df,
        generated_at=generated_at,
        total_logged_sec=total_logged_sec
    )

    if "segment_start_time" in result.columns and "segment_end_time" in result.columns:
        result["time_start"] = pd.to_datetime(result["segment_start_time"], errors="coerce")
        result["time_end"] = pd.to_datetime(result["segment_end_time"], errors="coerce")
    else:
        result["time_start"] = (
            session_start_time
            + pd.to_timedelta(result["minute_index"].astype(int), unit="m")
        )
        result["time_end"] = result["time_start"] + pd.Timedelta(minutes=1)

    # 마지막 구간이 실제 측정 종료 시간을 넘어가면 hover 표시용 끝 시간을 측정 종료 시간으로 잘라준다.
    result.loc[result["time_end"] > session_end_time, "time_end"] = session_end_time

    result["_session_start_time"] = session_start_time
    result["_session_end_time"] = session_end_time
    result["minute_label"] = result["time_start"].dt.strftime("%H:%M")
    result["date_label"] = result["time_start"].dt.strftime("%Y-%m-%d")

    return result


def make_time_bin_min_max_df(
    df: pd.DataFrame,
    value_cols: list[str],
    unit_min: int = 5,
    time_col: str = "time_start",
    minute_col: str = "minute_index"
) -> pd.DataFrame:
    """
    측정 시작 시간을 기준으로 원하는 시간 단위의 최저값/최고값을 만든다.

    예시:
    - 측정 시작: 14:12
    - unit_min=5
    - 14:12~14:17, 14:17~14:22처럼 측정 시작 기준으로 묶는다.
    - 그래프에는 각 구간의 최고값을 표시하고, hover에는 최저/최고를 같이 표시한다.
    """
    if df.empty:
        return pd.DataFrame()

    result = df.copy()
    unit_min = max(1, int(unit_min))
    unit_sec = unit_min * 60

    if time_col in result.columns:
        result[time_col] = pd.to_datetime(result[time_col], errors="coerce")
    elif minute_col in result.columns:
        # time_start가 없는 예외 상황용 fallback이다.
        result[minute_col] = pd.to_numeric(result[minute_col], errors="coerce")
        today_start = pd.Timestamp.today().normalize()
        result[time_col] = today_start + pd.to_timedelta(result[minute_col], unit="m")
    else:
        return pd.DataFrame()

    result = result.dropna(subset=[time_col])

    if result.empty:
        return pd.DataFrame()

    usable_cols = []
    for col in value_cols:
        if col in result.columns:
            result[col] = pd.to_numeric(result[col], errors="coerce")
            usable_cols.append(col)

    if not usable_cols:
        return pd.DataFrame()

    if "_session_start_time" in result.columns:
        session_start_time = pd.to_datetime(result["_session_start_time"].iloc[0], errors="coerce")
    else:
        session_start_time = result[time_col].min()

    if pd.isna(session_start_time):
        session_start_time = result[time_col].min()

    if "_session_end_time" in result.columns:
        session_end_time = pd.to_datetime(result["_session_end_time"].iloc[0], errors="coerce")
    elif "time_end" in result.columns:
        session_end_time = pd.to_datetime(result["time_end"], errors="coerce").max()
    else:
        session_end_time = result[time_col].max()

    if pd.isna(session_end_time) or session_end_time <= session_start_time:
        session_end_time = session_start_time + pd.Timedelta(minutes=1)

    # 측정 시작 시간을 0으로 보고 unit_min 단위로 묶는다.
    elapsed_sec = (result[time_col] - session_start_time).dt.total_seconds()
    result["time_bin_index"] = (elapsed_sec // unit_sec).astype(int)

    agg_dict = {}
    for col in usable_cols:
        agg_dict[f"{col}_min"] = (col, "min")
        agg_dict[f"{col}_max"] = (col, "max")

    group_keys = ["time_bin_index"]

    if "segment_index" in result.columns:
        group_keys = ["segment_index", "time_bin_index"]

    grouped = (
        result
        .groupby(group_keys, as_index=False)
        .agg(**agg_dict)
        .sort_values(group_keys)
    )

    grouped["time_bin_start_dt"] = (
        session_start_time
        + pd.to_timedelta(grouped["time_bin_index"] * unit_min, unit="m")
    )
    grouped["time_bin_end_dt"] = grouped["time_bin_start_dt"] + pd.to_timedelta(unit_min, unit="m")

    # 마지막 구간 끝은 실제 측정 종료 시간까지만 표시한다.
    grouped.loc[grouped["time_bin_end_dt"] > session_end_time, "time_bin_end_dt"] = session_end_time

    grouped["_session_start_time"] = session_start_time
    grouped["_session_end_time"] = session_end_time
    grouped["date_label"] = grouped["time_bin_start_dt"].dt.strftime("%Y-%m-%d")
    grouped["time_range_label"] = (
        grouped["time_bin_start_dt"].dt.strftime("%Y-%m-%d %H:%M")
        + " ~ "
        + grouped["time_bin_end_dt"].dt.strftime("%H:%M")
    )

    return grouped


def get_session_axis_range(chart_df: pd.DataFrame) -> tuple[pd.Timestamp, pd.Timestamp] | None:
    """
    x축 범위를 측정 시작 시간부터 측정 마지막 값까지로 만든다.
    """
    if chart_df.empty:
        return None

    if "_session_start_time" in chart_df.columns and "_session_end_time" in chart_df.columns:
        start_time = pd.to_datetime(chart_df["_session_start_time"].iloc[0], errors="coerce")
        end_time = pd.to_datetime(chart_df["_session_end_time"].iloc[0], errors="coerce")
    else:
        start_time = pd.to_datetime(chart_df.get("time_bin_start_dt"), errors="coerce").min()
        end_time = pd.to_datetime(chart_df.get("time_bin_end_dt"), errors="coerce").max()

    if pd.isna(start_time) or pd.isna(end_time):
        return None

    if end_time <= start_time:
        end_time = start_time + pd.Timedelta(minutes=1)

    return start_time, end_time


def get_axis_date_title(axis_range: tuple[pd.Timestamp, pd.Timestamp] | None) -> str:
    """
    x축 제목에 표시할 날짜 문자열을 만든다.
    """
    if axis_range is None:
        return ""

    start_time, end_time = axis_range

    if start_time.date() == end_time.date():
        return start_time.strftime("%Y-%m-%d")

    return f"{start_time.strftime('%Y-%m-%d')} ~ {end_time.strftime('%Y-%m-%d')}"

def get_day_axis_range(chart_df: pd.DataFrame) -> tuple[pd.Timestamp, pd.Timestamp] | None:
    """
    x축을 해당 날짜의 00:00~24:00으로 고정하기 위한 범위를 만든다.
    """
    if chart_df.empty or "time_bin_start_dt" not in chart_df.columns:
        return None

    first_time = pd.to_datetime(chart_df["time_bin_start_dt"], errors="coerce").dropna()
    if first_time.empty:
        return None

    day_start = first_time.min().normalize()
    day_end = day_start + pd.Timedelta(days=1)

    return day_start, day_end


def render_line_chart(
    df: pd.DataFrame,
    x_col: str,
    y_specs: list[dict],
    title: str,
    y_suffix: str = "",
    height: int = 360,
    unit_min: int = 5,
    show_max_by_unit: bool = True
) -> None:
    """
    라인 차트 렌더링 함수

    기본 동작:
    - 실제 시간 기준으로 unit_min 단위 묶음
    - 각 구간의 최고값을 라인으로 표시
    - hover에 실제 시간 구간, 최저값, 최고값 표시
    - x축 범위는 측정 시작 시간부터 측정 마지막 값까지로 제한
    """
    fig = go.Figure()

    if df.empty:
        st.info("그래프에 표시할 데이터가 없습니다.")
        return

    value_cols = [spec["col"] for spec in y_specs if spec.get("col") in df.columns]

    if not value_cols:
        st.info("그래프에 사용할 컬럼이 없습니다.")
        return

    if show_max_by_unit:
        chart_df = make_time_bin_min_max_df(
            df=df,
            value_cols=value_cols,
            unit_min=unit_min,
            time_col="time_start",
            minute_col="minute_index"
        )

        if chart_df.empty:
            st.info("그래프에 표시할 집계 데이터가 없습니다.")
            return

        for spec in y_specs:
            col = spec["col"]

            if f"{col}_max" not in chart_df.columns:
                continue

            if "segment_index" in chart_df.columns:
                segment_groups = chart_df.groupby("segment_index")
            else:
                segment_groups = [(None, chart_df)]

            for segment_index, segment_df in segment_groups:
                trace_name = spec["name"]

                fig.add_trace(
                    go.Scatter(
                        x=segment_df["time_bin_start_dt"],
                        y=segment_df[f"{col}_max"],
                        mode="lines+markers",
                        name=trace_name,
                        line=dict(color=spec["color"], width=3),
                        marker=dict(size=7),
                        customdata=segment_df[
                            [
                                "time_range_label",
                                f"{col}_min",
                                f"{col}_max"
                            ]
                        ],
                        hovertemplate=(
                            "<b>%{fullData.name}</b><br>"
                            "시간: %{customdata[0]}<br>"
                            "최고값: %{customdata[2]:.2f}" + y_suffix + "<br>"
                            "최저값: %{customdata[1]:.2f}" + y_suffix +
                            "<extra></extra>"
                        ),
                        showlegend=(segment_index is None or segment_index == 0)
                    )
                )

        axis_range = get_session_axis_range(chart_df)
        xaxis_title = get_axis_date_title(axis_range)

        xaxis_config = dict(
            title=xaxis_title,
            type="date",
            showgrid=True,
            gridcolor="#efefef",
            tickfont=dict(size=12, color="#666"),
            tickformat="%H:%M",
            dtick=unit_min * 60 * 1000,
            tick0=axis_range[0] if axis_range else None,
            range=list(axis_range) if axis_range else None,
            zeroline=False
        )

    else:
        for spec in y_specs:
            fig.add_trace(
                go.Scatter(
                    x=df[x_col],
                    y=df[spec["col"]],
                    mode="lines",
                    name=spec["name"],
                    line=dict(color=spec["color"], width=3)
                )
            )

        xaxis_config = dict(
            title="",
            showgrid=True,
            gridcolor="#efefef",
            tickfont=dict(size=12, color="#666")
        )

    fig.update_layout(
        title=dict(text=title, x=0.02, xanchor="left", font=dict(size=18, color="#222")),
        paper_bgcolor="#ffffff",
        plot_bgcolor="#ffffff",
        height=height,
        margin=dict(l=40, r=20, t=55, b=45),
        legend=dict(orientation="h", yanchor="bottom", y=1.01, xanchor="left", x=0.0),
        hovermode="x unified",
        xaxis=xaxis_config,
        yaxis=dict(
            title="",
            showgrid=True,
            gridcolor="#efefef",
            ticksuffix=y_suffix,
            tickfont=dict(size=12, color="#666"),
            zeroline=False
        )
    )

    st.plotly_chart(fig, width="stretch")



# ------------------------------------------------------------
# 날짜별 세션 데이터 처리
# ------------------------------------------------------------

REPORT_DIR = BASE_DIR / "data" / "reports"

# app.py 위치가 바뀌어도 session_logs를 찾을 수 있도록 후보 경로를 같이 본다.
SESSION_LOG_DIR_CANDIDATES = [
    SESSION_LOG_DIR,
    BASE_DIR / "data" / "session_log",
    Path.cwd() / "data" / "session_log",
]


def parse_date_from_text(value) -> str | None:
    """
    문자열 또는 timestamp 값을 YYYY-MM-DD 형태로 변환한다.
    파일명이 2026_05_13처럼 들어와도 2026-05-13으로 처리한다.
    """
    if value is None:
        return None

    normalized = str(value).strip().replace("_", "-")
    timestamp = pd.to_datetime(normalized, errors="coerce")

    if pd.isna(timestamp):
        return None

    return timestamp.strftime("%Y-%m-%d")


def get_session_log_file_map() -> dict[str, Path]:
    """
    data/session_logs/posture_log_YYYY-MM-DD.csv 형태의 파일을 날짜별로 찾는다.
    app.py 실행 위치가 달라도 찾을 수 있도록 여러 후보 경로를 확인한다.
    """
    file_map: dict[str, Path] = {}

    for log_dir in SESSION_LOG_DIR_CANDIDATES:
        if not log_dir.exists():
            continue

        for file_path in sorted(log_dir.glob("posture_log_*.csv")):
            date_text = file_path.stem.replace("posture_log_", "")
            parsed_date = parse_date_from_text(date_text)

            if parsed_date:
                file_map[parsed_date] = file_path

    return file_map


def get_report_summary_path(session_date: str) -> Path:
    """
    날짜별 리포트 JSON 저장 경로를 반환한다.
    예: data/reports/report_summary_2026-05-13.json
    """
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    return REPORT_DIR / f"report_summary_{session_date}.json"


def get_report_file_map() -> dict[str, Path]:
    """
    data/reports/report_summary_YYYY-MM-DD.json 형태의 리포트 파일을 날짜별로 찾는다.
    """
    file_map: dict[str, Path] = {}

    if not REPORT_DIR.exists():
        return file_map

    for file_path in sorted(REPORT_DIR.glob("report_summary_*.json")):
        date_text = file_path.stem.replace("report_summary_", "")
        parsed_date = parse_date_from_text(date_text)

        if parsed_date:
            file_map[parsed_date] = file_path

    return file_map


def get_available_session_dates(default_summary: dict, default_raw_df: pd.DataFrame) -> list[str]:
    """
    셀렉트 박스에 보여줄 세션 날짜 목록을 만든다.

    우선순위:
    1. data/session_logs/posture_log_YYYY-MM-DD.csv 파일 날짜
    2. data/reports/report_summary_YYYY-MM-DD.json 파일 날짜
    3. 기존 raw_df timestamp에 들어있는 날짜
    4. 기존 summary session.start_time 또는 generated_at 날짜
    """
    dates: set[str] = set()

    dates.update(get_session_log_file_map().keys())
    dates.update(get_report_file_map().keys())

    if default_raw_df is not None and not default_raw_df.empty and "timestamp" in default_raw_df.columns:
        timestamps = pd.to_datetime(default_raw_df["timestamp"], errors="coerce").dropna()
        for date_value in timestamps.dt.strftime("%Y-%m-%d").unique():
            dates.add(str(date_value))

    session_info = default_summary.get("session", {}) if isinstance(default_summary, dict) else {}
    summary_start_date = parse_date_from_text(session_info.get("start_time"))
    if summary_start_date:
        dates.add(summary_start_date)

    generated_date = parse_date_from_text(default_summary.get("generated_at")) if isinstance(default_summary, dict) else None
    if generated_date:
        dates.add(generated_date)

    if not dates:
        dates.add(pd.Timestamp.today().strftime("%Y-%m-%d"))

    return sorted(dates, reverse=True)


def filter_raw_df_by_date(raw_df: pd.DataFrame, selected_date: str) -> pd.DataFrame:
    """
    기존 단일 raw_df 안에 여러 날짜가 섞여 있을 때 선택 날짜만 분리한다.
    """
    if raw_df is None or raw_df.empty or "timestamp" not in raw_df.columns:
        return pd.DataFrame()

    result = raw_df.copy()
    result["timestamp"] = pd.to_datetime(result["timestamp"], errors="coerce")
    result = result.dropna(subset=["timestamp"])
    result = result[result["timestamp"].dt.strftime("%Y-%m-%d") == selected_date]
    return result.reset_index(drop=True)


@st.cache_data(show_spinner=False)
def load_session_log_data(csv_path_text: str, csv_modified_time: float) -> pd.DataFrame:
    """
    선택한 날짜의 CSV를 data_loader.py로 읽는다.
    csv_modified_time은 Streamlit cache 갱신용 파라미터다.
    """
    return load_posture_log(Path(csv_path_text))


@st.cache_data(show_spinner=False)
def load_or_generate_session_summary(
    selected_date: str,
    csv_path_text: str,
    csv_modified_time: float
) -> dict:
    """
    선택 날짜의 report_summary_YYYY-MM-DD.json을 읽거나 생성한다.

    동작:
    1. 리포트 JSON이 없으면 새로 생성
    2. CSV가 JSON보다 최신이면 리포트 재생성
    3. JSON이 최신이면 기존 리포트 사용

    csv_modified_time은 Streamlit cache 갱신용 파라미터다.
    """
    csv_path = Path(csv_path_text)
    report_path = get_report_summary_path(selected_date)

    should_generate = False

    if not report_path.exists():
        should_generate = True
    else:
        csv_mtime = csv_path.stat().st_mtime
        report_mtime = report_path.stat().st_mtime

        if csv_mtime > report_mtime:
            should_generate = True

    if should_generate:
        return generate_report_from_csv(
            csv_path=csv_path,
            output_path=report_path
        )

    return load_report_summary(report_path)


def load_selected_session_data(
    selected_date: str,
    default_summary: dict,
    default_raw_df: pd.DataFrame
) -> tuple[dict, pd.DataFrame]:
    """
    선택한 날짜의 summary/raw_df를 반환한다.

    1. 날짜별 CSV가 있으면 CSV를 data_loader.py로 읽고,
       report_writer.py / summary_builder.py를 통해 summary를 만든다.
    2. CSV는 없지만 날짜별 report JSON이 있으면 summary만 읽는다.
    3. 기존 단일 raw_df에 해당 날짜가 있으면 summary_builder.py로 즉시 summary를 만든다.
    4. 마지막으로 기존 default summary를 fallback으로 사용한다.
    """
    log_file_map = get_session_log_file_map()

    if selected_date in log_file_map:
        csv_path = log_file_map[selected_date]
        csv_modified_time = csv_path.stat().st_mtime

        session_raw_df = load_session_log_data(
            str(csv_path),
            csv_modified_time
        )

        session_summary = load_or_generate_session_summary(
            selected_date,
            str(csv_path),
            csv_modified_time
        )

        return session_summary, session_raw_df

    report_file_map = get_report_file_map()
    if selected_date in report_file_map:
        return load_report_summary(report_file_map[selected_date]), pd.DataFrame()

    filtered_raw_df = filter_raw_df_by_date(default_raw_df, selected_date)
    if not filtered_raw_df.empty:
        return build_report_summary(filtered_raw_df), filtered_raw_df

    return default_summary, default_raw_df


# ------------------------------------------------------------
# 데이터 로드
# ------------------------------------------------------------

@st.cache_data(show_spinner=False)
def load_default_report_data() -> dict:
    try:
        return load_report_summary()
    except FileNotFoundError:
        return {}


@st.cache_data(show_spinner=False)
def load_default_raw_log_data() -> pd.DataFrame:
    try:
        return load_posture_log()
    except Exception:
        return pd.DataFrame()


def prepare_minute_df(summary_data: dict, raw_log_df: pd.DataFrame) -> pd.DataFrame:
    session_data = summary_data.get("session", {})
    minute_summary_data = summary_data.get("minute_summary", [])
    prepared_df = pd.DataFrame(minute_summary_data) if minute_summary_data else pd.DataFrame()

    if not prepared_df.empty:
        prepared_df = attach_real_time_to_minute_df(
            minute_df=prepared_df,
            raw_df=raw_log_df,
            generated_at=summary_data.get("generated_at"),
            total_logged_sec=session_data.get("total_logged_sec")
        )

        if "good_ratio" in prepared_df.columns:
            prepared_df["good_ratio_percent"] = prepared_df["good_ratio"] * 100

        if "drowsy_ratio" in prepared_df.columns:
            prepared_df["drowsy_ratio_percent"] = prepared_df["drowsy_ratio"] * 100

    return prepared_df


def format_session_date_option(date_text: str) -> str:
    date_value = pd.to_datetime(date_text, errors="coerce")

    if pd.isna(date_value):
        return date_text

    return date_value.strftime("%Y-%m-%d")


def get_session_date_text(summary_data: dict, selected_date: str) -> str:
    session_info = summary_data.get("session", {}) if isinstance(summary_data, dict) else {}
    session_start_date = parse_date_from_text(session_info.get("start_time"))
    return session_start_date or selected_date


def get_session_time_text(raw_log_df: pd.DataFrame, summary_data: dict | None = None) -> str:
    if raw_log_df is not None and not raw_log_df.empty and "timestamp" in raw_log_df.columns:
        timestamps = pd.to_datetime(raw_log_df["timestamp"], errors="coerce").dropna()
        if not timestamps.empty:
            return f"{timestamps.min().strftime('%H:%M')} ~ {timestamps.max().strftime('%H:%M')}"

    if summary_data:
        session_info = summary_data.get("session", {})
        start_time = pd.to_datetime(session_info.get("start_time"), errors="coerce")
        end_time = pd.to_datetime(session_info.get("end_time"), errors="coerce")

        if not pd.isna(start_time) and not pd.isna(end_time):
            return f"{start_time.strftime('%H:%M')} ~ {end_time.strftime('%H:%M')}"

    return "-"


def clear_session_cache() -> None:
    st.cache_data.clear()
    st.rerun()


def close_linux_report_browser() -> None:
    """
    Raspberry Pi/Linux 키오스크 환경에서 Streamlit 리포트를 띄운 Chromium 창을 종료한다.

    PyQt 쪽에서 Chromium을 --kiosk 또는 --app 형태로 실행했을 때,
    명령줄에 localhost:8501이 포함되므로 해당 브라우저 프로세스만 종료한다.
    """

    if platform.system() != "Linux":
        return

    patterns = [
        "chromium.*localhost:8501",
        "chromium-browser.*localhost:8501",
        "google-chrome.*localhost:8501",
    ]

    for pattern in patterns:
        try:
            subprocess.run(
                ["pkill", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False
            )
        except Exception:
            pass


def shutdown_streamlit_report(delay_sec: float = 0.8) -> None:
    """
    종료 버튼 클릭 후 Streamlit 서버를 종료한다.

    버튼 클릭 응답이 브라우저에 먼저 전달될 수 있도록
    아주 짧은 지연 후 별도 스레드에서 종료한다.
    """

    def delayed_shutdown():
        time.sleep(delay_sec)

        close_linux_report_browser()

        # 현재 Streamlit 서버 프로세스 종료
        os._exit(0)

    threading.Thread(target=delayed_shutdown, daemon=True).start()


def render_exit_script() -> None:
    """
    브라우저 창 닫기를 요청한다.

    일반 브라우저에서는 보안 정책 때문에 window.close()가 막힐 수 있지만,
    PyQt에서 Chromium 앱/키오스크 모드로 연 경우에는 보조적으로 동작할 수 있다.
    """

    st.markdown(
        """
        <script>
            setTimeout(function() {
                window.open('', '_self');
                window.close();
            }, 300);
        </script>
        """,
        unsafe_allow_html=True
    )


def render_session_debug_info(available_dates: list[str]) -> None:
    """
    날짜 파일이 안 잡힐 때 확인용 디버그 UI.
    필요 없으면 expander 전체를 주석 처리해도 된다.
    """
    with st.expander("세션 로그 경로 디버그"):
        st.write("BASE_DIR:", str(BASE_DIR))
        st.write("현재 작업 경로:", str(Path.cwd()))
        st.write("확인 중인 session_logs 후보:", [str(path) for path in SESSION_LOG_DIR_CANDIDATES])
        st.write("찾은 로그 파일:", {date: str(path) for date, path in get_session_log_file_map().items()})
        st.write("찾은 리포트 파일:", {date: str(path) for date, path in get_report_file_map().items()})
        st.write("available_session_dates:", available_dates)


try:
    default_summary = load_default_report_data()
    default_raw_df = load_default_raw_log_data()
    available_session_dates = get_available_session_dates(default_summary, default_raw_df)
except Exception as e:
    st.error("초기 데이터를 불러오는 중 문제가 발생했습니다.")
    st.exception(e)
    st.stop()


# ------------------------------------------------------------
# 상단 헤더
# ------------------------------------------------------------

# 날짜 선택 박스가 헤더 아래에 보이도록 먼저 헤더 위치만 잡아두고,
# 선택된 날짜의 데이터를 로드한 뒤 아래에서 실제 헤더 내용을 채운다.
header_placeholder = st.empty()


# ------------------------------------------------------------
# 상단 세션 날짜 선택
# ------------------------------------------------------------

col_date, col_refresh, col_exit = st.columns([2.0, 1.0, 0.9], gap="small")

with col_date:
    selected_session_date = st.selectbox(
        "세션 날짜 선택",
        options=available_session_dates,
        index=0,
        key="selected_session_date",
        format_func=format_session_date_option
    )

with col_refresh:
    st.markdown('<div class="refresh-button-spacer"></div>', unsafe_allow_html=True)

    if st.button("새로고침", key="refresh_report_button", use_container_width=True):
        st.cache_data.clear()

        report_path = get_report_summary_path(selected_session_date)

        if report_path.exists():
            report_path.unlink()

        st.rerun()

with col_exit:
    st.markdown('<div class="exit-button-spacer"></div>', unsafe_allow_html=True)

    if st.button("종료", key="exit_report_button", use_container_width=True, type="primary"):
        render_exit_script()
        st.success("리포트를 종료합니다.")
        shutdown_streamlit_report()
        st.stop()


summary, raw_df = load_selected_session_data(
    selected_date=selected_session_date,
    default_summary=default_summary,
    default_raw_df=default_raw_df
)

if not summary:
    st.warning("선택한 세션의 리포트 데이터를 찾지 못했습니다. data/session_logs 폴더를 확인해주세요.")
    render_session_debug_info(available_session_dates)
    st.stop()

session = summary["session"]
score_summary = summary["score_summary"]
posture_summary = summary["posture_summary"]
fatigue_summary = summary["fatigue_summary"]
minute_df = prepare_minute_df(summary, raw_df)

selected_session_date_text = get_session_date_text(summary, selected_session_date)
selected_session_time_text = get_session_time_text(raw_df, summary)

header_placeholder.markdown(
    f"""
    <div class="main-header-wrap">
        <div class="main-title">VisionPoseCoach</div>
        <div class="main-sub">
            선택 세션 날짜: {escape(format_session_date_option(selected_session_date_text))}
            · 측정 시간: {escape(selected_session_time_text)}
            · 리포트 생성 시간: {escape(str(summary.get("generated_at", "-")))}
        </div>
    </div>
    """,
    unsafe_allow_html=True
)


# ------------------------------------------------------------
# 탭
# ------------------------------------------------------------

tab_report, tab_posture, tab_fatigue = st.tabs([
    "오늘의 리포트",
    "자세 분석",
    "피로도 분석"
])


# ------------------------------------------------------------
# 1. 오늘의 리포트
# ------------------------------------------------------------

with tab_report:
    st.markdown('<div class="section-title">오늘의 리포트</div>', unsafe_allow_html=True)

    posture_score = score_summary["posture_score"]
    fatigue_score = score_summary["fatigue_score"]

    score_bar_df = make_score_bar_df(
        posture_score=posture_score,
        fatigue_score=fatigue_score,
        posture_grade=score_summary.get("posture_grade", "-"),
        fatigue_grade=score_summary.get("fatigue_grade", "-")
    )
    left_space, chart_area, right_space = st.columns([1, 4, 1])

    with chart_area:
        render_score_bar_chart(
        score_bar_df,
        title="자세 / 피로도 점수"
    )


    st.write("")

    c1, c2, c3= st.columns(3)
    with c1:
        render_metric_card(
            "나쁜 자세 누적시간",
            posture_summary["bad_time_text"],
            # f"비율 {format_percent_from_ratio(posture_summary['bad_ratio'])}",
            "",
            "#f44444",
            "!",
            card_size="small"
        )
    with c2:
        render_metric_card(
            "피로 경고 누적시간",
            fatigue_summary["drowsy_time_text"],
            # f"비율 {format_percent_from_ratio(fatigue_summary['drowsy_ratio'])}",
            "",
            "#8b5cf6",
            "◔",
            card_size="small"
        )    
    with c3:
        render_metric_card(
            "총 측정 시간",
            session["total_logged_time_text"],
            # f"전체 로그 {session['row_count']}개",
            "",
            "#173a6a",
            "⏱",
            card_size="small"
        ) 


    st.write("")
    st.write("")
    st.write("")

    if not minute_df.empty:
        report_time_unit_min = st.selectbox(
        label="그래프 시간 단위",
        options=[5, 10, 15, 30],
        index=0,
        key="report_time_unit_min",
        format_func=lambda x: f"{x}분 단위",
        label_visibility="collapsed"
        )


        cc1, cc2 = st.columns(2)

        with cc1:
            render_line_chart(
                minute_df,
                x_col="minute_label",
                y_specs=[
                    {"col": "avg_posture_score", "name": "자세 점수", "color": "#1db67d"},
                    {"col": "avg_fatigue_score", "name": "피로도 점수", "color": "#3d7ee8"},
                ],
                title=f"{report_time_unit_min}분 단위 최고 점수 추이",
                unit_min=report_time_unit_min
            )

        with cc2:
            render_line_chart(
                minute_df,
                x_col="minute_label",
                y_specs=[
                    {"col": "good_ratio_percent", "name": "정자세 유지율", "color": "#1db67d"},
                    {"col": "drowsy_ratio_percent", "name": "피로 의심 비율", "color": "#8b5cf6"},
                ],
                title=f"{report_time_unit_min}분 단위 최고 비율 추이",
                y_suffix="%",
                unit_min=report_time_unit_min
            )

    st.write("")


# ------------------------------------------------------------
# 2. 자세 분석
# ------------------------------------------------------------

with tab_posture:
    st.markdown('<div class="section-title">자세 분석</div>', unsafe_allow_html=True)

    posture_type_seconds = posture_summary["posture_type_seconds"]
    total_logged_sec = int(session["total_logged_sec"])

    good_sec = int(posture_type_seconds.get(POSTURE_OPTIMAL, 0))
    forward_head_sec = int(posture_type_seconds.get(POSTURE_FORWARD_HEAD, 0))
    chin_rest_sec = int(posture_type_seconds.get(POSTURE_CHIN_PROPPING, 0))
    asymmetry_sec = int(posture_type_seconds.get(POSTURE_ASYMMETRIC, 0))

    p1, p2, p3, p4 = st.columns(4)
    with p1:
        render_metric_card(
            "거북목 누적 시간",
            seconds_to_text(forward_head_sec),
            # "ForwardHead 감지 누적",
            "",
            "#ff7a17",
            "◌",
            card_size="tiny"
        )
    with p2:
        render_metric_card(
            "턱괴기 누적 시간",
            seconds_to_text(chin_rest_sec),
            # "ChinRest 감지 누적",
            "",
            "#f5a000",
            "◍",
            card_size="tiny"
        )
    with p3:
        render_metric_card(
            "비대칭 누적 시간",
            seconds_to_text(asymmetry_sec),
            # "Asymmetry 감지 누적",
            "",
            "#f44444",
            "⚖",
            card_size="tiny"
        )
    with p4:
        render_metric_card(
            "정자세 유지 시간",
            seconds_to_text(good_sec),
            # f"정자세 유지율 {format_percent_from_ratio(posture_summary['good_ratio'])}",
            "",
            "#19b87a",
            "✓",
            card_size="tiny"
        )

    st.write("")

    chart_left, chart_right = st.columns(2)

    bad_ratio_df = make_bad_posture_ratio_df(posture_type_seconds, total_logged_sec)
    posture_dist_df = make_posture_distribution_df(posture_type_seconds)

    with chart_left:
        render_bad_posture_bar_chart(
            bad_ratio_df,
            title="나쁜 자세 유형별 비율"
        )

    with chart_right:
        render_donut_chart(
            posture_dist_df,
            title="자세 상태 분포",
            center_text="자세\n분포"
        )

    st.write("")

    table_rows = []
    for _, row in posture_dist_df.iterrows():
        ratio = (row["seconds"] / total_logged_sec) if total_logged_sec > 0 else 0
        table_rows.append([
            row["name"],
            seconds_to_text(int(row["seconds"])),
            format_percent_from_ratio(ratio)
        ])

    # render_pretty_table(
    #     title="자세 상태 비율 표",
    #     columns=["자세 유형", "누적 시간", "비율"],
    #     rows=table_rows
    # )

    st.markdown('<div class="section-title">TOP 3 자세 문제</div>', unsafe_allow_html=True)

    top_items = posture_summary.get("top_posture_issues", [])
    if top_items:
        issue_cols = st.columns(3)

        for idx, item in enumerate(top_items[:3]):
            label = item["label"]
            color = get_posture_color(label)
            name = get_posture_name(label)
            ratio_text = format_percent_from_ratio(item["ratio"])
            time_text = item["time_text"]
            tip = POSTURE_TIP_MAP.get(label, DEFAULT_POSTURE_TIP)

            with issue_cols[idx]:
                render_issue_card(
                    rank=idx + 1,
                    label=name,
                    ratio_text=ratio_text,
                    time_text=time_text,
                    tip=tip,
                    color=color
                )

    # st.markdown('<div class="section-title">자세 세부 지표</div>', unsafe_allow_html=True)

    # posture_metric = posture_summary["metric_summary"]
    # render_pretty_table(
    #     title="상세 지표 그래프용 요약",
    #     columns=["지표", "평균", "최대"],
    #     rows=[
    #         [
    #             "거북목 지표",
    #             format_number(posture_metric.get("avg_forward_head_ratio")),
    #             format_number(posture_metric.get("max_forward_head_ratio"))
    #         ],
    #         [
    #             "턱괴기 지표",
    #             format_number(posture_metric.get("avg_chin_rest_score")),
    #             format_number(posture_metric.get("max_chin_rest_score"))
    #         ],
    #         [
    #             "비대칭 지표",
    #             format_number(posture_metric.get("avg_asymmetry_angle")),
    #             format_number(posture_metric.get("max_asymmetry_angle"))
    #         ]
    #     ]
    # )

    if not minute_df.empty:
        st.write("")

        posture_time_unit_min = st.selectbox(
            "그래프 시간 단위",
            options=[5, 10, 15, 30],
            index=0,
            key="posture_time_unit_min"
        )

        # plot1, plot2 = st.columns(2)

        # with plot1:
        #     render_line_chart(
        #         minute_df,
        #         x_col="minute_label",
        #         y_specs=[
        #             {"col": "avg_forward_head_ratio", "name": "거북목 지표", "color": "#ff7a17"},
        #             {"col": "avg_chin_rest_score", "name": "턱괴기 지표", "color": "#f5a000"},
        #             {"col": "avg_asymmetry_angle", "name": "비대칭 지표", "color": "#f44444"},
        #         ],
        #         title=f"{posture_time_unit_min}분 단위 자세 상세 지표 최고값",
        #         unit_min=posture_time_unit_min
        #     )

        # with plot2:
        render_line_chart(
            minute_df,
            x_col="minute_label",
            y_specs=[
                {"col": "good_ratio_percent", "name": "정자세 유지율", "color": "#19b87a"},
            ],
            title=f"{posture_time_unit_min}분 단위 정자세 유지율 최고값",
            y_suffix="%",
            unit_min=posture_time_unit_min
            )


# ------------------------------------------------------------
# 3. 피로도 분석
# ------------------------------------------------------------

with tab_fatigue:
    st.markdown('<div class="section-title">피로도 분석</div>', unsafe_allow_html=True)

    fatigue_dist_df = make_fatigue_distribution_df(fatigue_summary["fatigue_label_seconds"])

    left_space, chart_area, right_space = st.columns([1, 4, 1])

    with chart_area:
        render_donut_chart(
            fatigue_dist_df,
            title="피로 상태 분포",
            center_text="피로\n분포"
        )

    # st.write("")

    # f1, f2, f3 = st.columns(3)
    # with f1:
    #     render_metric_card(
    #         "평균 피로도",
    #         f"{score_summary['fatigue_score']}점",
    #         "임시 Value",
    #         "#3d7ee8",
    #         "◉"
    #     )
    # with f2:
    #     render_metric_card(
    #         "평균 눈 감김 정도",
    #         format_number(fatigue_summary["avg_eye_closed_ratio"]),
    #         "",
    #         "#6366f1",
    #         "◎"
    #     )
    # with f3:
    #     render_metric_card(
    #         "하품 횟수",
    #         f"{fatigue_summary['total_yawn_count']}회",
    #         "",
    #         "#f59e0b",
    #         "◌"
    #     )
    # with f4:
    #     render_metric_card(
    #         "최대 눈 감김 지속",
    #         f"{format_number(fatigue_summary['max_eye_closed_duration'])}초",
    #         "연속 눈 감김 최대값",
    #         "#ef4444",
    #         "◡"
    #     )

    st.write("")

    f5, f6, f7 = st.columns(3)
    with f5:
        render_metric_card(
            "평균 피로도",
            f"{score_summary['fatigue_score']}점",
            "임시 Value",
            "#3d7ee8",
            "◉",
            card_size="small"
        )
    with f6:
    
        render_metric_card(
            "정상 시간",
            fatigue_summary["normal_time_text"],
            "",
            "#3d7ee8",
            "✓",
            card_size="small"
        )
    with f7:
        render_metric_card(
            "피로 경고 누적시간",
            fatigue_summary["drowsy_time_text"],
            # f"비율 {format_percent_from_ratio(fatigue_summary['drowsy_ratio'])}",
            "",
            "#8b5cf6",
            "◔",
            card_size="small"
        )
    # with f7:
    #     render_metric_card(
    #         "평균 입 벌림 비율",
    #         format_number(fatigue_summary["avg_mouth_open_ratio"]),
    #         "mouth_open_ratio 평균",
    #         "#f97316",
    #         "◠"
    #     )
    # with f7:
    #     render_metric_card(
    #         "총 측정 시간",
    #         session["total_elapsed_time_text"],
    #         "",
    #         "#173a6a",
    #         "⏱"
    #     )

    st.write("")

    # fatigue_dist_df = make_fatigue_distribution_df(fatigue_summary["fatigue_label_seconds"])
    # fd_left, fd_right = st.columns(2)


    # with fd_left:
    #     render_donut_chart(
    #         fatigue_dist_df,
    #         title="피로 상태 분포",
    #         center_text="피로\n분포"
    #     )

    # with fd_right:
    #     fatigue_rows = []
    #     total_fatigue_sec = sum(fatigue_summary["fatigue_label_seconds"].values())

    #     for _, row in fatigue_dist_df.iterrows():
    #         ratio = (row["seconds"] / total_fatigue_sec) if total_fatigue_sec > 0 else 0
    #         fatigue_rows.append([
    #             row["name"],
    #             seconds_to_text(int(row["seconds"])),
    #             format_percent_from_ratio(ratio)
    #         ])

    #     render_pretty_table(
    #         title="피로 상태 비율 표",
    #         columns=["상태", "누적 시간", "비율"],
    #         rows=fatigue_rows
    #     )

    st.markdown('<div class="section-title">피로도 세부 지표</div>', unsafe_allow_html=True)

    # fatigue_metric = fatigue_summary["metric_summary"]
    # render_pretty_table(
    #     title="피로도 지표 요약",
    #     columns=["지표", "평균", "최대"],
    #     rows=[
    #         [
    #             "눈 감김 비율",
    #             format_number(fatigue_metric.get("avg_eye_closed_ratio")),
    #             format_number(fatigue_metric.get("max_eye_closed_ratio"))
    #         ],
    #         [
    #             "연속 눈 감김 시간",
    #             format_number(fatigue_metric.get("avg_eye_closed_duration")),
    #             format_number(fatigue_metric.get("max_eye_closed_duration"))
    #         ],
    #         [
    #             "입 벌림 비율",
    #             format_number(fatigue_metric.get("avg_mouth_open_ratio")),
    #             format_number(fatigue_metric.get("max_mouth_open_ratio"))
    #         ]
    #     ]
    # )

    if not minute_df.empty:
        st.write("")

        fatigue_time_unit_min = st.selectbox(
            "그래프 시간 단위",
            options=[5, 10, 15, 30],
            index=0,
            key="fatigue_time_unit_min"
        )


        render_line_chart(
            minute_df,
            x_col="minute_label",
            y_specs=[{"col": "drowsy_ratio_percent", "name": "피로 경고 비율", "color": "#8b5cf6"},],
            title=f"{fatigue_time_unit_min}분 단위 피로 의심 비율 최고값",
            y_suffix="%",
            unit_min=fatigue_time_unit_min
            )

        # ff1, ff2 = st.columns(2)

        # with ff1:
        #     render_line_chart(
        #         minute_df,
        #         x_col="minute_label",
        #         y_specs=[
        #             {"col": "drowsy_ratio_percent", "name": "피로 경고 비율", "color": "#8b5cf6"},
        #         ],
        #         title=f"{fatigue_time_unit_min}분 단위 피로 의심 비율 최고값",
        #         y_suffix="%",
        #         unit_min=fatigue_time_unit_min
        #     )

        # with ff2:
        #     render_line_chart(
        #         minute_df,
        #         x_col="minute_label",
        #         y_specs=[
        #             {"col": "avg_eye_closed_ratio", "name": "눈 감김 비율", "color": "#6366f1"},
        #             {"col": "avg_mouth_open_ratio", "name": "입 벌림 비율", "color": "#f97316"},
        #         ],
        #         title=f"{fatigue_time_unit_min}분 단위 눈 감김 / 입 벌림 최고값", 
        #         unit_min=fatigue_time_unit_min
        #     )


# ------------------------------------------------------------
# 개발용 원본 로그
# ------------------------------------------------------------

# with st.expander("개발용 원본 로그 확인"):
#     if not raw_df.empty:
#         st.dataframe(raw_df, use_container_width=True)
#     else:
#         st.info("원본 로그를 불러오지 못했습니다.")