#!/bin/bash

sleep 3

APP_DIR="/home/willtek/VisionPoseCoach/WorkSpace/pyQt"
PYTHON="/home/willtek/VisionPoseCoach/.venv/bin/python"
LOG_FILE="/home/willtek/pyqt_app.log"

# 시스템에 en_US.UTF-8 locale이 생성되지 않은 경우에도 앱 로그 경고가 반복되지 않게
# 이 실행 스크립트에서만 UTF-8 locale을 안전한 기본값으로 사용한다.
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

cd "$APP_DIR" || exit 1

# native library에서 Segmentation fault가 재발하면 Python stack을 로그에 남긴다.
"$PYTHON" -X faulthandler mainpyQt.py >> "$LOG_FILE" 2>&1
