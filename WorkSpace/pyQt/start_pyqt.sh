#!/bin/bash

sleep 3

APP_DIR="/home/willtek/VisionPoseCoach/WorkSpace/pyQt"
PYTHON="/home/willtek/VisionPoseCoach/.venv/bin/python"
LOG_FILE="/home/willtek/pyqt_app.log"

cd "$APP_DIR" || exit 1

"$PYTHON" mainpyQt.py >> "$LOG_FILE" 2>&1
