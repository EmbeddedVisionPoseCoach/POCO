#!/bin/bash

set -e

echo "=========================================="
echo "      라즈베리 파이 AI 및 GUI 환경 통합 구축"
echo "=========================================="

# [중요] 이 스크립트 위치 기준으로 프로젝트 루트로 이동
# 예: scripts/setup_pi.sh 에서 실행하면 프로젝트 루트로 이동
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
echo "현재 작업 디렉토리: $(pwd)"

echo "=========================================="
echo "      시스템 패키지 사전 업데이트"
echo "=========================================="
sudo apt update

# venv 생성에 필요한 기본 패키지
sudo apt install -y python3-venv python3-pip

# [1/10] 가상환경 설정
if [ ! -d ".venv" ]; then
    echo "[1/10] 가상환경(.venv) 생성 중..."
    python3 -m venv .venv
else
    echo "[1/10] 기존 가상환경(.venv)을 사용합니다."
fi

# 가상환경이 시스템 패키지(PyQt5 등)를 인식하도록 설정 변경
echo "가상환경 설정을 시스템 패키지 허용으로 변경..."
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg

# 가상환경 활성화
source .venv/bin/activate

# [2/10] 필수 도구 업데이트
echo "[2/10] pip 및 기본 빌드 도구 업데이트..."
pip install --upgrade pip setuptools wheel

# [3/10] Git LFS로 받아온 Tensorflow 설치
echo "[3/10] 로컬 Tensorflow .whl 설치 중..."
TF_WHL="tensorflow-2.15.0.post1-cp311-none-linux_aarch64.whl"

if [ -f "$TF_WHL" ]; then
    FILE_SIZE=$(stat -c%s "$TF_WHL")
    if [ "$FILE_SIZE" -lt 1000000 ]; then
        echo "!!! 에러: 파일 용량이 너무 작습니다. (Git LFS pull 확인 필요) !!!"
        exit 1
    else
        pip install "$TF_WHL"
    fi
else
    echo "!!! 경고: 설치할 $TF_WHL 파일이 없습니다. TensorFlow 설치를 건너뜁니다. !!!"
fi

# [4/10] TFLite Runtime 설치
TFLITE_SH="download_tflite_runtime-2.15.0-cp311-none-linux_aarch64.whl.sh"
if [ -f "install/$TFLITE_SH" ]; then
    echo "[4/10] TFLite 설치 실행..."
    chmod +x "install/$TFLITE_SH"
    cd install
    ./"$TFLITE_SH"
    pip install tflite_runtime-2.15.0.post1-cp311-none-linux_aarch64.whl
    cd ..
else
    echo "[4/10] TFLite 다운로드 스크립트가 없어 건너뜁니다."
fi

# [5/10] OpenCV 및 AI 라이브러리 설치
echo "[5/10] OpenCV 및 AI 라이브러리 설치..."

pip uninstall -y \
    opencv-python \
    opencv-contrib-python \
    opencv-python-headless \
    opencv-contrib-python-headless || true

# TensorFlow / MediaPipe / Pandas 공통 호환 버전
pip install \
    "numpy==1.26.4" \
    "protobuf==4.25.9"

# MediaPipe 의존성
pip install \
    absl-py \
    "attrs>=19.1.0" \
    "flatbuffers>=2.0" \
    matplotlib \
    "sounddevice>=0.4.4" \
    "jax<0.4.20" \
    "jaxlib<0.4.20" \
    "ml-dtypes~=0.2.0" \
    "pandas==2.1.4" \
    h5py \
    ai-edge-litert

# PyQt가 GUI를 담당하므로 OpenCV GUI 제거
pip install "opencv-contrib-python-headless==4.10.0.84"

# OpenCV dependency를 다시 일반 버전으로 덮어쓰지 않도록 --no-deps
pip install --no-deps "mediapipe==0.10.14"
pip install --no-deps cvzone


# [6/10] Streamlit 및 데이터 분석 라이브러리
echo "[6/10] Streamlit, Plotly, Joblib, Scikit-learn, PySerial 설치..."

pip install \
    plotly \
    "streamlit==1.54.0" \
    joblib \
    scikit-learn \
    pyserial


# [7/10] NumPy 최종 확인
echo "[7/10] NumPy 버전 고정..."
pip install "numpy==1.26.4" "protobuf==4.25.9"

# [8/10] Streamlit 설정 파일 자동 생성
echo "[8/10] Streamlit 설정 파일 생성..."
mkdir -p "$HOME/.streamlit"
cat > "$HOME/.streamlit/config.toml" <<'CONFIG_EOF'
[browser]
gatherUsageStats = false

[server]
headless = true
CONFIG_EOF

echo "Streamlit 설정 완료: $HOME/.streamlit/config.toml"

# [9/10] QT5 시스템 패키지 및 PyQt5 설치
echo "[9/10] QT5, PyQt5, 한글 폰트, xcb 관련 패키지 설치..."
sudo apt install -y python3-pyqt5 \
    build-essential perl python-is-python3 2to3 git \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qtdeclarative5-dev cmake qtbase5-examples qt5-doc qt5-doc-html \
    qtmultimedia5-dev libqt5multimedia5-plugins \
    libxcb-xinerama0 fonts-unfonts-core fontconfig

# 한글 폰트 캐시 갱신
echo "폰트 캐시 갱신 중..."
sudo fc-cache -fv || true

# [10/10] 최종 결과 확인
echo "=========================================="
echo "          최종 환경 구축 완료 목록"
echo "=========================================="
pip list | grep -E "tensorflow|tflite|opencv|mediapipe|numpy|PyQt5|pandas|plotly|streamlit|joblib|scikit-learn|sklearn|pyserial" || true
echo "=========================================="
echo "모든 설치가 완료되었습니다!"
echo "사용법: source .venv/bin/activate"
echo "Streamlit 실행 예시: streamlit run WorkSpace/app.py"
echo "=========================================="
