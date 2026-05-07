#!/bin/bash

echo "=========================================="
echo "      라즈베리 파이 AI 및 GUI 환경 통합 구축"
echo "=========================================="

# [중요] 프로젝트 루트로 이동
cd "$(dirname "$0")/.."
echo "현재 작업 디렉토리: $(pwd)"

# [1/8] 가상환경 설정
if [ ! -d ".venv" ]; then
    echo "[1/8] 가상환경(.venv) 생성 중..."
    python3 -m venv .venv
fi

# 가상환경이 시스템 패키지(PyQt5 등)를 인식하도록 설정 변경
echo "가상환경 설정을 시스템 패키지 허용으로 변경..."
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg

# 가상환경 활성화
source .venv/bin/activate

# [2/8] 필수 도구 업데이트
echo "[2/8] pip 및 기본 빌드 도구 업데이트..."
pip install --upgrade pip setuptools wheel

# [3/8] Git LFS로 받아온 Tensorflow 설치
echo "[3/8] 로컬 Tensorflow .whl 설치 중..."
TF_WHL="tensorflow-2.15.0.post1-cp311-none-linux_aarch64.whl"

if [ -f "$TF_WHL" ]; then
    FILE_SIZE=$(stat -c%s "$TF_WHL")
    if [ "$FILE_SIZE" -lt 1000000 ]; then
        echo "!!! 에러: 파일 용량이 너무 작습니다. (LFS pull 확인 필요) !!!"
        exit 1
    else
        pip install "$TF_WHL"
    fi
else
    echo "!!! 에러: 설치할 $TF_WHL 파일이 없습니다. !!!"
fi

# [4/8] TFLite Runtime 설치
TFLITE_SH="download_tflite_runtime-2.15.0-cp311-none-linux_aarch64.whl.sh"
if [ -f "install/$TFLITE_SH" ]; then
    echo "[4/8] TFLite 설치 실행..."
    chmod +x "install/$TFLITE_SH"
    cd install
    ./$TFLITE_SH
    pip install tflite_runtime-2.15.0.post1-cp311-none-linux_aarch64.whl
    cd ..
fi

# [5/8] OpenCV 및 AI 라이브러리 설치 (4.10.0.84 동기화)
echo "[5/8] OpenCV 및 기타 라이브러리 설치..."
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless
pip install "opencv-python==4.10.0.84" "opencv-contrib-python==4.10.0.84"
pip install "mediapipe==0.10.14" "jax<0.4.20" "jaxlib<0.4.20" "ml-dtypes~=0.2.0" "pandas==2.1.4" cvzone h5py ai-edge-litert

# [6/8] NumPy 버전 고정
echo "[6/8] NumPy 버전 최적화 (1.x 고정)..."
pip install "numpy<2.0.0"

# [7/8] QT5 시스템 패키지 및 PyQt5 설치 (사용자 목록 반영)
echo "[7/8] QT5 및 파이썬 PyQt5 시스템 패키지 설치..."
sudo apt update
# 사용자님이 주신 목록 + 파이썬 연결용 python3-pyqt5 추가
sudo apt install -y python3-pyqt5 \
    build-essential perl python-is-python3 2to3 git \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qtdeclarative5-dev cmake qtbase5-examples qt5-doc qt5-doc-html \
    qtmultimedia5-dev libqt5multimedia5-plugins

# [8/8] 최종 결과 확인
echo "=========================================="
echo "          최종 환경 구축 완료 목록"
echo "=========================================="
pip list | grep -E "tensorflow|tflite|opencv|mediapipe|numpy|PyQt5"
echo "=========================================="
echo "모든 설치가 완료되었습니다!"
echo "사용법: source .venv/bin/activate"