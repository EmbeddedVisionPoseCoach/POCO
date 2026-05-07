#!/bin/bash

echo "=========================================="
echo "      라즈베리 파이 AI 환경 통합 구축 스크립트"
echo "=========================================="

# [중요] 스크립트 위치와 상관없이 프로젝트 루트(상위 폴더)로 이동
cd "$(dirname "$0")/.."
echo "현재 작업 디렉토리: $(pwd)"

# [1/7] 가상환경 설정 및 활성화
if [ ! -d ".venv" ]; then
    echo "[1/7] 가상환경(.venv) 생성 중..."
    python3 -m venv .venv
fi
source .venv/bin/activate

# [2/7] pip 및 기본 도구 업데이트
echo "[2/7] pip 및 기본 빌드 도구 업데이트..."
pip install --upgrade pip setuptools wheel

# [3/7] 구글 드라이브에서 Tensorflow .whl 다운로드 및 설치
TF_FILE_ID="18affo-8VwzCqS0EQfKPqgF8uD8L1XSXc"
TF_WHL_NAME="tensorflow-2.15.0.post1-cp311-none-linux_aarch64.whl"

echo "[3/7] 구글 드라이브에서 Tensorflow 다운로드 중..."
if [ ! -f "$TF_WHL_NAME" ]; then
    wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id='$TF_FILE_ID -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id="$TF_FILE_ID -O "$TF_WHL_NAME" && rm -rf /tmp/cookies.txt
fi
pip install "$TF_WHL_NAME"

# [4/7] TFLite Runtime 다운로드 및 설치 (install 폴더 내 스크립트 활용)
TFLITE_SH="download_tflite_runtime-2.15.0-cp311-none-linux_aarch64.whl.sh"
if [ -f "install/$TFLITE_SH" ]; then
    echo "[4/7] TFLite 다운로드 스크립트 실행..."
    chmod +x "install/$TFLITE_SH"
    cd install
    ./$TFLITE_SH
    TFLITE_WHL=$(ls tflite_runtime-2.15.0.post1-cp311-none-linux_aarch64.whl 2>/dev/null)
    if [ -f "$TFLITE_WHL" ]; then
        pip install "$TFLITE_WHL"
    fi
    cd ..
fi

# [5/7] OpenCV 버전 최적화 (재설치)
echo "[5/7] OpenCV 재설치 (4.10.0.84)..."
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless
pip install "opencv-python==4.10.0.84"

# [6/7] 나머지 추가 모듈 설치
echo "[6/7] 기타 라이브러리 설치 중..."
pip install "mediapipe==0.10.14" "jax<0.4.20" "jaxlib<0.4.20" "ml-dtypes~=0.2.0" "pandas==2.1.4" cvzone h5py ai-edge-litert

# [7/7] 결과 확인
echo "=========================================="
echo "          환경 구축 완료 목록"
echo "=========================================="
pip list | grep -E "tensorflow|tflite|opencv|mediapipe|jax"
echo "=========================================="
read -p "엔터를 누르면 종료됩니다."