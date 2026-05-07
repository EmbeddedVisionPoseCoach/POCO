#!/bin/bash

echo "=========================================="
echo "      라즈베리 파이 AI 환경 통합 구축 스크립트"
echo "=========================================="

# [중요] 프로젝트 루트로 이동
cd "$(dirname "$0")/.."

# [1/7] 가상환경 설정
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi
source .venv/bin/activate

# [2/7] 필수 도구 업데이트
pip install --upgrade pip setuptools wheel

# [3/7] 구글 드라이브 대용량 다운로드 로직 (수정됨)
TF_FILE_ID="18affo-8VwzCqS0EQfKPqgF8uD8L1XSXc"
TF_WHL_NAME="tensorflow-2.15.0.post1-cp311-none-linux_aarch64.whl"

echo "[3/7] Tensorflow 다운로드 중 (대용량 보안 통과 적용)..."
# 기존에 잘못 받은 2.4KB 파일이 있다면 삭제
rm -f "$TF_WHL_NAME"

# 대용량 파일 확인 절차를 통과하는 wget 명령어
wget --no-check-certificate "https://docs.google.com/uc?export=download&id=$TF_FILE_ID" -O "$TF_WHL_NAME"

# 만약 위 방법으로도 실패할 경우를 대비한 2차 시도 (confirm 코드 추출)
if [ $(stat -c%s "$TF_WHL_NAME") -lt 10000 ]; then
    echo "1차 다운로드 실패, 보안 코드 추출 후 재시도..."
    CONFIRM=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate "https://docs.google.com/uc?export=download&id=$TF_FILE_ID" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')
    wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$CONFIRM&id=$TF_FILE_ID" -O "$TF_WHL_NAME" && rm -rf /tmp/cookies.txt
fi

echo "Tensorflow 설치 시작..."
pip install "$TF_WHL_NAME"

# [4/7] TFLite Runtime 다운로드 및 설치
TFLITE_SH="download_tflite_runtime-2.15.0-cp311-none-linux_aarch64.whl.sh"
if [ -f "install/$TFLITE_SH" ]; then
    echo "[4/7] TFLite 다운로드 실행..."
    chmod +x "install/$TFLITE_SH"
    cd install
    ./$TFLITE_SH
    pip install tflite_runtime-2.15.0.post1-cp311-none-linux_aarch64.whl
    cd ..
fi

# [5/7] OpenCV 및 필수 라이브러리 설치
echo "[5/7] OpenCV 및 기타 라이브러리 설치 (버전 동기화)..."

# 기존 설치된 OpenCV가 있다면 모두 제거 후 새로 설치 (충돌 방지)
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless

# 요청하신 대로 두 버전 모두 4.10.0.84로 고정하여 설치
pip install "opencv-python==4.10.0.84" "opencv-contrib-python==4.10.0.84"

# 나머지 AI 모듈 설치
pip install "mediapipe==0.10.14" "jax<0.4.20" "jaxlib<0.4.20" "ml-dtypes~=0.2.0" "pandas==2.1.4" cvzone h5py ai-edge-litert

# [6/7] NumPy 버전 고정 (중요: Mediapipe/TFLite 충돌 방지)
echo "[6/7] NumPy 버전 최적화 (1.x대로 고정)..."
pip install "numpy<2.0.0"

# [7/7] 결과 확인
echo "=========================================="
echo "          최종 환경 구축 결과"
echo "=========================================="
pip list | grep -E "tensorflow|tflite|opencv|mediapipe|numpy"
echo "=========================================="