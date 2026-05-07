#!/bin/bash

echo "=========================================="
echo "      라즈베리 파이 AI 환경 통합 구축 스크립트"
echo "=========================================="

# [중요] 스크립트 위치와 상관없이 프로젝트 루트(상위 폴더)로 이동
cd "$(dirname "$0")/.."
echo "현재 작업 디렉토리: $(pwd)"

# [1/7] 가상환경 설정
if [ ! -d ".venv" ]; then
    echo "[1/7] 가상환경(.venv) 생성 중..."
    python3 -m venv .venv
fi
source .venv/bin/activate

# [2/7] 필수 도구 업데이트
echo "[2/7] pip 및 기본 빌드 도구 업데이트..."
pip install --upgrade pip setuptools wheel

# [3/7] Git LFS로 받아온 Tensorflow 설치 (수정됨)
echo "[3/7] 로컬 Tensorflow .whl 설치 중..."
TF_WHL="tensorflow-2.15.0.post1-cp311-none-linux_aarch64.whl"

if [ -f "$TF_WHL" ]; then
    # 파일 용량이 정상인지 체크 (LFS가 제대로 작동했는지 확인용)
    FILE_SIZE=$(stat -c%s "$TF_WHL")
    if [ "$FILE_SIZE" -lt 1000000 ]; then
        echo "!!! 에러: 파일 용량이 너무 작습니다. (LFS pull이 안 된 것 같습니다) !!!"
        echo "라즈베리 파이 터미널에서 'git lfs pull'을 먼저 실행하세요."
    else
        pip install "$TF_WHL"
    fi
else
    echo "!!! 에러: 설치할 $TF_WHL 파일이 없습니다. !!!"
fi

# [4/7] TFLite Runtime 다운로드 및 설치
TFLITE_SH="download_tflite_runtime-2.15.0-cp311-none-linux_aarch64.whl.sh"
if [ -f "install/$TFLITE_SH" ]; then
    echo "[4/7] TFLite 다운로드 및 설치 실행..."
    chmod +x "install/$TFLITE_SH"
    cd install
    ./$TFLITE_SH
    # 다운로드된 whl 설치
    pip install tflite_runtime-2.15.0.post1-cp311-none-linux_aarch64.whl
    cd ..
fi

# [5/7] OpenCV 및 필수 라이브러리 설치 (버전 동기화 반영)
echo "[5/7] OpenCV 및 기타 라이브러리 설치 (4.10.0.84 동기화)..."

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
echo "          최종 환경 구축 완료 목록"
echo "=========================================="
pip list | grep -E "tensorflow|tflite|opencv|mediapipe|numpy"
echo "=========================================="