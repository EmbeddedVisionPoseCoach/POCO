@echo off
title Vision AI Environment Setup
echo ==========================================
echo        환경 구축 자동화 스크립트 (Win)
echo ==========================================

:: [중요] 스크립트가 어느 위치에서 실행되든 'scripts' 폴더의 부모 폴더(루트)로 이동
cd /d "%~dp0.."

echo [1/5] 가상환경 존재 여부 확인 중...
:: 가상환경이 없을 때만 생성 (3.11 버전 우선 시도)
if not exist .venv (
    py -3.11 -m venv .venv || python -m venv .venv
)
timeout /t 1 > nul

echo [2/5] 가상환경 활성화 중...
:: 가상환경 경로를 루트 기준으로 호출
call .\.venv\Scripts\activate.bat
timeout /t 1 > nul

echo [3/5] pip 자체 업데이트 및 필수 도구 설치...
python.exe -m pip install --upgrade pip
pip install --upgrade setuptools wheel

echo [4/5] 패키지 설치 시작...
:: 루트 폴더에 있는 requirements 파일을 찾아 설치
if exist requirements.txt pip install -r requirements.txt
if exist requirements-win.txt pip install -r requirements-win.txt

echo [추가] Serial Streamlit 리포트 및 데이터 분석 패키지 설치 중...
pip install pandas plotly streamlit joblib scikit-learn pyserial

:: [추가] 텐서플로우 등 특정 버전 설치가 필요하다면 여기에 추가 가능
:: pip install "opencv-python==4.10.0.84"

echo [5/5] 설치 완료된 패키지 목록:
pip list

echo.
echo ------------------------------------------
echo 모든 설정이 완료되었습니다! (위치: %cd%)
echo 엔터를 누르면 종료됩니다.
echo ------------------------------------------
set /p final_enter=