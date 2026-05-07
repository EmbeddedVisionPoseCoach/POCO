@echo off

echo ==========================================
echo        환경 구축 자동화 스크립트
echo ==========================================

echo [1/5] 가상환경 존재 여부 확인 중...
if not exist .venv py -3.11 -m venv .venv
if not exist .venv python -m venv .venv
timeout /t 1 > nul

echo [2/5] 가상환경 활성화 중...
call .\.venv\Scripts\activate.bat
timeout /t 1 > nul

echo [3/5] pip 자체 업데이트 및 필수 도구 설치...
:: 안내문에 나온 명령어를 그대로 실행하여 경고를 제거합니다.
python.exe -m pip install --upgrade pip
:: 추가 빌드 도구 설치
pip install --upgrade setuptools wheel

echo [4/5] 패키지 설치 시작...
:: 파일명을 확인하여 있는 파일을 설치합니다.
if exist requirements.txt pip install -r requirements.txt
if exist requirements-win.txt pip install -r requirements-win.txt

echo [5/5] 설치 완료된 패키지 목록:
pip list

echo.
echo ------------------------------------------
echo 모든 설정이 완료되었습니다! 엔터를 누르면 종료됩니다.
echo ------------------------------------------
set /p final_enter=