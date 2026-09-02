# POCO Multi-Process 성능평가 사용법

현재 단계에서는 **멀티프로세스 버전만** 측정합니다. 싱글프로세스 코드는 이후 같은 JSON 스키마/측정 방식으로 추가하면 됩니다.

## 핵심 비교 지표

1. **Camera 입력 FPS**: Main Process가 초당 공급한 카메라 Frame 수
2. **Pose 처리 FPS**: Pose Process가 초당 실제 처리한 Frame 수
3. **Frame → Pose 완료(E2E)**: Frame 생성부터 Pose 처리 완료까지 걸린 시간
4. **CPU 사용률**: Main Process / Pose Process 각각의 CPU 사용량

보조 지표로 Shared Memory Write, Pose Frame 대기시간, MediaPipe 처리시간, Ring Pending/Skip/Overrun도 같이 저장합니다.

## 측정 방법

1. 평소처럼 `mainpyQt.py`를 실행합니다.
2. Calibration/준비 과정을 진행한 뒤 **실제 측정(MEASURING)** 을 시작합니다.
3. 성능평가용으로 최소 30초, 가능하면 60초 이상 유지합니다.
4. JSON은 약 2초 단위 sample로 자동 저장됩니다.
5. 한 번의 비교 실험이 끝나면 앱을 재실행하면 새로운 session 폴더가 생성됩니다.

Preview / Calibration 수치는 JSON 성능평가에 넣지 않고 **MEASURING 구간만 저장**합니다.

## JSON 저장 위치

```text
WorkSpace/data/performance/
└── multiprocess_YYYYMMDD_HHMMSS_mmm/
    ├── main_profile.json
    └── pose_profile.json
```

- `main_profile.json`: Camera FPS, Main CPU, Shared Memory Write, Camera Loop 등
- `pose_profile.json`: Pose FPS, Pose CPU, Queue Latency, E2E, MediaPipe, Pending/Skip/Overrun 등

CPU 값은 추가 패키지 없이 `process CPU time / wall time` 방식으로 측정하며 **100% ≈ 논리 CPU 코어 1개 사용**을 의미합니다. 이후 싱글프로세스 측정도 같은 계산식을 사용해야 공정하게 비교할 수 있습니다.

## Tkinter 확인 Tool

라즈베리파이 GUI 환경에서:

```bash
cd ~/POCO/WorkSpace/pyQt
python multiprocess_performance_viewer.py
```

최신 결과를 터미널에서 빠르게 확인하려면:

```bash
python multiprocess_performance_viewer.py --print-latest
```

Tool의 상단 핵심 카드에는 다음 5개가 표시됩니다.

- Camera FPS
- Pose FPS
- 자세 분석 완료 시간(E2E)
- Main CPU
- Pose CPU

아래 표에서는 Shared Memory Write, Frame 대기시간, MediaPipe 시간, Pending/Skip/Overrun을 추가 확인할 수 있습니다.

## 비교 실험 권장 조건

나중에 Single Process와 비교할 때 다음 조건은 동일하게 맞춥니다.

- 동일 Raspberry Pi
- 동일 Camera / 해상도 / 목표 30 FPS
- 동일 Pose 모델 및 설정
- 동일 촬영 환경
- 동일 측정 시간

권장: **60초 × 3회** 측정 후 평균 비교.
