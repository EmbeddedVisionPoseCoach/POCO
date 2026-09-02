# POCO Multi-Process 성능평가 사용법

현재 단계에서는 **멀티프로세스 버전만** 측정합니다. 싱글프로세스는 이후 동일한 항목/JSON 스키마로 비교합니다.

## 저장되는 핵심 지표

1. **Camera 입력 FPS** — Main Process가 초당 공급한 Camera Frame 수
2. **Pose 처리 FPS** — Pose Process가 초당 실제 처리한 Frame 수
3. **Frame → Pose 완료(E2E)** — Frame 생성부터 Pose 결과 완료까지의 실제 응답시간
4. **Main Process CPU** — 카메라/UI/IPC를 포함한 Main Process CPU 사용량
5. **Pose Process CPU** — Pose 추론 Process CPU 사용량

보조 진단값으로 Shared Memory Write, Pose Frame 대기시간, MediaPipe 처리시간, Ring Pending/Skip/Overrun도 같이 저장합니다.

## 성능 데이터는 일반 로그와 별도 저장

성능평가 데이터는 사용자 세션/자세 로그와 섞지 않고 아래 전용 폴더에 저장합니다.

```text
WorkSpace/data/performance/
└── multiprocess_YYYYMMDD_HHMMSS_mmm/
    ├── main_profile.json   # Main 원본 sample
    ├── pose_profile.json   # Pose 원본 sample
    └── summary.json        # Tool/PPT용 핵심 평균값 요약
```

- **앱 1회 실행 = 성능평가 run 1개**
- Preview / Calibration은 저장하지 않고 **MEASURING 구간의 sample만 저장**
- 약 2초마다 원본 sample을 추가하고 `summary.json`을 자동 갱신
- 비교 실험 1회가 끝나면 앱을 재실행하면 새 run 폴더가 생김

실행 시 터미널에 다음처럼 실제 저장 경로가 1회 출력됩니다.

```text
[PERFORMANCE] JSON 저장 폴더: .../WorkSpace/data/performance/multiprocess_...
```

## summary.json에서 바로 확인 가능한 값

```text
core_metrics
  camera_fps_avg
  pose_fps_avg
  pose_e2e_ms_avg
  main_cpu_percent_avg
  pose_cpu_percent_avg

diagnostic_metrics
  shared_memory_write_ms_avg
  queue_latency_ms_avg
  mediapipe_ms_avg
  ring_pending_max
  ring_skipped_final
  ring_overrun_final
```

## Tkinter 확인 Tool

```bash
cd ~/POCO/WorkSpace/pyQt
python multiprocess_performance_viewer.py
```

최신 run만 터미널에서 확인:

```bash
python multiprocess_performance_viewer.py --print-latest
```

상단 핵심 카드:
- Camera FPS
- Pose FPS
- 자세 분석 완료 시간(E2E)
- Main CPU
- Pose CPU

아래 표:
- Shared Memory Write
- Pose Frame 대기시간
- MediaPipe 시간
- Pending / Skip / Overrun

## 권장 측정

- 동일 Raspberry Pi / Camera / 해상도 / 목표 FPS / Pose 모델
- 실제 MEASURING 상태로 **60초** 유지
- 가능하면 **3회 반복** 후 평균 비교

CPU는 `process CPU time / wall time` 방식이며 **100% ≈ 논리 CPU 코어 1개 사용**입니다. 나중에 Single Process도 동일 계산식을 사용합니다.
