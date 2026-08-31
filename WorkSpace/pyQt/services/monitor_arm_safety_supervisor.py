"""Pure safety state machine used to gate all four monitor-arm motors."""

from __future__ import annotations

import time


class MonitorArmSafetySupervisor:
    TRACKING = "AUTO_TRACKING"
    SENSOR_GRACE = "SENSOR_GRACE_HOLD"
    ABSENT = "USER_ABSENT_RETURN"
    POSTURE = "POSTURE_HOLD"
    WAITING_POSTURE = "POSTURE_RESULT_HOLD"

    def __init__(
        self,
        absence_timeout_sec=5.0,
        reacquire_stable_sec=1.0,
        posture_confidence=0.70,
        posture_stale_sec=1.0,
    ):
        self.absence_timeout_sec = max(0.1, float(absence_timeout_sec))
        self.reacquire_stable_sec = max(0.0, float(reacquire_stable_sec))
        self.posture_confidence = min(1.0, max(0.0, float(posture_confidence)))
        self.posture_stale_sec = max(0.1, float(posture_stale_sec))
        self.missing_since = None
        self.reacquired_since = None
        self.return_requested = False
        self.state = self.WAITING_POSTURE
        self.reason = "측정 결과 대기"

    def reset(self):
        self.missing_since = None
        self.reacquired_since = None
        self.return_requested = False
        self.state = self.WAITING_POSTURE
        self.reason = "측정 결과 대기"

    def update(self, tof_valid, landmark_valid, inference, now=None):
        now = time.monotonic() if now is None else float(now)
        sensors_ok = bool(tof_valid and landmark_valid)
        request_return = False

        if not sensors_ok:
            self.reacquired_since = None
            if self.missing_since is None:
                self.missing_since = now
            elapsed = max(0.0, now - self.missing_since)
            if elapsed >= self.absence_timeout_sec:
                self.state = self.ABSENT
                self.reason = "사용자/센서 미검출 지속"
                if not self.return_requested:
                    self.return_requested = True
                    request_return = True
            else:
                self.state = self.SENSOR_GRACE
                self.reason = "사용자/센서 재검출 대기"
            return self.snapshot(now, request_return)

        self.missing_since = None
        if self.return_requested:
            if self.reacquired_since is None:
                self.reacquired_since = now
            if now - self.reacquired_since < self.reacquire_stable_sec:
                self.state = self.SENSOR_GRACE
                self.reason = "복귀 안정화 확인"
                return self.snapshot(now, False)
            self.return_requested = False
        else:
            self.reacquired_since = None

        if not isinstance(inference, dict):
            self.state = self.WAITING_POSTURE
            self.reason = "자세 판정 대기"
            return self.snapshot(now, False)
        try:
            confidence = float(inference.get("confidence", 0.0))
            timestamp = float(inference.get("timestamp", inference.get("created_at", 0.0)))
        except (TypeError, ValueError):
            confidence, timestamp = 0.0, 0.0
        # Pose process timestamps are wall-clock seconds; missing timestamps are
        # accepted for compatibility and still require the confidence threshold.
        stale = timestamp > 0.0 and abs(time.time() - timestamp) > self.posture_stale_sec
        posture = str(inference.get("posture_type", ""))
        if stale or confidence < self.posture_confidence or not posture:
            self.state = self.WAITING_POSTURE
            self.reason = "자세 판정 신뢰도 부족"
        elif posture != "Optimal":
            self.state = self.POSTURE
            self.reason = f"비정상 자세: {posture}"
        else:
            self.state = self.TRACKING
            self.reason = "정상 자세 자동 추종"
        return self.snapshot(now, False)

    def snapshot(self, now=None, request_return=False):
        now = time.monotonic() if now is None else float(now)
        missing_elapsed = (
            0.0 if self.missing_since is None else max(0.0, now - self.missing_since)
        )
        return {
            "state": self.state,
            "reason": self.reason,
            "tracking_allowed": self.state == self.TRACKING,
            "request_return": bool(request_return),
            "return_latched": bool(self.return_requested),
            "missing_elapsed_sec": missing_elapsed,
            "absence_timeout_sec": self.absence_timeout_sec,
        }
