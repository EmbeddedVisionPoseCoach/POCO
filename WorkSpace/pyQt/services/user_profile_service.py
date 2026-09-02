"""Persistent four-slot calibration profiles for the POCO application."""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_PROFILE_DIR = WORKSPACE_DIR / "data" / "user_profiles"
MONITOR_CALIBRATION_PATH = (
    WORKSPACE_DIR / "data" / "settings" / "monitor_arm_user_calibration.json"
)


class UserProfileService:
    """Save and activate complete calibration bundles in four fixed slots."""

    SLOT_COUNT = 4

    def __init__(self, root: Path = DEFAULT_PROFILE_DIR):
        self.root = Path(root)

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as file:
            value = json.load(file)
        if not isinstance(value, dict):
            raise ValueError(f"JSON object가 아닙니다: {path}")
        return value

    @staticmethod
    def _atomic_json(path: Path, value: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as file:
                json.dump(value, file, ensure_ascii=False, indent=2)
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary_name, path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
            raise

    def slot_dir(self, slot: int) -> Path:
        slot = int(slot)
        if not 1 <= slot <= self.SLOT_COUNT:
            raise ValueError("프로필 슬롯은 1~4만 사용할 수 있습니다.")
        return self.root / f"slot_{slot}"

    def list_profiles(self) -> list[dict[str, Any]]:
        profiles = []
        for slot in range(1, self.SLOT_COUNT + 1):
            metadata_path = self.slot_dir(slot) / "profile.json"
            if metadata_path.exists():
                try:
                    metadata = self._read_json(metadata_path)
                    metadata.update({"slot": slot, "occupied": True})
                except Exception as error:
                    metadata = {
                        "slot": slot,
                        "occupied": True,
                        "name": "손상된 프로필",
                        "error": str(error),
                    }
            else:
                metadata = {"slot": slot, "occupied": False, "name": "빈 슬롯"}
            profiles.append(metadata)
        return profiles

    def save_profile(
        self,
        slot: int,
        name: str,
        pose_baseline: Path | None,
        face_baseline: Path | None,
        hardware_state: dict[str, Any],
        require_pose: bool,
        require_face: bool,
    ) -> dict[str, Any]:
        target = self.slot_dir(slot)
        clean_name = str(name).strip()[:24]
        if not clean_name:
            raise ValueError("프로필 이름을 입력해주세요.")
        if require_pose and (pose_baseline is None or not Path(pose_baseline).exists()):
            raise FileNotFoundError("Pose 보정 기준값이 없습니다.")
        if require_face and (face_baseline is None or not Path(face_baseline).exists()):
            raise FileNotFoundError("Face 보정 기준값이 없습니다.")

        arm = dict(hardware_state.get("monitor_arm", {}))
        calibration = dict(arm.get("calibration", {}))
        imu = dict(hardware_state.get("imu", {}))
        if not calibration.get("session_ready", False):
            raise ValueError("현재 세션의 ToF/눈 간격 보정값이 준비되지 않았습니다.")
        if not imu.get("calibrated", False):
            raise ValueError("현재 세션의 IMU 보정값이 준비되지 않았습니다.")

        self.root.mkdir(parents=True, exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix=f".slot_{slot}.", dir=str(self.root)))
        try:
            if require_pose:
                shutil.copy2(Path(pose_baseline), temporary / "pose_baseline.pkl")
            if require_face:
                shutil.copy2(Path(face_baseline), temporary / "face_baseline.pkl")
            calibration["ready"] = True
            calibration["session_ready"] = True
            self._atomic_json(temporary / "monitor_arm_calibration.json", calibration)
            imu_record = {
                "x_reference_g": imu.get("imu_x_reference_g"),
                "y_reference_g": imu.get("imu_y_reference_g"),
                "x_reference_raw": imu.get("imu_x_reference_raw"),
                "y_reference_raw": imu.get("imu_y_reference_raw"),
                "pitch_reference_deg": imu.get("pitch_reference_deg"),
                "roll_reference_deg": imu.get("roll_reference_deg"),
            }
            if any(value is None for value in imu_record.values()):
                raise ValueError("저장할 IMU 기준값 일부가 없습니다.")
            self._atomic_json(temporary / "imu_calibration.json", imu_record)
            metadata = {
                "version": 1,
                "slot": int(slot),
                "name": clean_name,
                "saved_at": time.time(),
                "has_pose": bool(require_pose),
                "has_face": bool(require_face),
            }
            self._atomic_json(temporary / "profile.json", metadata)

            backup = self.root / f".slot_{slot}.old"
            if backup.exists():
                shutil.rmtree(backup)
            if target.exists():
                os.replace(target, backup)
            try:
                os.replace(temporary, target)
            except Exception:
                if backup.exists() and not target.exists():
                    os.replace(backup, target)
                raise
            if backup.exists():
                shutil.rmtree(backup)
            return metadata
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    def load_profile(self, slot: int) -> dict[str, Any]:
        source = self.slot_dir(slot)
        metadata = self._read_json(source / "profile.json")
        calibration = self._read_json(source / "monitor_arm_calibration.json")
        imu = self._read_json(source / "imu_calibration.json")
        return {
            "metadata": metadata,
            "calibration": calibration,
            "imu": imu,
            "motor_angles_deg": dict(calibration.get("motor_angles_deg", {})),
        }

    def activate_profile(
        self,
        slot: int,
        pose_destination: Path | None,
        face_destination: Path | None,
        require_pose: bool,
        require_face: bool,
    ) -> dict[str, Any]:
        source = self.slot_dir(slot)
        bundle = self.load_profile(slot)
        metadata = bundle["metadata"]
        for required, flag, filename, destination in (
            (require_pose, metadata.get("has_pose"), "pose_baseline.pkl", pose_destination),
            (require_face, metadata.get("has_face"), "face_baseline.pkl", face_destination),
        ):
            if not required:
                continue
            if not flag or not (source / filename).exists() or destination is None:
                raise ValueError("현재 실행 모드에 필요한 비전 기준값이 프로필에 없습니다.")
            destination = Path(destination)
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_name(f".{destination.name}.profile.tmp")
            shutil.copy2(source / filename, temporary)
            os.replace(temporary, destination)
        self._atomic_json(MONITOR_CALIBRATION_PATH, bundle["calibration"])
        return bundle

    def delete_profile(self, slot: int) -> None:
        target = self.slot_dir(slot)
        if target.exists():
            shutil.rmtree(target)
