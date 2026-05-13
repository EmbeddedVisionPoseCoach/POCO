import time
import numpy as np
import cv2
import config

class PoseCalibrator:
    def __init__(self):
        self.start_time = None
        self.is_finished = False
        self.features_buffer = []
        self.baseline = None
        
    def adjust_motor_horizontal(self, tilt_angle):
        """
        [하드웨어 제어 영역]
        F4(Head Roll Angle) 등을 참고하여 모터를 구동, 웹캠의 수평을 맞춥니다.
        모터 제어 라이브러리 로직을 여기에 넣으시면 됩니다.
        """
        if abs(tilt_angle) > 2.0: # 2도 이상 기울었을 때만 조정
            # print(f"🔧 모터 가동: {tilt_angle}도 보정 중...")
            pass

    def process(self, frame, results_pose, calculate_features_func):
        """
        매 프레임 호출되어 교정을 진행합니다.
        Returns: (is_done, baseline_result)
        """
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        
        if elapsed < config.CALIBRATION_TIME:
            if results_pose and results_pose.pose_landmarks:
                # 1. 피처 추출 및 버퍼 저장
                landmark_list = [results_pose.pose_landmarks.landmark]
                feats = calculate_features_func(landmark_list)
                
                if any(feats):
                    self.features_buffer.append(feats)
                    
                    # 2. 실시간 모터 수평 조절 (F4 피처 활용 예시)
                    # 11개 피처 버전에서 F4는 고개 기울기이므로, 이를 웹캠 수평의 힌트로 사용
                    tilt = feats[3] 
                    self.adjust_motor_horizontal(tilt)
            
            return False, None
        
        else:
            # 설정된 시간이 지나면 평균값 계산
            if self.features_buffer:
                self.baseline = np.mean(self.features_buffer, axis=0)
                self.is_finished = True
                return True, self.baseline
            return True, None