import cv2
import numpy as np
import mediapipe as mp
from camera import CameraStream
from features import calculate_features
from visualizer import Visualizer
import config

def draw_feature_bar_dashboard(features):
    """
    12개의 피처 이름, 수치, 막대 그래프를 한 줄에 정렬하여 시각화
    """
    # 대시보드 크기 (가로를 넓혀서 이름을 수용)
    width, height = 750, 600
    db = np.zeros((height, width, 3), dtype=np.uint8)
    
    feature_names = [
        "Neck Vert Ratio", "Hand-Face Prox", "Shoulder Tilt",
        "Head Roll Ang", "Nose-Shld Hgt", "Center Offset",
        "Eye-Ear Horiz", "Fwd Head Scale", "Hand-Eye Dist",
        "Hand-Nose Dist", "Hand Visible", "Ear-Shld Gap" # F12 추가
    ]

    cv2.putText(db, "--- RAW Feature Dashboard ---", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # 시작 좌표 설정
    name_x = 20        # 이름 시작
    value_x = 220      # 숫자 시작
    bar_x = 350        # 막대 시작
    max_bar_width = 350

    for i, (name, val) in enumerate(zip(feature_names, features)):
        y_pos = 100 + (i * 45)
        
        # 1. 피처 이름 (왼쪽 정렬)
        cv2.putText(db, f"F{i+1}. {name}", (name_x, y_pos - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 2. 실시간 수치 (중앙 정렬)
        cv2.putText(db, f"{val:.4f}", (value_x, y_pos - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 3. 배경 바 (어두운 회색)
        cv2.rectangle(db, (bar_x, y_pos - 25), (bar_x + max_bar_width, y_pos), (40, 40, 40), -1)
        
        # --- 시각화를 위한 정규화 로직 ---
        if i == 3:  # F4: Head Roll (각도 -30~30 가정)
            norm_val = np.clip((val + 30) / 60, 0, 1)
        elif i == 7: # F8: Z-Depth (중요 피처 - 변화폭 강조)
            # 수치가 작을수록 바가 길어지게 하거나 절댓값 강조
            norm_val = np.clip(abs(val) * 5, 0, 1) 
        elif i == 10: # F11: Visibility Flag
            norm_val = val
        
        elif i==11: # F12: Ear-Shoulder Gap (예시로 -0.5~0.5 범위 가정)
            norm_val = np.clip(val, 0, 1) # Gap은 보통 0~1 사이
            color = (255, 255, 0) # 하늘색 등 별도 색상
        else:
            # 일반 비율 데이터
            norm_val = np.clip(abs(val), 0, 1)

        bar_w = int(max_bar_width * norm_val)
        
        # 색상 선정
        color = (0, 255, 0) # 기본 초록
        if i == 10 and val == 0: color = (0, 0, 255) # 손 미인식 빨간색
        elif i == 7 and norm_val > 0.5: color = (0, 165, 255) # 거북목 주의 주황색

        # 4. 실제 수치 바 (네모) 그리기
        cv2.rectangle(db, (bar_x, y_pos - 25), (bar_x + bar_w, y_pos), color, -1)

    return db

def main():
    # 1. 기존 모듈 초기화
    cam = CameraStream(src=0).start()
    viz = Visualizer()
    
    # MediaPipe 초기화 (포즈 + 페이스메쉬)
    mp_pose = mp.solutions.pose
    mp_face_mesh = mp.solutions.face_mesh
    
    pose_detector = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    face_detector = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)

    print("🔍 Feature Monitoring Start...")

    try:
        while True:
            frame = cam.read()
            if frame is None: continue

            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            results_pose = pose_detector.process(img_rgb)
            results_face = face_detector.process(img_rgb)

            current_features = [0.0] * 11

            if results_pose.pose_landmarks:
                # [Visualizer] 전신 랜드마크 렌더링
                face_lms = results_face.multi_face_landmarks[0] if results_face.multi_face_landmarks else None
                viz.draw_landmarks(frame, results_pose.pose_landmarks, face_lms)
                
                # [Features] RAW 피처 추출
                landmark_list = [results_pose.pose_landmarks.landmark]
                current_features = calculate_features(landmark_list)

            # 대시보드 생성
            dashboard = draw_feature_bar_dashboard(current_features)

            # 두 개의 창으로 분리 출력
            cv2.imshow('[View] Landmark All-in-One', frame)
            cv2.imshow('[Analysis] Real-time Feature Bar', dashboard)

            if cv2.waitKey(1) & 0xFF == 27: # ESC
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()