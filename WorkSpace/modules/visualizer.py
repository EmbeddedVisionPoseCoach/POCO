import mediapipe as mp
import cv2
import numpy as np
from mediapipe.framework.formats import landmark_pb2

class Visualizer:
    def __init__(self):
        # 1. MediaPipe 그리기 도구 및 포즈 연결 정보 초기화
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh  # 얼굴 그물망 모듈 추가
        self.mp_drawing = mp.solutions.drawing_utils
        
        # 2. 선과 점의 스타일 설정 (이 부분이 있어야 에러가 안 납니다)
        # 점(Dot) 스타일: 초록색, 두께 2, 반지름 2
        self.pose_dot_spec = self.mp_drawing.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2
        )
        # 선(Connection) 스타일: 빨간색, 두께 2
        self.pose_con_spec = self.mp_drawing.DrawingSpec(
            color=(0, 0, 255), thickness=2
        )
        # 3. 얼굴(Face Mesh) 그리기 스타일 설정
        # 선(Connection): 하늘색(Cyan), 두께 1, 점은 그리지 않음
        self.face_spec = self.mp_drawing.DrawingSpec(
            color=(255, 255, 0), thickness=1, circle_radius=1
        )
        
    # def draw_landmarks(self, frame, pose_landmarks, face_landmarks=None):
    #     """
    #     웹캠 프레임 위에 MediaPipe Pose 랜드마크와 연결선을 그립니다.
    #     """
    #     if pose_landmarks:
    #         # 병현님이 작성하신 코드의 로직을 그대로 유지하되, 
    #         # 위에서 초기화한 속성들을 사용하여 그립니다.
    #         self.mp_drawing.draw_landmarks(
    #             image=frame,
    #             landmark_list=pose_landmarks,
    #             connections=self.mp_pose.POSE_CONNECTIONS,
    #             landmark_drawing_spec=self.pose_dot_spec,
    #             connection_drawing_spec=self.pose_con_spec
    #         )
        
    #     if face_landmarks:
    #         self.mp_drawing.draw_landmarks(
    #             image=frame,
    #             landmark_list=face_landmarks,
    #             connections=self.mp_face_mesh.FACEMESH_CONTOURS,
    #             landmark_drawing_spec=None, # 점은 생략하고 선만 그림
    #             connection_drawing_spec=self.face_spec
    #         )
    def draw_landmarks(self, frame, pose_landmarks, face_landmarks=None):
        """
        웹캠 프레임 위에 Pose와 Face 랜드마크를 그립니다.
        """
        # 1. Pose Landmark 시각화 (기존 로직 유지)
        if pose_landmarks:
            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=pose_landmarks,
                connections=self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.pose_dot_spec,
                connection_drawing_spec=self.pose_con_spec
            )
        
        # 2. Face Landmark 시각화 (신형 리스트 대응 로직)
        if face_landmarks:
            # 입력된 데이터가 일반 리스트(mp.tasks 결과)일 경우 변환
            if isinstance(face_landmarks, list):
                render_data = landmark_pb2.NormalizedLandmarkList()
                render_data.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z) 
                    for lm in face_landmarks
                ])
            else:
                render_data = face_landmarks

            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=render_data,
                connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                landmark_drawing_spec=None, # 점 생략
                connection_drawing_spec=self.face_spec # Cyan 색상 선
            )

    
    
    def draw_confidence_dashboard(self, prediction, labels, colors):
        """모든 라벨의 Confidence를 바 차트 형태로 그리는 대시보드 생성"""
        width, height = 600, 400
        db = np.zeros((height, width, 3), dtype=np.uint8)
        margin = 40
        bar_height = 50
        gap = 30
        
        cv2.putText(db, "Analysis Details (All Confidences)", (margin, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        for i in range(len(labels)):
            label_text = labels[i]
            conf = prediction[i]
            color = colors[i]
            y_pos = 100 + i * (bar_height + gap)
            
            # 클래스 이름 및 확률 텍스트
            text_display = f"{label_text}: {conf*100:.1f}%"
            cv2.putText(db, text_display, (margin, y_pos - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            
            # 확률 바 배경 (회색)
            cv2.rectangle(db, (margin, y_pos), (width - margin, y_pos + bar_height), (50, 50, 50), -1)
            
            # 실제 확률 바 (클래스 색상)
            filled_width = int((width - 2 * margin) * conf)
            cv2.rectangle(db, (margin, y_pos), (margin + filled_width, y_pos + bar_height), color, -1)

        return db

    
    def draw_calibration_ui(self, frame, elapsed, total):
        """교정 단계 전용 UI: 프로그레스 바와 안내 메시지"""
        h, w, _ = frame.shape
        progress = min(elapsed / total, 1.0)
        
        # 1. 상단 안내 영역 (반투명 검정 바)
        cv2.rectangle(frame, (0, 0), (w, 100), (0, 0, 0), -1)
        
        # 안내 텍스트
        msg = "PLEASE MAINTAIN OPTIMAL POSTURE"
        cv2.putText(frame, msg, (int(w*0.1), 45), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        sub_msg = f"Calibrating... {elapsed:.1f}s / {total}s"
        cv2.putText(frame, sub_msg, (int(w*0.1), 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # 2. 하단 프로그레스 바
        bar_x1, bar_x2 = int(w*0.1), int(w*0.9)
        bar_y = h - 40
        cv2.rectangle(frame, (bar_x1, bar_y), (bar_x2, bar_y + 15), (50, 50, 50), -1)
        
        filled_w = int((bar_x2 - bar_x1) * progress)
        cv2.rectangle(frame, (bar_x1, bar_y), (bar_x1 + filled_w, bar_y + 15), (0, 255, 0), -1)

        # 3. 바른 자세 가이드라인 (어깨선 수평 가이드 등 시각적 보조)
        # 중앙 수직선
        cv2.line(frame, (int(w/2), 120), (int(w/2), h-80), (100, 100, 100), 1, cv2.LINE_AA)

    

    def draw_result_on_frame(frame, latest_result, elapsed_sec, window_sec, valid_frame_count):
        """
        카메라 화면 위에 현재 수집 상태와 최근 예측 결과를 표시한다.
        """

        # 상단 검은 박스
        cv2.rectangle(frame, (5, 5), (315, 80), (0, 0, 0), -1)

        # 30초 수집 진행률
        progress_text = f"Collecting: {elapsed_sec:.1f}/{window_sec:.0f}s | frames: {valid_frame_count}"

        cv2.putText(
            frame,
            progress_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        # 아직 예측 결과가 없을 때
        if latest_result is None:
            result_text = "Result: waiting..."
            color = (255, 255, 255)

        # 예측 결과가 있을 때
        else:
            label = latest_result["label"]
            fatigue_score = latest_result["fatigue_score"]
            probability = latest_result["drowsy_probability"]

            if label == "drowsy":
                result_text = f"DROWSY | fatigue: {fatigue_score:.1f}% | prob: {probability:.3f}"
                color = (0, 0, 255)
            else:
                result_text = f"NORMAL | fatigue: {fatigue_score:.1f}% | prob: {probability:.3f}"
                color = (0, 255, 0)

        cv2.putText(
            frame,
            result_text,
            (10, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA
        )


    def create_feature_panel(
        elapsed_sec,
        window_sec,
        valid_frame_count,
        latest_result,
        latest_features
    ):
        """
        별도 Feature Monitor 창에 현재 상태와 최근 30초 feature 값을 표시한다.

        주의:
            OpenCV 기본 putText는 한글 출력이 어렵기 때문에
            영어 중심으로 표시한다.
        """

        panel = np.zeros((520, 520, 3), dtype=np.uint8)

        y = 30

        cv2.putText(
            panel,
            "Feature Monitor",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        y += 35

        cv2.putText(
            panel,
            f"Window: {elapsed_sec:.1f} / {window_sec:.0f} sec",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        y += 25

        cv2.putText(
            panel,
            f"Valid blendshape frames: {valid_frame_count}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        y += 35

        if latest_result is None:
            cv2.putText(
                panel,
                "Prediction: waiting for first 30 sec window",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )
            y += 30

        else:
            label = latest_result["label"]
            probability = latest_result["drowsy_probability"]
            fatigue_score = latest_result["fatigue_score"]
            threshold = latest_result["threshold"]

            color = (0, 0, 255) if label == "drowsy" else (0, 255, 0)

            cv2.putText(
                panel,
                f"Prediction: {label}",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                1,
                cv2.LINE_AA
            )

            y += 25

            cv2.putText(
                panel,
                f"Drowsy probability: {probability:.4f}",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            y += 25

            cv2.putText(
                panel,
                f"Fatigue score: {fatigue_score:.2f}",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            y += 25

            cv2.putText(
                panel,
                f"Threshold: {threshold:.4f}",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            y += 35

        # 최근 30초 feature 값 표시
        if latest_features is not None:
            cv2.putText(
                panel,
                "Last 30sec Features",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            y += 25

            feature_names = get_feature_names()

            for name, value in zip(feature_names, latest_features):
                if y > 500:
                    break

                text = f"{name}: {value:.4f}"

                cv2.putText(
                    panel,
                    text,
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (200, 200, 200),
                    1,
                    cv2.LINE_AA
                )

                y += 20

        return panel