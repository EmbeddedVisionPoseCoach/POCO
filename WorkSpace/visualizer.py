import mediapipe as mp
import cv2

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
        
    def draw_landmarks(self, frame, pose_landmarks, face_landmarks=None):
        """
        웹캠 프레임 위에 MediaPipe Pose 랜드마크와 연결선을 그립니다.
        """
        if pose_landmarks:
            # 병현님이 작성하신 코드의 로직을 그대로 유지하되, 
            # 위에서 초기화한 속성들을 사용하여 그립니다.
            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=pose_landmarks,
                connections=self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.pose_dot_spec,
                connection_drawing_spec=self.pose_con_spec
            )
        
        if face_landmarks:
            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=face_landmarks,
                connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                landmark_drawing_spec=None, # 점은 생략하고 선만 그림
                connection_drawing_spec=self.face_spec
            )