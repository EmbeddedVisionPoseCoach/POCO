import cv2
import config

class Visualizer:
    def draw(self, frame, face_res, pose_res, fps):
        # Pose 그리기
        if pose_res.pose_landmarks:
            for landmarks in pose_res.pose_landmarks:
                for lm in landmarks:
                    cv2.circle(frame, (int(lm.x * config.FRAME_WIDTH), int(lm.y * config.FRAME_HEIGHT)), 2, (0, 255, 0), -1)
                for conn in config.POSE_CONNECTIONS:
                    p1, p2 = landmarks[conn[0]], landmarks[conn[1]]
                    c1 = (int(p1.x * config.FRAME_WIDTH), int(p1.y * config.FRAME_HEIGHT))
                    c2 = (int(p2.x * config.FRAME_WIDTH), int(p2.y * config.FRAME_HEIGHT))
                    cv2.line(frame, c1, c2, (0, 255, 0), 2)

        # Face 그리기
        if face_res.face_landmarks:
            for landmarks in face_res.face_landmarks:
                for lm in landmarks:
                    cv2.circle(frame, (int(lm.x * config.FRAME_WIDTH), int(lm.y * config.FRAME_HEIGHT)), 1, (255, 255, 255), -1)

        # Blendshapes 및 FPS
        self.render_blendshapes(frame, face_res.face_blendshapes)
        cv2.putText(frame, f"{fps:.1f}", (config.FRAME_HEIGHT, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    def render_blendshapes(self, canvas, blendshapes):
        if not blendshapes or len(blendshapes) == 0: return
        for i, cat in enumerate(blendshapes[0]):
            if i > 25: break
            y = 30 + i * 15
            cv2.putText(canvas, cat.category_name, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            cv2.rectangle(canvas, (130, y-8), (130 + int(cat.score * 150), y), (0, 255, 0), -1)