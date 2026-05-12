import cv2
import time
import mediapipe as mp
from camera import CameraStream
from detector import LandmarkerDetector
from visualizer import Visualizer
from features import calculate_8_features

def main():
    detector = LandmarkerDetector()
    visualizer = Visualizer()
    stream = CameraStream(src=0).start()
    
    cv2.namedWindow("Webcam Stream")
    cv2.namedWindow("Feature Dashboard")
    cv2.moveWindow("Webcam Stream", 100, 100)
    cv2.moveWindow("Feature Dashboard", 760, 100)

    prev_time = 0
    print("🚀 자세 분석 및 피처 검증 중... 'q'를 눌러 종료하세요.")

    try:
        while True:
            frame = stream.read()
            if frame is None: continue

            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            face_res, pose_res = detector.detect(mp_image)
            current_features = calculate_8_features(pose_res.pose_landmarks)
            
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            webcam_frame = visualizer.draw_webcam(frame, face_res, pose_res, fps)
            dashboard_frame = visualizer.draw_dashboard(current_features)

            cv2.imshow("Webcam Stream", webcam_frame)
            cv2.imshow("Feature Dashboard", dashboard_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        stream.stop()
        detector.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()