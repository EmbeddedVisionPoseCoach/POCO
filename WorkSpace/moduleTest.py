import tensorflow as tf
import cv2
import mediapipe as mp
import pandas as pd
import jax

print("--- 설치 확인 결과 ---")
print(f"TensorFlow: {tf.__version__}")
print(f"OpenCV: {cv2.__version__}")
print(f"MediaPipe: {mp.__version__}")
print(f"Pandas: {pd.__version__}")
print(f"JAX: {jax.__version__}")
print("----------------------")


import sys
from PyQt5.QtWidgets import QApplication, QWidget

app = QApplication(sys.argv)
w = QWidget()
w.setWindowTitle('Qt5 윈도우 설치 확인')
w.show()
sys.exit(app.exec_())