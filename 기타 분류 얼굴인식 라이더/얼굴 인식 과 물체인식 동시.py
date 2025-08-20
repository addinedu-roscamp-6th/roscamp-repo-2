import sys
import os
import cv2
import numpy as np
import face_recognition
import logging
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
import dlib
from ultralytics import YOLO

# dlib线程数限制
dlib.DLIB_USE_MAX_NUM_THREADS = 4

# 日志配置
logging.basicConfig(
    filename='recognition.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class RecognitionThread(QThread):
    finished = pyqtSignal(list, list, list)

    def __init__(self, frame, known_face_encodings, known_face_names, yolo_model):
        super().__init__()
        self.frame = frame
        self.known_face_encodings = known_face_encodings
        self.known_face_names = known_face_names
        self.yolo_model = yolo_model

    def run(self):
        try:
            # 人脸识别
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame, model='hog')
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations, model='large') if face_locations else []
            names = []

            for encoding in face_encodings:
                matches = face_recognition.compare_faces(self.known_face_encodings, encoding, tolerance=0.5)
                name = "Unknown"
                face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
                if len(face_distances) > 0:
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                names.append(name)

            # 物体识别
            results = self.yolo_model(self.frame, conf=0.4)
            objects = [(int(box[0]), int(box[1]), int(box[2]), int(box[3]), self.yolo_model.names[int(cls)])
                       for r in results for box, cls in zip(r.boxes.xyxy, r.boxes.cls)]

            self.finished.emit(face_locations, names, objects)
        except Exception as e:
            logging.error(f"识别线程错误: {e}")
            self.finished.emit([], [], [])

class FaceObjectRecognition(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("人脸与物体识别系统")
        self.setFixedSize(1280, 720)
        
        # 初始化变量
        self.known_face_encodings = []
        self.known_face_names = []
        self.current_face_locations = []
        self.current_names = []
        self.current_objects = []
        
        # 初始化YOLO模型
        self.yolo_model = YOLO("yolov8n.pt")
        
        # 初始化UI
        self.init_ui()
        
        # 加载已知人脸
        self.load_known_faces()
        
        # 初始化摄像头
        self.init_camera()
        
        # 启动定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def init_ui(self):
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black;")
        
        self.status_label = QLabel("系统初始化中...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("font-size: 16px; color: white;")
        
        self.warning_label = QLabel("")
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setStyleSheet("color: red; font-size: 28px; font-weight: bold; background-color: rgba(0,0,0,0.5);")
        
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.status_label)
        layout.addWidget(self.warning_label)
        self.setLayout(layout)

    def init_camera(self):
        self.cap = cv2.VideoCapture('http://192.168.4.1:8889')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        
        if not self.cap.isOpened():
            error_msg = "无法打开摄像头"
            logging.error(error_msg)
            self.status_label.setText(error_msg)
            self.status_label.setStyleSheet("color: red;")
            QTimer.singleShot(3000, self.close)

    def load_known_faces(self):
        folder_path = "known_faces"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            logging.warning(f"创建了已知人脸目录: {folder_path}")
            self.status_label.setText(f"请将已知人脸图片放入 {folder_path} 目录")
            return

        loaded_count = 0
        for person_name in os.listdir(folder_path):
            person_path = os.path.join(folder_path, person_name)
            if os.path.isdir(person_path):
                for filename in os.listdir(person_path):
                    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                        try:
                            image_path = os.path.join(person_path, filename)
                            image = face_recognition.load_image_file(image_path)
                            image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
                            image = cv2.convertScaleAbs(image, alpha=1.1, beta=5)
                            encodings = face_recognition.face_encodings(image, model='large')
                            if encodings:
                                for encoding in encodings:
                                    self.known_face_encodings.append(encoding)
                                    self.known_face_names.append(person_name)
                                    loaded_count += 1
                                    logging.info(f"已注册: {person_name} ({filename})")
                            else:
                                logging.warning(f"未检测到人脸: {filename}")
                        except Exception as e:
                            logging.error(f"加载 {filename} 失败: {e}")

        self.status_label.setText(f"系统就绪 | 已加载 {loaded_count} 张人脸")
        if loaded_count == 0:
            self.status_label.setStyleSheet("color: orange;")

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            logging.error("无法获取视频帧")
            return

        frame = self.adjust_color_balance(frame)
        self.display_frame(frame)
        
        if not hasattr(self, 'frame_count'):
            self.frame_count = 0
        self.frame_count += 1
            
        if self.frame_count % 4 == 0:
            self.recognition_thread = RecognitionThread(
                frame.copy(), 
                self.known_face_encodings, 
                self.known_face_names,
                self.yolo_model
            )
            self.recognition_thread.finished.connect(self.update_recognition_results)
            self.recognition_thread.start()

    def adjust_color_balance(self, frame):
        b, g, r = cv2.split(frame)
        b = cv2.addWeighted(b, 0.9, np.zeros_like(b), 0, 0)
        frame = cv2.merge([b, g, r])
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(frame)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        frame = cv2.merge([l, a, b])
        frame = cv2.cvtColor(frame, cv2.COLOR_LAB2BGR)
        return frame

    def update_recognition_results(self, face_locations, names, objects):
        self.current_face_locations = face_locations
        self.current_names = names
        self.current_objects = objects
        
        if "Unknown" in names:
            self.warning_label.setText("警告: 检测到未授权人员!")
            logging.warning(f"检测到未授权人员: {names}")
        elif names or objects:
            self.warning_label.setText("")
            logging.info(f"识别到: 人脸 {names}, 物体 {objects}")
        else:
            self.warning_label.setText("")

    def display_frame(self, frame):
        # 绘制人脸框
        for (top, right, bottom, left), name in zip(self.current_face_locations, self.current_names):
            color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
            cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
            cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)

        # 绘制物体框
        for (x1, y1, x2, y2, label) in self.current_objects:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        logging.info("应用程序关闭")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = FaceObjectRecognition()
    window.show()
    sys.exit(app.exec())
