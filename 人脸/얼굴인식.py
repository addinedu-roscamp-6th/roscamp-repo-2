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

# dlib线程数限制
dlib.DLIB_USE_MAX_NUM_THREADS = 4

# 日志配置
logging.basicConfig(
    filename='face_recognition.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class FaceRecognitionThread(QThread):
    finished = pyqtSignal(list, list)

    def __init__(self, frame, known_face_encodings, known_face_names):
        super().__init__()
        self.frame = frame
        self.known_face_encodings = known_face_encodings
        self.known_face_names = known_face_names

    def run(self):
        try:
            # 转换颜色空间为RGB
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            
            # 检测人脸位置
            face_locations = face_recognition.face_locations(rgb_frame, model='hog')
            if not face_locations:
                self.finished.emit([], [])
                return

            # 获取人脸编码
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations, model='large')
            names = []
            
            for encoding in face_encodings:
                matches = face_recognition.compare_faces(
                    self.known_face_encodings, 
                    encoding, 
                    tolerance=0.5  # 调整识别阈值
                )
                name = "Unknown"
                
                # 计算人脸距离
                face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
                if len(face_distances) > 0:
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                
                names.append(name)
            
            self.finished.emit(face_locations, names)
            
        except Exception as e:
            logging.error(f"人脸识别线程错误: {e}")
            self.finished.emit([], [])

class FaceRecognitionSecurity(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("高级人脸识别安全系统")
        self.setFixedSize(1280, 720)
        
        # 初始化变量
        self.known_face_encodings = []
        self.known_face_names = []
        self.current_face_locations = []  # 初始化属性
        self.current_names = []  # 初始化属性
        
        # 初始化UI
        self.init_ui()
        
        # 加载已知人脸
        self.load_known_faces()
        
        # 初始化摄像头
        self.init_camera()
        
        # 启动定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # ~33fps

    def init_ui(self):
        """初始化用户界面"""
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black;")
        
        self.status_label = QLabel("系统初始化中...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("font-size: 16px; color: white;")
        
        self.warning_label = QLabel("")
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setStyleSheet(
            "color: red; font-size: 28px; font-weight: bold; background-color: rgba(0,0,0,0.5);"
        )
        
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.status_label)
        layout.addWidget(self.warning_label)
        self.setLayout(layout)
###############################CameraCameraCameraCameraCameraCameraCameraCamera#############################
    def init_camera(self):
        """初始化摄像头"""
        self.cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture('http://192.168.4.1:8889')  # Use Raspberry Pi IP and port







###############################CameraCameraCameraCameraCameraCameraCameraCamera#############################


        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # 尝试设置自动白平衡
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        
        if not self.cap.isOpened():
            error_msg = "无法打开摄像头"
            logging.error(error_msg)
            self.status_label.setText(error_msg)
            self.status_label.setStyleSheet("color: red;")
            QTimer.singleShot(3000, self.close)

    def load_known_faces(self):
        """加载已知人脸数据库"""
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
                            
                            # 调整图像大小和质量
                            image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
                            image = cv2.convertScaleAbs(image, alpha=1.1, beta=5)
                            
                            # 检测人脸编码
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
        """更新视频帧"""
        ret, frame = self.cap.read()
        if not ret:
            logging.error("无法获取视频帧")
            return

        # 颜色校正 - 减少蓝色偏色
        frame = self.adjust_color_balance(frame)
        
        # 显示当前帧（即使不进行人脸识别也要显示）
        self.display_frame(frame)
        
        # 每4帧处理一次人脸识别以降低CPU负载
        if not hasattr(self, 'frame_count'):
            self.frame_count = 0
        self.frame_count += 1
            
        if self.frame_count % 4 == 0:
            # 使用线程处理人脸识别
            self.recognition_thread = FaceRecognitionThread(
                frame.copy(), 
                self.known_face_encodings, 
                self.known_face_names
            )
            self.recognition_thread.finished.connect(self.update_recognition_results)
            self.recognition_thread.start()

    def adjust_color_balance(self, frame):
        """调整颜色平衡以减少蓝色偏色"""
        # 分离颜色通道
        b, g, r = cv2.split(frame)
        
        # 减少蓝色通道强度
        b = cv2.addWeighted(b, 0.9, np.zeros_like(b), 0, 0)
        
        # 合并通道
        frame = cv2.merge([b, g, r])
        
        # 应用自动白平衡
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(frame)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        frame = cv2.merge([l, a, b])
        frame = cv2.cvtColor(frame, cv2.COLOR_LAB2BGR)
        
        return frame

    def update_recognition_results(self, face_locations, names):
        """更新人脸识别结果"""
        self.current_face_locations = face_locations
        self.current_names = names
        
        if "Unknown" in names:
            self.warning_label.setText("警告: 检测到未授权人员!")
            logging.warning(f"检测到未授权人员: {names}")
        elif names:
            self.warning_label.setText("")
            logging.info(f"识别到: {names}")
        else:
            self.warning_label.setText("")

    def display_frame(self, frame):
        """显示视频帧"""
        # 绘制人脸框和名称（如果有）
        if hasattr(self, 'current_face_locations') and hasattr(self, 'current_names'):
            for (top, right, bottom, left), name in zip(self.current_face_locations, self.current_names):
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                cv2.putText(
                    frame, name, (left + 6, bottom - 6), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1
                )
        
        # 转换为Qt图像格式并显示
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def closeEvent(self, event):
        """关闭事件处理"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        logging.info("应用程序关闭")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle('Fusion')
    
    window = FaceRecognitionSecurity()
    window.show()
    
    sys.exit(app.exec())
