import os
import sys
import cv2
import time
from datetime import datetime
from PyQt6.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, 
                            QPushButton, QLineEdit, QMessageBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
import face_recognition

class FaceCaptureApp(QWidget):
    update_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("人脸数据采集系统")
        self.setGeometry(100, 100, 800, 600)
        
        self.base_dir = "/home/lc/Desktop/Project文件开始日期_250702/人脸/known_faces"
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)
        
        self.init_ui()
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.capture_count = 0
        self.max_captures = 10
        self.min_face_size = 100
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
        
        self.update_signal.connect(self.update_status)

    def init_ui(self):
        layout = QVBoxLayout()
        
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        layout.addWidget(self.video_label)
        
        self.name_input = QLineEdit(self)
        self.name_input.setPlaceholderText("输入人员姓名")
        layout.addWidget(self.name_input)
        
        self.capture_btn = QPushButton("开始采集", self)
        self.capture_btn.clicked.connect(self.start_capture)
        layout.addWidget(self.capture_btn)
        
        self.status_label = QLabel("准备就绪", self)
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status_label)
        
        self.setLayout(layout)

    def apply_pink_filter(self, frame):
        """整体偏粉色滤镜"""
        pink_overlay = frame.copy()
        pink_overlay[:, :, 0] = cv2.addWeighted(frame[:, :, 0], 1, 255 * 0.4, 0.6, 0)  # 蓝色通道
        pink_overlay[:, :, 2] = cv2.addWeighted(frame[:, :, 2], 1, 255 * 0.4, 0.6, 0)  # 红色通道
        return pink_overlay

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.flip(frame, 1)
            
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame)
            
            frame = self.apply_pink_filter(frame)

            for top, right, bottom, left in face_locations:
                cv2.rectangle(frame, (left, top), (right, bottom), (250, 182, 197), 2)
                if hasattr(self, 'current_person') and self.capture_count > 0:
                    cv2.putText(frame, f"{self.capture_count}/{self.max_captures}", 
                               (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.8, (250, 182, 197), 2)
            
            self.display_image(frame)

    def display_image(self, frame):
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        convert_to_qt_format = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        p = convert_to_qt_format.scaled(self.video_label.width(), self.video_label.height(), 
                                       Qt.AspectRatioMode.KeepAspectRatio)
        self.video_label.setPixmap(QPixmap.fromImage(p))

    def start_capture(self):
        person_name = self.name_input.text().strip()
        if not person_name:
            QMessageBox.warning(self, "错误", "请输入人员姓名")
            return
            
        self.current_person = person_name
        self.capture_count = 0
        self.person_dir = os.path.join(self.base_dir, person_name)
        
        if not os.path.exists(self.person_dir):
            os.makedirs(self.person_dir)
        
        self.update_status(f"开始采集: {person_name}")
        self.capture_btn.setEnabled(False)
        self.name_input.setEnabled(False)
        
        self.capture_timer = QTimer(self)
        self.capture_timer.timeout.connect(self.auto_capture)
        self.capture_timer.start(1000)

    def auto_capture(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        
        if len(face_locations) == 0:
            self.update_status("未检测到人脸，请正对摄像头")
            return
        elif len(face_locations) > 1:
            self.update_status("检测到多张人脸，请单独拍摄")
            return
            
        top, right, bottom, left = face_locations[0]
        face_height = bottom - top
        if face_height < self.min_face_size:
            self.update_status(f"人脸太小，请靠近摄像头 (当前: {face_height}px)")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.current_person}_{timestamp}_{self.capture_count+1}.jpg"
        save_path = os.path.join(self.person_dir, filename)
        
        face_img = frame[top:bottom, left:right]
        cv2.imwrite(save_path, face_img)
        
        self.capture_count += 1
        self.update_status(f"已采集 {self.capture_count}/{self.max_captures} - {filename}")
        
        if self.capture_count >= self.max_captures:
            self.capture_timer.stop()
            self.update_status(f"采集完成: {self.current_person} 共 {self.max_captures} 张")
            self.capture_btn.setEnabled(True)
            self.name_input.setEnabled(True)
            self.show_capture_result()

    def show_capture_result(self):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setWindowTitle("采集完成")
        msg.setText(f"成功采集 {self.max_captures} 张人脸照片\n存储位置: {self.person_dir}")
        msg.setStandardButtons(QMessageBox.StandardButton.Ok)
        msg.exec()

    def update_status(self, message):
        self.status_label.setText(message)
        print(message)

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        if hasattr(self, 'capture_timer') and self.capture_timer.isActive():
            self.capture_timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FaceCaptureApp()
    window.show()
    sys.exit(app.exec())
