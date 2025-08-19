#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QGroupBox, QTextEdit, QLabel
)
from PyQt6.QtGui import QPalette, QColor, QImage, QPixmap
from PyQt6.QtCore import Qt, QTimer
import logging
from datetime import datetime
import cv2

logging.basicConfig(filename='user_log.txt', level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')

class UserClientUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("양로원 식사 배달 로봇 사용자 인터페이스")
        self.resize(1200, 800)

        # 색상 팔레트
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#fff2cc"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#647687"))
        palette.setColor(QPalette.ColorRole.Button, QColor("#e1d5e7"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor("#647687"))
        palette.setColor(QPalette.ColorRole.Base, QColor("#fff2cc"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#647687"))
        self.setPalette(palette)

        # 메인 레이아웃
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # QGroupBox 공통 스타일
        groupbox_style = """
        QGroupBox {
            color:#647687;
            border:2px solid #647687;
            border-radius:10px;
            font-size:18px;
            background-color:#bac8d3;
            margin-top: 30px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top center;
            padding: 6px;
        }
        """

        # 중앙 패널: 101~103호 요청 및 퇴식
        control_group = QGroupBox("식사 배달 요청 및 퇴식")
        control_group.setStyleSheet(groupbox_style)
        control_layout = QVBoxLayout(control_group)

        for room in [101, 102, 103]:
            room_widget = QWidget()
            room_layout = QHBoxLayout(room_widget)
            
            btn_request = QPushButton(f"{room}호 식사 요청")
            btn_request.setStyleSheet("background-color:#e1d5e7; color:#647687; border-radius:6px; font-size:20px; font-weight:bold; border:2px solid #647687;")
            btn_request.setFixedHeight(100)
            btn_request.clicked.connect(lambda checked, r=room: self.request_meal(r))
            
            btn_clear = QPushButton(f"{room}호 퇴식")
            btn_clear.setStyleSheet("background-color:#e1d5e7; color:#647687; border-radius:6px; font-size:20px; font-weight:bold; border:2px solid #647687;")
            btn_clear.setFixedHeight(100)
            btn_clear.clicked.connect(lambda checked, r=room: self.clear_meal(r))
            
            room_layout.addWidget(btn_request)
            room_layout.addWidget(btn_clear)
            control_layout.addWidget(room_widget)

        main_layout.addWidget(control_group)

        # 카메라 패널
        camera_group = QGroupBox("실시간 카메라 화면")
        camera_group.setStyleSheet(groupbox_style)
        camera_layout = QVBoxLayout(camera_group)
        self.camera_label = QLabel()
        self.camera_label.setFixedHeight(400)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("background-color:black; border-radius:6px;")
        camera_layout.addWidget(self.camera_label)
        main_layout.addWidget(camera_group)

        # 하단 패널: 알림 + 로그
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setSpacing(15)

        alert_group = QGroupBox("알림")
        alert_group.setStyleSheet(groupbox_style)
        alert_layout = QVBoxLayout(alert_group)
        self.alert_text = QTextEdit()
        self.alert_text.setReadOnly(True)
        self.alert_text.setStyleSheet("font-size:14px; background-color:#fff2cc;")
        alert_layout.addWidget(self.alert_text)

        log_group = QGroupBox("요청 기록")
        log_group.setStyleSheet(groupbox_style)
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("font-size:14px; background-color:#fff2cc;")
        log_layout.addWidget(self.log_text)

        bottom_layout.addWidget(alert_group, stretch=1)
        bottom_layout.addWidget(log_group, stretch=1)
        main_layout.addWidget(bottom_widget, stretch=1)

        # 카메라 시작
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera)
        self.timer.start(30)

        self.log_action("사용자 인터페이스 초기화 완료")

    def request_meal(self, room_number):
        message = f"{room_number}호 식사 배달 요청"
        self.log_action(message)
        self.alert_action(f"{room_number}호 식사 배달이 시작되었습니다")

    def clear_meal(self, room_number):
        message = f"{room_number}호 퇴식 요청"
        self.log_action(message)
        self.alert_action(f"{room_number}호 퇴식이 시작되었습니다")

    def log_action(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        logging.info(message)

    def alert_action(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.alert_text.append(f"[{timestamp}] {message}")

    def update_camera(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            qimg = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.camera_label.width(), self.camera_label.height(),
                Qt.AspectRatioMode.KeepAspectRatio))

    def closeEvent(self, event):
        if self.cap.isOpened():
            self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication([])
    window = UserClientUI()
    window.show()
    app.exec()
