from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QGroupBox, QTextEdit, QLabel, QDialog
)
from PyQt6.QtGui import QPalette, QColor, QImage, QPixmap
from PyQt6.QtCore import Qt, QTimer
import cv2
import logging
from datetime import datetime

# Simulated hardware imports (replace with actual libraries)
# from jetcobot import JetCobotArm
# from pinky import PinkyCar
# from rplidar import RPLidar
# import RPi.GPIO as GPIO

# 로깅 설정
logging.basicConfig(filename='robot_log.txt', level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("양로원 식사 배달 로봇 제어 인터페이스")
        self.showMaximized()

        # 색상 팔레트 설정 (커스텀 색상)
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
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # 좌측 패널: 카메라 뷰 (50%)
        left_widget = QWidget()
        left_widget.setStyleSheet("background-color: #bac8d3; border-radius: 10px; padding: 15px; border: 2px solid #647687;")
        left_layout = QGridLayout(left_widget)
        left_layout.setSpacing(8)

        # 카메라 디스플레이
        self.camera_labels = []
        camera_names = ["JetCobot 카메라 1", "JetCobot 카메라 2", "Pinky 카메라 1", "Pinky 카메라 2"]
        for i in range(4):
            label = QLabel(camera_names[i])
            label.setStyleSheet("background-color: #fff2cc; border: 2px solid #647687; border-radius: 6px; color: #647687; font-size: 16px; font-family: 'Arial';")
            label.setFixedSize(640, 480)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.camera_labels.append(label)
            left_layout.addWidget(label, i // 2, i % 2)

        main_layout.addWidget(left_widget, stretch=1)

        # 우측 패널: 제어 및 로그 (50%)
        right_widget = QWidget()
        right_widget.setStyleSheet("background-color: #bac8d3; border-radius: 10px; padding: 15px; border: 2px solid #647687;")
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(15)

        # 이상 알림 그룹
        alert_group = QGroupBox("알림")
        alert_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        alert_layout = QVBoxLayout(alert_group)
        self.alert_text = QTextEdit()
        self.alert_text.setStyleSheet("background-color: #fff2cc; color: #647687; border: 2px solid #647687; border-radius: 6px; font-size: 14px; font-family: 'Arial';")
        self.alert_text.setReadOnly(True)
        alert_layout.addWidget(self.alert_text)

        # 로봇 선택 그룹
        robot_group = QGroupBox("로봇 선택")
        robot_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        robot_layout = QGridLayout(robot_group)

        robot_buttons = [
            ("Pinky 1 제어", self.open_pinky1_control),
            ("Pinky 2 제어", self.open_pinky2_control),
            ("Pinky 3 제어", self.open_pinky3_control),
            ("JetCobot 제어", self.open_jetcobot_control)
        ]
        for i, (text, func) in enumerate(robot_buttons):
            btn = QPushButton(text)
            btn.setStyleSheet("background-color: #e1d5e7; color: #647687; border-radius: 6px; padding: 22.5px; font-size: 24px; font-family: 'Arial'; border: 2px solid #647687;")
            btn.setFixedSize(360, 180)  # 50% 증가 (240x120 -> 360x180)
            btn.clicked.connect(func)
            robot_layout.addWidget(btn, i // 2, i % 2)

        # 로그 표시
        log_group = QGroupBox("작업 로그")
        log_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setStyleSheet("background-color: #fff2cc; color: #647687; border: 2px solid #647687; border-radius: 6px; font-size: 14px; font-family: 'Arial';")
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

        right_layout.addWidget(alert_group)
        right_layout.addWidget(robot_group)
        right_layout.addWidget(log_group)
        right_layout.addStretch()

        main_layout.addWidget(right_widget, stretch=1)

        # 카메라 초기화 (시뮬레이션)
        self.cameras = [cv2.VideoCapture(i) for i in range(4)]
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_cameras)
        self.timer.start(100)

        # 로그 및 알림 초기화
        self.log_action("인터페이스 초기화 완료")

    def update_cameras(self):
        for i, cap in enumerate(self.cameras):
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = cv2.resize(frame, (640, 480))
                    h, w, ch = frame.shape
                    bytes_per_line = ch * w
                    q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    self.camera_labels[i].setPixmap(QPixmap.fromImage(q_img))

    def open_pinky1_control(self):
        self.open_pinky_control("Pinky 1")

    def open_pinky2_control(self):
        self.open_pinky_control("Pinky 2")

    def open_pinky3_control(self):
        self.open_pinky_control("Pinky 3")

    def open_pinky_control(self, pinky_name):
        dialog = QDialog(self)
        dialog.setWindowTitle(f"{pinky_name} 제어")
        dialog.setStyleSheet("background-color: #fff2cc;")
        layout = QGridLayout(dialog)
        layout.setSpacing(10)

        buttons = [
            ("주방으로 이동", lambda: self.move_to_kitchen(pinky_name)),
            ("방으로 이동", lambda: self.move_to_room(pinky_name)),
            ("위치 확인", lambda: self.report_position(pinky_name)),
            ("충전소로 이동", lambda: self.move_to_charge(pinky_name)),
            ("정지", lambda: self.pinky_stop(pinky_name)),
            ("대기", lambda: self.pinky_standby(pinky_name))
        ]
        for i, (text, func) in enumerate(buttons):
            btn = QPushButton(text)
            btn.setStyleSheet("background-color: #e1d5e7; color: #647687; border-radius: 6px; padding: 10px; font-size: 18px; font-family: 'Arial'; border: 2px solid #647687;")
            btn.setFixedSize(300, 200)
            btn.clicked.connect(func)
            layout.addWidget(btn, i // 2, i % 2)

        dialog.setFixedSize(1000, 800)
        dialog.exec()

    def open_jetcobot_control(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("JetCobot 제어")
        dialog.setStyleSheet("background-color: #fff2cc;")
        layout = QGridLayout(dialog)
        layout.setSpacing(10)

        buttons = [
            ("식사 적재", self.load_meal),
            ("식사 하역", self.unload_meal),
            ("대기", self.jetcobot_standby),
            ("정지", self.jetcobot_stop)
        ]
        for i, (text, func) in enumerate(buttons):
            btn = QPushButton(text)
            btn.setStyleSheet("background-color: #e1d5e7; color: #647687; border-radius: 6px; padding: 10px; font-size: 18px; font-family: 'Arial'; border: 2px solid #647687;")
            btn.setFixedSize(300, 200)
            btn.clicked.connect(func)
            layout.addWidget(btn, i // 2, i % 2)

        dialog.setFixedSize(1000, 600)
        dialog.exec()

    def move_to_kitchen(self, pinky_name):
        self.log_action(f"{pinky_name} 주방으로 이동")
        self.alert_action(f"{pinky_name} 주방 이동 중 이상 감지됨" if random.random() > 0.7 else "")

    def move_to_room(self, pinky_name):
        self.log_action(f"{pinky_name} 방으로 이동")
        self.alert_action(f"{pinky_name} 방 이동 중 이상 감지됨" if random.random() > 0.7 else "")

    def report_position(self, pinky_name):
        self.log_action(f"{pinky_name} 위치 보고")
        self.alert_action(f"{pinky_name} 위치 보고 중 이상 감지됨" if random.random() > 0.7 else "")

    def move_to_charge(self, pinky_name):
        self.log_action(f"{pinky_name} 충전소로 이동")
        self.alert_action(f"{pinky_name} 충전 이동 중 이상 감지됨" if random.random() > 0.7 else "")

    def pinky_stop(self, pinky_name):
        self.log_action(f"{pinky_name} 정지")
        self.alert_action(f"{pinky_name} 정지 중 이상 감지됨" if random.random() > 0.7 else "")

    def pinky_standby(self, pinky_name):
        self.log_action(f"{pinky_name} 대기")
        self.alert_action(f"{pinky_name} 대기 중 이상 감지됨" if random.random() > 0.7 else "")

    def load_meal(self):
        self.log_action("JetCobot 식사 적재 시작")
        self.alert_action("JetCobot 적재 중 이상 감지됨" if random.random() > 0.7 else "")

    def unload_meal(self):
        self.log_action("JetCobot 식사 하역 시작")
        self.alert_action("JetCobot 하역 중 이상 감지됨" if random.random() > 0.7 else "")

    def jetcobot_standby(self):
        self.log_action("JetCobot 대기")
        self.alert_action("JetCobot 대기 중 이상 감지됨" if random.random() > 0.7 else "")

    def jetcobot_stop(self):
        self.log_action("JetCobot 정지")
        self.alert_action("JetCobot 정지 중 이상 감지됨" if random.random() > 0.7 else "")

    def log_action(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        self.log_text.append(log_message)
        logging.info(message)

    def alert_action(self, message):
        if message:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            alert_message = f"[{timestamp}] 경고: {message}"
            self.alert_text.append(alert_message)

    def closeEvent(self, event):
        for cap in self.cameras:
            if cap.isOpened():
                cap.release()
        super().closeEvent(event)

if __name__ == "__main__":
    import random
    app = QApplication([])
    window = RobotControlUI()
    window.show()
    app.exec()
