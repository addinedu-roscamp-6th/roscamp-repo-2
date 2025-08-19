from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QGroupBox, QTextEdit, QLabel, QDialog
)
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import Qt
import logging
from datetime import datetime

# 로깅 설정
logging.basicConfig(filename='user_log.txt', level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')

class UserClientUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("양로원 식사 배달 로봇 사용자 인터페이스")
        self.resize(1000, 800)

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
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # 상단 패널: 상태 정보
        status_group = QGroupBox("로봇 상태")
        status_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        status_layout = QHBoxLayout(status_group)
        
        self.status_labels = []
        status_info = [
            ("Pinky 1 상태", "대기 중"),
            ("Pinky 2 상태", "대기 중"),
            ("Pinky 3 상태", "대기 중"),
            ("JetCobot 상태", "대기 중")
        ]
        
        for name, status in status_info:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            label_name = QLabel(name)
            label_name.setStyleSheet("font-size: 16px; font-family: 'Arial';")
            label_status = QLabel(status)
            label_status.setStyleSheet("font-size: 24px; font-family: 'Arial'; font-weight: bold;")
            layout.addWidget(label_name)
            layout.addWidget(label_status)
            self.status_labels.append(label_status)
            status_layout.addWidget(widget)

        main_layout.addWidget(status_group)

        # 중앙 패널: 제어 버튼
        control_group = QGroupBox("식사 배달 요청")
        control_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        control_layout = QGridLayout(control_group)

        room_buttons = [
            ("101호 식사 요청", lambda: self.request_meal(101)),
            ("102호 식사 요청", lambda: self.request_meal(102)),
            ("103호 식사 요청", lambda: self.request_meal(103)),
            ("104호 식사 요청", lambda: self.request_meal(104)),
            ("105호 식사 요청", lambda: self.request_meal(105)),
            ("106호 식사 요청", lambda: self.request_meal(106)),
            ("107호 식사 요청", lambda: self.request_meal(107)),
            ("108호 식사 요청", lambda: self.request_meal(108))
        ]
        
        for i, (text, func) in enumerate(room_buttons):
            btn = QPushButton(text)
            btn.setStyleSheet("background-color: #e1d5e7; color: #647687; border-radius: 6px; padding: 15px; font-size: 18px; font-family: 'Arial'; border: 2px solid #647687;")
            btn.setFixedHeight(100)
            btn.clicked.connect(func)
            control_layout.addWidget(btn, i // 4, i % 4)

        main_layout.addWidget(control_group)

        # 하단 패널: 알림 및 로그
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setSpacing(15)

        # 알림 그룹
        alert_group = QGroupBox("알림")
        alert_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        alert_layout = QVBoxLayout(alert_group)
        self.alert_text = QTextEdit()
        self.alert_text.setStyleSheet("background-color: #fff2cc; color: #647687; border: 2px solid #647687; border-radius: 6px; font-size: 14px; font-family: 'Arial';")
        self.alert_text.setReadOnly(True)
        alert_layout.addWidget(self.alert_text)

        # 로그 그룹
        log_group = QGroupBox("요청 기록")
        log_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px; font-family: 'Arial'; background-color: #bac8d3;")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setStyleSheet("background-color: #fff2cc; color: #647687; border: 2px solid #647687; border-radius: 6px; font-size: 14px; font-family: 'Arial';")
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

        bottom_layout.addWidget(alert_group, stretch=1)
        bottom_layout.addWidget(log_group, stretch=1)
        main_layout.addWidget(bottom_widget)

        # 초기 로그
        self.log_action("사용자 인터페이스 초기화 완료")

    def request_meal(self, room_number):
        message = f"{room_number}호 식사 배달 요청"
        self.log_action(message)
        self.alert_action(f"{room_number}호 식사 배달이 시작되었습니다")
        self.update_status(f"Pinky {room_number % 3 + 1}", f"{room_number}호 배달 중")

    def update_status(self, robot_name, status):
        index = ["Pinky 1", "Pinky 2", "Pinky 3", "JetCobot"].index(robot_name)
        if index >= 0:
            self.status_labels[index].setText(status)

    def log_action(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        self.log_text.append(log_message)
        logging.info(message)

    def alert_action(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        alert_message = f"[{timestamp}] {message}"
        self.alert_text.append(alert_message)

if __name__ == "__main__":
    app = QApplication([])
    window = UserClientUI()
    window.show()
    app.exec()
