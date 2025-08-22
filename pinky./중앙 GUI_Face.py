#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ── (选择) Qt 日志简化 ────────────────────────────────────────────────
import os
os.environ["QT_LOGGING_RULES"] = "*.debug=false;*.info=false;qt.qpa.*=false"
# ───────────────────────────────────────────────────────────────────────

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QGroupBox, QTextEdit, QLabel
)
from PyQt6.QtGui import QPalette, QColor, QImage, QPixmap
from PyQt6.QtCore import Qt, QTimer, qInstallMessageHandler, QObject, pyqtSignal
qInstallMessageHandler(lambda *args: None)  # Qt 消息静音（不需要可注释）

import sys
from datetime import datetime
import math
import urllib.request
import threading, time  # ← MJPEG 读取线程
import socket  # ← UDP
import cv2
import numpy as np
import face_recognition
import logging
from PyQt6.QtCore import QThread

# ===== ROS2 =====
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ===== 视频/位置话题设置 =====
IMAGE_TOPICS = {
    "Global":   "/marker/debug_image",                 # ← ROS Image 话题
    "Pinky-1":  "http://192.168.247.64:9817/video_feed",  # ← MJPEG (HTTP)
    "Pinky-2":  "http://192.168.247.101:9818/video_feed", # ← MJPEG (HTTP)
    "Pinky-3":  "http://192.168.4.1:8889", # ← 人脸识别相机
}
POSE_TOPICS = {
    "Pinky-1": "pinky1/marker_map/ID25",
    "Pinky-2": "pinky2/marker_map/ID26",
    "Pinky-3": "pinky3/marker_map/ID27",
}
CELL_W, CELL_H = 640, 360

# 机器人臂 MJPEG 流地址 - Flask 的 /video_feed
JETCOBOT_STREAM_URL = "http://192.168.247.164:9816/video_feed"

# UDP 接收端口
UDP_PORT = 9999

# ───────────────────────────────────────────────────────────────────────
# 延迟优化 MJPEG 读取器：大块连续读取 + 仅保留最新帧
# ───────────────────────────────────────────────────────────────────────
class MjpegReader:
    """快速读取 MJPEG 流，仅将最新帧（jpg bytes）传递给回调"""
    def __init__(self, url: str, on_frame_bytes):
        self.url = url
        self.on_frame_bytes = on_frame_bytes
        self._buf = bytearray()
        self._stop = False
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop = True

    def _run(self):
        conn = None
        while not self._stop:
            try:
                req = urllib.request.Request(self.url, headers={'User-Agent': 'Mozilla/5.0'})
                conn = urllib.request.urlopen(req, timeout=3.0)
                while not self._stop:
                    chunk = conn.read(65536)  # 64KB
                    if not chunk:
                        break
                    self._buf += chunk

                    while True:
                        soi = self._buf.find(b'\xff\xd8')  # Start Of Image
                        if soi < 0:
                            if len(self._buf) > 2_000_000:
                                del self._buf[:-2]
                            break
                        eoi = self._buf.find(b'\xff\xd9', soi + 2)  # End Of Image
                        if eoi < 0:
                            if soi > 1_000_000:
                                del self._buf[:soi]
                            break
                        jpg = bytes(self._buf[soi:eoi+2])
                        del self._buf[:eoi+2]
                        self.on_frame_bytes(jpg)

                    if len(self._buf) > 3_000_000:
                        self._buf = self._buf[-1_000_000:]

            except Exception:
                time.sleep(0.3)
            finally:
                try:
                    if conn:
                        conn.close()
                except Exception:
                    pass
                conn = None

# 人脸识别线程
class FaceRecognitionThread(QThread):
    finished = pyqtSignal(list, list, np.ndarray)

    def __init__(self, frame, known_face_encodings, known_face_names):
        super().__init__()
        self.frame = frame
        self.known_face_encodings = known_face_encodings
        self.known_face_names = known_face_names

    def run(self):
        try:
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame, model='hog')
            if not face_locations:
                self.finished.emit([], [], self.frame)
                return

            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations, model='large')
            names = []
            
            for encoding in face_encodings:
                matches = face_recognition.compare_faces(
                    self.known_face_encodings, 
                    encoding, 
                    tolerance=0.5
                )
                name = "Unknown"
                
                face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
                if len(face_distances) > 0:
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                
                names.append(name)
            
            for (top, right, bottom, left), name in zip(face_locations, names):
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                cv2.rectangle(self.frame, (left, top), (right, bottom), color, 2)
                cv2.rectangle(self.frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                cv2.putText(
                    self.frame, name, (left + 6, bottom - 6), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1
                )
            
            self.finished.emit(face_locations, names, self.frame)
            
        except Exception as e:
            logging.error(f"人脸识别错误: {e}")
            self.finished.emit([], [], self.frame)

# UDP 接收器（发送到通知面板）
class UdpNotifyReceiver(QObject):
    received = pyqtSignal(str)

    def __init__(self, port=UDP_PORT, parent=None):
        super().__init__(parent)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", port))
        self._thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thr.start()

    def _run(self):
        while True:
            try:
                data, addr = self._sock.recvfrom(8192)
                msg = data.decode("utf-8", "ignore")
                self.received.emit(msg)
            except Exception:
                continue

class RosBridge(Node):
    """ROS2 发布/订阅 + 最新状态保存（多相机 & 3个位置 + Bool 命令专用）"""
    def __init__(self, image_topics: dict[str, str], pose_topics: dict[str, str]):
        super().__init__('delivery_robot_gui')
        self.bridge = CvBridge()
        self.latest_qimages: dict[str, QImage | None] = {name: None for name in image_topics.keys()}
        self.latest_poses: dict[str, tuple[float, float, float] | None] = {name: None for name in pose_topics.keys()}
        self.image_subs = {}
        for name, topic in image_topics.items():
            if isinstance(topic, str) and topic.startswith('/'):
                self.image_subs[name] = self.create_subscription(
                    Image, topic, self._make_img_cb(name),
                    QoSPresetProfiles.SENSOR_DATA.value
                )
        self.pose_subs = {}
        for name, topic in pose_topics.items():
            self.pose_subs[name] = self.create_subscription(
                PoseStamped, topic, self._make_pose_cb(name), 10
            )
        self.pub_log = self.create_publisher(String, '/gui/log', 10)
        self.pub_p1_kitchen = self.create_publisher(Bool, '/pinky1/go_to_kitchen', 10)
        self.pub_p1_serving = self.create_publisher(Bool, '/pinky1/go_to_serving', 10)
        self.pub_p1_charge = self.create_publisher(Bool, '/pinky1/go_to_charge', 10)
        self.pub_p1_room = self.create_publisher(Bool, '/pinky1/go_to_room', 10)
        self.pub_p1_recall = self.create_publisher(Bool, '/pinky1/go_to_recall', 10)
        self.pub_p1_return = self.create_publisher(Bool, '/pinky1/go_to_return', 10)
        self.pub_p1_stop = self.create_publisher(Bool, '/pinky1/stop', 10)
        self.pub_p2_kitchen = self.create_publisher(Bool, '/pinky2/go_to_kitchen', 10)
        self.pub_p2_serving = self.create_publisher(Bool, '/pinky2/go_to_serving', 10)
        self.pub_p2_charge = self.create_publisher(Bool, '/pinky2/go_to_charge', 10)
        self.pub_p2_room = self.create_publisher(Bool, '/pinky2/go_to_room', 10)
        self.pub_p2_recall = self.create_publisher(Bool, '/pinky2/go_to_recall', 10)
        self.pub_p2_return = self.create_publisher(Bool, '/pinky2/go_to_return', 10)
        self.pub_p2_stop = self.create_publisher(Bool, '/pinky2/stop', 10)
        self.pub_p3_kitchen = self.create_publisher(Bool, '/pinky3/go_to_kitchen', 10)
        self.pub_p3_serving = self.create_publisher(Bool, '/pinky3/go_to_serving', 10)
        self.pub_p3_charge = self.create_publisher(Bool, '/pinky3/go_to_charge', 10)
        self.pub_p3_room = self.create_publisher(Bool, '/pinky3/go_to_room', 10)
        self.pub_p3_recall = self.create_publisher(Bool, '/pinky3/go_to_recall', 10)
        self.pub_p3_return = self.create_publisher(Bool, '/pinky3/go_to_return', 10)
        self.pub_p3_stop = self.create_publisher(Bool, '/pinky3/stop', 10)
        self.gui_append_log = None
        self.gui_append_alert = None
        self.gui_set_pose_text = None
        self.sub_coord_log = self.create_subscription(String, '/coordinator/gui_log', self._cb_coord_log, 10)

    def _make_img_cb(self, name: str):
        def _cb(msg: Image):
            try:
                cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_rgb = cv_bgr[:, :, ::-1]
                h, w = cv_rgb.shape[:2]
                bytes_per_line = w * 3
                qimg = QImage(cv_rgb.tobytes(), w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
                self.latest_qimages[name] = qimg
            except Exception as e:
                self._log_gui(f"[{name}] cv_bridge 转换失败: {e}")
        return _cb

    def _make_pose_cb(self, name: str):
        def _cb(msg: PoseStamped):
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.orientation.z
            w = msg.pose.orientation.w
            yaw = 2.0 * math.atan2(z, w)
            yaw_deg = math.degrees(yaw)
            self.latest_poses[name] = (x, y, yaw_deg)
            if self.gui_set_pose_text:
                text = " | ".join(
                    f"{n}: x={p[0]:.2f}, y={p[1]:.2f}, yaw={p[2]:.1f}°"
                    for n, p in self.latest_poses.items() if p is not None
                )
                self.gui_set_pose_text(text if text else "当前位置: -")
        return _cb

    def set_callbacks(self, append_log, append_alert, set_pose_text, append_coord_log):
        self.gui_append_log = append_log
        self.gui_append_alert = append_alert
        self.gui_set_pose_text = set_pose_text
        self.gui_append_coord_log = append_coord_log

    def _log_gui(self, text: str):
        if self.gui_append_log:
            self.gui_append_log(text)
        self.pub_log.publish(String(data=text))

    def _alert_gui(self, text: str):
        if self.gui_append_alert:
            self.gui_append_alert(text)

    def _send_goal(self, robot: int, goal_name: str):
        table = {
            1: {"kitchen": self.pub_p1_kitchen, "serving": self.pub_p1_serving, "charge": self.pub_p1_charge,
                "room": self.pub_p1_room, "recall": self.pub_p1_recall, "return": self.pub_p1_return},
            2: {"kitchen": self.pub_p2_kitchen, "serving": self.pub_p2_serving, "charge": self.pub_p2_charge,
                "room": self.pub_p2_room, "recall": self.pub_p2_recall, "return": self.pub_p2_return},
            3: {"kitchen": self.pub_p3_kitchen, "serving": self.pub_p3_serving, "charge": self.pub_p3_charge,
                "room": self.pub_p3_room, "recall": self.pub_p3_recall, "return": self.pub_p3_return},
        }
        pub = table.get(robot, {}).get(goal_name)
        if pub is None:
            self._log_gui(f"[Pinky{robot}] 未知目标: {goal_name}")
            return
        pub.publish(Bool(data=True))
        self._log_gui(f"[Pinky{robot}] /pinky{robot}/go_to_{goal_name} = True  (桥接→/go_to_{goal_name})")

    def _send_stop(self, robot: int):
        pub = {1: self.pub_p1_stop, 2: self.pub_p2_stop, 3: self.pub_p3_stop}.get(robot)
        if pub is None:
            self._log_gui(f"[Pinky{robot}] STOP 发布者不存在")
            return
        pub.publish(Bool(data=True))
        self._log_gui(f"[Pinky{robot}] /pinky{robot}/stop = True  (桥接→/stop)")

    def _cb_coord_log(self, msg: String):
        if hasattr(self, "gui_append_coord_log") and self.gui_append_coord_log:
            self.gui_append_coord_log(msg.data)

    def send_p1_goal(self, goal_name: str): self._send_goal(1, goal_name)
    def send_p2_goal(self, goal_name: str): self._send_goal(2, goal_name)
    def send_p3_goal(self, goal_name: str): self._send_goal(3, goal_name)
    def send_p1_stop(self): self._send_stop(1)
    def send_p2_stop(self): self._send_stop(2)
    def send_p3_stop(self): self._send_stop(3)

class GUI(QMainWindow):
    def __init__(self, ros: RosBridge, image_topics: dict[str, str]):
        super().__init__()
        self.ros = ros
        self.image_names = list(image_topics.keys())
        self.image_topics = image_topics
        self.setWindowTitle("养老院餐饮配送机器人控制 (ROS2 话题专用)")
        self.showMaximized()

        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#FFF2CC"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#647687"))
        palette.setColor(QPalette.ColorRole.Button, QColor("#E1D5E7"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor("#647687"))
        palette.setColor(QPalette.ColorRole.Base, QColor("#FFF2CC"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#647687"))
        self.setPalette(palette)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        left_widget = QWidget()
        left_widget.setStyleSheet("background-color: #BAC8D3; border-radius: 10px; padding: 15px; border: 2px solid #647687;")
        left_layout = QGridLayout(left_widget)
        left_layout.setSpacing(8)

        jet_group = QGroupBox("JetCobot MJPEG 流")
        jet_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px;")
        jet_layout = QVBoxLayout(jet_group)
        self.image_jetcobot = QLabel("JetCobot 待机中…")
        self.image_jetcobot.setStyleSheet("background-color: #FFF2CC; border: 2px solid #647687; border-radius: 6px; color: #647687;")
        self.image_jetcobot.setFixedSize(CELL_W, CELL_H)
        self.image_jetcobot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        jet_layout.addWidget(self.image_jetcobot)
        left_layout.addWidget(jet_group, 2, 1)

        pose_group = QGroupBox("当前位置 (ID25~27)")
        pose_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px;")
        pose_layout = QVBoxLayout(pose_group)
        self.pose_p1 = QLabel("Pinky-1: 未检测")
        self.pose_p2 = QLabel("Pinky-2: 未检测")
        self.pose_p3 = QLabel("Pinky-3: 未检测")
        for label in (self.pose_p1, self.pose_p2, self.pose_p3):
            label.setStyleSheet("color: #647687; font-size: 14px;")
            pose_layout.addWidget(label)
        left_layout.addWidget(pose_group, 0, 0, 1, 2)

        right_widget = QWidget()
        right_widget.setStyleSheet("background-color: #BAC8D3; border-radius: 10px; padding: 15px; border: 2px solid #647687;")
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(15)

        alert_group = QGroupBox("通知 (UDP)")
        alert_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px;")
        alert_layout = QVBoxLayout(alert_group)
        self.alert_text = QTextEdit()
        self.alert_text.setReadOnly(True)
        self.alert_text.setStyleSheet("background-color: #FFF2CC; border: 2px solid #647687; border-radius: 6px;")
        alert_layout.addWidget(self.alert_text)

        ctrl_group = QGroupBox("机器人控制 (桥接 Bool 话题)")
        ctrl_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px;")
        ctrl_layout = QGridLayout(ctrl_group)
        bool_cmds = [
            ("厨房", "kitchen"),
            ("配餐台", "serving"),
            ("充电站", "charge"),
            ("房间", "room"),
            ("召回", "recall"),
            ("返回", "return"),
        ]
        base_row = 0
        for i, (label, goal) in enumerate(bool_cmds):
            btn = QPushButton(f"Pinky1: {label}")
            btn.setStyleSheet("background-color: #E1D5E7; border-radius: 6px; padding: 10px; font-size: 16px; border: 1px solid #647687;")
            btn.clicked.connect(lambda _, g=goal: self.ros.send_p1_goal(g))
            ctrl_layout.addWidget(btn, base_row + i, 2)
        btn_p1_stop = QPushButton("Pinky1: STOP")
        btn_p1_stop.setStyleSheet("background-color: #ffdddd; border-radius: 6px; padding: 10px; font-size: 16px; border: 2px solid #aa0000;")
        btn_p1_stop.clicked.connect(lambda _: self.ros.send_p1_stop())
        ctrl_layout.addWidget(btn_p1_stop, base_row + len(bool_cmds), 2)
        for i, (label, goal) in enumerate(bool_cmds):
            btn = QPushButton(f"Pinky2: {label}")
            btn.setStyleSheet("background-color: #E1D5E7; border-radius: 6px; padding: 10px; font-size: 16px; border: 1px solid #647687;")
            btn.clicked.connect(lambda _, g=goal: self.ros.send_p2_goal(g))
            ctrl_layout.addWidget(btn, base_row + i, 3)
        btn_p2_stop = QPushButton("Pinky2: STOP")
        btn_p2_stop.setStyleSheet("background-color: #ffdddd; border-radius: 6px; padding: 10px; font-size: 16px; border: 2px solid #aa0000;")
        btn_p2_stop.clicked.connect(lambda _: self.ros.send_p2_stop())
        ctrl_layout.addWidget(btn_p2_stop, base_row + len(bool_cmds), 3)
        for i, (label, goal) in enumerate(bool_cmds):
            btn = QPushButton(f"Pinky3: {label}")
            btn.setStyleSheet("background-color: #E1D5E7; border-radius: 6px; padding: 10px; font-size: 16px; border: 1px solid #647687;")
            btn.clicked.connect(lambda _, g=goal: self.ros.send_p3_goal(g))
            ctrl_layout.addWidget(btn, base_row + i, 4)
        btn_p3_stop = QPushButton("Pinky3: STOP")
        btn_p3_stop.setStyleSheet("background-color: #ffdddd; border-radius: 6px; padding: 10px; font-size: 16px; border: 2px solid #aa0000;")
        btn_p3_stop.clicked.connect(lambda _: self.ros.send_p3_stop())
        ctrl_layout.addWidget(btn_p3_stop, base_row + len(bool_cmds), 4)

        coordinator_box = QGroupBox("Coordinator 控制")
        coordinator_box.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px;")
        coordinator_layout = QHBoxLayout(coordinator_box)
        coordinator_layout.setSpacing(10)
        coordinator_layout.setContentsMargins(10, 10, 10, 10)
        self.coordinator_pub = self.ros.create_publisher(Int32, '/coordinator/command', 10)
        def add_coord_btn(label: str, cmd: int):
            btn = QPushButton(label)
            btn.setMinimumHeight(45)
            btn.setStyleSheet("background-color: #D5E8D4; border-radius: 6px; padding: 10px; font-size: 16px; border: 1px solid #647687;")
            btn.clicked.connect(lambda _, c=cmd: self.coordinator_pub.publish(Int32(data=c)))
            coordinator_layout.addWidget(btn)
        add_coord_btn("餐盘装载", 1)
        add_coord_btn("餐盘回收", 2)
        add_coord_btn("暂停", 3)
        add_coord_btn("初始化", 4)

        log_group = QGroupBox("操作日志 (/gui/log)")
        log_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px;")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #FFF2CC; border: 2px solid #647687; border-radius: 6px;")
        log_layout.addWidget(self.log_text)

        coord_log_group = QGroupBox("Coordinator 日志 (/coordinator/gui_log)")
        coord_log_group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 10px; font-size: 18px;")
        coord_log_layout = QVBoxLayout(coord_log_group)
        self.coord_log_text = QTextEdit()
        self.coord_log_text.setReadOnly(True)
        self.coord_log_text.setStyleSheet("background-color: #FFF2CC; border: 2px solid #647687; border-radius: 6px;")
        coord_log_layout.addWidget(self.coord_log_text)

        self.video_labels = {}
        for i, name in enumerate(self.image_names):
            group = QGroupBox(f"{name} ({image_topics[name]})")
            group.setStyleSheet("color: #647687; border: 2px solid #647687; border-radius: 8px; font-size: 14px;")
            group_layout = QVBoxLayout(group)
            label = QLabel(f"{name} 待机中…")
            label.setStyleSheet("background-color: #FFF2CC; border: 2px solid #647687; border-radius: 6px; color: #647687;")
            label.setFixedSize(CELL_W, CELL_H)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            group_layout.addWidget(label)
            row, col = divmod(i, 2)
            left_layout.addWidget(group, row, col)
            self.video_labels[name] = label

        right_layout.addWidget(alert_group)
        right_layout.addWidget(ctrl_group)
        right_layout.addWidget(coordinator_box)
        right_layout.addWidget(log_group)
        right_layout.addWidget(coord_log_group)
        right_layout.addStretch()
        main_layout.addWidget(left_widget, stretch=1)
        main_layout.addWidget(right_widget, stretch=1)

        self._mjpeg_last_imgs = {name: None for name in self.image_names}
        self._mjpeg_readers = {}
        for name, topic in image_topics.items():
            if topic.startswith('http'):
                self._mjpeg_readers[name] = MjpegReader(topic, lambda jpg, n=name: self._on_mjpeg_jpeg(n, jpg))
        self._jet_last_img = None
        self._jet_reader = MjpegReader(JETCOBOT_STREAM_URL, self._on_jet_jpeg)

        self.known_face_encodings = []
        self.known_face_names = []
        self.current_face_locations = []
        self.current_names = []
        self.load_known_faces()
        self.init_pinky3_camera()

        self.log_action("GUI 启动。多相机/位置订阅；/pinkyN/go_to_*, /pinkyN/stop 发布")

        self._udp_rx = UdpNotifyReceiver(port=UDP_PORT)
        self._udp_rx.received.connect(self.alert_action)
        self._udp_rx.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(33)
        self.ros.set_callbacks(
            append_log=self.log_action,
            append_alert=self.alert_action,
            set_pose_text=self._update_pose_ui,
            append_coord_log=self.append_coord_log
        )

    def init_pinky3_camera(self):
        """初始化 Pinky-3 相机"""
        self.cap_pinky3 = cv2.VideoCapture('http://192.168.4.1:8889')
        self.cap_pinky3.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap_pinky3.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap_pinky3.set(cv2.CAP_PROP_AUTO_WB, 1)
        if not self.cap_pinky3.isOpened():
            self.log_action("Pinky-3 相机打开失败")
            self.video_labels["Pinky-3"].setText("Pinky-3: 相机错误")
            self.video_labels["Pinky-3"].setStyleSheet("color: red;")

    def load_known_faces(self):
        """加载已知人脸数据库"""
        folder_path = "/home/lc/Desktop/Project文件开始日期_250702/人脸/known_faces"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            self.log_action(f"创建 known_faces 目录: {folder_path}")
            return

        loaded_count = 0
        self.log_action(f"扫描 known_faces 目录: {folder_path}")
        for person_name in os.listdir(folder_path):
            person_path = os.path.join(folder_path, person_name)
            self.log_action(f"检查目录: {person_path}")
            if os.path.isdir(person_path):
                for filename in os.listdir(person_path):
                    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                        try:
                            image_path = os.path.join(person_path, filename)
                            self.log_action(f"加载图像: {image_path}")
                            image = face_recognition.load_image_file(image_path)
                            image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
                            image = cv2.convertScaleAbs(image, alpha=1.1, beta=5)
                            encodings = face_recognition.face_encodings(image, model='large')
                            if encodings:
                                for encoding in encodings:
                                    self.known_face_encodings.append(encoding)
                                    self.known_face_names.append(person_name)
                                    loaded_count += 1
                                    self.log_action(f"注册成功: {person_name} ({filename})")
                            else:
                                self.log_action(f"未检测到人脸: {filename}")
                        except Exception as e:
                            self.log_action(f"加载失败 {filename}: {e}")
            else:
                self.log_action(f"跳过非目录项: {person_path}")

        self.log_action(f"共加载 {loaded_count} 个人脸")
        if loaded_count == 0:
            self.log_action("未加载到任何人脸")

    def _on_jet_jpeg(self, jpg: bytes):
        img = QImage.fromData(jpg)
        if not img.isNull():
            self._jet_last_img = img

    def _on_mjpeg_jpeg(self, name: str, jpg: bytes):
        img = QImage.fromData(jpg)
        if not img.isNull():
            self._mjpeg_last_imgs[name] = img

    def update_pinky3_frame(self):
        """更新 Pinky-3 帧并进行人脸识别"""
        ret, frame = self.cap_pinky3.read()
        if not ret:
            self.log_action("无法获取 Pinky-3 帧")
            return

        # 垂直翻转帧以纠正倒挂图像
        frame = cv2.flip(frame, 0)

        # 颜色校正
        b, g, r = cv2.split(frame)
        b = cv2.addWeighted(b, 0.9, np.zeros_like(b), 0, 0)
        frame = cv2.merge([b, g, r])
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(frame)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        frame = cv2.merge([l, a, b])
        frame = cv2.cvtColor(frame, cv2.COLOR_LAB2BGR)

        if not hasattr(self, 'frame_count'):
            self.frame_count = 0
        self.frame_count += 1

        if self.frame_count % 4 == 0:
            self.recognition_thread = FaceRecognitionThread(
                frame.copy(), 
                self.known_face_encodings, 
                self.known_face_names
            )
            self.recognition_thread.finished.connect(self._update_pinky3_recognition)
            self.recognition_thread.start()
        else:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pix = QPixmap.fromImage(qimg.scaled(CELL_W, CELL_H, Qt.AspectRatioMode.KeepAspectRatio))
            self.video_labels["Pinky-3"].setPixmap(pix)

    def _update_pinky3_recognition(self, face_locations, names, frame):
        """更新 Pinky-3 人脸识别结果并触发躲避"""
        self.current_face_locations = face_locations
        self.current_names = names
        
        if "Unknown" in names:
            self.alert_action(f"P既定目标: Pinky-3: 检测到未经授权人员: {names}")
            self.ros.send_p3_stop()  # 检测到未知人员时发布停止命令
            self.log_action("Pinky-3: 检测到未经授权人员，发布 /pinky3/stop 命令")
        elif names:
            self.alert_action(f"Pinky-3: 识别到: {names}")

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(qimg.scaled(CELL_W, CELL_H, Qt.AspectRatioMode.KeepAspectRatio))
        self.video_labels["Pinky-3"].setPixmap(pix)

    def _on_timer(self):
        rclpy.spin_once(self.ros, timeout_sec=0.0)
        if self._jet_last_img is not None:
            pix = QPixmap.fromImage(
                self._jet_last_img.scaled(
                    self.image_jetcobot.width(),
                    self.image_jetcobot.height(),
                    Qt.AspectRatioMode.KeepAspectRatio
                )
            )
            self.image_jetcobot.setPixmap(pix)

        if "Pinky-3" in self.image_names:
            self.update_pinky3_frame()

        for name in self.image_names:
            if name == "Pinky-3":
                continue
            lbl = self.video_labels.get(name)
            if not lbl:
                continue
            frame_qimg = None
            if name in self._mjpeg_last_imgs and self._mjpeg_last_imgs[name] is not None:
                frame_qimg = self._mjpeg_last_imgs[name]
            elif self.ros.latest_qimages.get(name) is not None:
                frame_qimg = self.ros.latest_qimages[name]
            if frame_qimg is not None:
                pix = QPixmap.fromImage(frame_qimg.scaled(CELL_W, CELL_H, Qt.AspectRatioMode.KeepAspectRatio))
                lbl.setPixmap(pix)

    def append_coord_log(self, message: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.coord_log_text.append(f"[Coordinator] [{ts}] {message}")

    def log_action(self, message: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_text.append(f"[{ts}] {message}")

    def alert_action(self, message: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.alert_text.append(f"[{ts}] {message}")

    def _update_pose_ui(self, _incoming_text: str):
        def fmt(p):
            return f"x={p[0]:.2f}, y={p[1]:.2f}, yaw={p[2]:.1f}°"
        p1 = self.ros.latest_poses.get("Pinky-1")
        p2 = self.ros.latest_poses.get("Pinky-2")
        p3 = self.ros.latest_poses.get("Pinky-3")
        self.pose_p1.setText(f"Pinky-1: {fmt(p1)}" if p1 else "Pinky-1: 未检测")
        self.pose_p2.setText(f"Pinky-2: {fmt(p2)}" if p2 else "Pinky-2: 未检测")
        self.pose_p3.setText(f"Pinky-3: {fmt(p3)}" if p3 else "Pinky-3: 未检测")

    def closeEvent(self, event):
        try:
            if hasattr(self, "_mjpeg_readers"):
                for r in self._mjpeg_readers.values():
                    try:
                        r.stop()
                    except Exception:
                        pass
            if hasattr(self, "_jet_reader") and self._jet_reader:
                self._jet_reader.stop()
            if hasattr(self, 'cap_pinky3') and self.cap_pinky3.isOpened():
                self.cap_pinky3.release()
            self.ros.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)

def main():
    rclpy.init(args=None)
    ros = RosBridge(IMAGE_TOPICS, POSE_TOPICS)
    app = QApplication(sys.argv)
    w = GUI(ros, IMAGE_TOPICS)
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
