import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import json
import tkinter as tk
from threading import Thread, Lock
import time
import math
import logging

# 配置日志
logging.basicConfig(filename='aruco_control.log', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

# 常量定义
CAMERA_MATRIX = np.array([
    [284.03, 0, 960],
    [0, 282.317, 540],
    [0, 0, 1]
])
DIST_COEFFS = np.array([-1.06489, 4.85019, 0.01332, 0.00156, -9.18456])
HOST = '192.168.4.1'
PORT = 8899
GRID_ROWS = 8
GRID_COLS = 16
MARKER_SIZE = 50.0  # mm

class ArucoCarControl:
    def __init__(self, root):
        self.root = root
        self.root.title("ArUco Car Control")
        
        # 初始化状态变量
        self.running = True
        self.lock = Lock()
        self.car_position = None
        self.car_heading = None
        self.target = None
        self.marker_coords = {}
        self.scale_x, self.scale_y = 1.0, 1.0
        self.corner_markers = {20: (0, 0), 21: (0, GRID_ROWS-1), 
                              22: (GRID_COLS-1, GRID_ROWS-1), 23: (GRID_COLS-1, 0)}
        
        # 先定义会被UI调用的方法
        self.stop_car = self._stop_car
        
        # 设置UI界面
        self.setup_ui()
        
        # 初始化网络连接
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((HOST, PORT))
            logging.info("Connected to Raspberry Pi")
            self.safe_update_label(self.status_label, "Status: Connected")
        except socket.error as e:
            self.safe_update_label(self.status_label, "Error: Connection failed")
            logging.error(f"Connection error: {str(e)}")
            return
        
        # 初始化ArUco检测
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # 初始化标记
        self.initialize_markers()
        if len(self.marker_coords) == 4:
            Thread(target=self.video_loop, daemon=True).start()
        else:
            self.safe_update_label(self.status_label, "Error: Need 4 corner markers")
            self.running = False

    def setup_ui(self):
        """设置用户界面"""
        self.status_label = tk.Label(self.root, text="Status: Initializing...")
        self.status_label.grid(row=0, column=0, columnspan=2, pady=5)
        
        self.position_label = tk.Label(self.root, text="Car position: (Unknown, Unknown)")
        self.position_label.grid(row=1, column=0, columnspan=2, pady=5)
        
        tk.Label(self.root, text="Target X (0-15):").grid(row=2, column=0)
        self.x_entry = tk.Entry(self.root)
        self.x_entry.grid(row=2, column=1)
        
        tk.Label(self.root, text="Target Y (0-7):").grid(row=3, column=0)
        self.y_entry = tk.Entry(self.root)
        self.y_entry.grid(row=3, column=1)
        
        tk.Button(self.root, text="Go", command=self.move_to_target).grid(row=4, column=0)
        tk.Button(self.root, text="Stop", command=self.stop_car).grid(row=4, column=1)
        
        # 绑定窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _stop_car(self):
        """停止小车运动"""
        self.send_command("stop", 0, 0)
        with self.lock:
            self.target = None
        self.safe_update_label(self.status_label, "Stopped")

    def send_command(self, command, speed_x, speed_y):
        """发送控制命令给小车"""
        try:
            data = {"command": command, "speed_x": int(speed_x), "speed_y": int(speed_y)}
            self.client_socket.send(json.dumps(data).encode())
        except Exception as e:
            logging.error(f"Send command error: {str(e)}")

    def safe_update_label(self, label, text):
        """线程安全的标签更新"""
        def _update():
            if self.running and tk._default_root:
                try:
                    label.config(text=text)
                except tk.TclError:
                    pass
        self.root.after(0, _update)

    def initialize_markers(self):
        """检测并初始化四个角点标记"""
        start_time = time.time()
        while time.time() - start_time < 10 and len(self.marker_coords) < 4:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for i, id_val in enumerate(ids):
                    if id_val[0] in self.corner_markers:
                        corner = corners[i][0]
                        self.marker_coords[id_val[0]] = (int(corner[:, 0].mean()), int(corner[:, 1].mean()))
            
            time.sleep(0.1)
        
        if len(self.marker_coords) == 4:
            x_coords = [self.marker_coords[i][0] for i in self.corner_markers]
            y_coords = [self.marker_coords[i][1] for i in self.corner_markers]
            self.scale_x = (max(x_coords) - min(x_coords)) / (GRID_COLS - 1)
            self.scale_y = (max(y_coords) - min(y_coords)) / (GRID_ROWS - 1)
            self.safe_update_label(self.status_label, "Status: Ready")

    def pixel_to_grid(self, px, py):
        """将像素坐标转换为网格坐标"""
        if not all(i in self.marker_coords for i in [20, 21, 22, 23]):
            return 0, 0
            
        min_x = min(self.marker_coords[20][0], self.marker_coords[23][0])
        min_y = min(self.marker_coords[20][1], self.marker_coords[21][1])
        
        return (max(0, min(GRID_COLS-1, int((px - min_x) / self.scale_x))),
                max(0, min(GRID_ROWS-1, int((py - min_y) / self.scale_y))))

    def grid_to_pixel(self, grid_x, grid_y):
        """将网格坐标转换为像素坐标"""
        if not all(i in self.marker_coords for i in [20, 21, 22, 23]):
            return 0, 0
            
        min_x = min(self.marker_coords[20][0], self.marker_coords[23][0])
        min_y = min(self.marker_coords[20][1], self.marker_coords[21][1])
        
        return (min_x + grid_x * self.scale_x,
                min_y + grid_y * self.scale_y)

    def angle_between(self, v1, v2):
        """计算两个向量之间的角度"""
        unit_v1 = v1 / (np.linalg.norm(v1) + 1e-6)
        unit_v2 = v2 / (np.linalg.norm(v2) + 1e-6)
        dot = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
        angle = np.arccos(dot)
        cross = np.cross(unit_v1, unit_v2)
        return np.degrees(angle) * np.sign(cross)

    def video_loop(self):
        """主视频处理循环"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for i, id_val in enumerate(ids):
                    if id_val[0] == 25:  # 小车标记ID
                        corner = corners[i][0]
                        cx, cy = int(corner[:, 0].mean()), int(corner[:, 1].mean())
                        heading = np.array([corner[0][0] - corner[1][0], corner[0][1] - corner[1][1]])
                        
                        with self.lock:
                            self.car_position = (cx, cy)
                            self.car_heading = heading
                            grid_x, grid_y = self.pixel_to_grid(cx, cy)
                            self.safe_update_label(self.position_label, f"Car position: ({grid_x}, {grid_y})")
                        
                        # 可视化
                        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                        cv2.putText(frame, f"{grid_x},{grid_y}", (cx + 10, cy), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            cv2.imshow("Aruco Detection", cv2.resize(frame, (960, 540)))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        self.cleanup()

    def move_to_target(self):
        """处理移动到目标位置的命令"""
        try:
            x = int(self.x_entry.get())
            y = int(self.y_entry.get())
            if 0 <= x < GRID_COLS and 0 <= y < GRID_ROWS:
                with self.lock:
                    self.target = (x, y)
                Thread(target=self.navigate, daemon=True).start()
                self.safe_update_label(self.status_label, f"Moving to: ({x}, {y})")
            else:
                self.safe_update_label(self.status_label, "Error: Coordinates out of range")
        except ValueError:
            self.safe_update_label(self.status_label, "Error: Invalid input")








    def navigate(self):
        """改进后的导航算法，解决摇晃问题"""
        # 优化后的控制参数
        BASE_SPEED = 20       # 降低基础速度
        MIN_SPEED = 10        # 最小速度
        TURN_GAIN = 0.3       # 降低转向增益
        ANGLE_TOLERANCE = 5   # 角度容差(度)
        DISTANCE_TOLERANCE = 25 # 距离容差(像素)
        SLOWDOWN_START = 200  # 开始减速的距离(像素)
        STOP_TIME = 1.0       # 到达后确认时间(秒)
        DAMPING_FACTOR = 0.7  # 阻尼系数
        
        # 状态变量
        last_angle_diff = 0
        stop_start_time = None
        
        while self.running and self.target:
            with self.lock:
                if self.car_position is None or self.car_heading is None:
                    time.sleep(0.1)
                    continue
                    
                # 获取当前位置和目标位置
                cx, cy = self.car_position
                heading = self.car_heading
                tx, ty = self.target
                
                # 转换为像素坐标
                px_target, py_target = self.grid_to_pixel(tx, ty)
                target_vec = np.array([px_target - cx, py_target - cy])
                distance = np.linalg.norm(target_vec)
                
                # 到达判定
                if distance < DISTANCE_TOLERANCE:
                    if stop_start_time is None:
                        stop_start_time = time.time()
                    elif time.time() - stop_start_time > STOP_TIME:
                        self._stop_car()
                        self.safe_update_label(self.status_label, f"精确到达: ({tx}, {ty})")
                        break
                    continue
                
                # 计算角度差
                angle_diff = self.angle_between(heading, target_vec)
                
                # 添加阻尼项 (减少振荡)
                damping = DAMPING_FACTOR * (angle_diff - last_angle_diff)
                last_angle_diff = angle_diff
                
                # 动态速度控制
                if distance < SLOWDOWN_START:
                    speed = BASE_SPEED * (0.3 + 0.7 * (distance / SLOWDOWN_START))
                else:
                    speed = BASE_SPEED
                speed = max(MIN_SPEED, speed)
                
                # 转向控制 (更平滑)
                if abs(angle_diff) > ANGLE_TOLERANCE:
                    turn = TURN_GAIN * (angle_diff + damping)
                    turn = np.clip(turn, -30, 30)  # 限制最大转向
                else:
                    turn = 0
                
                # 计算电机PWM值
                left_pwm = int(np.clip(speed - turn, -50, 50))
                right_pwm = int(np.clip(speed + turn, -50, 50))
                
                # 发送移动命令
                self.send_command("move", left_pwm, right_pwm)
            
            time.sleep(0.1)






    def on_closing(self):
        """处理窗口关闭事件"""
        self.running = False
        time.sleep(0.2)  # 给线程时间退出
        self.root.destroy()

    def cleanup(self):
        """清理资源"""
        self.running = False
        self._stop_car()
        
        try:
            if hasattr(self, 'client_socket') and self.client_socket:
                self.client_socket.close()
        except:
            pass
            
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
        except:
            pass
            
        cv2.destroyAllWindows()

if __name__ == "__main__":
    root = tk.Tk()
    app = ArucoCarControl(root)
    try:
        root.mainloop()
    finally:
        app.cleanup()