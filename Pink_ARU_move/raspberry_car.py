import socket
import json
from pinkylib import Motor
import logging

# 配置日志
logging.basicConfig(filename='car_control.log', level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')

HOST = '192.168.4.1'  # Raspberry Pi IP
PORT = 8899

# 初始化socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    logging.info("Server started, waiting for connection...")
    print("Waiting for PC connection...")
    conn, addr = server_socket.accept()
    logging.info(f"Connected to PC: {addr}")
    print(f"Connected to PC: {addr}")
except socket.error as e:
    logging.error(f"Socket setup failed: {e}")
    print(f"Socket setup failed: {e}")
    exit(1)

# 初始化电机
pinky = Motor()
pinky.enable_motor()
pinky.start_motor()

try:
    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                logging.warning("Connection closed by PC")
                break
            try:
                command = json.loads(data)
            except json.JSONDecodeError:
                logging.error("Invalid JSON received")
                print("Invalid JSON received")
                continue
            
            if command["command"] == "heartbeat":
                logging.info("Received heartbeat")
                continue
            elif command["command"] == "move":
                left_pwm = command["speed_x"]
                right_pwm = command["speed_y"]
                pinky.move(left_pwm, right_pwm)
                logging.info(f"Executing move: left_pwm={left_pwm:.1f}, right_pwm={right_pwm:.1f}")
                print(f"Executing move: left_pwm={left_pwm:.1f}, right_pwm={right_pwm:.1f}")
            elif command["command"] == "stop":
                pinky.stop()
                logging.info("Car stopped")
                print("Car stopped")
        except socket.error as e:
            logging.error(f"Socket error: {e}")
            print(f"Socket error: {e}")
            break

finally:
    pinky.stop()
    pinky.disable_motor()
    pinky.clean()
    conn.close()
    server_socket.close()
    logging.info("Connection closed, resources cleaned up")
    print("Connection closed, resources cleaned up")