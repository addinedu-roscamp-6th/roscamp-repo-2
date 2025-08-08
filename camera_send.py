from picamera2 import Picamera2
from http.server import HTTPServer, BaseHTTPRequestHandler
import time
import cv2  # 缺失的导入

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (1280, 720)}))
picam2.start()

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
        self.end_headers()
        try:
            while True:
                frame = picam2.capture_array()
                _, img = cv2.imencode('.jpg', frame)
                self.wfile.write(b'--jpgboundary\r\n')
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-length', str(len(img)))
                self.end_headers()
                self.wfile.write(img.tobytes())
                self.wfile.write(b'\r\n')
                time.sleep(0.05)  # 可调节帧率
        except BrokenPipeError:
            print("客户端断开连接")
        except Exception as e:
            print(f"发生错误: {e}")

server = HTTPServer(('192.168.4.1', 8889), CamHandler)

server.serve_forever()
