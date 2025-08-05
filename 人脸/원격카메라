替换摄像头初始化：
原代码：
python

Collapse


self.cap = cv2.VideoCapture(0)
修改为从树莓派视频流获取（如使用MJPEG或RTSP流）：

self.cap = cv2.VideoCapture('http://<树莓派IP>:8081')  # 替换为树莓派IP和端口
需确保树莓派上运行了视频流服务器（如motion或Flask）。
树莓派端配置：
安装motion或类似工具：

sudo apt install motion
sudo nano /etc/motion/motion.conf

设置stream_port 8081和stream_localhost off，然后启动：
sudo service motion start
或使用Flask/Picamera2创建流服务器（CSI摄像头）：


from flask import Flask, Response
from picamera2 import Picamera2
app = Flask(__name__)
picam2 = Picamera2()
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)


网络和性能优化：
确保树莓派和运行代码的设备在同一网络，或配置端口转发。
降低分辨率或帧率以减少延迟（如self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)）。
原代码中frame_count % 4逻辑可保留，以降低CPU负载。
无需修改的部分：
人脸识别逻辑（FaceRecognitionThread）、UI显示（display_frame）等无需更改，只要确保输入帧格式兼容。
总结：主要修改init_camera方法，使用远程视频流URL替换本地摄像头索引。树莓派需运行流服务器。其余逻辑可保持不变，但需测试网络延迟和性能。

