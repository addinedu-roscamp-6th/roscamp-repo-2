#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

def main():
    # 替换为你的树莓派实际 IP 和端口
    url = "http://192.168.4.1:8889"

    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("无法连接到视频流，请检查服务器是否运行以及IP/端口是否正确")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取失败，尝试重新连接...")
            break

        cv2.imshow("Raspberry Pi Camera Stream", frame)

        # 按 q 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
