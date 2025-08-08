from pinkylib import Motor, Ultrasonic
import time

# 初始化
motor = Motor()
ultrasonic = Ultrasonic()

motor.enable_motor()
motor.start_motor()

start_time = time.time()
duration = 60  # 最长运行时间（秒）

try:
    while time.time() - start_time < duration:
        dist = ultrasonic.get_dist()
        print(f"[INFO] 当前距离: {dist:.2f} cm")

        if dist > 10:
            # 前方安全，继续前进
            motor.move(50, 50)
        else:
            print("[WARN] 障碍物检测到，开始旋转寻找新方向...")
            motor.stop()
            time.sleep(0.5)

            found_path = False
            # 执行原地旋转，共扫描360度（例如每次转45度，总共扫描8次）
            for i in range(8):
                print(f"[SCAN] 扫描方向 {i+1}/8")
                motor.move(40, -40)  # 左轮前进，右轮后退，原地转动
                time.sleep(0.5)       # 旋转一段时间
                motor.stop()
                time.sleep(0.4)

                new_dist = ultrasonic.get_dist()
                print(f"[SCAN] 新方向距离: {new_dist:.2f} cm")

                if new_dist > 20:  # 找到安全路径
                    print("[OK] 找到无障碍方向，继续前进")
                    found_path = True
                    break

            if not found_path:
                print("[FAIL] 未找到安全方向，停止")
                motor.stop()
                break

        time.sleep(0.1)

finally:
    # 停止与清理
    print("[SYSTEM] 程序结束，清理资源")
    motor.stop()
    motor.disable_motor()
    motor.stop_motor()
    motor.clean()
    ultrasonic.clean()
