#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool
import threading
import time
import sys


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        # QoS
        self.qos1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 상태
        self.current_step = None
        self.step_list = []
        self.idx = 0

        self.a_ready = -1
        self.b_ready = -1
        self.a_done = -1
        self.b_done = -1
        self.waiting_ready = False

        self.last_step_sent = 0.0
        self.last_go_sent = 0.0
        self.go_sent_once = False

        self._paused = False
        self._pause_lock = threading.Lock()
        self._menu_thread = None
        self._menu_start_evt = threading.Event()
        self._menu_alive = False
        self._cmd_thread_started = False
        self._in_menu = False

        # GUI 일시정지 상태 대응용
        self._pause_input_waiting = False

        # ─── 구독 ───
        self.create_subscription(Int32, '/a_1_done', self.cb_a_done, self.qos1)
        self.create_subscription(Int32, '/b_1_done', self.cb_b_done, self.qos1)
        self.create_subscription(Int32, '/a_ready_step', self.cb_a_ready, self.qos1)
        self.create_subscription(Int32, '/b_ready_step', self.cb_b_ready, self.qos1)
        self.create_subscription(Int32, '/coordinator/command', self.cb_command, self.qos1)

        # ─── 발행 ───
        self.pub_a = self.create_publisher(Int32, '/robot_a_1_node', self.qos1)
        self.pub_b = self.create_publisher(Int32, '/robot_b_1_node', self.qos1)
        self.pub_go = self.create_publisher(Int32, '/go_step', self.qos1)

        self.pub_a_pause      = self.create_publisher(Bool, '/a_pause', self.qos1)
        self.pub_b_pause      = self.create_publisher(Bool, '/b_pause', self.qos1)
        self.pub_a_move_ready = self.create_publisher(Bool, '/a_move_ready', self.qos1)
        self.pub_b_move_ready = self.create_publisher(Bool, '/b_move_ready', self.qos1)

        self.create_timer(0.2, self.tick)

        self._ensure_menu_thread()
        self._menu_start_evt.set()

        if not self._cmd_thread_started:
            self._cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
            self._cmd_thread.start()
            self._cmd_thread_started = True

    def cb_command(self, msg):
        cmd = int(msg.data)

        if self._pause_input_waiting:
            if cmd == 3:
                self._unpause_from_any_source()
            elif cmd == 4:
                self._reset_from_any_source()
            return

        if cmd == 1:
            self.select_function(1)
        elif cmd == 2:
            self.select_function(2)
        elif cmd == 3:
            self.trigger_pause()
        elif cmd == 4:
            self._reset_from_any_source()
        elif cmd == 9:
            self.get_logger().info("🛑 GUI 종료 명령 수신")
            rclpy.shutdown()
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 명령: {cmd}")

    def trigger_pause(self):
        with self._pause_lock:
            if self._paused:
                print("⏸ 이미 일시정지 상태입니다.")
                return
            self._paused = True

        self.get_logger().warn("⏸ 일시정지 요청")
        self.pulse_bool(self.pub_a_pause)
        self.pulse_bool(self.pub_b_pause)
        self.pulse_bool(self.pub_a_move_ready)
        self.pulse_bool(self.pub_b_move_ready)

        self.get_logger().info("3=계속 / 4=초기화 명령 대기 중 (GUI 또는 콘솔 입력 가능)")
        self._pause_input_waiting = True

        threading.Thread(target=self._wait_console_input, daemon=True).start()

    def _wait_console_input(self):
        while self._pause_input_waiting:
            try:
                ans = input("3=계속 / 4=초기화 ? ").strip()
            except EOFError:
                time.sleep(0.1)
                continue

            if ans == '3':
                self._unpause_from_any_source()
                break
            elif ans == '4':
                self._reset_from_any_source()
                break
            else:
                print("⚠️ '3' 또는 '4'만 입력하세요.")

    def _unpause_from_any_source(self):
        with self._pause_lock:
            self._paused = False
        self._pause_input_waiting = False
        self.get_logger().info("▶ 계속 진행")

    def _reset_from_any_source(self):
        with self._pause_lock:
            self._paused = False
        self.current_step = None
        self.step_list = []
        self.idx = 0
        self.a_ready = self.b_ready = -1
        self.a_done = self.b_done = -1
        self.waiting_ready = False
        self.last_step_sent = 0.0
        self.last_go_sent = 0.0
        self.go_sent_once = False
        self._pause_input_waiting = False
        self._menu_start_evt.set()
        self.get_logger().info("🔄 초기화: 메뉴로 복귀")

    def pulse_bool(self, pub, ms: int = 250, value: bool = True):
        pub.publish(Bool(data=value))
        threading.Timer(ms / 1000.0, lambda: pub.publish(Bool(data=not value))).start()

    def _ensure_menu_thread(self):
        if self._menu_alive:
            return
        def menu_loop():
            self._menu_alive = True
            while rclpy.ok():
                self._menu_start_evt.wait()
                self._menu_start_evt.clear()
                self._choose_function_blocking()
        self._menu_thread = threading.Thread(target=menu_loop, daemon=True)
        self._menu_thread.start()

    def _choose_function_blocking(self):
        self._in_menu = True
        try:
            print("\n🌟 기능을 선택하세요:")
            print("  1. 식판 적재 (101~117)")
            print("  2. 식판 회수 (201~214)")
            print("  3. (실행 중 언제든) 일시정지")
            while True:
                try:
                    option = int(input("▶ 기능 번호 입력: "))
                    if option in (1, 2):
                        self.select_function(option)
                        break
                    elif option == 3:
                        self.trigger_pause()
                        continue
                    else:
                        print("⚠️ 1, 2, 또는 3만 입력하세요.")
                except Exception:
                    print("⚠️ 숫자를 입력하세요.")
            print("ℹ️ 실행 중 콘솔 입력: 1=적재, 2=회수, 3=일시정지, q=종료")
        finally:
            self._in_menu = False

    def _cmd_loop(self):
        print("⌨️ 런타임 명령: 1=적재, 2=회수, 3=일시정지, q=종료")
        while rclpy.ok():
            if self._in_menu:
                time.sleep(0.1)
                continue
            try:
                s = sys.stdin.readline()
                if not s:
                    time.sleep(0.05)
                    continue
                s = s.strip()
            except Exception:
                break

            if s == '1':
                self.select_function(1)
            elif s == '2':
                self.select_function(2)
            elif s == '3':
                self.trigger_pause()
            elif s == 'q':
                print("👋 종료 요청")
                rclpy.shutdown()
                break

    def select_function(self, option: int):
        if option == 1:
            new_steps = list(range(101, 118))
            name = "식판 적재"
        elif option == 2:
            new_steps = list(range(201, 215))
            name = "식판 회수"
        else:
            self.get_logger().warn("⚠️ select_function: 잘못된 옵션")
            return

        with self._pause_lock:
            self._paused = True
        self.pulse_bool(self.pub_a_pause)
        self.pulse_bool(self.pub_b_pause)
        self.pulse_bool(self.pub_a_move_ready)
        self.pulse_bool(self.pub_b_move_ready)
        time.sleep(0.35)
        self.pub_a_pause.publish(Bool(data=False))
        self.pub_b_pause.publish(Bool(data=False))

        self.current_step = None
        self.step_list = new_steps
        self.idx = 0
        self.a_ready = self.b_ready = -1
        self.a_done = self.b_done = -1
        self.waiting_ready = False
        self.last_step_sent = 0.0
        self.last_go_sent = 0.0
        self.go_sent_once = False

        with self._pause_lock:
            self._paused = False

        self.get_logger().info(f"🔁 기능 전환: {name} 시작")

        time.sleep(1.0)
        self.start_step()

    def start_step(self):
        self.current_step = self.step_list[self.idx]
        self.a_ready = self.b_ready = -1
        self.a_done = self.b_done = -1
        self.waiting_ready = True
        self.go_sent_once = False
        self.send_step()
        self._maybe_send_go()

    def send_step(self):
        msg = Int32(data=int(self.current_step))
        self.pub_a.publish(msg)
        self.pub_b.publish(msg)
        self.last_step_sent = time.time()
        self.get_logger().info(f"📤 STEP {self.current_step} 전송 → A, B")

    def cb_a_ready(self, msg):
        step = int(msg.data)
        cur = self.current_step if self.current_step is not None else -1
        if step >= cur:
            self.a_ready = step
            self._maybe_send_go()

    def cb_b_ready(self, msg):
        step = int(msg.data)
        cur = self.current_step if self.current_step is not None else -1
        if step >= cur:
            self.b_ready = step
            self._maybe_send_go()

    def cb_a_done(self, msg):
        self.a_done = int(msg.data)
        self.get_logger().info(f"✅ A DONE: {self.a_done}")

    def cb_b_done(self, msg):
        self.b_done = int(msg.data)
        self.get_logger().info(f"✅ B DONE: {self.b_done}")

    def _maybe_send_go(self):
        if self.current_step is None or not self.waiting_ready:
            return
        if (self.a_ready >= self.current_step) and (self.b_ready >= self.current_step):
            if not self.go_sent_once:
                self.pub_go.publish(Int32(data=int(self.current_step)))
                self.last_go_sent = time.time()
                self.go_sent_once = True
                self.get_logger().info(f"🚦 GO {self.current_step} 발행")
            self.waiting_ready = False

    def tick(self):
        if self.current_step is None:
            return
        with self._pause_lock:
            if self._paused:
                return

        now = time.time()

        if self.waiting_ready:
            self._maybe_send_go()
            if not self.waiting_ready:
                return
            if now - self.last_step_sent > 0.5:
                self.send_step()
            return

        if self.a_done == self.current_step and self.b_done == self.current_step:
            self.get_logger().info(f"🎉 STEP {self.current_step} 완료")
            self.idx += 1
            if self.idx >= len(self.step_list):
                self.get_logger().info("🏁 모든 STEP 완료 → 기능 선택으로 복귀")
                self.current_step = None
                self._menu_start_evt.set()
            else:
                self.start_step()


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[중단됨]")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
