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

        # READY/DONE
        self.a_ready = -1
        self.b_ready = -1
        self.a_done = -1
        self.b_done = -1
        self.waiting_ready = False

        # 타임스탬프
        self.last_step_sent = 0.0
        self.last_go_sent = 0.0
        self.go_sent_once = False

        # 일시정지/입력 스레드 상태
        self._paused = False
        self._pause_lock = threading.Lock()

        self._menu_thread = None
        self._menu_start_evt = threading.Event()
        self._menu_alive = False
        self._cmd_thread_started = False
        self._in_menu = False          # ← 입력 경쟁 방지용 플래그

        # 구독
        self.create_subscription(Int32, '/a_1_done', self.cb_a_done, self.qos1)
        self.create_subscription(Int32, '/b_1_done', self.cb_b_done, self.qos1)
        self.create_subscription(Int32, '/a_ready_step', self.cb_a_ready, self.qos1)
        self.create_subscription(Int32, '/b_ready_step', self.cb_b_ready, self.qos1)

        # 발행
        self.pub_a = self.create_publisher(Int32, '/robot_a_1_node', self.qos1)
        self.pub_b = self.create_publisher(Int32, '/robot_b_1_node', self.qos1)
        self.pub_go = self.create_publisher(Int32, '/go_step', self.qos1)

        self.pub_a_pause      = self.create_publisher(Bool, '/a_pause', self.qos1)
        self.pub_b_pause      = self.create_publisher(Bool, '/b_pause', self.qos1)
        self.pub_a_move_ready = self.create_publisher(Bool, '/a_move_ready', self.qos1)
        self.pub_b_move_ready = self.create_publisher(Bool, '/b_move_ready', self.qos1)

        # 타이머
        self.create_timer(0.2, self.tick)

        # 메뉴/커맨드 스레드 기동
        self._ensure_menu_thread()
        self._menu_start_evt.set()  # 최초 메뉴 표시

        if not self._cmd_thread_started:
            self._cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
            self._cmd_thread.start()
            self._cmd_thread_started = True

    # ─────────────────────────────────────────
    # 유틸: Bool 펄스
    def pulse_bool(self, pub, ms: int = 250, value: bool = True):
        pub.publish(Bool(data=value))
        threading.Timer(ms / 1000.0, lambda: pub.publish(Bool(data=not value))).start()

    # ─────────────────────────────────────────
    # 메뉴 스레드
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
        # 메뉴 시작: 커맨드 스레드가 stdin을 읽지 못하게 막음
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
            # 메뉴 종료: 커맨드 스레드 읽기 허용
            self._in_menu = False

    # ─────────────────────────────────────────
    # 런타임 커맨드(입력 경쟁 방지)
    def _cmd_loop(self):
        print("⌨️ 런타임 명령: 1=적재, 2=회수, 3=일시정지, q=종료 (기타 입력은 무시)")
        while rclpy.ok():
            if self._in_menu:
                time.sleep(0.1)
                continue  # 메뉴가 열려 있으면 절대 stdin을 읽지 않음
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
            else:
                pass

    # ─────────────────────────────────────────
    # 기능 전환
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

        # 안전 정지/준비자세 (펄스)
        with self._pause_lock:
            self._paused = True
        self.pulse_bool(self.pub_a_pause)
        self.pulse_bool(self.pub_b_pause)
        self.pulse_bool(self.pub_a_move_ready)
        self.pulse_bool(self.pub_b_move_ready)
        time.sleep(0.35)
        self.pub_a_pause.publish(Bool(data=False))
        self.pub_b_pause.publish(Bool(data=False))

        # 내부 상태 초기화
        self.current_step = None
        self.step_list = new_steps
        self.idx = 0
        self.a_ready = self.b_ready = -1
        self.a_done = self.b_done = -1
        self.waiting_ready = False
        self.last_step_sent = 0.0
        self.last_go_sent = 0.0
        self.go_sent_once = False

        # 재개
        with self._pause_lock:
            self._paused = False

        self.get_logger().info(f"🔁 기능 전환: {name} ({self.step_list[0]}~{self.step_list[-1]}) 시작")

        # 준비자세 안정화 후 시작
        time.sleep(1.0)
        self.start_step()

    # ─────────────────────────────────────────
    # STEP 전개
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

    # ─────────────────────────────────────────
    # 콜백
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

    # ─────────────────────────────────────────
    # GO 발행 (READY 충족 시 1회)
    def _maybe_send_go(self):
        if self.current_step is None or not self.waiting_ready:
            return
        if (self.a_ready >= self.current_step) and (self.b_ready >= self.current_step):
            if not self.go_sent_once:
                self.pub_go.publish(Int32(data=int(self.current_step)))
                self.last_go_sent = time.time()
                self.go_sent_once = True
                self.get_logger().info(f"🚦 GO {self.current_step} 발행(조건 충족, 1회)")
            self.waiting_ready = False

    # ─────────────────────────────────────────
    # 일시정지
    def trigger_pause(self):
        with self._pause_lock:
            if self._paused:
                print("⏸ 이미 일시정지 상태입니다.")
                return
            self._paused = True

        self.get_logger().warn("⏸ 일시정지 요청: 로봇 정지 및 준비자세 이동 지시")
        self.pulse_bool(self.pub_a_pause)
        self.pulse_bool(self.pub_b_pause)
        self.pulse_bool(self.pub_a_move_ready)
        self.pulse_bool(self.pub_b_move_ready)

        while True:
            ans = input("3=계속 / 4=초기화 ? ").strip()
            if ans == '3':
                with self._pause_lock:
                    self._paused = False
                self.get_logger().info("▶ 계속 진행합니다.")
                break
            elif ans == '4':
                self.get_logger().info("🔄 초기화: 처음 메뉴로 돌아갑니다.")
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
                # 메뉴 다시 띄우기
                self._menu_start_evt.set()
                break
            else:
                print("⚠️ '3' 또는 '4'만 입력하세요.")

    # ─────────────────────────────────────────
    # 메인 루프
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
            self.get_logger().info(f"🎉 STEP {self.current_step} 모두 완료")
            self.idx += 1
            if self.idx >= len(self.step_list):
                self.get_logger().info("🏁 모든 STEP 완료. 기능 선택으로 돌아갑니다.")
                self.current_step = None
                # 콜백 스레드에서 input()을 절대 호출하지 않음
                self._menu_start_evt.set()   # 메뉴 스레드에게 알림
            else:
                self.start_step()
            return
        # GO 주기 재전송 없음


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
