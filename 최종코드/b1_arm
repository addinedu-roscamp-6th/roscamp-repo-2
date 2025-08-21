import time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool
from pymycobot.mycobot280 import MyCobot280

class RobotB1Node(Node):
    def __init__(self):
        super().__init__('robot_b_1_node')

        # QoS
        self.qos1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # HW
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.get_logger().info("🤖 JetCobot B 연결 완료")

        # 상태(기존)
        self.step_running = threading.Event()
        self.waiting_barrier = False
        self.executing_step = None
        self.pending_step = None
        self.last_step_time = None
        self.worker_thread = None
        self.step_done = False

        # ───────── 추가: 일시정지/복구 상태 + 체크포인트 ─────────
        self._pause_event = threading.Event()
        self._resume_lock = threading.Lock()
        self._last_cmd = None      # ("angles", angles, speed, None) 등
        self._need_resume = False

        # ───────── 추가: 속도/모드 & 포즈 상수 ─────────
        self.FAST_SPEED = 50
        self.SLOW_SPEED = 12
        self.BLEND_MODE = 1
        self.READY_POSE = [0, 0, -70, -20, 0, 45, 100]  # A와 동일
        # ★ 초기화(리셋) 시 이동할 "제로 포즈" (관절 모두 0도, 그리퍼 100)
        #    나중에 바꾸려면 이 줄만 수정하세요.
        self.RESET_ZERO_POSE = [0, 0, -70, -20, 0, 45, 100]

        # Pub/Sub (기존 + 추가)
        self.done_pub = self.create_publisher(Int32, '/b_1_done', self.qos1)
        self.ready_pub = self.create_publisher(Int32, '/b_ready_step', self.qos1)
        self.create_subscription(Int32, '/robot_b_1_node', self.step_callback, self.qos1)
        self.create_subscription(Int32, '/go_step', self.go_callback, self.qos1)

        # ───────── 추가: 일시정지/준비자세/리셋 제어 토픽 ─────────
        self.create_subscription(Bool, '/b_pause', self.pause_callback, self.qos1)
        self.create_subscription(Bool, '/b_move_ready', self.move_ready_callback, self.qos1)
        self.create_subscription(Bool, '/b_reset', self.reset_callback, self.qos1)  # ★ 추가

        # 타이머(기존)
        self.create_timer(0.3, self.tick_timeout)

        # 동작 맵(기존 유지)
        self.action_map = {
            101: [0, 0, -70, -20, 0, 45, 100],
            102:"NOP",
            103: [0, 40, -130, 0, -60, 45, 100],
            104: [0, 40, -130, 0, -10, 45, 100],
            105: [0, 40, -130, 0, -10, 45, 0],
            106: [0, 0, -40, -48, -10, 45, 0],
            107: [0, -15, -85, 10, -11, 45, 0],
            108: [0, -15, -85, 10, -11, 45, 100],
            109: [0, 0, -40, -48, -10, 45, 100],
            110:"NOP",
            111: [0, 40, -130, 0, -60, 45, 100],
            112: [0, 40, -130, 0, -10, 45, 100],
            113: [0, 40, -130, 0, -10, 45, 0],
            114: [0, 0, -40, -48, -10, 45, 0],
            115: [0, -15, -75, 0, -11, 45, 0],
            116: [0, -15, -75, 0, -11, 45, 100],
            117: [0, 0, -70, -20, 0, 45, 100],

            201: [0, 0, -70, -20, 0, 45, 100],
            202: [0, -15, -75, 0, -60, 45, 100],
            203: [0, -15, -75, 0, -11, 45, 100],
            204: [0, -15, -75, 0, -11, 45, 0],
            205: [0, 0, -40, -48, -11, 45, 0],
            206: [0, 40, -130, 0, -10, 45, 0],
            207: [0, 40, -130, 0, -10, 45, 100],
            208: [0, -15, -85, 10, -60, 45, 100],
            209: [0, -15, -85, 10, -11, 45, 100],
            210: [0, -15, -85, 10, -11, 45, 0],
            211: [0, 0, -40, -48, -10, 45, 0],
            212: [0, 40, -130, 0, -10, 45, 0],
            213: [0, 40, -130, 0, -10, 45, 100],
            214: [0, 0, -70, -20, 0, 45, 100],
        }

        # ▶ 초기화 직후 준비자세로 이동(0.3초 지연, 비차단) — 기존 의미 유지
        self.get_logger().info("🚀 초기화 완료: 곧 준비자세로 이동합니다…")
        threading.Timer(0.3, self._move_to_ready_safe).start()

    # ───────── 체크포인트 래퍼들 ─────────
    def _ckpt_send_angles(self, angles, speed, save=True):
        if save:
            with self._resume_lock:
                self._last_cmd = ("angles", angles[:], speed, None)
        self.mc.send_angles(angles, speed)

    def _ckpt_send_angle(self, joint_index, degree, speed, save=True):
        if save:
            with self._resume_lock:
                self._last_cmd = ("angle", int(joint_index), float(degree), speed)
        self.mc.send_angle(joint_index, degree, speed)

    def _ckpt_send_coords(self, coords6, speed, mode, save=True):
        if save:
            with self._resume_lock:
                self._last_cmd = ("coords", coords6[:], speed, mode)
        self.mc.send_coords(coords6, speed, mode)

    def _ckpt_set_gripper(self, value, speed, save=True):
        if save:
            with self._resume_lock:
                self._last_cmd = ("gripper", int(value), speed, None)
        self.mc.set_gripper_value(value, speed)

    def _reissue_last_cmd_if_needed(self):
        with self._resume_lock:
            if not self._need_resume or self._last_cmd is None:
                return
            kind, a, b, c = self._last_cmd
            try:
                if kind == "angles":
                    self.mc.send_angles(a, b)
                elif kind == "angle":
                    self.mc.send_angle(a, b, c)
                elif kind == "coords":
                    self.mc.send_coords(a, b, c)
                elif kind == "gripper":
                    self.mc.set_gripper_value(a, b)
                self.get_logger().info(f"↻ 일시정지 복구: 마지막 명령 재전송({kind})")
            except Exception as e:
                self.get_logger().warn(f"일시정지 복구 중 재전송 실패: {e}")
            finally:
                self._need_resume = False

    # ───────── 일시정지/준비자세/리셋 ─────────
    def pause_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("⏸ [B] 일시정지 수신: 현재 동작 정지")
            self._pause_event.set()
            try: self.mc.stop()
            except Exception: pass
            with self._resume_lock:
                self._need_resume = True
        else:
            self.get_logger().info("▶ [B] 재개 수신")
            self._pause_event.clear()

    def move_ready_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("🟦 [B] 준비자세 이동 명령 수신")
            threading.Thread(target=self._move_to_ready_safe, daemon=True).start()

    def reset_callback(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().warn("🔄 [B] 초기화 명령 수신: 모든 관절을 0도로 이동합니다")
        def _reset_seq():
            # 내부 상태 클리어 (기존 로직은 손대지 않음)
            self.waiting_barrier = False
            self.step_running.clear()
            self.executing_step = None
            self.pending_step = None
            self.step_done = False
            try: self.mc.stop()
            except Exception: pass
            self._pause_event.clear()

            # ZERO_POSE로 블로킹 이동
            self._move_to_pose_blocking(self.RESET_ZERO_POSE)

            # 준비 완료 알림 (IDLE 동기화)
            self.ready_pub.publish(Int32(data=0))
            self.get_logger().info("🟢 [B] 초기화 완료: ZERO_POSE 도달, READY(0) 발행")
        threading.Thread(target=_reset_seq, daemon=True).start()

    def _wait_if_paused(self):
        while self._pause_event.is_set():
            time.sleep(0.05)
        self._reissue_last_cmd_if_needed()

    def _move_to_ready_safe(self):
        """준비자세는 체크포인트 저장 없이 보내 재개 로직 간섭을 피함(기존 유지)"""
        try:
            try: self.mc.stop()
            except Exception: pass
            angles = self.READY_POSE[:6]
            grip = self.READY_POSE[6]
            self._ckpt_send_angles(angles, self.FAST_SPEED, save=False); time.sleep(0.6)
            self._ckpt_set_gripper(grip, 40, save=False); time.sleep(0.4)
            self.get_logger().info(f"✅ [B] 준비자세 이동 완료: {self.READY_POSE}")
        except Exception as e:
            self.get_logger().error(f"❌ [B] 준비자세 이동 실패: {e}")

    # ★ 공용: 지정 포즈로 블로킹 이동(각도 도달 대기 포함)
    def _move_to_pose_blocking(self, pose_with_grip):
        try:
            try: self.mc.stop()
            except Exception: pass
            angles = pose_with_grip[:6]
            grip = pose_with_grip[6]
            self._ckpt_send_angles(angles, self.FAST_SPEED, save=False)
            self._wait_reached_angles(angles, timeout=10.0)
            self._ckpt_set_gripper(grip, 40, save=False); time.sleep(0.3)
            self.get_logger().info(f"✅ [B] 이동 완료: {pose_with_grip}")
        except Exception as e:
            self.get_logger().error(f"❌ [B] 이동 실패: {e}")

    def _wait_reached_angles(self, target_deg, timeout=10.0, eps=2.0):
        t_end = time.time() + timeout
        last = None
        while time.time() < t_end:
            self._wait_if_paused()
            try:
                cur = self.mc.get_angles()
                if cur:
                    last = cur[:6]
                    if all(abs((last[i] or 0) - target_deg[i]) <= eps for i in range(6)):
                        return True
            except Exception:
                pass
            time.sleep(0.1)
        if last:
            self.get_logger().warn(f"[B] 현재각(미도달): {[round(x,1) for x in last]}")
        return False

    # ────────────────────────────── 배리어 프로토콜 (기존 로직 유지)
    def step_callback(self, msg):
        step = int(msg.data)
        self.get_logger().info(f"📨 [B] STEP 수신: {step}")

        if self.step_running.is_set() or self.waiting_barrier:
            if step != self.executing_step:
                self.pending_step = step
                self.get_logger().warn(f"🧺 [B] 실행/대기 중({self.executing_step}) → Step {step} 큐잉")
            return

        self.executing_step = step
        self.step_done = False
        self.last_step_time = time.time()
        self.waiting_barrier = True
        self.ready_pub.publish(Int32(data=step))
        self.get_logger().info(f"🟡 [B] READY 전송: {step} (GO 대기)")

    def go_callback(self, msg):
        go = int(msg.data)
        self.get_logger().info(f"📨 [B] GO 수신: {go} (executing_step={self.executing_step})")
        if self.waiting_barrier and go == self.executing_step:
            self.waiting_barrier = False
            self.step_running.set()
            self.worker_thread = threading.Thread(target=self.execute_step, args=(go,), daemon=True)
            self.worker_thread.start()
        else:
            self.get_logger().warn("🚫 [B] GO 무시 (배리어상태 아님 또는 step 불일치)")

    def execute_step(self, step):
        angles = self.action_map[step]
        try:
            if angles == "NOP":
                self.get_logger().info(f"➖ [B] Step {step}: NOP")
                time.sleep(0.1)
            else:
                self._wait_if_paused()
                self._ckpt_send_angles(angles[:6], 30)   # 기존 속도 유지
                self._wait_if_paused()
                self._ckpt_set_gripper(angles[6], 30)    # 기존 속도 유지

            time.sleep(2)  # 기존 대기 유지
            self.step_done = True
            self.get_logger().info(f"✅ [B] Step {step} 완료")
        except Exception as e:
            self.step_done = False
            self.get_logger().error(f"❌ [B] Step {step} 실패: {e}")
        finally:
            self.step_running.clear()
            self.done_pub.publish(Int32(data=int(step)))
            self.executing_step = None
            if self.pending_step is not None:
                nxt = self.pending_step; self.pending_step = None
                self.step_callback(Int32(data=nxt))

    def tick_timeout(self):
        if self.waiting_barrier and self.executing_step is not None:
            if time.time() - self.last_step_time > 3.0:
                self.ready_pub.publish(Int32(data=int(self.executing_step)))
                self.last_step_time = time.time()
                self.get_logger().warn(f"⏳ [B] READY 재전송: {self.executing_step}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotB1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
