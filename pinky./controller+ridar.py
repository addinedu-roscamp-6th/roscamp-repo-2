#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Bool
from collections import deque
from sensor_msgs.msg import LaserScan
from pinky_controller.goal_points import GOAL_PATHS, FINAL_YAWS_DEG

# ===== 사용자 설정값 =====
YAW_THRESHOLD_DEG    = 7.0
ROTATE_SPEED_DEG     = 0.1
LINEAR_SPEED         = 0.1
GOAL_TOLERANCE       = 0.1

# ---- 라이다 관련 (필수 조정 포인트) ----
FRONT_FOV_DEG        = 60.0     # 전방 검사 시야각(정면 기준 ±30°)
LIDAR_YAW_OFFSET_DEG = 180.0      # 라이다 정면 오프셋 보정(+이면 좌측으로 치운 좌표계)
MIN_OBS_DIST_ON      = 0.2    # 장애물 감지(ON) 임계거리
MIN_OBS_DIST_OFF     = 0.3    # 장애물 해제(OFF) 임계거리(ON보다 크게)
USE_PCTL             = True     # 최솟값 대신 하위 백분위수 사용
PCTL                 = 20       # 하위 20% 지점

# ---- 회피 파라미터 ----
AVOID_FORWARD_DIST   = 0.08     # 측방 전진 거리(m)
PULSE_ON_SEC         = 0.15     # 펄스 회전 on 시간
PULSE_RESET_SEC      = 0.20     # 펄스 타이머 리셋 간격

# ===== [추가] 좌표 기반 회피 ON/OFF 지점 =====
# (x, y, r) 형식. r은 판정 반경(m). 필요 지점 자유롭게 추가/수정.
ENABLE_POINTS  = [
    # 예) (0.11, 0.88, 0.15),  # 이 원 안으로 진입하면 회피 '활성화'
]
DISABLE_POINTS = [
    (0.54, 0.8, 0.15),        # kitchen 경로 중 이 지점 진입 시 회피 '비활성화'
]
# ------------------------------------------------

# 라디안 변환
YAW_THRESHOLD = math.radians(YAW_THRESHOLD_DEG)
ROTATE_SPEED  = math.radians(ROTATE_SPEED_DEG)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ===== [추가] 간단한 원형 지오펜스 게이트 =====
class _CircleGate:
    """원 경계 바깥→안쪽으로 진입할 때 한 번만 True를 반환"""
    def __init__(self, x, y, r):
        self.x = float(x); self.y = float(y); self.r = float(r)
        self.prev_outside = True

    def update_and_fired(self, px, py):
        dx = px - self.x; dy = py - self.y
        inside = (dx*dx + dy*dy) <= (self.r * self.r)
        fired = (self.prev_outside and inside)
        self.prev_outside = (not inside)
        return fired
# ==============================================

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 로봇 상태
        self.robot_pose = None
        self.robot_yaw = None
        self.raw_yaw = None
        self.goal_pose = None
        self.state = 'IDLE'
        self.auto_start = False
        self.last_pulse_time = self.get_clock().now()

        # 목표 큐
        self.goal_queue = deque()
        # 경로별 최종 정렬 목표 각도(rad). 경로 로드 시 갱신됨. 기본 -90도
        self.final_target_yaw = -0.5 * math.pi

        # 파라미터
        marker_id = self.declare_parameter('marker_id', 26).get_parameter_value().integer_value
        topic_name = f"marker_map/ID{marker_id}"

        # 구독/퍼블리셔
        self.create_subscription(PoseStamped, topic_name, self.robot_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.15, self.control_loop)

        self.create_subscription(Bool, 'go_to_kitchen', lambda msg: self.load_path('kitchen', msg), 10)
        self.create_subscription(Bool, 'go_to_serving', lambda msg: self.load_path('serving', msg), 10)
        self.create_subscription(Bool, 'go_to_charge',  lambda msg: self.load_path('charge', msg), 10)
        self.create_subscription(Bool, 'go_to_recall',  lambda msg: self.load_path('recall', msg), 10)
        self.create_subscription(Bool, 'go_to_return',  lambda msg: self.load_path('return', msg), 10)

        # 라이다
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.front_blocked = False

        # 회피 상태
        self.avoid_target_yaw = None
        self.avoid_start_pos = None

        # ===== [추가] 회피 ON/OFF 플래그 및 게이트 준비 =====
        self.obstacle_enabled = True  # 기본 ON (초기값은 네가 원하면 변경)
        self.enable_gates  = [_CircleGate(*p) for p in ENABLE_POINTS]
        self.disable_gates = [_CircleGate(*p) for p in DISABLE_POINTS]

        self.get_logger().info("[INIT] controller_node 초기화 완료. 명령 토픽을 대기 중.")

    # ---------- 경로 로드/전환 ----------
    def load_path(self, path_name, msg):
        if msg.data:
            if path_name in GOAL_PATHS:
                self.goal_queue.clear()
                for x, y in GOAL_PATHS[path_name]:
                    pt = Point(); pt.x, pt.y, pt.z = x, y, 0.0
                    self.goal_queue.append(pt)
                final_deg = FINAL_YAWS_DEG.get(path_name, -90.0)
                self.final_target_yaw = math.radians(final_deg)
                self.auto_start = True
                self.set_next_goal()
                self.get_logger().info(
                    f"[ROUTE] '{path_name}' 시작 "
                    f"(총 {len(self.goal_queue)+1 if self.goal_pose else len(self.goal_queue)}개 좌표, 최종각 {final_deg:.1f}°)"
                )
            else:
                self.get_logger().warn(f"[ERROR] '{path_name}' 경로가 존재하지 않음")

    def set_next_goal(self):
        if self.goal_queue:
            self.goal_pose = self.goal_queue.popleft()
            self.state = 'ROTATE'
            self.last_pulse_time = self.get_clock().now()
            self.get_logger().info(f"[GOAL] 다음 목표: x={self.goal_pose.x:.2f}, y={self.goal_pose.y:.2f}")
        else:
            self.goal_pose = None
            self.state = 'IDLE'
            self.auto_start = False
            self.get_logger().info("[GOAL] 모든 목표 완료. 대기 상태로 전환.")

    # ---------- 상태 업데이트 ----------
    def robot_callback(self, msg):
        self.robot_pose = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.raw_yaw = math.atan2(siny, cosy)
        self.robot_yaw = self.raw_yaw

        # ===== [추가] 좌표 기반 회피 ON/OFF 트리거 =====
        if self.robot_pose is not None:
            px, py = self.robot_pose.x, self.robot_pose.y

            # 비활성화 존 진입
            for gate in self.disable_gates:
                if gate.update_and_fired(px, py):
                    if self.obstacle_enabled:
                        self.obstacle_enabled = False
                        self.front_blocked = False  # 즉시 차단 해제
                        self.get_logger().warn("장애물 인식 비활성화됨!")

            # 활성화 존 진입
            for gate in self.enable_gates:
                if gate.update_and_fired(px, py):
                    if not self.obstacle_enabled:
                        self.obstacle_enabled = True
                        self.get_logger().info("장애물 인식 활성화됨!")

    # ---------- 라이다(정면 각도 구간만 검사 + 히스테리시스) ----------
    def scan_callback(self, scan: LaserScan):
        # ===== [추가] 회피 OFF일 때는 라이다 무시 =====
        if not getattr(self, 'obstacle_enabled', True):
            return
        try:
            if not scan.ranges:
                return

            half = math.radians(FRONT_FOV_DEG * 0.5)
            offset = math.radians(LIDAR_YAW_OFFSET_DEG)
            ang_min, ang_max, inc = scan.angle_min, scan.angle_max, scan.angle_increment

            # 검사구간: [ -half + offset , +half + offset ]
            from_ang = max(ang_min, -half + offset)
            to_ang   = min(ang_max, +half + offset)
            if to_ang <= from_ang:
                return

            i_from = max(0, int((from_ang - ang_min) / inc))
            i_to   = min(len(scan.ranges) - 1, int((to_ang   - ang_min) / inc))

            # 유효값만
            window = []
            rmax = scan.range_max if math.isfinite(scan.range_max) and scan.range_max > 0.0 else float('inf')
            for r in scan.ranges[i_from:i_to+1]:
                if math.isfinite(r) and 0.01 < r < rmax:
                    window.append(r)
            if not window:
                return

            # 노이즈 완화
            if USE_PCTL:
                window.sort()
                k = max(0, min(len(window)-1, int(len(window)*PCTL/100.0)))
                front_near = window[k]
            else:
                front_near = min(window)

            # 히스테리시스
            if not self.front_blocked and front_near < MIN_OBS_DIST_ON:
                self.front_blocked = True
                self.get_logger().info(f"[LIDAR] 전방 차단 감지: ≈{front_near:.2f} m (ON<{MIN_OBS_DIST_ON}m)")
            elif self.front_blocked and front_near > MIN_OBS_DIST_OFF:
                self.front_blocked = False
                self.get_logger().info(f"[LIDAR] 전방 차단 해제: ≈{front_near:.2f} m (OFF>{MIN_OBS_DIST_OFF}m)")

        except Exception as e:
            self.get_logger().warn(f"[LIDAR] 예외: {e}")

    # ---------- 제어 루프 ----------
    def control_loop(self):
        if not self.auto_start:
            return
        if self.robot_pose is None or self.robot_yaw is None or self.goal_pose is None:
            return

        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y
        # 좌표계 보정: 맵 정의에 맞춰 -0.5*pi 유지
        target_yaw = math.atan2(dy, dx) - 0.5 * math.pi
        yaw_err = normalize_angle(target_yaw - self.robot_yaw)
        yaw_err_deg = math.degrees(yaw_err)
        distance = math.hypot(dx, dy)

        cmd = Twist()

        # [SAFETY] 전방 장애물 감지 시, 어떤 상태든 즉시 정지 + 회피 진입(회피 상태가 아닐 때만)
        # ===== [추가] 회피가 '활성화' 상태일 때만 진입 =====
        if getattr(self, 'obstacle_enabled', True) and self.front_blocked and self.state not in ('AVOID_TURN_RIGHT', 'AVOID_FORWARD', 'AVOID_TURN_LEFT'):
            self.cmd_pub.publish(Twist())  # ★즉시 정지
            self.avoid_target_yaw = normalize_angle(self.robot_yaw - math.pi/2.5)  # 우회전 70°
            self.avoid_start_pos = None
            self.state = 'AVOID_TURN_RIGHT'
            self.last_pulse_time = self.get_clock().now()
            self.get_logger().warn("[SAFETY] 전방 장애물 감지 → 즉시 정지 & 회피 진입")
            return

        # ➊ FINAL_ALIGN 우선 처리
        if self.state == 'FINAL_ALIGN':
            target_yaw_final = self.final_target_yaw
            yaw_err2 = normalize_angle(target_yaw_final - self.robot_yaw)
            if abs(yaw_err2) < YAW_THRESHOLD:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("[STATE] FINAL_ALIGN 완료 → IDLE")
                self.state = 'IDLE'
                self.auto_start = False
                return
            now = self.get_clock().now()
            t = (now - self.last_pulse_time).nanoseconds / 1e9
            cmd.linear.x = 0.0
            cmd.angular.z = ROTATE_SPEED * math.copysign(1.0, yaw_err2) if t < PULSE_ON_SEC else 0.0
            if t > PULSE_RESET_SEC:
                self.last_pulse_time = now
            self.get_logger().info(f"[FINAL_ALIGN] 정렬 중 | yaw_err={math.degrees(yaw_err2):.2f}° → 목표 {math.degrees(target_yaw_final):.2f}°")
            self.cmd_pub.publish(cmd)
            return

        # ➋ 목표 도달 처리 (중간/최종 구분)
        if distance < GOAL_TOLERANCE:
            self.cmd_pub.publish(Twist())
            if self.goal_queue:
                self.get_logger().info(f"[DONE] 중간 목표 도착 (±{GOAL_TOLERANCE}m). 다음 좌표로 이동.")
                self.set_next_goal()
                return
            else:
                self.get_logger().info("[DONE] 최종 목표 도착. 최종 각도 정렬 시작.")
                self.state = 'FINAL_ALIGN'
                self.last_pulse_time = self.get_clock().now()
                return  # 다음 틱에서 FINAL_ALIGN가 최우선으로 실행

        # ➌ 일반 상태머신 (ROTATE / FORWARD)
        if self.state == 'ROTATE':
            if abs(yaw_err) < YAW_THRESHOLD:
                self.get_logger().info(f"[STATE] ROTATE → FORWARD | yaw 오차 {yaw_err_deg:.2f}° < {YAW_THRESHOLD_DEG}°")
                self.state = 'FORWARD'
                cmd.linear.x = LINEAR_SPEED
                self.cmd_pub.publish(cmd)
                return
            now = self.get_clock().now()
            t = (now - self.last_pulse_time).nanoseconds / 1e9
            cmd.angular.z = ROTATE_SPEED * math.copysign(1.0, yaw_err) if t < PULSE_ON_SEC else 0.0
            cmd.linear.x = 0.0
            if t > PULSE_RESET_SEC:
                self.last_pulse_time = now
            self.get_logger().info(f"[ROTATE] 회전 중 | yaw_err={yaw_err_deg:.2f}°")
            self.cmd_pub.publish(cmd)

        elif self.state == 'FORWARD':
            # (여기까지 오면 front_blocked는 False 상태)
            cmd.linear.x = LINEAR_SPEED
            if abs(yaw_err) > YAW_THRESHOLD:
                self.get_logger().info(f"[STATE] FORWARD → ROTATE | yaw 오차 {yaw_err_deg:.2f}° > {YAW_THRESHOLD_DEG}°")
                self.state = 'ROTATE'
                self.cmd_pub.publish(cmd)
                return
            cmd.angular.z = 0.0
            self.get_logger().info(f"[FORWARD] 전진 중 | 거리={distance:.2f} m, yaw_err={yaw_err_deg:.2f}°")
            self.cmd_pub.publish(cmd)

        # ➍ 회피 시퀀스 (우회전 → 측방 전진 → 좌회전)
        elif self.state == 'AVOID_TURN_RIGHT':
            yaw_err_r = normalize_angle(self.avoid_target_yaw - self.robot_yaw)
            if abs(yaw_err_r) < YAW_THRESHOLD:
                self.cmd_pub.publish(Twist())
                self.avoid_start_pos = (self.robot_pose.x, self.robot_pose.y)
                self.state = 'AVOID_FORWARD'
                self.get_logger().info(f"[AVOID] 우회전 완료 → 측방 전진 {AVOID_FORWARD_DIST:.2f} m")
                return
            now = self.get_clock().now()
            t = (now - self.last_pulse_time).nanoseconds / 1e9
            cmd.angular.z = ROTATE_SPEED * math.copysign(1.0, yaw_err_r) if t < PULSE_ON_SEC else 0.0
            cmd.linear.x = 0.0
            if t > PULSE_RESET_SEC:
                self.last_pulse_time = now
            self.cmd_pub.publish(cmd)

        elif self.state == 'AVOID_FORWARD':
            if self.avoid_start_pos is None:
                self.avoid_start_pos = (self.robot_pose.x, self.robot_pose.y)
            moved = math.hypot(self.robot_pose.x - self.avoid_start_pos[0],
                               self.robot_pose.y - self.avoid_start_pos[1])
            if moved >= AVOID_FORWARD_DIST:
                self.cmd_pub.publish(Twist())
                self.avoid_target_yaw = normalize_angle(self.robot_yaw + math.pi/3.0)  # 좌회전 70°
                self.state = 'AVOID_TURN_LEFT'
                self.last_pulse_time = self.get_clock().now()
                self.get_logger().info("[AVOID] 측방 전진 완료 → 좌회전 70°")
                return
            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        elif self.state == 'AVOID_TURN_LEFT':
            yaw_err_l = normalize_angle(self.avoid_target_yaw - self.robot_yaw)
            if abs(yaw_err_l) < YAW_THRESHOLD:
                self.cmd_pub.publish(Twist())
                self.state = 'ROTATE'  # 원래 목표를 향해 다시 정렬
                self.get_logger().info("[AVOID] 좌회전 완료 → 경로 복귀(ROTATE)")
                return
            now = self.get_clock().now()
            t = (now - self.last_pulse_time).nanoseconds / 1e9
            cmd.angular.z = ROTATE_SPEED * math.copysign(1.0, yaw_err_l) if t < PULSE_ON_SEC else 0.0
            cmd.linear.x = 0.0
            if t > PULSE_RESET_SEC:
                self.last_pulse_time = now
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

