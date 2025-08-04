#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
# 사용자 설정값 (도 단위)
YAW_THRESHOLD_DEG = 5.0       # 허용 오차 (deg)
ROTATE_SPEED_DEG  = 17.0      # 회전 속도 (deg/s)
LINEAR_SPEED      = 0.1       # 직진 속도 (m/s)
GOAL_TOLERANCE    = 0.1      # 목표 도달 거리 (m)
# 내부 연산용 (도 → 라디안)
YAW_THRESHOLD = math.radians(YAW_THRESHOLD_DEG)
ROTATE_SPEED  = math.radians(ROTATE_SPEED_DEG)
def normalize_angle(angle):
    """라디안 각도를 -π ~ +π 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.robot_pose = None
        self.robot_yaw = None
        self.goal_pose = None
        self.state = 'IDLE'
        self.create_subscription(PoseStamped, '/marker_map/ID25', self.robot_callback, 10)
        self.create_subscription(PointStamped, '/manual_goal', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.control_loop)
    def robot_callback(self, msg):
        self.robot_pose = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.robot_yaw = math.atan2(siny, cosy)
    def goal_callback(self, msg):
        self.goal_pose = msg.point
        self.state = 'ROTATE'
        self.get_logger().info(f"[GOAL] New goal: x={self.goal_pose.x:.2f}, y={self.goal_pose.y:.2f}")
    def control_loop(self):
        if self.robot_pose is None or self.robot_yaw is None or self.goal_pose is None:
            return
        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y
        # :흰색_확인_표시: Y축 반전 주의할 필요 없음: 좌표계 맞게 atan2 사용
        target_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(target_yaw - self.robot_yaw)
        yaw_err_deg = ((math.degrees(yaw_err) + 180) % 360) - 180  # :흰색_확인_표시: 디버깅용 정규화
        distance = math.hypot(dx, dy)
        cmd = Twist()
        if distance < GOAL_TOLERANCE:
            self.get_logger().info("[DONE] Goal reached.")
            self.state = 'IDLE'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.state == 'ROTATE':
            if abs(yaw_err) < YAW_THRESHOLD:
                self.get_logger().info("[STATE] ROTATE → FORWARD")
                self.state = 'FORWARD'
            else:
                cmd.angular.z = ROTATE_SPEED * math.copysign(1.0, yaw_err)
                cmd.linear.x = 0.0
                self.get_logger().info(f"[ROTATE] yaw_err={yaw_err_deg:.2f}°")
        elif self.state == 'FORWARD':
            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = 0.0
            self.get_logger().info(f"[FORWARD] dist={distance:.2f} m")
        self.cmd_pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main() 
