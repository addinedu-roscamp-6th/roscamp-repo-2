import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RobotA1TestNode(Node):
    def __init__(self):
        super().__init__('robot_a1_test_node')

        self.subscription = self.create_subscription(
            String,
            'robot_a_command',
            self.command_callback,
            10
        )

        self.publisher_ = self.create_publisher(String, 'robot_a_status', 10)
        self.get_logger().info('🤖 로봇 A 노드 시작됨')

    def command_callback(self, msg):
        self.get_logger().info(f'📥 받은 명령 (로봇 A): {msg.data}')

        # 여기서 실제 동작 (예: 로봇팔 움직임)
        time.sleep(2)  # 동작 수행 시간 시뮬레이션

        done_msg = String()
        done_msg.data = f'{msg.data}_done'
        self.publisher_.publish(done_msg)
        self.get_logger().info(f'📤 완료 전송 (로봇 A): {done_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotA1TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
