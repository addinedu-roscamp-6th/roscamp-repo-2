#!/usr/bin/env python3

####需要 ros2 launch pinky_bringup bringup.launch.xml 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')
        try:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info("Publishing to /cmd_vel topic")
        except Exception as e:
            self.get_logger().error(f"Failed to create /cmd_vel publisher: {str(e)}")
            raise

        try:
            self.subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self.lidar_callback,
                10
            )
            self.get_logger().info("Subscribed to /scan topic")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to /scan: {str(e)}")
            raise

        self.min_distance = 0.2  # 最小避障距离（米），适配1m×2m实验台
        self.linear_speed = 0.2  # 前进线速度（m/s）
        self.angular_speed = 0.5  # 转弯角速度（rad/s）
        self.get_logger().info(f"Parameters: min_distance={self.min_distance}m, linear_speed={self.linear_speed}m/s, angular_speed={self.angular_speed}rad/s")

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info("LaserScan callback triggered")
        try:
            if not msg.ranges:
                self.get_logger().warning("Empty ranges in LaserScan message")
                self.publish_cmd_vel(0.0, 0.0)
                return

            front_ranges = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
            valid_ranges = [r for r in front_ranges if r > 0.0 and r < msg.range_max]
            
            if not valid_ranges:
                self.get_logger().warning("No valid range data in front 90 degrees")
                self.publish_cmd_vel(0.0, 0.0)
                return

            min_range = min(valid_ranges)
            self.get_logger().info(f"Minimum distance: {min_range:.2f}m")

            if min_range < self.min_distance:
                self.get_logger().info(f"Obstacle detected at {min_range:.2f}m, stopping")
                self.publish_cmd_vel(0.0, 0.0)
                time.sleep(1)
                self.get_logger().info("Moving backward")
                self.publish_cmd_vel(-0.1, 0.0)
                time.sleep(1)
                self.get_logger().info("Turning left")
                self.publish_cmd_vel(0.0, self.angular_speed)
                time.sleep(2)
            else:
                self.get_logger().info(f"Path clear, moving forward at {min_range:.2f}m")
                self.publish_cmd_vel(self.linear_speed, 0.0)
        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {str(e)}")
            self.publish_cmd_vel(0.0, 0.0)

    def publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def shutdown(self):
        self.get_logger().info("Shutting down node...")
        try:
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().info("Stopped movement")
            self.destroy_node()
            self.get_logger().info("Node destroyed")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = LidarObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.shutdown()
        rclpy.shutdown()
        node.get_logger().info("ROS 2 shutdown complete")

if __name__ == '__main__':
    main()
