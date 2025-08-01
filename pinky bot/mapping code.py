#!/usr/bin/env python3
import os
import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.pub = self.create_publisher(PoseStamped, '/marker/world_pose', 10)
        self.pinky_pub = self.create_publisher(PoseStamped, '/marker_map/ID25', 10)
        self.pinky_id = 25
        pkg_share = get_package_share_directory('marker_mapping')
        calib_path = os.path.join(pkg_share, 'config', 'calibration_result.npz')
        calib = np.load(calib_path)
        self.K, self.dist = calib['camera_matrix'], calib['dist_coeffs']
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_len = 0.1
        self.world_coords = {
            20: (0.0, 0.0),
            21: (0.0, 1.0),
            22: (2.0, 1.0),
            23: (2.0, 0.0),
        }
        self.last_M = None
        self.last_yaw = None
        self.ema_alpha = 0.3
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다")
            raise RuntimeError("Could not open camera")
        self.create_timer(0.1, self.timer_callback)
    def estimate_affine(self, corners, ids):
        image_pts, world_pts = [], []
        for i, id_arr in enumerate(ids):
            mid = int(id_arr[0])
            if mid in self.world_coords:
                pts = corners[i].reshape(4, 2)
                center = pts.mean(axis=0)
                image_pts.append(center)
                world_pts.append(self.world_coords[mid])
        if len(image_pts) < 3:
            return None
        M, _ = cv2.estimateAffine2D(
            np.array(image_pts, dtype=np.float32),
            np.array(world_pts, dtype=np.float32)
        )
        return M
    @staticmethod
    def _angle_lerp(prev, new, alpha):
        if prev is None:
            return new
        diff = math.atan2(math.sin(new - prev), math.cos(new - prev))
        return prev + alpha * diff
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        img = cv2.undistort(frame, self.K, self.dist)
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            cv2.imshow("Mapped", img)
            cv2.waitKey(1)
            return
        for i, mid in enumerate(ids.flatten()):
            if mid in self.world_coords:
                wx, wy = self.world_coords[mid]
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'map'
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = 0.0
                ps.pose.orientation.x = 0.0
                ps.pose.orientation.y = 0.0
                ps.pose.orientation.z = 0.0
                ps.pose.orientation.w = 1.0
                self.pub.publish(ps)
                text = f"ID{mid}: ({wx:.2f},{wy:.2f})m"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                pts = corners[i].reshape(4, 2)
                cx, cy = pts.mean(axis=0).astype(int)
                tx = cx + 10 if mid in (20, 21) else cx - tw - 10
                ty = cy + th // 2
                cv2.putText(img, text, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        M = self.estimate_affine(corners, ids)
        if M is not None:
            self.last_M = M
        else:
            M = self.last_M
        if M is not None:
            ids_list = ids.flatten().tolist()
            if self.pinky_id in ids_list:
                idx = ids_list.index(self.pinky_id)
                pts_img = corners[idx].reshape(4, 2).astype(np.float32)
                center_img = pts_img.mean(axis=0)
                center_map = cv2.transform(center_img[None, None, :], M)[0, 0, :]
                p0_img = pts_img[0]
                p1_img = pts_img[1]
                p0_map = cv2.transform(p0_img[None, None, :], M)[0, 0, :]
                p1_map = cv2.transform(p1_img[None, None, :], M)[0, 0, :]
                vx, vy = (p1_map - p0_map).tolist()
                yaw_map = math.atan2(vy, vx)
                filtered_yaw = self._angle_lerp(self.last_yaw, yaw_map, self.ema_alpha)
                self.last_yaw = filtered_yaw
                # :흰색_확인_표시: 정규화 적용 (이 라인만 수정됨)
                yaw_deg = ((math.degrees(filtered_yaw) + 180) % 360) - 180
                self.get_logger().info(f"Pinky in map → x={center_map[0]:.2f}m, y={center_map[1]:.2f}m")
                self.get_logger().info(f"Pinky yaw(map) → {yaw_deg:.1f}°")
                cx, cy = center_img.astype(int)
                overlay = f"Pinky: ({center_map[0]:.2f},{center_map[1]:.2f})m, {yaw_deg:.1f}deg"
                (tw, th), _ = cv2.getTextSize(overlay, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                tx = cx - tw - 10
                ty = cy - 10
                cv2.circle(img, (cx, cy), 6, (0, 0, 255), 2)
                cv2.putText(img, overlay, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'map'
                ps.pose.position.x = float(center_map[0])
                ps.pose.position.y = float(center_map[1])
                ps.pose.position.z = 0.0
                ps.pose.orientation.x = 0.0
                ps.pose.orientation.y = 0.0
                ps.pose.orientation.z = math.sin(filtered_yaw / 2.0)
                ps.pose.orientation.w = math.cos(filtered_yaw / 2.0)
                self.pinky_pub.publish(ps)
        cv2.imshow("Mapped", img)
        cv2.waitKey(1)
def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main() 
