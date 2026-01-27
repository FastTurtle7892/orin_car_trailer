import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import sys
import math
import os

def euler_to_quaternion(yaw):
    """yaw(rad)를 quaternion(x, y, z, w)으로 변환"""
    # z축 회전만 고려
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class PathPublisher(Node):
    def __init__(self, json_path):
        super().__init__('path_publisher_node')
        
        # Publisher 생성 (토픽 이름: /planned_path)
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.json_path = json_path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # RViz의 Fixed Frame과 맞춰야 함

        self.load_path()

    def load_path(self):
        if not os.path.exists(self.json_path):
            self.get_logger().error(f"File not found: {self.json_path}")
            return

        with open(self.json_path, 'r') as f:
            data = json.load(f)

        if not data.get("ok", False):
            self.get_logger().warn("This path is marked as failed (ok=false). Publishing anyway...")

        xs = data["x"]
        ys = data["y"]
        yaws = data["yaw"]

        self.get_logger().info(f"Loaded {len(xs)} points from JSON.")

        # JSON 좌표를 nav_msgs/Path 메시지로 변환
        for x, y, yaw in zip(xs, ys, yaws):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # Orientation (Yaw -> Quaternion)
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.path_msg.poses.append(pose)

    def timer_callback(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path_msg)
        # self.get_logger().info('Published path message...')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 pub_path.py <path_to_json_file>")
        return

    json_file = sys.argv[1]

    rclpy.init(args=args)
    node = PathPublisher(json_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
