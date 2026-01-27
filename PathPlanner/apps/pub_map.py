import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import numpy as np
from PIL import Image
import sys
import os
import math

class MapPublisher(Node):
    def __init__(self, yaml_file):
        super().__init__('map_publisher')
        
        # QoS 설정 (Late Latching: 늦게 들어온 RViz도 지도를 받을 수 있게 설정)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos)
        
        self.yaml_file = yaml_file
        self.load_and_publish_map()

    def load_and_publish_map(self):
        # 1. YAML 파일 읽기
        if not os.path.exists(self.yaml_file):
            self.get_logger().error(f"YAML file not found: {self.yaml_file}")
            return

        with open(self.yaml_file, 'r') as f:
            map_data = yaml.safe_load(f)

        # 2. PGM 이미지 경로 찾기 (YAML 파일 기준 상대 경로 처리)
        image_file = map_data['image']
        if not os.path.isabs(image_file):
            yaml_dir = os.path.dirname(os.path.abspath(self.yaml_file))
            image_path = os.path.join(yaml_dir, image_file)
        else:
            image_path = image_file

        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            return

        # 3. 이미지 로드 및 처리
        self.get_logger().info(f"Loading map image: {image_path}")
        img = Image.open(image_path)
        
        # ROS 좌표계에 맞게 이미지 상하 반전 (PGM은 위에서 아래로 읽지만, ROS는 아래에서 위로)
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        
        # Grayscale 변환
        if img.mode != 'L':
            img = img.convert('L')
            
        img_data = np.array(img)
        
        # 4. OccupancyGrid 메시지 생성
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # 메타데이터 설정 (YAML 값 사용)
        resolution = map_data['resolution']
        origin = map_data['origin'] # [x, y, z]

        msg.info.resolution = float(resolution)
        msg.info.width = img_data.shape[1]
        msg.info.height = img_data.shape[0]
        
        # Origin 설정
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0 # No rotation

        # 픽셀 데이터 변환 (0~255 -> 0~100, -1)
        # ROS 표준: 흰색(255) -> 0 (Free), 검은색(0) -> 100 (Occupied), 회색 -> -1 (Unknown)
        # 보통 임계값(thresh)을 기준으로 나눔 (기본 0.65 / 0.196)
        
        # 배열 평탄화 (Flatten)
        flat_data = img_data.flatten()
        
        # 변환 로직 (간소화)
        # 255(흰색) 가까우면 0, 0(검정) 가까우면 100
        # thresh_occupied = 0.65 (255 * (1-0.65) = 89 이하)
        # thresh_free = 0.196 (255 * (1-0.196) = 205 이상)
        
        occ_grid = np.full(flat_data.shape, -1, dtype=np.int8)
        
        # 검은색(장애물) -> 100
        occ_grid[flat_data <= 100] = 100
        # 흰색(빈 공간) -> 0
        occ_grid[flat_data >= 200] = 0
        
        msg.data = occ_grid.tolist()

        # 5. 게시 (Publish)
        self.publisher_.publish(msg)
        self.get_logger().info("Map published to /map topic!")

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 pub_map.py <path_to_map_yaml>")
        return

    rclpy.init(args=args)
    node = MapPublisher(sys.argv[1])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
