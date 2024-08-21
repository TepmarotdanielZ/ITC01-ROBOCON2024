import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

class ImagePublisher(Node):
    
    def __init__(self):
        super().__init__('image_publisher')
        self.color_publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self.br = CvBridge()

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            color_msg = self.br.cv2_to_imgmsg(color_image)
            self.color_publisher_.publish(color_msg)
            self.get_logger().info('Publishing color image')
        
        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg = self.br.cv2_to_imgmsg(depth_image)
            self.depth_publisher_.publish(depth_msg)
            self.get_logger().info('Publishing depth image')
            
            # Create point cloud
            points = rs.points()
            pc = rs.pointcloud()
            pc.map_to(depth_frame)
            pc_data = pc.calculate(depth_frame)
            points.export_to_ply('temp.ply', depth_frame)
            
            # Read the created point cloud
            pcd = o3d.io.read_point_cloud('temp.ply')
            # Convert to ROS message
            pointcloud_msg = o3d.io.read_point_cloud(pcd)
            self.pointcloud_publisher_.publish(pointcloud_msg)
            self.get_logger().info('Publishing point cloud')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()