import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class YoloCameraSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_camera_subscriber')
        
        self.img = None
        self.depth = None
        self.cnt = 0

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
        self.yolo_subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)

        self.ball_publisher = self.create_publisher(Point, "/ball_coordinates", 10)
        self.bounding_publisher = self.create_publisher(Image, "/bounding_box", 10)

    def camera_callback(self, data):
        self.img = bridge.imgmsg_to_cv2(data)

    def depth_callback(self, data):
        self.depth = bridge.imgmsg_to_cv2(data)

    def yolo_callback(self, data):
        ball_msg = Point()  # Initialize ball_msg here
        
        for r in data.yolov8_inference:
            class_name = r.class_name
            y_max = r.top
            y_min = r.bottom
            x_min = r.left
            x_max = r.right
            self.get_logger().info(f"{self.cnt} {class_name} : {y_max}, {x_min}, {y_min}, {x_max}")

            # Draw rectangle on the image
            cv2.rectangle(self.img, (x_min, y_max), (x_max, y_min), (255, 255, 0), thickness=3)
            
            ball_x = (x_min + x_max) / 2
            ball_y = (y_max + y_min) / 2
            
            # depth[xmin:xmax, ymin:ymax]
            
            depth_at_center = self.depth[x_min:x_max, x_max:x_min]
            
            # depth_at_center = self.depth[int(ball_y), int(ball_x)]
        
            cv2.circle(self.img, (int(ball_x), int(ball_y)), 4, (0, 0, 255), -1)  # Draw a red circle at the center of the ball
            
            distance = float(depth_at_center)

            ball_msg.x = ball_x  
            ball_msg.y = ball_y
            
            
            ball_msg.z = distance
            
            self.ball_publisher.publish(ball_msg)  # Publish ball_msg
            
            self.cnt += 1
                
        self.cnt = 0
        
        # img_msg = bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
            
        box_msg = bridge.cv2_to_imgmsg(self.img, encoding="bgr8") 

        self.bounding_publisher.publish(box_msg)



def main(args=None):
    rclpy.init(args=args)

    node = YoloCameraSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    executor_thread.start()

    rclpy.spin(node)

    executor.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
