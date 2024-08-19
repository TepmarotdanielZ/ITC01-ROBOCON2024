import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

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
        self.image_publisher = self.create_publisher(Image, "/image_with_info", 10)

    def camera_callback(self, data):
        self.img = bridge.imgmsg_to_cv2(data)

    def depth_callback(self, data):
        # Convert depth image to meters (assuming depth values are in millimeters)
        self.depth = bridge.imgmsg_to_cv2(data)
        self.depth = self.depth * 0.001  # Convert depth values from mm to meters

    def yolo_callback(self, data):
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
            
            depth_at_center = self.depth[int(ball_y), int(ball_x)]
            distance = float(depth_at_center)

            # Draw a red circle at the center of the ball
            cv2.circle(self.img, (int(ball_x), int(ball_y)), 4, (0, 0, 255), -1)
            
            # Add text displaying the distance
            cv2.putText(self.img, f"{distance:.2f}m", (int(ball_x) + 10, int(ball_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish ball coordinates
            ball_msg = Point()
            ball_msg.x = ball_x  
            ball_msg.y = ball_y
            ball_msg.z = distance
            self.ball_publisher.publish(ball_msg)
            
            self.cnt += 1
                
        self.cnt = 0
        
        # Publish the image with the information
        img_msg = bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
        
        self.image_publisher.publish(img_msg)

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
