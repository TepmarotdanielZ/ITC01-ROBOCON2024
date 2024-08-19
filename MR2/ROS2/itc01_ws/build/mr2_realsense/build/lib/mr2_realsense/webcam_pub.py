import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.cap = cv2.VideoCapture(4)  # Try index 0 first
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        # Check if the webcam is opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam')
            return

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warning('Failed to capture frame from webcam')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
