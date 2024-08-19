import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ultralytics import YOLO

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov9_rs', 'best.engine')) 
        self.image_publisher = self.create_publisher(Image, 'webcam_image', 10)  
        self.ball_count_publisher = self.create_publisher(Int32, '/total_ball_count', 10)
        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480
        self.capture = cv2.VideoCapture(0) 

    def detect_objects(self):
       
        self.red_ball_counter = 0
        self.blue_ball_counter = 0
        
        ret, color_image = self.capture.read() 
        if not ret:
            return

        results = self.model(color_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
               
                c = box.cls  
    
                if int(c) == 1:
                    self.red_ball_counter += 1
                elif int(c) == 0:
                    self.blue_ball_counter += 1
                    pass

        total_count = self.red_ball_counter + self.blue_ball_counter
        total_count_msg = Int32()
        total_count_msg.data = total_count
        
        self.get_logger().info(f'Red balls: {self.red_ball_counter}, Blue balls: {self.blue_ball_counter}, Total balls: {total_count}')
        self.ball_count_publisher.publish(total_count_msg)

        annotated_frame = results[0].plot()

        # Display the image using OpenCV
        cv2.imshow('Annotated Frame', annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    yolo_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
