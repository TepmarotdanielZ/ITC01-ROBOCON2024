import sys
import os
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import pyrealsense2 as rs
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov8_rs', 'ball_yolov8.pt'))
        self.bridge = CvBridge()
        self.annotated_image_publisher = self.create_publisher(Image, '/yolo_detection_annotated_image', 10)  # Publisher for annotated depth image
        self.raw_depth_image_publisher = self.create_publisher(Image, '/yolo_detection_raw_depth_image', 10)  # Publisher for raw depth image
        self.object_position_publisher = self.create_publisher(Point, '/object_position', 10)
        self.object_distance_publisher = self.create_publisher(Float64, '/object_distance', 10)
        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def detect_objects(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        results = self.model(color_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                #get box
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls #class
                # define coordinate
                x_center = int((b[0] + b[2]) / 2)
                y_center = int((b[1] + b[3]) / 2)
                depth = depth_frame.get_distance(x_center, y_center)

                cv2.rectangle(depth_image, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255),
                            thickness=2, lineType=cv2.LINE_4)
                cv2.putText(depth_image, text=self.model.names[int(c)], org=(int(b[0]), int(b[1])),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 255),
                            thickness=2, lineType=cv2.LINE_4)

                object_position = Point()
                object_position.x = float(x_center)
                object_position.y = float(y_center)
                self.object_position_publisher.publish(object_position)

                object_distance = Float64()
                object_distance.data = depth
                self.object_distance_publisher.publish(object_distance)

        annotated_frame = results[0].plot()

        ros_annotated_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.annotated_image_publisher.publish(ros_annotated_image)

        ros_raw_depth_image = self.bridge.cv2_to_imgmsg(depth_image, "passthrough")
        self.raw_depth_image_publisher.publish(ros_raw_depth_image)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLODetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
