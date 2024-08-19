import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
from ultralytics import YOLO

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov8_rs', 'ball_yolov8t03.engine')) 
    
        self.ball_distance_publisher = self.create_publisher(Float32MultiArray, '/ball_distance', 10)

        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480
        self.HFOV = 62

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        self.logger = self.get_logger()
        
    # def calculate_angle(cx, cy, bbox):
    #     # x, y, w, h = bbox
    #      # object_center_x = x + w / 2
    #     #object_center_y = y + h / 2
    #     delta_x = object_center_x - cx
    #     delta_y = object_center_y - cy
    #     angle_x = np.degrees(np.arctan(delta_x / fx))
    #     angle_y = np.degrees(np.arctan(delta_y / fy))
    #     return angle_x, angle_y

    def detect_objects(self):
        self.min_distance = float('inf')  # Reset min_distance for each detection cycle
        self.nearest_object = None  # Reset nearest_object for each detection cycle

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
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls  # Class ID
                class_name = self.model.names[int(c)] 
                
                xmin, ymin, xmax, ymax = map(int, b[:4])  
                x_center = int((xmin + xmax) / 2)
                y_center = int((ymin + ymax) / 2)
                depth = depth_frame.get_distance(x_center, y_center)
                Horizontal_Angle  = ((x_center - self.W / 2) / (self.W / 2)) * (self.HFOV / 2)

                if int(c) == 1:
                    
                    object_depth = depth_image[ymin:ymax, xmin:xmax]

                    # Get the median distance within the bounding box
                    median_distance = np.median(object_depth)

                    # Compare to find the nearest object
                    if median_distance < self.min_distance:
                        self.min_distance = median_distance
                        self.nearest_object = [xmin, ymin, xmax, ymax]
                        self.x_center = x_center
                        self.Horizontal_Angle = Horizontal_Angle

                    cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    cv2.putText(color_image, class_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if self.nearest_object is not None:
            object_distance = Float32MultiArray()
            object_distance.data = [self.min_distance, self.Horizontal_Angle]
            Yaw = -1 * (np.pi * self.Horizontal_Angle) / 180
            self.ball_distance_publisher.publish(object_distance)
            self.logger.info('distance: %f, radian: %f, degree: %f' % (self.min_distance, Yaw, -1 * self.Horizontal_Angle))
            print(f"The nearest object is at a distance of {self.min_distance / 1000} m with bounding box: {self.nearest_object}")

        cv2.imshow('Color Image', color_image)
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

if __name__ == '__main__':
    main()
