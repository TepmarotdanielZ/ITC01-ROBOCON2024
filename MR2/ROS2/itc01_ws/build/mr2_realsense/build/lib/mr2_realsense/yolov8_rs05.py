import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs

from ultralytics import YOLO

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov8_rs', 'ball_yolov8t03.engine')) 
        
        # self.ball_position_publisher = self.create_publisher(Point, '/ball_coordinate', 10)
        self.ball_distance_publisher = self.create_publisher(Float32MultiArray, '/ball_distance', 10)

        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480
        self.HFOV = 62
        self.nearest_object = None
        self.min_distance = float('inf')
        self.Horizontal_Angle = 0.0
        self.x_center = 0
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        self.logger = self.get_logger()

    def detect_objects(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        # depth_image = np.asanyarray(depth_frame.get_data())
        results = self.model(color_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
            
                # b = box.xyxy[0].to('cuda').detach().numpy().copy()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()

                c = box.cls  # Class ID
                class_name = self.model.names[int(c)] 
                
                xmin, ymin, xmax, ymax = map(int, b[:4])  
                x_center = int((xmin + xmax) / 2)
                y_center = int((ymin + ymax) / 2)
                depth = depth_frame.get_distance(x_center, y_center)
                Horizontal_Angle  = ((x_center - self.W/2)/(self.W/2)) * (self.HFOV/2)

                if int(c) == 1:
                    # object_position = Point()
                    # object_position.x = float(x_center)
                    # object_position.y = float(y_center)
                    # # self.ball_position_publisher.publish(object_position)
                #     object_depth = depth_image[ymin:ymin+ymax, xmin:xmin+xmin]

                #    # Get the median distance within the bounding box
                #     median_distance = np.median(object_depth)

                #    # Compare to find the nearest object
                #     if median_distance < self.min_distance:
                #         self.min_distance = median_distance
                #         self.nearest_object = map(int, b[:4])  
                #     self.x_center = int((xmin + xmax) / 2)
                #     self.Horizontal_Angle  = ((self.x_center - self.W/2)/(self.W/2)) * (self.HFOV/2)
                    
                    object_distance = Float32MultiArray()
                    object_distance.data = [depth,Horizontal_Angle]
                    Yaw = -1*(np.pi*Horizontal_Angle)/180
                    self.ball_distance_publisher.publish(object_distance)
                    self.get_logger().info('distance :%f,radian : %f, degree: %f]'%(depth,Yaw,-1*Horizontal_Angle))

                    # Draw bounding box
                    # cv2.rectangle(color_image, (xmin, ymin), (xmin+xmax,ymin+ymax), (0, 255, 0), 2)
                    
                    cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    # cv2.putText(color_image, class_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                elif int(c) == 0:
                    pass
            
            # if self.nearest_object is not None:
            #     b = self.nearest_object
            #     print(f"The nearest object is at a distance of {self.min_distance/1000} m with bounding box: {self.nearest_object}")
                
            
        # # Display images
        cv2.imshow('Color Image', color_image)
        # cv2.imshow('Depth Image', depth_image)
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
