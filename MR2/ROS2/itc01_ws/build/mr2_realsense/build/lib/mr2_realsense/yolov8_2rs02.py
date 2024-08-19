import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
import pyrealsense2 as rs
from ultralytics import YOLO
import sys

class YOLODetector(Node):

    def __init__(self, camera_selection):
        super().__init__('yolo_detector')
        self.camera_selection = camera_selection

        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov9_rs', 'yolov9_3color.engine'))
        self.ball_distance_publisher = self.create_publisher(Float32MultiArray, '/ball_distance', 10)
        self.pur_dis_pub = self.create_publisher(Float32, '/purple_dis', 10)
        self.ball_class_publisher = self.create_publisher(String, '/ball_class', 10)

        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480
        self.HFOV = 45

        self.pipeline1 = None
        self.pipeline2 = None

        self.initialize_cameras()

        self.logger = self.get_logger()

    def initialize_cameras(self):
        # if self.camera_selection in ['both', 'camera2']:
        # self.pipeline2 = rs.pipeline()
        # self.config2 = rs.config()
        # self.config2.enable_device('239722073716')  # Replace with the actual device ID for camera 2
        # self.config2.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        # self.config2.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        # self.profile2 = self.pipeline2.start(self.config2)
        # self.align_to2 = rs.stream.color
        # self.align2 = rs.align(self.align_to2)
            # self.logger.info("Camera 2 initialized")
        
        # elif self.camera_selection in ['both', 'camera1']:
        self.pipeline1 = rs.pipeline()
        self.config1 = rs.config()
        self.config1.enable_device('230122070111') 
        self.config1.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        self.config1.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.profile1 = self.pipeline1.start(self.config1)
        self.align_to1 = rs.stream.color
        self.align1 = rs.align(self.align_to1)
            # self.logger.info("Camera 1 initialized")


    def detect_objects(self):
        ball_class = "no_ball"
        self.min_distance = float('inf')  # Reset min_distance for each detection cycle
        self.nearest_object = None  # Reset nearest_object for each detection cycle

        if self.pipeline1:
            frames1 = self.pipeline1.wait_for_frames()
            aligned_frames1 = self.align1.process(frames1)
            color_frame1 = aligned_frames1.get_color_frame()
            depth_frame1 = aligned_frames1.get_depth_frame()
            if color_frame1 and depth_frame1:
                self.process_frame(color_frame1, depth_frame1, ball_class)

        if self.pipeline2:
            frames2 = self.pipeline2.wait_for_frames()
            aligned_frames2 = self.align2.process(frames2)
            color_frame2 = aligned_frames2.get_color_frame()
            depth_frame2 = aligned_frames2.get_depth_frame()
            if color_frame2 and depth_frame2:
                self.process_frame(color_frame2, depth_frame2, ball_class)

    def process_frame(self, color_frame, depth_frame, ball_class):
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        results = self.model(color_image, conf=0.4)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls  # Class ID
                class_name = self.model.names[int(c)]

                xmin, ymin, xmax, ymax = map(int, b[:4])
                x_center = int((xmin + xmax) / 2)
                y_center = int((ymin + ymax) / 2)
                Horizontal_Angle = ((x_center - self.W / 2) / (self.W / 2)) * (self.HFOV / 2)

                if int(c) == 2:  # Blue ball or red ball
                    ball_class = "blue ball" if class_name == "blue" else "red ball"

                    object_depth_red = depth_image[ymin:ymax, xmin:xmax]
                    median_distance_r = np.median(object_depth_red)

                    if median_distance_r < self.min_distance:
                        self.min_distance = median_distance_r / 1000
                        self.nearest_object = [xmin, ymin, xmax, ymax]
                        self.x_center = x_center
                        self.Horizontal_Angle = Horizontal_Angle
                elif int(c) == 1:  # Purple ball
                    ball_class = "purple ball"
                    self.object_dis_pur = depth_frame.get_distance(x_center, y_center)
                    dis_pur = Float32()
                    dis_pur.data = self.object_dis_pur
                    self.pur_dis_pub.publish(dis_pur)
                    self.logger.info('purple_distance: %f' % self.object_dis_pur)

                cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(color_image, class_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.ball_class_publisher.publish(String(data=ball_class))

        if self.nearest_object is not None:
            object_distance = Float32MultiArray()
            object_distance.data = [self.min_distance, self.Horizontal_Angle]
            Yaw = (-1 * (np.pi * self.Horizontal_Angle) / 180) + 0.02
            self.ball_distance_publisher.publish(object_distance)
            self.logger.info('distance: %f, radian: %f, degree: %f' % (self.min_distance, Yaw, -1 * self.Horizontal_Angle))
            print(f"The nearest object is at a distance of {self.min_distance} m with bounding box: {self.nearest_object}")

        cv2.imshow('Color Image', color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    # Check command-line argument for camera selection
    camera_selection = 'both'
    if len(sys.argv) > 1:
        if sys.argv[1] in ['camera1', 'camera2']:
            camera_selection = sys.argv[1]
    
    yolo_detector = YOLODetector(camera_selection)
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
