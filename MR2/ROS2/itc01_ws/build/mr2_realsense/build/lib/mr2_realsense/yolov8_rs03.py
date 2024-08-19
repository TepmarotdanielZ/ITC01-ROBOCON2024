import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int32 , Float32MultiArray
import pyrealsense2 as rs

from ultralytics import YOLO

class YOLODetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO(os.path.join(os.environ['HOME'], 'yolov8_rs', 'ball_yolov8t03.engine'))
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '/camera/color/image_raw', 10)  
        self.depth_publisher = self.create_publisher(Image, '/camera/depth/image_raw', 10) 
        # self.ball_position_publisher = self.create_publisher(Point, '/ball_coordinate', 10)
        self.ball_distance_publisher = self.create_publisher(Float32MultiArray, '/ball_distance', 10)
        self.ball_count_publisher = self.create_publisher(Int32, '/total_ball_count', 10)
        self.timer = self.create_timer(0.1, self.detect_objects)
        self.W = 640
        self.H = 480
        self.HFOV = 62
        self.VFOV = 46 

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        # self.red_ball_counter = 0
        # self.blue_ball_counter = 0
    # def pixel_to_3d(self,u, v, distance, intrinsics):
    #     # Convert pixel coordinates and depth to 3D coordinates
    #     x = (u - intrinsics.ppx) * distance / intrinsics.fx
    #     y = (v - intrinsics.ppy) * distance / intrinsics.fy
    #     z = distance
        
        # return x, y, z
    def detect_objects(self):
        # Reset counters
        self.red_ball_counter = 0
        self.blue_ball_counter = 0
        
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
                # Get box
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls  # Class ID
                class_name = self.model.names[int(c)]  # Class name
                
                # Define coordinate
                x_center = int((b[0] + b[2]) / 2)
                y_center = int((b[1] + b[3]) / 2)
                
                # yaw = ((x_center - 320) / 320 * 45.0) 
                Horizontal_Angle  = ((x_center - self.W/2)/(self.W/2)) * (self.HFOV/2)
                
                # Vertical_Angle = ((y_center - self.H/2)/(self.H/2)* (self.VFOV/2))
                # Yaw = np.sqrt(Horizontal_Angle**2 + Vertical_Angle**2)
                # self.get_logger().info('Yaw :%f ]'%(Yaw))

                # distance = 0.18*480/b[2]

                # diff_x = b[2] - b[0]
                # diff_y = b[3] - b[1]
                # obj_x_center = b[0]+(diff_x/2)
                # obj_y_center = b[1]+(diff_y/2)
       
                depth = depth_frame.get_distance(x_center, y_center)
                
                cv2.rectangle(depth_image, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255),
                            thickness=2, lineType=cv2.LINE_4)
                cv2.putText(depth_image, text=class_name, org=(int(b[0]), int(b[1])),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 255),
                            thickness=2, lineType=cv2.LINE_4)

                # detect red
                if int(c) == 1:
                    # object_position = Point()
                    # object_position.x = float(x_center)
                    # object_position.y = float(y_center)
                    # self.ball_position_publisher.publish(object_position)

                    object_distance = Float32MultiArray()
                    object_distance.data = [depth,Horizontal_Angle]
                    Yaw = -1*(np.pi*Horizontal_Angle)/180
                    
                    self.ball_distance_publisher.publish(object_distance)

                    self.red_ball_counter += 1
                    self.get_logger().info('distance :%f,radian : %f, degree: %f]'%(depth,Yaw,-1*Horizontal_Angle))

                # Check if the detected object is a blue ball (Class ID 0)
                elif int(c) == 0:
                    self.blue_ball_counter += 1
                    pass
                
                #  # detect blue
                # if int(c) == 0:
                #     object_position = Point()
                #     object_position.x = float(x_center)
                #     object_position.y = float(y_center)
                #     self.ball_position_publisher.publish(object_position)

                #     object_distance = Float64()
                #     object_distance.data = depth
                #     self.ball_distance_publisher.publish(object_distance)

                #     self.red_ball_counter += 1

                # # Check if the detected object is a blue ball (Class ID 0)
                # elif int(c) == 1:
                #     self.blue_ball_counter += 1
                #     pass

        # Publish total ball count
        total_count = self.red_ball_counter + self.blue_ball_counter
        total_count_msg = Int32()
        total_count_msg.data = total_count
        self.ball_count_publisher.publish(total_count_msg)

        annotated_frame = results[0].plot()

        ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.image_publisher.publish(ros_image)

        ros_raw_depth_image = self.bridge.cv2_to_imgmsg(depth_image, "passthrough")
        self.depth_publisher.publish(ros_raw_depth_image)


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
