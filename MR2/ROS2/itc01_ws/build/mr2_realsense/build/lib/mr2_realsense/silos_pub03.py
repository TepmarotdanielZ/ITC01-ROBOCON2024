import torch
import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult

bridge = CvBridge()

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/vipho/abu_farmer/src/abu_farmer/cv_test/cv_test/last.pt') #last.pt
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model.to(device)
conf_threshold = 0.5
iou_threshold = 0.5

def detect_objects(frame):
    # Perform inference
    results = model(frame)

    # Filter detections based on confidence threshold
    detections = results.pred[0]
    filtered_detections = detections[detections[:, 4] > conf_threshold]

    return filtered_detections

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',#'/camera/color/image_raw',
            self.camera_callback,
            10)
        self.subscription

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/silos_Inference", 30)
        self.img_pub = self.create_publisher(Image, "/inference_result", 30)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data)
        cvt = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Perform object detection
        filtered_detections = detect_objects(cvt)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for detection in filtered_detections:
            self.inference_result = InferenceResult()
            
            class_index = int(detection[5])
            class_name = model.names[class_index]
            
            confidence = round(float(detection[4]), 2)
            percentage = f'{confidence * 100:.2f}%'

            # Populate InferenceResult message
            self.inference_result.class_name = class_name
            self.inference_result.top = int(detection[3])
            self.inference_result.bottom = int(detection[1])
            self.inference_result.left = int(detection[0])
            self.inference_result.right = int(detection[2])

            # Print inference result
            print(f'Class: {class_name}, Confidence: {percentage}, Coordinates: ({self.inference_result.left}, {self.inference_result.top}) to ({self.inference_result.right}, {self.inference_result.bottom})')

            self.yolov8_inference.yolov8_inference.append(self.inference_result)


        self.yolov8_pub.publish(self.yolov8_inference)

        annotated_frame = cvt.copy()
        for detection in filtered_detections:
            x1, y1, x2, y2 = map(int, detection[:4])
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f'{class_name}: {percentage}', (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cvt_2 = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        img_msg = bridge.cv2_to_imgmsg(cvt_2)

        self.img_pub.publish(img_msg)

        self.yolov8_inference.yolov8_inference.clear()
        
        cv2.imshow("camera", cvt_2)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
