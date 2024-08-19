import torch
import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult


bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
          path='/home/jsagx-6/itc01_ws/src/itc01_mr2/cv_test/cv_test/best_18.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)
        self.subscription

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 30)
        self.img_pub = self.create_publisher(Image, "/inference_result", 30)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data)
        cvt = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        results = self.model(cvt)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results.pred[0]:
            self.inference_result = InferenceResult()
            b = r.tolist()
            # print(b)
            self.inference_result.class_name = self.model.names[int(b[5])] # string of the ball
            # print(c)
            self.inference_result.top = int(b[3])
            self.inference_result.bottom = int(b[1])
            self.inference_result.left = int(b[0])
            self.inference_result.right = int(b[2])
            # self.yolov8_inference.append(self.inference_result)
            self.yolov8_inference.yolov8_inference.append(self.inference_result)
            
            print(self.yolov8_inference)
            # self.yolov8_pub.publish(self.yolov8_inference)

        self.yolov8_pub.publish(self.yolov8_inference)
        annotated_frame = results.render()[0]
        cvt_2 = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        img_msg = bridge.cv2_to_imgmsg(cvt_2)

        self.img_pub.publish(img_msg)
        
        self.yolov8_inference.yolov8_inference.clear()  # we can see it in yolobot subscriber.command this line to see the code in sub_9.
        cv2.imshow("camera",cvt_2 )
        cv2.waitKey(1)
def main(args=None):
  rclpy.init(args=args)
  camera_subscriber = CameraSubscriber()
  rclpy.spin(camera_subscriber)
  camera_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()


