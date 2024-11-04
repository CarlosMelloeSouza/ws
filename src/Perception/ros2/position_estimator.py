#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
#from fs_msgs.msg import Cone

bridge = CvBridge()

class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')

        #<-- aqui mais ou menos tem que iniciar a classe da camera
        self.detections_sub = Subscriber(self, Yolov8Inference, "/Yolov8_Inference")
        self.image_left_sub = Subscriber(self, Image, "/fsds/cameracam1/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cameracam2/image_color")
        self.camera_left_info_sub = Subscriber(self, CameraInfo, "/fsds/cameracam1/camera_info")
        queue_size = 10
        max_delay = 0.015
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub,self.detections_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)

    def sync_callback(self, img_left,img_right,det):
        img_left_temp = img_left.header.stamp.sec
        img_right_temp = img_right.header.stamp.sec
        det_temp = det.header.stamp.sec
        cv2img_left=bridge.cv2_to_imgmsg(img_left, desired_encoding='passthrough')
        cv2img_right=bridge.cv2_to_imgmsg(img_right)
        #<-- e aqui mais ou menos tem que chamar a funçao da classe da camera que estima a posiçao 
        self.get_logger().info(f'Sync callback with {img_left_temp} , {img_right_temp} and {det_temp} as times')
        
    
    
    



def main(args=None):
    rclpy.init(args=args)

    position_estimator = PositionEstimator()

    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()