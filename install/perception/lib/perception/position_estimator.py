#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
#from fs_msgs.msg import Cone

bridge = CvBridge()

class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')
        self.detections_sub = Subscriber(self, Yolov8Inference, "/Yolov8_Inference")
        self.image_left_sub = Subscriber(self, Image, "/fsds/cameracam1/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cameracam2/image_color")
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub,self.detections_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)

    def sync_callback(self, img_left,img_right,det):
        img_left_temp = img_left.header.stamp.sec
        img_right_temp = img_right.header.stamp.sec
        det_temp = det.header.stamp.sec
        self.get_logger().info(f'Sync callback with {img_left_temp} and {img_right_temp} as times')
        
    
    
    



def main(args=None):
    rclpy.init(args=args)

    position_estimator = PositionEstimator()

    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()