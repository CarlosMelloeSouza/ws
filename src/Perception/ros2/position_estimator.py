#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from position_estimation.stereo_pipeline import StereoPipeline

bridge = CvBridge()
keypoints_path="/home/carlosmello/ws/src/Perception/position_estimation/keypoints_net_v1-1.pt"

class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')

        
        self.detections_sub = Subscriber(self, Yolov8Inference, "/Yolov8_Inference")
        self.image_left_sub = Subscriber(self, Image, "/fsds/cameracam1/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cameracam2/image_color")
        self.camera_left_info_sub = Subscriber(self, CameraInfo, "/fsds/cameracam1/camera_info")
        self.camera_right_info_sub = Subscriber(self, CameraInfo, "/fsds/cameracam2/camera_info")
        self.vision_pipeline=StereoPipeline(self.camera_left_info_sub,self.camera_right_info_sub)
        queue_size = 10
        max_delay = 0.015
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub,self.detections_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)

    def sync_callback(self, img_left,img_right,yolo_result):
        img_left_temp = img_left.header.stamp.sec
        img_right_temp = img_right.header.stamp.sec
        det_temp = yolo_result.header.stamp.sec
        cv2img_left=bridge.cv2_to_imgmsg(img_left, desired_encoding='passthrough')
        cv2img_right=bridge.cv2_to_imgmsg(img_right)
        objects=self.vision_pipeline.get_object_position(img_left,img_right,yolo_result,keypoints_path)
        self.get_logger().info(f'cone position: {objects}')
        
    
    
    



def main(args=None):
    rclpy.init(args=args)

    position_estimator = PositionEstimator()

    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()