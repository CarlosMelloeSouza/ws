#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import Track
from fs_msgs.msg import Cone
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from stereo_msgs.msg._disparity_image import DisparityImage
from position_estimation.stereo_pipeline import StereoPipeline
import sensor_msgs_py.point_cloud2 as pc2
from position_estimation.disparity_estimator import DisparityEstimator
import yaml

bridge = CvBridge()
keypoints_path="/home/carlosmello/ws/src/Perception/position_estimation/keypoints_net_v1-1.pt"
left_path = "/home/carlosmello/ws/src/Perception/config/fsds_left.yaml"
right_path = "/home/carlosmello/ws/src/Perception/config/fsds_right.yaml"

class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')
        

        self.disparity_sub = Subscriber(self,DisparityImage,"/sm2/disparity/disparity_image")
        self.detections_sub = Subscriber(self, Yolov8Inference, "/Yolov8_Inference")
        self.image_left_sub = Subscriber(self, Image, "/fsds/cameracam2/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cameracam1/image_color")
        self.publishers_ = self.create_publisher(Track, '/position_estimation/track',10)
        self.publishers_point_clound = self.create_publisher(PointCloud2, '/position_estimation/point_clound',10)
        
        
        queue_size = 10
        max_delay = 1
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.detections_sub,self.disparity_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        

        with open(left_path) as arquivo:
            self.left_camera_info = yaml.load(arquivo, Loader=yaml.FullLoader)

        with open(right_path) as arquivo:
            self.right_camera_info = yaml.load(arquivo,Loader=yaml.FullLoader)
        self.disparity=DisparityEstimator(self.left_camera_info)
        

    def sync_callback(self, img_left,yolo_result,disp_map):

        
        cv2disp_map=bridge.imgmsg_to_cv2(disp_map.image)
        cv2img_left=bridge.imgmsg_to_cv2(img_left)
        
        track=self.disparity.get_object_on_map(cv2img_left,cv2disp_map,yolo_result.yolov8_inference,disp_map.t,disp_map.f)
        self.publishers_.publish(track)
        pointclound=self.track_to_point_cloud(track)
        self.publishers_point_clound.publish(pointclound)

    
    def track_to_point_cloud(self,track_msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "/down"  # Ajuste para o frame de referência correto

        points = []
        for cone in track_msg.track:  # Supondo que track_msg.tracks é a lista de rastreamentos
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            points.append([x, y, z])

    # Define os campos da nuvem de pontos
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            
        ]
        

    # Cria a mensagem PointCloud2
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        return pointcloud_msg
    
    
    

        
    
    
    



def main(args=None):
    rclpy.init(args=args)


    position_estimator = PositionEstimator()
    #position_estimator.get_logger().info(args)

    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()