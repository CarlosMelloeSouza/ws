#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cone_detection import ConeDetection

yoloPath="a"


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/fsds/cameracam1/image_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.stereo=ConeDetection(yoloPath)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.width)
        self.stereo.get_boundingbox(self.br.imgmsg_to_cv2(msg))
        #Processamento de image aqui
        


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber(yolopath)

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()