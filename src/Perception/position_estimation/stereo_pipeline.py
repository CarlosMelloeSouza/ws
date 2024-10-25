import cv2

class StereoPipeline:
    def __init__(self,cam_left_info,cam_right_info):
        self.left_intrinsics=cam_left_info.header.k
        self.right_intrinsics=cam_right_info.header.k


    def boundingbox_propagator(self, img_left,img_right,):
        pass
