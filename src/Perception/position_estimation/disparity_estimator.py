import cv2 
from fs_msgs.msg import Track
from fs_msgs.msg import Cone
import numpy as np
import rclpy

class DisparityEstimator:
    def __init__(self,intrinsics):
        self.intrinsic_matrix=intrinsics['camera_matrix']['data']

        
    def get_object_on_map(self,left_img,disp_map,yolo,baseline,focal_length):
        cone_list=[]

        self.bb_on_left=yolo
        self.left_img=left_img
        self.disp_map=disp_map
        
        track=Track()
        
        
        for box in self.bb_on_left:
            cone = Cone()
            cor=box.class_name
            centro_x=int((int(box.top)+int(box.bottom))/2)
            centro_y=int((int(box.left)+int(box.right))/2)
            lado=15
            disp_map=np.array(disp_map)
        
            disp_slice=disp_map[(centro_y-lado//2):(centro_y+lado//2),(centro_x-lado//2):(centro_x+lado//2)]
        
            disparity=np.median(disp_slice)
            
            if disparity >= 85:
                X,Y,Z=self.triangulation(self.disp_map,centro_x,centro_y,baseline,focal_length)
                cone.location.x = X
                cone.location.y = Y
                cone.location.z = Z
                if cor == 'blue_cone':
                    cone.color=0
                elif cor == 'yellow_cone':
                    cone.color=1
                elif cor == 'large_orange_cone':
                    cone.color=2
            
                cone_list.append(cone)
        
            # cv2.circle(left_img,(centro_x,centro_y),1,(0,0,255))
            # cv2.putText(left_img,str(Z),(centro_x,centro_y),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2,cv2.LINE_AA)
            # cv2.imshow('left image',left_img)
            # cv2.waitKey(0)
        track.track=cone_list
        
        return track
    
    def triangulation(self,disp_map,ponto_x,ponto_y,baseline,focal_length):
        lado=15
        disp_map=np.array(disp_map)
        
        disp_slice=disp_map[(ponto_y-lado//2):(ponto_y+lado//2),(ponto_x-lado//2):(ponto_x+lado//2)]
        
        disparity=np.median(disp_slice)


        cx=self.intrinsic_matrix[0][2]
        cy=self.intrinsic_matrix[1][2]
        
        focal_length_x=self.intrinsic_matrix[0][0]
        focal_length_y=self.intrinsic_matrix[1][1]
        
        baseline=0.32
        #rclpy.get_logger().info(disparity)
        
        Z=((baseline*focal_length_x)/disparity)
        Z= Z*10
        X=(((ponto_x-cx)*Z)/focal_length_x)
        Y=(((ponto_y-cy)*Z)/focal_length_y)
        
        return X,Y,Z


        
        