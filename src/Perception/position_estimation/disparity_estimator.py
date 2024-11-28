import cv2 
from fs_msgs.msg import Track
from fs_msgs.msg import Cone


class DisparityEstimator:
    def __init__(self,intrinsics):
        self.intrinsic_matrix=intrinsics['rectification_matrix']['data']

        
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
            if disp_map[(centro_x,centro_y)] != 0:
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
        track.track=cone_list

        
        return track
    
    def triangulation(self,disp_map,ponto_x,ponto_y,baseline,focal_length):
        cx=self.intrinsic_matrix[0][2]
        cy=self.intrinsic_matrix[1][2]
        
        Z=(baseline*focal_length)/disp_map[(ponto_x,ponto_y)]
        X=(ponto_x-cx)*Z/focal_length
        Y=(ponto_y-cy)*Z/focal_length
        
        return X*1000,Y*1000,Z*1000


        
        