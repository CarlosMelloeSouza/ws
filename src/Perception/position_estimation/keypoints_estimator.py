import cv2
import torch.nn as nn
import numpy as np
import torch
from ultralytics import YOLO
from torchvision import transforms

class KeypointsEstimator:
    def __init__(self,camera_info,keypoints_path,yolo):
        self.model_keypoints = KeyPoints_Net()
        self.keypoints_net = self.model_keypoints.load_state_dict(torch.load(keypoints_path, map_location=torch.device('cuda')))
        self.model_keypoints.eval()
        self.model_keypoints.cuda()
        self.object3D_points = np.array([[0.0,0.0 , 227.0], [0.0, -37.0, 144.0], [0.0, 37.0, 144.0], [0.0, -48.0, 78.0], [0.0, 48.0, 78.0], [0.0, -60.0, 0.0], [0.0, 60.0, 0.0]], dtype=np.float32)
        self.camera_intrinsics=camera_info #garantir que seja de fato os intrisicos da camera
    
    
    def get_image_keypoints(self,boundingbox_list,image):
        keypoints_list=[]
        
        for box in boundingbox_list:
            x1,y1 = int(box[0]),int(box[1])
            x2,y2 = int(box[2]),int(box[3])
            crop = image[y1:y2,x1:x2]
            imagem_redimencionada=cv2.resize(crop,(80,80))
            im = image.fromarray(cv2.cvtColor(imagem_redimencionada, cv2.COLOR_BGR2RGB))
            test_transforms = transforms.Compose([transforms.Resize((80, 80)), transforms.ToTensor()])
            im = test_transforms(im)
            pytorch_image = im.unsqueeze(0).to('cuda')
            result = self.model_keypoints(pytorch_image)
            result = result.tolist()[0]
            keypoints_x,keypoints_y = result[0],result[1]
            largura_orig=x2-x1
            altura_orig= y2-y1
            xy_imag=[]
            for j, (x, y) in enumerate(zip(result[::2], result[1::2])):
                x_keypoint = int(((x / 80) * (x2 - x1)) + x1)
                y_keypoint = int(((y / 80) * (y2 - y1)) + y1)

                xy_imag.append([x_keypoint,y_keypoint])
           
            keypoints_list.append(xy_imag)
        return keypoints_list

    def get_boxes(self,result_f):
        boundingbox_list=[]
        conf_list=[]
        label_list=[]
        for l in result_f:
            boxes = l.boxes
            
            for box in boxes:
                
                boundingbox_list.append(box.xyxy[0])
                conf_list.append(box.conf[0])
                label_list.append(box.cls[0])
        
        self.boundingbox_list=boundingbox_list
        return boundingbox_list,conf_list,label_list

    def get_position_estimation(self, image):
        cameraMatrix = self.camera_intrinsics
        bounding_boxes = self.yolo_model.predict(image)

        boundingbox_list,conf_list,label_list = self.get_boxes(bounding_boxes)
        keypoints = self.get_image_keypoints(boundingbox_list,image)
        cameraMatrix = np.array(cameraMatrix, dtype=np.float32)

        obstacles = list()
        rvec_list=[]
        tvec_list=[]
        
        
        for i,objects in enumerate(keypoints):
            
            xy_imag = np.array([objects], dtype=np.float32)
            funciona,rvec,tvec= cv2.solvePnP(self.object3D_points, xy_imag, cameraMatrix, np.array(self.distortion),flags=0)
            obstacles.append(Obstacle(tvec[0][0],tvec[2][0],conf_list[i].item(),label_list[i].item())) 
            rvec_list.append(rvec)
            tvec_list.append(tvec)
            
            
            
        timing=self.get_timing()
        return obstacles,timing,rvec_list,tvec_list








class Obstacle:

    def __init__(self, x, y, confidence, label,deviation=1e-15, count=1, id=0):
        self.x = x
        self.y = y
        self.confidence = confidence
        self.label = label
        self.count = count
        self.deviation = deviation
        self.id = id

class ResNet(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ResNet, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=2, padding=4)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=2, padding=4)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.relu2 = nn.ReLU()

        self.shortcut_conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=1, stride=1)
        self.shortcut_bn = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        c1 = self.conv1(x)
        b1 = self.bn1(c1)
        act1 = self.relu1(b1)
        out = self.bn2(self.conv2(act1))
        return out

class KeyPoints_Net(nn.Sequential):
    def __init__(self):
        super().__init__(
           nn.Conv2d(in_channels=3,out_channels=64,kernel_size=(11,11), stride=10, dilation=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            ResNet(in_channels=64, out_channels=64),
            ResNet(in_channels=64, out_channels=128),
            ResNet(in_channels=128, out_channels=256),
            ResNet(in_channels=256, out_channels=512),
            nn.Flatten(),
            nn.Linear(in_features=(512*7*7), out_features = 14))