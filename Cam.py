from qibullet import SimulationManager
from qibullet import RomeoVirtual
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import Camera
#from qibullet import joint_parameters
import cv2
import threading,time
import numpy as np
import pybullet as p
from math import pi

class Cam(threading.Thread):
    """ 
    ID_CAM : 

    top_cam -- pour ID_CAMERA_TOP, 
    bottom_cam -- ID_CAMERA_BOTTOM, 
    depth_cam -- pour ID_CAMERA_DEPTH

    RESOLUTION : 

     K_QQVGA = CameraResolution(160, 120)
     K_QVGA = CameraResolution(320, 240)
     K_VGA = CameraResolution(640, 480)
     K_QQ720p = CameraResolution(320, 180)
     K_Q720p = CameraResolution(640, 360) 
     K_720p = CameraResolution(1280, 720)
    """
    def __init__(self,pepper,id_cam,do_predictions=False,draw_predictions=True):
        threading.Thread.__init__(self)
        self.pepper=pepper
        self.id_cam=id_cam
        self.do_prediction=do_predictions
        self.disable_cam = False
        self.whT = 320
        self.confThreshold = 0.5
        self.nmsThreshold = 0.8
        self.classNames = []
        self.draw = draw_predictions
        self.centers=[]
        
    
    def __del__(self):
        print("detruire Cam")

    def run(self):
        print("i'm in")
        if self.do_prediction:
            modelConfiguration = "configs yolo/custom-yolov4-detector.cfg"
            modelWeights = 'configs yolo/custom-yolov4-detector_best.weights'
            net = cv2.dnn.readNetFromDarknet(modelConfiguration,modelWeights)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            classesFile = 'configs yolo/coco.names'

            with open(classesFile,'rt') as f:
                self.classNames = f.read().rstrip('\n').split('\n')+['table','chair']


        if self.id_cam == "top_cam" :
                handle = self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, resolution=Camera.K_VGA)
        elif self.id_cam == "depth_cam" :
                handle = self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH, resolution=Camera.K_VGA)
        elif self.id_cam == "bottom_cam" :
                handle = self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM, resolution=Camera.K_VGA)

        while True :
                img = self.pepper.getCameraFrame(handle)
                if self.do_prediction:
                    blob = cv2.dnn.blobFromImage(img,1/255,(self.whT,self.whT),[0,0,0],1,crop=False)
                    net.setInput(blob)
                    
                    layerNames = net.getLayerNames()
                    #print(layerNames)
                    outputNames = [layerNames[i[0]-1] for  i in net.getUnconnectedOutLayers()]

                    outputs = net.forward(outputNames)
                    centers = self.findObjects(outputs,img)
                cv2.imshow('Image',img)

                cv2.waitKey(1)
                if self.disable_cam == True:
                    self.pepper.unsubscribeCamera(handle)
                    break

                if cv2.waitKey(1)!=-1:
                    break

    def findObjects(self, outputs,img):
        hT,wT,cT = img.shape
        bbox=[]
        classIds=[]
        confs=[]
        centers,tmp_centers=[],[]
        
        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w,h = int(det[2]*wT), int(det[3]*hT)
                    x,y = int(det[0]*wT-w/2), int(det[1]*hT-h/2)
                    tmp_centers.append([det[0],det[1]])
                    bbox.append([x,y,w,h])
                    classIds.append(classId)
                    confs.append(float(confidence))
            indices = cv2.dnn.NMSBoxes(bbox,confs,self.confThreshold,self.nmsThreshold)
            
            for i in indices:
                i = i[0]
                name = self.classNames[classIds[i]].upper()
                if name=='BICYCLE': name='TABLE'
                if name=='PERSON': name='CHAIR'
                centers.append((name,tmp_centers[i]))
                if self.draw:
                    box = bbox[i]
                    x,y,w,h = box
                    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,255),2)
                    cv2.putText(img,f'{name}'+
                                    f'{int(confs[i]*100)}%',
                                (x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
        self.centers = centers
                            
    def setDisableCam(self,bol):
        self.disable_cam = bol







class HumanCam(threading.Thread):
    def __init__(self,pepper):
        threading.Thread.__init__(self)
        self.pepper=pepper
        
    def __del__(self):
        print("detruire Cam")

    def on_press(self,keys):
        z_,q_,s_,d_=ord('z'),ord('q'),ord('s'),ord('d')
        
        print('op_press',keys)
        if z_ in keys and keys[z_]&2:
            self.pepper.move(1,0,0)
        elif s_ in keys and keys[s_]&2:
            self.pepper.move(-1,0,0)
        elif q_ in keys and keys[q_]&2:
            self.pepper.move(0,0,pi)
        elif d_ in keys and keys[d_]&2:
            self.pepper.move(0,0,-pi)
            
    def on_release(self,keys):
        z_,q_,s_,d_=ord('z'),ord('q'),ord('s'),ord('d')
        if (z_ in keys and keys[z_]&p.KEY_WAS_RELEASED) or (q_ in keys and keys[q_]&p.KEY_WAS_RELEASED) or (s_ in keys and keys[s_]&p.KEY_WAS_RELEASED) or (d_ in keys and keys[d_]&p.KEY_WAS_RELEASED):
            self.pepper.move(0,0,0)
            self.pepper.stopMove()

    def run(self):
        #listener = keyboard.Listener(on_press=self.on_press,
        #                             on_release=self.on_release)
        #listener.start()
        handle = self.pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, resolution=Camera.K_720p)
        while True:
            events = p.getKeyboardEvents()
            self.on_press(events)
            self.on_release(events)
            img = self.pepper.getCameraFrame(handle)
            cv2.imshow('Human Vision',img)
            cv2.waitKey(1)
    



























