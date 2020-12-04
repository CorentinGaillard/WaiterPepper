import cv2,time,math,random
import numpy as np
from qibullet import SimulationManager
from qibullet import RomeoVirtual, PepperVirtual, NaoVirtual
import pybullet as p
import pybullet_data
from PIL import Image
from itertools import combinations

def changeObj(obj,angle=None,position=None):
    if angle is not None:
        quat = p.getQuaternionFromEuler(angle)
        pos,_ = p.getBasePositionAndOrientation(obj)
        p.resetBasePositionAndOrientation(obj,pos,quat)
    if position is not None:
        _,ang = p.getBasePositionAndOrientation(obj)
        p.resetBasePositionAndOrientation(obj,position,ang)
    
def setVisible(obj,visible=True):
    pos,ang = p.getBasePositionAndOrientation(obj)
    pos = list(pos)
    if visible:
        pos[-1]=0        
    else:
        pos[-1]=-100
    p.resetBasePositionAndOrientation(obj,pos,ang)

simulation_manager = SimulationManager()
client_id = simulation_manager.launchSimulation(gui=True)
pepper = simulation_manager.spawnPepper(client_id,
                                        spawn_ground_plane=True,
                                        translation=[0,0,0],
                                        quaternion=p.getQuaternionFromEuler((0,0,math.pi/2)))
p.connect(p.DIRECT)
path = pybullet_data.getDataPath() + '/'
table0 = p.loadURDF(path+'table/table.urdf',basePosition=[-100,-100,-100])
table1 = p.loadURDF(path+'table/table.urdf',basePosition=[-100,-100,-100])
table2 = p.loadURDF(path+'table/table.urdf',basePosition=[-100,-100,-100])

chair0 = p.loadURDF("urdf/chair_1/chair.urdf", basePosition=[-100,-100,-100],
        globalScaling=1)#, baseOrientation=[0,0,1,0])
chair1 = p.loadURDF("urdf/chair_1/chair.urdf", basePosition=[-100,-100,-100],
        globalScaling=1)#, baseOrientation=[0,0,1,0])
chair2 = p.loadURDF("urdf/chair_1/chair.urdf", basePosition=[-100,-100,-100],
        globalScaling=1)#, baseOrientation=[0,0,1,0])
        
tables=[table0,table1,table2]
chairs=[chair0,chair1,chair2]
    
handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)


nAngles = 10

angles = np.linspace(0,math.pi*2,nAngles)


def checkCollisions(coords):
    #coords is a list of tuples
    assert type(coords)==list
    assert len(coords)>1
    assert type(coords[0])==tuple
    

    

'''
objs = [table,chair1]
    for obj in objs:
        setVisible(obj,True)
        print('loooooop')
        for x in range(-15,15):
            for y in range(0,15,1):
                if y<=3*abs(x)-3 or y==0:
                    continue
                print('loop')
                changeObj(obj,position=[x,y,0])
                for angle in angles:
                  changeObj(obj,angle=[0,0,angle])
                  img = pepper.getCameraFrame(handle) 
                  cv2.imshow('Image',img)
                  cv2.imwrite(f'db/img_{obj}_{x}_{y}_{int(angle*100)}.png',img)
                #time.sleep(0.1)
                  cv2.waitKey(1)
            
        setVisible(obj,False)
'''

try:
    for i in range(500):
        print(i)
        nb_table = int(random.random()>0.05)+int(random.random()>0.8)+int(random.random()>0.95)
        nb_chair = int(random.random()>0.05)+int(random.random()>0.8)+int(random.random()>0.95)

        
        idxya=[]#(idObj,x,y,angle)
        for table in range(nb_table):
            x = random.random()*30-15
            y = random.random()*15
            angle = random.random()*2*math.pi
            while (y<=3*abs(x)-3) or y<1:
                x = random.random()*30-15
                y = random.random()*15
                    
            idxya.append((tables[table],x,y,angle))
        
        for chair in range(nb_chair):
            x = random.random()*30-15
            y = random.random()*15
            angle = random.random()*2*math.pi
            while (y<=3*abs(x)-3) or y<1:
                x = random.random()*30-15
                y = random.random()*15
                    
            idxya.append((chairs[chair],x,y,angle))

        #checking that there is no overlapping
        dist_min = 1
        for (obj1,x1,y1,a1),(obj2,x2,y2,a2) in combinations(idxya,2):
            if abs((x1**2+y1**2)**0.5-(x2**2+y2**2)**0.5)<dist_min:
                if random.random()<0.5:
                    try: idxya.pop(idxya.index((obj1,x1,y1,a1))) 
                    except : pass
                else:
                    try: idxya.pop(idxya.index((obj2,x2,y2,a2)))
                    except: pass 
                print('removing a object to avoid collisions')


        for (obj,x,y,angle) in idxya:
            changeObj(obj,position=[x,y,0],angle=[0,0,angle])       
        
        
        img = pepper.getCameraFrame(handle)
        #cv2.imshow('Image',img)
        cv2.imwrite(f'db2/img_{i}.png',img)
        time.sleep(0.1)
        
        for (obj,_,_,_) in idxya:
            setVisible(obj,False)


        
    print('done')
    simulation_manager.stopSimulation(client_id)
except KeyboardInterrupt:
    simulation_manager.stopSimulation(client_id)
