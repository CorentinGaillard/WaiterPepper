from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import Camera
#from qibullet import joint_parameters
import pybullet as p
import math,time,random
from Cam import Cam,HumanCam
from Laser import Laser
from itertools import combinations
import pybullet_data
import numpy as np



def genereObjs():  
    print('populating world with tables and chairs')                              
    objs=[]
    for i in range(10):
        x,y = random.random()*30-15,random.random()*30-15
        a = random.random()*2*math.pi-math.pi
        objs.append((p.loadURDF("urdf/chair_1/chair.urdf", 
                            basePosition=[x,y,0],
                            globalScaling=1,
                            baseOrientation=p.getQuaternionFromEuler((0,0,a))),
                        'chair'))
                                            
    
    path = pybullet_data.getDataPath() + '/'
    for i in range(10):
        x,y = random.random()*30-15,random.random()*30-15
        a = random.random()*2*math.pi-math.pi
        objs.append((p.loadURDF(path+'table/table.urdf', 
                            basePosition=[x,y,0],
                            globalScaling=1,
                            baseOrientation=p.getQuaternionFromEuler((0,0,a))),
                        'table'))
    
    #checking that there is no overlapping
    print('removing overlapping object')
    dist_min = 1
    for (obj1,_),(obj2,_) in combinations(objs,2):
        (x1,y1,z1),a1=p.getBasePositionAndOrientation(obj1)
        (x2,y2,z2),a2=p.getBasePositionAndOrientation(obj2)
        if abs((x1**2+y1**2)**0.5-(x2**2+y2**2)**0.5)<dist_min:
            print('.',end='')
            if random.random()<0.5:
                try: objs.pop(objs.index((obj1,x1,y1,a1))) 
                except : pass
            else:
                try: objs.pop(objs.index((obj2,x2,y2,a2)))
                except: pass 
    print(f'\ndone, {len(objs)} objects generated')    
    return objs

def goToPosture(pepper,posture):
    for name,value in posture.items():
            pepper.setAngles(name,value,1.0)

def changeObj(obj,angle=None,position=None):
    if angle is not None:
        quat = p.getQuaternionFromEuler(angle)
        pos,_ = p.getBasePositionAndOrientation(obj)
        p.resetBasePositionAndOrientation(obj,pos,quat)
    if position is not None:
        _,ang = p.getBasePositionAndOrientation(obj)
        p.resetBasePositionAndOrientation(obj,position,ang)

def avancePepper(pepper,speed):
    pepper.moveTo(speed,0,0,_async=True)
    
def pushChair(pepper,chair):
    pos=pepper.getPosition()
    pos = list(pos)
    pos[1]+=0.5
    pos[2]=0
    changeObj(chair,position=pos)

def turnToObj(pepper,camera,obj='chair'):
    print(f'looking for {obj}')
    closestYEver=10
    inc = 60
    closestY=10
    while not(0.495<closestYEver<0.505):
        if 0.4<closestYEver<0.6:
            inc = 120
        else: inc = 60
        direction=-abs(closestY-0.5)/(closestY-0.5)
        pepper.moveTo(0,0,direction*math.pi/inc)
        time.sleep(0.5)
        closestY=10
        for center in camera.centers:
            if center[0]!=obj.upper():
                continue
            if abs(center[1][0]-0.5) < abs(closestY-0.5):
                closestY=center[1][0]
        if abs(closestY-0.5) < abs(closestYEver-0.5):
            closestYEver = closestY
        print(closestYEver) 
    print('I got One')

def goToObj(pepper,camera,obj='chair'):
    t = time.time()+10
    while 1:
        if t-time.time()>5:
            turnToObj(pepper,cam,obj)
            t = time.time()
        pepper.moveTo(5,0,0,_async=True)
        for l in pepper.getFrontLaserValue():
            if l<0.5: 
                pepper.stopMove()
                return
        

def guideToTable(pepper):
    pepper.moveTo(4,0,0)
    pepper.moveTo(0,0,math.pi/2)
    #accompHumanToTable(pepper)

def accomHumanToTable(pepper):
    far=True
    while far:
        avancePepper(pepper,0.5)
        for l in pepper.getFrontLaserValue():
            if l<0.3:
                far=False
    #checkOnHuman()

def bringBackChair(pepper,cam): 
    global chair,grabChairPosture
    start = pepper.getPosition()
    chairGrabed=False
    print('going to the chair')
    if DEMO_TURN_TO_OBJ:
        goToObj(pepper,cam,'chair')
    else:
        pepper.moveTo(5,0,0,_async=True)
        a=1
        while a:
            for l in pepper.getFrontLaserValue():
                if l<0.5: 
                    a = 0
                    print('ok')
                    pepper.stopMove()
    
    print('grabing the chair')
    goToPosture(pepper,grabChairPosture)
    if DEMO_TURN_TO_OBJ:
        for obj,_ in objs:
            posc,_ = p.getBasePositionAndOrientation(obj)
            if np.allclose(posc[:2],pepper.getPosition()[:2],atol=0.7):
                chair = obj
                break
            
    print('going back')
    x1,y1,_ = start
    x2,y2,_ = pepper.getPosition()
    dist = ((x2-x1)**2+(y2-y1)**2)**0.5
    print(dist)
    pepper.moveTo(-dist,0,0,_async=True)
    while not np.allclose(pepper.getPosition()[:2],start[:2]):
        pushChair(pepper,chair)
    print("I'm back")

if __name__=='__main__':
    DEMO_CAM_HUMAN = 1
    DEMO_TURN_TO_OBJ = 0
    DEMO_BRING_BACK_CHAIR = 0

    defaultPosture = {'LShoulderPitch':1.427,
                      'RShoulderPitch':1.427}
    grabChairPosture = {'LShoulderPitch':0.6,
                         'LShoulderRoll':0,
                         'LElbowYaw':-1.7,
                         'LElbowRoll':-1.4,
                         'LWristYaw':1.5,
                         'RShoulderPitch':0.6,
                         'RShoulderRoll':0,
                         'RElbowYaw':1.7,
                         'RElbowRoll':1.4,
                         'RWristYaw':-1.5}


    simulation_manager = SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True)

    if not DEMO_CAM_HUMAN:
        pepper = simulation_manager.spawnPepper(client_id,
                                            spawn_ground_plane=True,
                                            translation=[-3,-1,0])
        cam = Cam(pepper,"top_cam",do_predictions=DEMO_TURN_TO_OBJ)
        cam.start()
        laser = Laser(pepper)
        laser.start()
        goToPosture(pepper,defaultPosture)

                              
    if DEMO_CAM_HUMAN:
        human  = simulation_manager.spawnPepper(client_id,
                                              spawn_ground_plane=True,
                                              translation=[0,0,0])
        humanCam = HumanCam(human)
        humanCam.start()
      
      


    p.connect(p.DIRECT)

    if DEMO_BRING_BACK_CHAIR and not DEMO_TURN_TO_OBJ:
        chair = p.loadURDF("urdf/chair_1/chair.urdf", basePosition=[0,-1,0],
                globalScaling=1, baseOrientation=p.getQuaternionFromEuler((0,0,math.pi)))

    time.sleep(3) #waiting for pepper and all to be initialized

    #path = pybullet_data.getDataPath() + '/'
    #table = p.loadURDF(path+'table/table.urdf', basePosition=[1,0,0],
    #        globalScaling=1, baseOrientation=p.getQuaternionFromEuler((0,0,math.pi)))

    try:
        #objs = genereObjs()
        #turnToObj(pepper,cam,'table')
        #guideToTable(pepper)
        #turnToObj(pepper,cam,'chair')
        #bringBackChair(pepper,cam)
        #simulation_manager.stopSimulation(client_id)
        
        if DEMO_CAM_HUMAN: #demo caméra Humain
            while True: pass
        if DEMO_TURN_TO_OBJ: #turnToObj
            objs = genereObjs()
            turnToObj(pepper,cam,'chair')
        if DEMO_BRING_BACK_CHAIR: #bringBackChair
            bringBackChair(pepper,cam)
        while True: pass
        simulation_manager.stopSimulation(client_id)    
    except KeyboardInterrupt:
        simulation_manager.stopSimulation(client_id)
