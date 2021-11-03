import pybullet as p
import time
import numpy as np
import pybullet_data

# class Dog:
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("/plane.urdf") # "plane.urdf"
p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)
#p.setDefaultContactERP(0)
#urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("a1/urdf/a1.urdf",[0,0,0.48],[0,0,0,1], flags = urdfFlags,useFixedBase=False) # "a1/urdf/a1.urdf"

p.createMultiBody(0, 0)

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.05]) 

mass = 20
visualShapeId = -1

basePosition = [0.05,0,1]
baseOrientation = [0,0,0,1]

p.setRealTimeSimulation(0)
timeStep = 0.001
Kp = 15.
Kd = 0.5
lower_legs = [2,5,8,11]

# for l0 in lower_legs:
#     for l1 in lower_legs:
#         if (l1>l0):
#             enableCollision = 1 # enableCollision = 1
#             # print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12], 
#             # 		p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
#             p.setCollisionFilterPair(quadruped, quadruped, 2,5,enableCollision)

jointIds=[]
paramIds=[]

maxForceId = p.addUserDebugParameter("maxForce",0,100,10.)
massId = p.addUserDebugParameter("mass",0.1,100,10)
mass_c = p.readUserDebugParameter(massId)
boxUid = p.createMultiBody(mass_c, colBoxId, visualShapeId, basePosition, baseOrientation)

for j in range (p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped,j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        jointIds.append(j)

# print(jointIds) # [2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 15, 16]

p.getCameraImage(480,320)
p.setRealTimeSimulation(0)

joints=[]
test_angles = []

calfs = [4,8,12,16]
hips = [2, 6, 10, 14]
thighs = [3,7,11,15]
hipsAndThighs = [2,3,6,7,10,11,14,15]
joints = [0.033172, 0.662264, -1.258133, -0.030680,0.624135, -1.258133,0.041513, 0.678391, -1.258133,-0.051197, 0.645650,-1.258133]

while(1):
    maxForce = p.readUserDebugParameter(maxForceId)
    mass_c = p.readUserDebugParameter(massId)

    p.changeDynamics(boxUid, linkIndex=-1, mass=mass_c, lateralFriction=0.7)    

    quadPos, quadOrn = p.getBasePositionAndOrientation(quadruped)
                               
    for j in range(12):
        if j!=2 or j!=5 or j!=8 or j!=11:
          targetPos = float(joints[j])
          p.setJointMotorControl2(quadruped, jointIndex=jointIds[j], controlMode=p.POSITION_CONTROL, targetPosition=targetPos, force=maxForce)

    for j in range(4):
        p.setJointMotorControl2(quadruped, 
                                jointIndex=calfs[j], 
                                controlMode=p.TORQUE_CONTROL, 
                                targetPosition=-1.258133, 
                                targetVelocity=0,
                                positionGain=timeStep * (Kp / 10.),
                                velocityGain=Kd, 
                                force=maxForce)



    # p.setJointMotorControl2(quadruped, 
    #                         jointIndex=4, 
    #                         controlMode=p.TORQUE_CONTROL, 
    #                         targetPosition=-1.258133, 
    #                         targetVelocity=0,
    #                         positionGain=timeStep * (Kp / 10.),
    #                         velocityGain=Kd, 
    #                         force=maxForce)
    # p.setJointMotorControl2(quadruped, 
    #                         jointIndex=8, 
    #                         controlMode=p.TORQUE_CONTROL, 
    #                         targetPosition=-1.258133, 
    #                         targetVelocity=0,
    #                         positionGain=timeStep * (Kp / 10.),
    #                         velocityGain=Kd, 
    #                         force=maxForce)
    # p.setJointMotorControl2(quadruped, 
    #                         jointIndex=12, 
    #                         controlMode=p.TORQUE_CONTROL, 
    #                         targetPosition=-1.258133, 
    #                         targetVelocity=0,
    #                         positionGain=timeStep * (Kp / 10.),
    #                         velocityGain=Kd, 
    #                         force=maxForce)    
    # p.setJointMotorControl2(quadruped, 
    #                         jointIndex=16, 
    #                         controlMode=p.TORQUE_CONTROL, 
    #                         targetPosition=-1.258133, 
    #                         targetVelocity=0,
    #                         positionGain=timeStep * (Kp / 10.),
    #                         velocityGain=Kd, 
    #                         force=maxForce)

            # # for i in range(len(test_angles)):    
            # #     for j in range (12):
            # #         targetPos = float(joints[i][j])
            # #         # print(targetPos,'\n')
            # #         targetVel = 0.5
            # #         p.setJointMotorControl2(quadruped, jointIndex=jointIds[j], controlMode=p.POSITION_CONTROL, targetPosition=targetPos, force=maxForce)

    p.stepSimulation()
    time.sleep(timeStep)

p.disconnect()