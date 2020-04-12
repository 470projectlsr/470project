# -*- coding: utf-8 -*-
"""
Created on Sun Apr 12 10:43:50 2020

@author: 10246
"""

import sim
import math
import time
import numpy as np

def SetJointPosition(theta):
	sim.simxSetJointPosition (clientID, joint_one_handle, (theta[0]+180)*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
	sim.simxSetJointPosition (clientID, joint_two_handle, (theta[1]+90)*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
	sim.simxSetJointPosition (clientID, joint_three_handle, theta[2]*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
	sim.simxSetJointPosition (clientID, joint_four_handle, theta[3]*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
	sim.simxSetJointPosition (clientID, joint_five_handle, theta[4]*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
	sim.simxSetJointPosition (clientID, joint_six_handle, theta[5]*np.pi/180, sim.simx_opmode_oneshot)
	time.sleep(0.2)
    
def getobjectposition(sphere1_handle,sphere2_handle):
    errorcode, position1=sim.simxGetObjectPosition(clientID,sphere1_handle,ur3WorldFrame,sim.simx_opmode_blocking)
    if errorcode != sim.simx_return_ok:
        raise Exception('could not get object position for first sphere1 positioon')
        
    errorcode, position2=sim.simxGetObjectPosition(clientID,sphere2_handle,ur3WorldFrame,sim.simx_opmode_blocking)
    if errorcode != sim.simx_return_ok:
        raise Exception('could not get object position for first sphere2 position')
    
    x1=position1[0]
    y1=position1[1]
    
    x2=position2[0]
    y2=position2[1]
    
    d = np.sqrt((x1-x2)**2+(y1-y2)**2)
    r =0.000
    
    x = x1 + r/d*(x1-x2)
    y = y1 + r/d*(y1-y2)
    z = position1[2]
    
    position3=[x,y,z]
    return(position3)
    
def getjointangle(position):
    L1=0.152
    L2=0.12
    L3=0.244
    L4=0.093
    L5=0.213
    L6=0.083
    L7=0.083
    L8=0.082
    L9=0.0535
    L10=0.059
    A=L2-L4+L6

    def atan2(y,x):
        if(x==0 and y>0):
            return np.pi/2
        if(x==0 and y<0):
            return -np.pi/2
        else:
            return math.atan(y/x)
        
    yawg=np.pi
    theta5=-90    
    xg=position[0]+0.15
    yg=position[1]-0.15
    zg=position[2]-0.01
    xc=xg-L9*np.cos(yawg)
    yc=yg-L9*np.sin(yawg)
    zc=zg  
    theta1=2*atan2(-xc+(xc**2-A**2+yc**2)**0.5,A+yc)
    theta6=np.pi/2+theta1-yawg  
    x3e=xc-L7*np.cos(theta1)+(A)*np.sin(theta1)
    y3e=yc-L7*np.sin(theta1)-(A)*np.cos(theta1)
    z3e=zc+L10+L8
    print((L3**2+x3e**2+y3e**2+(z3e-L1)**2-L5**2)/(2*L3*((x3e**2+y3e**2+(z3e-L1)**2)**0.5)))
    t21=math.acos((L3**2+x3e**2+y3e**2+(z3e-L1)**2-L5**2)/(2*L3*((x3e**2+y3e**2+(z3e-L1)**2)**0.5)))
    theta2=-t21-atan2(z3e-L1,(x3e**2+y3e**2)**0.5)
    theta3=np.pi-math.acos((L3**2-x3e**2-y3e**2-(z3e-L1)**2+L5**2)/(2*L3*L5))
    theta4=-theta3-theta2
    theta=[theta1*180/np.pi,theta2*180/np.pi,theta3*180/np.pi,theta4*180/np.pi,theta5,theta6*180/np.pi]
    return theta
    
    

print('program started')

sim.simxFinish(-1)

clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)
if clientID!=-1:
    print('connected to remote api server')
print('connection success')

sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
print('simulation start')

errorcode,ur3WorldFrame = sim.simxGetObjectHandle(clientID,'ReferenceFrame',sim.simx_opmode_blocking)

sim.simxSynchronous(clientID,True)

sim.simxSynchronousTrigger(clientID)

errorcode,handle=sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_blocking)

if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for base frame')

errorcode, joint_one_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for first joint')
errorcode, joint_two_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for second joint')
errorcode, joint_three_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for third joint')
errorcode, joint_four_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
errorcode, joint_five_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
errorcode, joint_six_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
errorcode, joint_seven_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint7', sim.simx_opmode_blocking)
if errorcode != sim.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
    
result, end_handle = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for end effector')
    
result, sphere1_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for end effector')
    
result, sphere2_handle = sim.simxGetObjectHandle(clientID, 'Sphere0', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for end effector')

position=getobjectposition(sphere1_handle,sphere2_handle) 
Goal_joint_angles=getjointangle(position)
print(position)
test = position
test[0] += 0
test[1] += 0.1
test[2] += 0
print(position)
Goal_joint_angles_init = getjointangle(test)
SetJointPosition(Goal_joint_angles_init)
time.sleep(1)
#SetJointPosition(Goal_joint_angles)



#time.sleep(1)
sim.simxSetJointPosition (clientID, joint_five_handle, -np.pi/2, sim.simx_opmode_oneshot)