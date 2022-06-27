
from ur5_fk import ur5_fk

import numpy as np
from kinematics import UR5_arm

from zmqRemoteApi import RemoteAPIClient
import math

client = RemoteAPIClient()
ur5 = UR5_arm()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

def moveToAngle(target_angles, targetVels, joint_handles):
    done = [False for x in range(0, len(target_angles))]
    while (not all(done)):
        joint_angles = [sim.getJointPosition(x) for x in joint_handles]
        for idx, value in enumerate(target_angles):
            if (abs(joint_angles[idx] - target_angles[idx]) > 0.1):
                print(abs(joint_angles[idx] - target_angles[idx]))
                print(targetVels[idx])
                sim.setJointTargetVelocity(joint_handles[idx], targetVels[idx]*10)
                sim.setJointMaxForce(joint_handles[idx], 100)
            else:
                done[idx] = True
        client.step()

jointHandles = [ 
    sim.getObject('/UR5/UR5_joint1'),
    sim.getObject('/UR5/UR5_joint2'),
    sim.getObject('/UR5/UR5_joint3'),
    sim.getObject('/UR5/UR5_joint4'),
    sim.getObject('/UR5/UR5_joint5'),
    sim.getObject('/UR5/UR5_joint6'),
    ]
currentConf = [ 0, 0, 0, 0, 0, 0 ]
for x in jointHandles:
    sim.setJointTargetPosition(x, 0)
target_angles = [90, 0, 45, 45, 45, 60]
targetConfig = [x*math.pi/180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)

pos = []
vel = []
accel = []

for i, value in enumerate(targetConfig):
    pos_temp, vel_temp, accel_temp = ur5.cubic(currentConf[i], targetConfig[i], 0, 0, 0, 10)
    pos.append(pos_temp)
    vel.append(vel_temp)
    accel.append(accel_temp)

for idx, value in enumerate(pos[0]):
    moveToAngle([v[idx] for v in pos], [v[idx] for v in vel], jointHandles)
    print("step: " + str(idx))

sim.stopSimulation()

print('Program ended')