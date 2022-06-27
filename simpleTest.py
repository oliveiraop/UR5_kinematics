# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time
from ur5_fk import ur5_fk

import numpy as np
from kinematics import UR5_arm

from zmqRemoteApi import RemoteAPIClient
import math

print('Program started')


def movCallback(config,vel,accel,handles):
    for i in range(len(handles)):
        if sim.getJointMode(handles[i])[0]==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i],config[i])
        else:
            sim.setJointPosition(handles[i],config[i])

client = RemoteAPIClient()
sim = client.getObject('sim')
jointHandles = [ 
    sim.getObject('/UR5/UR5_joint1'),
    sim.getObject('/UR5/UR5_joint2'),
    sim.getObject('/UR5/UR5_joint3'),
    sim.getObject('/UR5/UR5_joint4'),
    sim.getObject('/UR5/UR5_joint5'),
    sim.getObject('/UR5/UR5_joint6'),
    ]
# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

mVel = 100 * math.pi / 180
mAccel = 150 * math.pi / 180
mJerk = 100 * math.pi / 180
maxVel=[mVel, mVel, mVel, mVel, mVel, mVel]
maxAccel=[mAccel, mAccel, mAccel, mAccel, mAccel, mAccel]
maxJerk=[mJerk, mJerk, mJerk, mJerk, mJerk, mJerk]
ur5 = UR5_arm()

sim.startSimulation()


currentConf = [ 0, 0, 0, 0, 0, 0 ]
target_angles = [90, 0, 45, 45, 45, 60]
targetConfig = [x*math.pi/180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)
print(f"Forward kinematic 1 = \n{matrix}")
matrix = ur5_fk(targetConfig)
print(f"Forward kinematic 2 = \n{matrix}")

sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

tool = sim.getObject('/UR5/UR5_connection')
tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
tool_matrix = np.matrix([tool_matrix[0:4], tool_matrix[4:8], tool_matrix[8:12], [0,0,0,1]])
tool_matrix[abs(tool_matrix) < 5e-4] = 0
print(f"Object Matrix = \n{tool_matrix}")


currentConf = targetConfig
target_angles = [45, 30, 60, 30, 90, 30]
targetConfig = [x*math.pi/180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)
print(f"Forward kinematic = \n{matrix}")

sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

tool = sim.getObject('/UR5/UR5_connection')
tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
tool_matrix = np.matrix([tool_matrix[0:4], tool_matrix[4:8], tool_matrix[8:12], [0,0,0,1]])
tool_matrix[abs(tool_matrix) < 5e-4] = 0
print(f"Object Matrix = \n{tool_matrix}")

currentConf = targetConfig
target_angles = [85, 15, 40, 50, 60, 20]
targetConfig = [x*math.pi/180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)
print(f"Forward kinematic = \n{matrix}")

sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

tool = sim.getObject('/UR5/UR5_connection')
tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
tool_matrix = np.matrix([tool_matrix[0:4], tool_matrix[4:8], tool_matrix[8:12], [0,0,0,1]])
tool_matrix[abs(tool_matrix) < 5e-4] = 0
print(f"Object Matrix = \n{tool_matrix}")





sim.stopSimulation()

client.setStepping(True)

sim.startSimulation()

def moveToAngle(target_angles, targetVels, joint_handles):
    done = [False for x in range(0, len(target_angles))]
    while (not all(done)):
        joint_angles = [sim.getJointPosition(x) for x in joint_handles]
        for idx, value in enumerate(target_angles):
            if (abs(joint_angles[idx] - target_angles[idx]) > 0.1 * math.pi / 180):
                sim.setJointTargetVelocity(joint_handles[idx], targetVels[idx])
                sim.setJointMaxForce(joint_handles[idx], 100)
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
target_angles = [90, 0, 45, 45, 45, 60]
targetConfig = [x*math.pi/180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)

pos = []
vel = []
accel = []

for i, value in enumerate(targetConfig):
    pos[i], vel[i], accel[i] = ur5.cubic(currentConf[i], targetConfig[i], 0, 0, 0, 10)

for idx, value in enumerate(pos[0]):
    moveToAngle([v[idx] for v in pos], [v[idx] for v in vel], jointHandles)

sim.stopSimulation()


# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

print('Program ended')
