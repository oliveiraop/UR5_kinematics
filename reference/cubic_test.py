import math

import numpy as np

from kinematics import UR5_arm
from ur5_fk import ur5_fk
from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
ur5 = UR5_arm()
sim = client.getObject("sim")
client.setStepping(True)


def moveToAngle(target_angles, targetVels, joint_handles, target_force):
    done = [False for x in range(0, len(target_angles))]
    while not all(done):
        joint_angles = [sim.getJointPosition(x) for x in joint_handles]
        for idx, value in enumerate(target_angles):
            if abs(joint_angles[idx] - target_angles[idx]) > 0.1 * math.pi / 180:
                print(target_force[idx])
                sim.setJointTargetPosition(joint_handles[idx], target_angles[idx])
                sim.setJointTargetVelocity(joint_handles[idx], targetVels[idx])
                sim.setJointMaxForce(joint_handles[idx], target_force[idx])
            else:
                done[idx] = True
        client.step()


jointHandles = [
    sim.getObject("/UR5/UR5_joint1"),
    sim.getObject("/UR5/UR5_joint2"),
    sim.getObject("/UR5/UR5_joint3"),
    sim.getObject("/UR5/UR5_joint4"),
    sim.getObject("/UR5/UR5_joint5"),
    sim.getObject("/UR5/UR5_joint6"),
]
currentConf = [0, 0, 0, 0, 0, 0]
for x in jointHandles:
    sim.setJointTargetPosition(x, 0)
    sim.setJointTargetVelocity(x, 2 * math.pi)
target_angles = [90, 0, 45, 45, 45, 60]
targetConfig = [x * math.pi / 180 for x in target_angles]
matrix = ur5.forward_kinematic(*targetConfig)

pos = []
vel = []
accel = []

for i, value in enumerate(targetConfig):
    pos_temp, vel_temp, accel_temp = ur5.cubic(
        currentConf[i], targetConfig[i], 0, 0, 0, 10
    )
    pos.append(pos_temp)
    vel.append(vel_temp)
    accel.append(accel_temp)

sim.startSimulation()

for idx, value in enumerate(pos[0]):
    moveToAngle(
        [v[idx] for v in pos],
        [v[idx] for v in vel],
        jointHandles,
        [v[idx] for v in accel],
    )
    print("step: " + str(idx))

sim.stopSimulation()

print("Program ended")
