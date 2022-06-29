import numpy as np

from kinematics import UR5_arm

ur5 = UR5_arm()


class Aux:
    def __init__(self, client, sim):
        self.client = client
        self.sim = sim

    # Move para a orientacao final indicada (step)
    def moveToAngle(self, targetPos, targetVels, jointHandles):
        done = [False for _ in range(0, len(targetPos))]
        while not all(done):
            joint_angles = [self.sim.getJointPosition(x) for x in jointHandles]
            for idx, _ in enumerate(targetPos):
                if abs(joint_angles[idx] - targetPos[idx]) > 0.1 * np.pi / 180:
                    self.sim.setJointTargetPosition(jointHandles[idx], targetPos[idx])
                    self.sim.setJointTargetVelocity(jointHandles[idx], targetVels[idx])
                else:
                    done[idx] = True
            self.client.step()

    # Computa trajetoria cubica e move as juntas
    def run_trajectory(self, jointHandles, currentConf, targetConfig, tf):
        pos = []
        vel = []
        accel = []
        currentPos = [
            currentConf[0],
            currentConf[1] + np.pi / 2,
            currentConf[2],
            currentConf[3] + np.pi / 2,
            currentConf[4],
            currentConf[5],
        ]
        targetPos = [
            targetConfig[0],
            targetConfig[1] + np.pi / 2,
            targetConfig[2],
            targetConfig[3] + np.pi / 2,
            targetConfig[4],
            targetConfig[5],
        ]
        for i, _ in enumerate(targetPos):
            pos_temp, vel_temp, accel_temp = ur5.cubic(
                self.sim, currentPos[i], targetPos[i], 0, 0, 0, tf
            )
            pos.append(pos_temp.tolist())
            vel.append(vel_temp.tolist())
            accel.append(accel_temp.tolist())
        for idx, _ in enumerate(pos[0]):
            targetPos = np.array(pos)[:, idx].tolist()
            targetVel = np.array(vel)[:, idx].tolist()
            self.moveToAngle(
                targetPos,
                targetVel,
                jointHandles,
            )

    # Reseta posicao das juntas
    def reset_joints(self):
        currentConf = [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]
        targetConfig = [0, 0, 0, 0, 0, 0]
        tf = 5
        joint1 = self.sim.getObject("/UR5/UR5_joint1")
        joint2 = self.sim.getObject("/UR5/UR5_joint2")
        joint3 = self.sim.getObject("/UR5/UR5_joint3")
        joint4 = self.sim.getObject("/UR5/UR5_joint4")
        joint5 = self.sim.getObject("/UR5/UR5_joint5")
        joint6 = self.sim.getObject("/UR5/UR5_joint6")
        jointHandles = [joint1, joint2, joint3, joint4, joint5, joint6]
        self.run_trajectory(jointHandles, currentConf, targetConfig, tf)
        return jointHandles
