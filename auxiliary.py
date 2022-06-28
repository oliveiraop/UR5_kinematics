import numpy as np


class Aux:
    # Altera posicoes das juntas
    def set_pos(self, sim, joint_handles, theta_list):
        vel = 20
        accel = 40
        jerk = 20
        current_vel = [0, 0, 0, 0, 0, 0, 0]
        current_accel = [0, 0, 0, 0, 0, 0, 0]
        max_vel = [
            vel * np.pi / 180,
            vel * np.pi / 180,
            vel * np.pi / 180,
            vel * np.pi / 180,
            vel * np.pi / 180,
            vel * np.pi / 180,
        ]
        max_accel = [
            accel * np.pi / 180,
            accel * np.pi / 180,
            accel * np.pi / 180,
            accel * np.pi / 180,
            accel * np.pi / 180,
            accel * np.pi / 180,
        ]
        max_jerk = [
            jerk * np.pi / 180,
            jerk * np.pi / 180,
            jerk * np.pi / 180,
            jerk * np.pi / 180,
            jerk * np.pi / 180,
            jerk * np.pi / 180,
        ]
        target_vel = [0, 0, 0, 0, 0, 0]
        target_pos = [
            theta_list[0],
            theta_list[1] + np.pi / 2,
            theta_list[2],
            theta_list[3] + np.pi / 2,
            theta_list[4],
            theta_list[5],
        ]
        sim.rmlMoveToJointPositions(
            joint_handles,
            -1,
            current_vel,
            current_accel,
            max_vel,
            max_accel,
            max_jerk,
            target_pos,
            target_vel,
        )

    # Reseta posicao das juntas
    def reset_joints(self, sim):
        joint1 = sim.getObject("/UR5/UR5_joint1")
        joint2 = sim.getObject("/UR5/UR5_joint2")
        joint3 = sim.getObject("/UR5/UR5_joint3")
        joint4 = sim.getObject("/UR5/UR5_joint4")
        joint5 = sim.getObject("/UR5/UR5_joint5")
        joint6 = sim.getObject("/UR5/UR5_joint6")
        joint_handles = [joint1, joint2, joint3, joint4, joint5, joint6]
        self.set_pos(sim, joint_handles, [0, 0, 0, 0, 0, 0])
        sim.wait(2)
        return joint_handles
