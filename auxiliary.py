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

    # Testes
    def run_test(self, sim, ur5, joint_handles, name, thetas, unit="deg"):
        print(f"---------------------- {name} ----------------------")
        if unit == "deg":
            target_angles = thetas
            targetPos = [x * np.pi / 180 for x in target_angles]
        else:
            targetPos = thetas
        print(f"Angulos = {np.array(targetPos)}")
        matrix = ur5.ur5_fk(targetPos)
        print(f"Posicao Calculada (CD) = {matrix[:3,3].T}")

        self.set_pos(sim, joint_handles, targetPos)
        sim.wait(2)
        tool = sim.getObject("/UR5/UR5_connection")
        tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
        tool_matrix = np.matrix(
            [tool_matrix[0:4], tool_matrix[4:8], tool_matrix[8:12], [0, 0, 0, 1]]
        )
        tool_matrix[abs(tool_matrix) < 5e-4] = 0
        tool_pos = tool_matrix[:3, 3].T
        print(f"Transf Simulada = {tool_pos}")

        target_pos = tool_matrix
        base_pos = ur5.t01()
        calc_thetas = ur5.inverse_kinematic(base_pos, target_pos)
        print("Angulos Calculados (CI) =")
        for row in calc_thetas:
            print(f"  {[round(x,4) for x in row.tolist()]}")
        print()
