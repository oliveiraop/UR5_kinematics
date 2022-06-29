# Testes do UR5 utilizando FK e IK
# Por Joao Carneiro e Osmar Oliveira

import numpy as np

from auxiliary import Aux
from kinematics import UR5_arm
from zmqRemoteApi import RemoteAPIClient

np.set_printoptions(precision=4, suppress=True)
np.seterr(all="ignore")

# Qual teste rodar
which_test = input(
    "\n".join(
        ("Selecione o teste:", "  1 - Validar CD e CI", "  2 - Planejar trajetoria\n")
    )
)

# Inicia simulador
client = RemoteAPIClient()
client.setStepping(True)
sim = client.getObject("sim")

# Improve simulation response
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

# Classes com metodos de interesse
ur5 = UR5_arm()
aux = Aux(client, sim)

# Movimentacao sincrona
def moveToAngle(target_angles, targetVels, joint_handles):
    done = [False for _ in range(0, len(target_angles))]
    while not all(done):
        joint_angles = [sim.getJointPosition(x) for x in joint_handles]
        for idx, _ in enumerate(target_angles):
            if abs(joint_angles[idx] - target_angles[idx]) > 0.1 * np.pi / 180:
                sim.setJointTargetVelocity(joint_handles[idx], targetVels[idx])
                sim.setJointMaxForce(joint_handles[idx], 100)
        client.step()


if which_test == "1":
    print(f"---------------------- Teste 1 ----------------------")
    # Comeca simulacao
    sim.startSimulation()
    gripperHandle = sim.getObject("/UR5/ROBOTIQ_85")
    joint_handles = aux.reset_joints()

    # CD
    target_angles = [-90, -90, -90, -90, -90, -90]
    # target_angles = [0, 0, 0, 0, 0, 0]
    targetConfig = np.deg2rad(target_angles).tolist()
    # targetConfig = [0.0012, 0.0191, 0.0, -0.0191, 0.0025, -1.5702]
    print(f"Angulos = {np.array(targetConfig)}")
    matrix = ur5.forward_kinematic(targetConfig)
    print(f"Posicao Calculada (CD) = {matrix[:3,3].T}")

    # Simula
    currentConf = [0, 0, 0, 0, 0, 0]
    aux.run_trajectory(joint_handles, currentConf, targetConfig, 5)
    tool = sim.getObject("/UR5/UR5_connection")
    tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
    tool_matrix = np.matrix(
        [tool_matrix[0:4], tool_matrix[4:8], tool_matrix[8:12], [0, 0, 0, 1]]
    )
    tool_matrix[abs(tool_matrix) < 5e-4] = 0
    tool_pos = tool_matrix[:3, 3].T
    print(f"Transf Simulada = {tool_pos}")

    # CI
    # cup rotation = [0, 90, 90]
    t_o_6 = tool_matrix
    t_o_0 = ur5.t01()
    t_0_6 = np.linalg.inv(t_o_0) @ t_o_6
    calc_thetas = ur5.inverse_kinematic(t_0_6)
    print("Combinacoes Calculadas (CI) =")
    for row in calc_thetas:
        print(f"  {[round(x,4) for x in row.tolist()]}")
    print()

    # encerra simulacao
    sim.stopSimulation()

elif which_test == "2":
    print(f"---------------------- Teste 2 ----------------------")
    # Comeca simulacao
    sim.startSimulation()
    gripperHandle = sim.getObject("/UR5/ROBOTIQ_85")
    joint_handles = aux.reset_joints()
    sim.wait(1)

    # Aproximacao

    # Calcula transformada de ida
    target_ori = [0, np.pi / 2, 0]
    target_pos = [0.25, -0.35, 0.70]
    t_o_6 = ur5.get_trans_matrix(target_ori, target_pos)

    # CI ate a posicao da ida
    t_o_0 = ur5.t01()
    t_0_6 = np.linalg.inv(t_o_0) @ t_o_6
    calc_target = ur5.inverse_kinematic(t_0_6)
    print("Combinacoes Calculadas CI (ida) =")
    for row in calc_target:
        print(f"  {[round(x,4) for x in row.tolist()]}")
    print(f"Escolhida = {calc_target[-1]}")
    print()

    # Movimenta braco ate a posicao
    currentConf = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    targetConfig = calc_target[0]
    aux.run_trajectory(joint_handles, currentConf, targetConfig, 5)
    sim.wait(1)

    # Alvo

    # Calcula transformada do alvo
    target_ori = [0, np.pi / 2, 0]
    target_pos = [0.25, -0.35, 0.49]
    t_o_6 = ur5.get_trans_matrix(target_ori, target_pos)

    # CI ate o alvo
    t_o_0 = ur5.t01()
    t_0_6 = np.linalg.inv(t_o_0) @ t_o_6
    calc_target = ur5.inverse_kinematic(t_0_6)
    print("Combinacoes Calculadas CI (ida) =")
    for row in calc_target:
        print(f"  {[round(x,4) for x in row.tolist()]}")
    print(f"Escolhida = {calc_target[-1]}")
    print()

    # Movimenta braco ate o alvo
    currentConf = targetConfig
    targetConfig = calc_target[0]
    aux.run_trajectory(joint_handles, currentConf, targetConfig, 5)
    sim.wait(1)

    # Volta

    # Movimenta braco de volta para posicao original
    currentConf = targetConfig
    targetConfig = [0, 0, 0, 0, 0, 0]
    aux.run_trajectory(joint_handles, currentConf, targetConfig, 5)
    sim.wait(1)

    # encerra simulacao
    sim.stopSimulation()
