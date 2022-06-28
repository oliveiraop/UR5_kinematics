# Testes do UR5 utilizando FK e IK
# Por Joao Carneiro e Osmar Oliveira

import numpy as np

from auxiliary import Aux
from kinematics import UR5_arm
from zmqRemoteApi import RemoteAPIClient

np.set_printoptions(precision=4, suppress=True)
np.seterr(all="ignore")

# Inicia simulador
client = RemoteAPIClient()
sim = client.getObject("sim")
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

# Classes com metodos de interesse
ur5 = UR5_arm()
aux = Aux()

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


print(f"---------------------- Teste ----------------------")
# Comeca simulacao
sim.startSimulation()
joint_handles = aux.reset_joints(sim)

currentConf = [0, 0, 0, 0, 0, 0]
target_angles = [-90, -90, -90, -90, -90, -90]
targetConfig = [x * np.pi / 180 for x in target_angles]
print(f"Angulos = {np.array(targetConfig)}")
matrix = ur5.ur5_fk(targetConfig)
print(f"Posicao Calculada (CD) = {matrix[:3,3].T}")

pos = []
vel = []
accel = []

for i, value in enumerate(targetConfig):
    pos[i], vel[i], accel[i] = ur5.cubic(currentConf[i], targetConfig[i], 0, 0, 0, 10)

for idx, value in enumerate(pos[0]):
    moveToAngle([v[idx] for v in pos], [v[idx] for v in vel], joint_handles)


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

# encerra simulacao
sim.stopSimulation()
