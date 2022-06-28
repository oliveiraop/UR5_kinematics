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

# Comeca simulacao
sim.startSimulation()
joint_handles = aux.reset_joints(sim)

# Roda testes
aux.run_test(
    sim,
    ur5,
    joint_handles,
    "Teste 1",
    [-0.0075, -0.1038, 0.2297, -0.2092, -0.0076, -1.522],
    "rad",
)
aux.run_test(
    sim,
    ur5,
    joint_handles,
    "Teste 2",
    [-0.0063, -0.1344, 0.3355, -0.353, -0.0069, -3.0238],
    "rad",
)
aux.run_test(sim, ur5, joint_handles, "Teste 3", [90, -70, 70, 50, 60, 90])

# encerra simulacao
sim.stopSimulation()
