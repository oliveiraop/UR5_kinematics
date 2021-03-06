# Teste de 3 posicoes do UR5 via Cinematica Direta (angulo nas juntas).
# Por:
# Joao Victor Carneiro
# Osmar Oliveira

import numpy as np

from zmqRemoteApi import RemoteAPIClient


# Funcao de Cinematica Direta (utilizando parametros DH extraidos do cenario)
def ur5_fk(theta_list):
    a = [0, 0, -0.4251, -0.3921, 0, 0]
    alpha = [0, np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2]
    d = [0.0892, 0, 0, 0.1102, 0.0947, -0.0288 + 0.075]
    theta = [
        theta_list[0],
        theta_list[1],
        theta_list[2],
        theta_list[3],
        theta_list[4],
        theta_list[5],
    ]

    # Transformacao homogenea do sistema de coordenadas original ate a base do UR5
    r_base = np.array(
        [
            [np.cos(np.pi / 2), -np.sin(np.pi / 2), 0],
            [np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
            [0, 0, 1],
        ]
    )
    d_base = np.array([[-0.214, -0.314, 0.41]])
    last_row = np.array([[0, 0, 0, 1]])
    h_base = np.concatenate(
        (np.concatenate((r_base, d_base.T), axis=1), last_row), axis=0
    )

    # Multiplica as transformacoes homogeneas entre cada junta
    t_matrix = h_base
    for i in np.arange(6):
        a_matrix = np.array(
            [
                [np.cos(theta[i]), -np.sin(theta[i]), 0, a[i]],
                [
                    np.sin(theta[i]) * np.cos(alpha[i]),
                    np.cos(theta[i]) * np.cos(alpha[i]),
                    -np.sin(alpha[i]),
                    -np.sin(alpha[i]) * d[i],
                ],
                [
                    np.sin(theta[i]) * np.sin(alpha[i]),
                    np.cos(theta[i]) * np.sin(alpha[i]),
                    np.cos(alpha[i]),
                    np.cos(alpha[i]) * d[i],
                ],
                [0, 0, 0, 1],
            ]
        )
        t_matrix = np.matmul(t_matrix, a_matrix)
    t_matrix[abs(t_matrix) < 1e-15] = 0
    return t_matrix


# Altera posicoes das juntas
def set_pos(theta_list):
    joint_handles = [joint1, joint2, joint3, joint4, joint5, joint6]
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

    # Aplica angulos nas juntas, compensando a diferenca na posicao inicial do cenario
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


if __name__ == "__main__":
    # Inicia cliente e salva instancia do simulador atraves de API ZMQ
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    # Inicia simulacao
    sim.startSimulation()

    # Salva referencia das juntas
    joint1 = sim.getObject("/UR5/UR5_joint1")
    joint2 = sim.getObject("/UR5/UR5_joint2")
    joint3 = sim.getObject("/UR5/UR5_joint3")
    joint4 = sim.getObject("/UR5/UR5_joint4")
    joint5 = sim.getObject("/UR5/UR5_joint5")
    joint6 = sim.getObject("/UR5/UR5_joint6")

    # Teste
    print("Teste FK:")
    tool = sim.getObject("/UR5/UR5_connection")
    test_angles = [np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4]
    # test_angles = [-np.pi/4, -np.pi/4, -np.pi/4, -np.pi/4, -np.pi/4, -np.pi/4]
    # test_angles = [-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2]
    print(f"theta_input = {test_angles}")
    t_calc = ur5_fk(test_angles)
    print(f"t_calculado =\n{t_calc}")
    print(f"p_calculado = {[np.round(x,4) for x in t_calc[:3,3].tolist()]}")
    set_pos(test_angles)
    sim.wait(3)
    tool_position = sim.getObjectPosition(tool, sim.handle_world)
    print(f"p_simulado = {[np.round(x,4) for x in tool_position]}")

    # sim.stopSimulation()
