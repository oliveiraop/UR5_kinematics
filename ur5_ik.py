# Teste para mover garra do UR5 ao copo e de volta via Cinematica Inversa.
# Por Joao Victor Carneiro

import numpy as np
from numpy import cos, sin
from zmqRemoteApi import RemoteAPIClient


# Normaliza angulos para intervalo [0, 2pi]
def normalize_angle(angle):
    new_angle = np.arctan2(np.sin(angle), np.cos(angle))
    if new_angle < 0:
        new_angle = abs(new_angle) + 2 * (np.pi - abs(new_angle))
    return new_angle


# Gera matriz de transformacao de uma junta utilizando tabela DH
def get_t_dh(joint, theta):
    a = [0, 0, -0.7391+0.314, -1.1312+0.7391, 0, 0]
    alpha = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]
    d = [0.4992-0.41, 0, 0, -0.1038+0.2140, 0.4994-0.4047, -0.0288+0.1038]
    cur_a = a[joint-1]
    cur_alpha = alpha[joint-1]
    cur_d = d[joint-1]
    t_dh = np.array(
        [[np.cos(theta), -np.sin(theta), 0, cur_a],
         [np.sin(theta)*np.cos(cur_alpha), np.cos(theta)*np.cos(cur_alpha), -np.sin(cur_alpha), -np.sin(cur_alpha)*cur_d],
         [np.sin(theta)*np.sin(cur_alpha), np.cos(theta)*np.sin(cur_alpha), np.cos(cur_alpha), np.cos(cur_alpha)*cur_d],
         [0, 0, 0, 1]])
    return t_dh


# Funcao de Cinematica Inversa do UR5 (utilizando posicao e orientacao)
def ur5_ik(t_o_0, t_o_6, d6):
    t_0_6 = np.linalg.inv(t_o_0) @ t_o_6
    d4 = 0.1102

    theta = np.zeros((6,8))

    # Theta 1
    p_0_5 = np.matmul(t_0_6, np.array([[0, 0, -d6, 1]]).T)
    phi1 = np.arctan2(p_0_5[1,0], p_0_5[0,0])
    phi2 = np.arccos(d4/np.sqrt(p_0_5[0,0]**2 + p_0_5[1,0]**2))
    result1 = phi1 + phi2 + np.pi/2
    result2 = phi1 - phi2 + np.pi/2
    theta[0,:4] = normalize_angle(result1)
    theta[0,4:] = normalize_angle(result2)
    theta = np.real(theta)

    # Theta 5
    p_0_6 = t_0_6[:,3]
    m = 1
    for i in np.arange(0,8,2):
        result = m*np.arccos((p_0_6[0]*sin(theta[0,i])-p_0_6[1]*cos(theta[0,i]) - d4) / d6)
        theta[4,i:i+2] = normalize_angle(result)
        m *= -1
    theta = np.real(theta)

    # Theta 6
    t_6_0 = np.linalg.inv(t_0_6)
    x_6_0_x = t_6_0[0, 0]
    x_6_0_y = t_6_0[1, 0]
    y_6_0_x = t_6_0[0, 1]
    y_6_0_y = t_6_0[1, 1]
    for i in np.arange(8):
        arg1 = (-x_6_0_y*sin(theta[0,i])+y_6_0_y*cos(theta[0,i]))/sin(theta[4,i])
        arg2 = (x_6_0_x*sin(theta[0,i])-y_6_0_x*cos(theta[0,i]))/sin(theta[4,i])
        theta[5,i] = normalize_angle(np.arctan2(arg1, arg2))
    theta = np.real(theta)

    # Theta 3
    for i in np.arange(0,8,2):
        t_0_1 = get_t_dh(1, theta[0,i])
        t_4_5 = get_t_dh(5, theta[4,i])
        t_5_6 = get_t_dh(6, theta[5,i])
        t_1_6 = np.linalg.inv(t_0_1) @ t_0_6
        t_6_4 = np.linalg.inv(t_4_5 @ t_5_6)
        t_1_4 = t_1_6 @ t_6_4
        p_1_4_xz = [[t_1_4[0,3], t_1_4[2,3]]]
        result = np.arccos((np.linalg.norm(p_1_4_xz)**2-0.4251**2-0.3921**2)/(2*0.4251*0.3921))
        theta[2,i] = normalize_angle(result)
        theta[2,i+1] = normalize_angle(-result)
    theta = np.real(theta)

    # Theta 2
    for i in np.arange(8):
        t_0_1 = get_t_dh(1, theta[0,i])
        t_4_5 = get_t_dh(5, theta[4,i])
        t_5_6 = get_t_dh(6, theta[5,i])
        t_1_6 = np.linalg.inv(t_0_1) @ t_0_6
        t_6_4 = np.linalg.inv(t_4_5 @ t_5_6)
        t_1_4 = t_1_6 @ t_6_4
        p_1_4_x = t_1_4[0,3]
        p_1_4_z = t_1_4[2,3]
        p_1_4_xz = [[p_1_4_x, p_1_4_z]]
        result = np.arctan2(-p_1_4_z, -p_1_4_x) - np.arcsin((0.3921*sin(theta[2,i]))/np.linalg.norm(p_1_4_xz))
        theta[1,i] = normalize_angle(result)

    # Theta 4
        t_1_2 = get_t_dh(2, theta[1,i])
        t_2_3 = get_t_dh(3, theta[2,i])
        t_1_3 = t_1_2 @ t_2_3
        t_3_4 = np.linalg.inv(t_1_3) @ t_1_4
        x_3_4_y = t_3_4[1,0]
        x_3_4_x = t_3_4[0,0]
        result = np.arctan2(x_3_4_y, x_3_4_x)
        theta[3,i] = normalize_angle(result)
    theta = np.real(theta)

    return theta


# Gera matriz de transformacao a partir de posicao e orientacao
def get_t_matrix(pos, ori):
    [a, b, g] = ori[0]
    r_x_a = np.array([[1, 0, 0],
                      [0, cos(a), -sin(a)],
                      [0, sin(a), cos(a)]])
    r_y_b = np.array([[cos(b), 0, sin(b)],
                      [0, 1, 0],
                      [-sin(b), 0, cos(b)]])
    r_z_g = np.array([[cos(g), -sin(g), 0],
                      [sin(g), cos(g), 0],
                      [0, 0, 1]])
    r_total = r_x_a @ r_y_b @ r_z_g
    last_row = np.array([[0, 0, 0, 1]])
    t_matrix = np.concatenate((np.concatenate((r_total, pos.T), axis=1), last_row), axis=0)
    return t_matrix

# Altera posicoes das juntas
def set_pos(theta_list):
    joint1 = sim.getObject('/UR5/UR5_joint1')
    joint2 = sim.getObject('/UR5/UR5_joint2')
    joint3 = sim.getObject('/UR5/UR5_joint3')
    joint4 = sim.getObject('/UR5/UR5_joint4')
    joint5 = sim.getObject('/UR5/UR5_joint5')
    joint6 = sim.getObject('/UR5/UR5_joint6')
    joint_handles = [joint1, joint2, joint3, joint4, joint5, joint6]
    vel = 20
    accel = 40
    jerk = 20
    current_vel = [0,0,0,0,0,0,0]
    current_accel = [0,0,0,0,0,0,0]
    max_vel = [vel*np.pi/180,vel*np.pi/180,vel*np.pi/180,vel*np.pi/180,vel*np.pi/180,vel*np.pi/180]
    max_accel = [accel*np.pi/180,accel*np.pi/180,accel*np.pi/180,accel*np.pi/180,accel*np.pi/180,accel*np.pi/180]
    max_jerk = [jerk*np.pi/180,jerk*np.pi/180,jerk*np.pi/180,jerk*np.pi/180,jerk*np.pi/180,jerk*np.pi/180]
    target_vel = [0,0,0,0,0,0]
    target_pos=[theta_list[0], theta_list[1]+np.pi/2, theta_list[2], theta_list[3]+np.pi/2, theta_list[4], theta_list[5]]
    sim.rmlMoveToJointPositions(joint_handles,-1,current_vel,current_accel,max_vel,max_accel,max_jerk,target_pos,target_vel)

# Inicia cliente e salva instancia do simulador atraves de API ZMQ
client = RemoteAPIClient()
sim = client.getObject('sim')

# Inicia simulacao
sim.startSimulation()

# Reseta posicao da garra
set_pos([0,0,0,0,0,0])

# Encontra matriz de transformacao da base para o copo
base_pos = np.array([[-0.214, -0.314, 0.41]])
base_ori = np.array([[0, 0, np.pi/2]])
cup_pos = np.array([[0.4, -0.358, 0.47]])
cup_ori = np.array([[0, 0, np.pi/2]])
t_o_0 = get_t_matrix(base_pos, base_ori)
t_o_6 = get_t_matrix(cup_pos, cup_ori)

# Distancia entre junta 6 e posicao da garra
d6 = 0.075

# Gera angulos com Cinematica Inversa
theta = ur5_ik(t_o_0, t_o_6, d6)

sim.stopSimulation()
