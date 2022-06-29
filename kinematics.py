import numpy as np
from numpy import cos, sin


class UR5_arm:
    # Transformacao via rotacao e deslocamento
    def get_trans_matrix(self, rot, dis):
        [a, b, g] = rot
        pos = np.array([dis])
        r_x_a = np.array([[1, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]])
        r_y_b = np.array([[cos(b), 0, sin(b)], [0, 1, 0], [-sin(b), 0, cos(b)]])
        r_z_g = np.array([[cos(g), -sin(g), 0], [sin(g), cos(g), 0], [0, 0, 1]])
        r_total = r_x_a @ r_y_b @ r_z_g
        last_row = np.array([[0, 0, 0, 1]])
        t_matrix = np.concatenate(
            (np.concatenate((r_total, pos.T), axis=1), last_row), axis=0
        )
        return t_matrix

    # Tranformacao da origem para a base
    def t01(self):
        T01 = np.matrix(
            [[0, -1, 0, -0.214], [1, 0, 0, -0.314], [0, 0, 1, 0.4246], [0, 0, 0, 1]]
        )
        return T01

    # Funcao de Cinematica Direta do UR5
    def forward_kinematic(self, theta_list):
        a = [0, 0, -0.425, -0.39225, 0, 0]
        alpha = [0, np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2]
        d = [0.089159, 0, 0, 0.10915, 0.09465, 0.075]
        theta = [
            theta_list[0],
            theta_list[1],
            theta_list[2],
            theta_list[3],
            theta_list[4],
            theta_list[5],
        ]
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

    # Gera matriz de transformacao de uma junta utilizando tabela DH
    def _get_t_dh(self, joint, theta):

        a = [0, 0, -0.425, -0.39225, 0, 0]
        alpha = [0, np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2]
        d = [0.089159, 0, 0, 0.10915, 0.09465, 0.075]

        cur_a = a[joint - 1]
        cur_alpha = alpha[joint - 1]
        cur_d = d[joint - 1]
        t_dh = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0, cur_a],
                [
                    np.sin(theta) * np.cos(cur_alpha),
                    np.cos(theta) * np.cos(cur_alpha),
                    -np.sin(cur_alpha),
                    -np.sin(cur_alpha) * cur_d,
                ],
                [
                    np.sin(theta) * np.sin(cur_alpha),
                    np.cos(theta) * np.sin(cur_alpha),
                    np.cos(cur_alpha),
                    np.cos(cur_alpha) * cur_d,
                ],
                [0, 0, 0, 1],
            ]
        )
        return t_dh

    # Funcao de Cinematica Inversa do UR5
    def inverse_kinematic(self, t_0_6):
        def r(num):
            abs_num = np.fabs(num)
            if abs_num > 1:
                if abs_num - 1 < 1e-2:
                    return np.modf(num)[1]
            return num

        # Distancia entre junta 6 e posicao da garra
        d6 = 0.075

        d4 = 0.10915

        theta = np.empty((8, 6))

        # Theta 1
        p_0_5 = np.matmul(t_0_6, np.array([[0, 0, -d6, 1]]).T)
        phi1 = np.arctan2((p_0_5[1, 0]), (p_0_5[0, 0]))
        phi2 = np.arccos(r((d4) / (np.sqrt(p_0_5[0, 0] ** 2 + p_0_5[1, 0] ** 2))))
        result1 = phi1 + phi2 + np.pi / 2
        result2 = phi1 - phi2 + np.pi / 2
        theta[0:4, 0] = result1  # Ombro esquerda
        theta[4:8, 0] = result2  # Ombro direita

        # Theta 5
        for i in np.arange(0, 8, 4):
            p_0_6 = t_0_6[:, 3]
            result1 = np.arccos(
                r(
                    ((p_0_6[0] * sin(theta[i, 0]) - p_0_6[1] * cos(theta[i, 0])) - d4)
                    / d6
                )
            )
            result2 = -np.arccos(
                r(
                    ((p_0_6[0] * sin(theta[i, 0]) - p_0_6[1] * cos(theta[i, 0]) - d4))
                    / d6
                )
            )
            theta[i : i + 2, 4] = result1  # Punho cima
            theta[i + 2 : i + 4, 4] = result2  # Punho baixo

        # Theta 6
        for i in np.arange(0, 8, 2):
            if abs(theta[i, 4]) == 0.0:
                theta[i, 5] = 0.0
            else:
                t_6_0 = np.linalg.inv(t_0_6)
                x_6_0_x = t_6_0[0, 0]
                x_6_0_y = t_6_0[1, 0]
                y_6_0_x = t_6_0[0, 1]
                y_6_0_y = t_6_0[1, 1]
                arg1 = (-x_6_0_y * sin(theta[i, 0]) + y_6_0_y * cos(theta[i, 0])) / sin(
                    theta[i, 4]
                )
                arg2 = (x_6_0_x * sin(theta[i, 0]) - y_6_0_x * cos(theta[i, 0])) / sin(
                    theta[i, 4]
                )
                theta[i : i + 2, 5] = np.arctan2((arg1), (arg2))

        # Theta 3
        for i in np.arange(0, 8, 2):
            t_0_1 = self._get_t_dh(1, theta[i, 0])
            t_4_5 = self._get_t_dh(5, theta[i, 4])
            t_5_6 = self._get_t_dh(6, theta[i, 5])
            t_1_6 = np.linalg.inv(t_0_1) @ t_0_6
            t_6_4 = np.linalg.inv(t_4_5 @ t_5_6)
            t_1_4 = t_1_6 @ t_6_4
            p_1_4_xz = [[(t_1_4[0, 3]), (t_1_4[2, 3])]]
            result1 = np.arccos(
                r(
                    ((np.linalg.norm(p_1_4_xz) ** 2 - 0.425**2 - 0.39225**2))
                    / ((2 * 0.425 * 0.39225))
                )
            )
            result2 = -np.arccos(
                r(
                    ((np.linalg.norm(p_1_4_xz) ** 2 - 0.425**2 - 0.39225**2))
                    / ((2 * 0.425 * 0.39225))
                )
            )
            theta[i, 2] = result1  # Cotovelo cima
            theta[i + 1, 2] = result2  # Cotovelo baixo

        # Theta 2
        for i in np.arange(0, 8):
            t_0_1 = self._get_t_dh(1, theta[i, 0])
            t_4_5 = self._get_t_dh(5, theta[i, 4])
            t_5_6 = self._get_t_dh(6, theta[i, 5])
            t_1_6 = np.linalg.inv(t_0_1) @ t_0_6
            t_6_4 = np.linalg.inv(t_4_5 @ t_5_6)
            t_1_4 = t_1_6 @ t_6_4
            p_1_4_x = t_1_4[0, 3]
            p_1_4_z = t_1_4[2, 3]
            p_1_4_xz = [[p_1_4_x, p_1_4_z]]
            result = np.arctan2((-p_1_4_z), (-p_1_4_x)) - np.arcsin(
                r(((0.39225 * sin(theta[i, 2]))) / (np.linalg.norm(p_1_4_xz)))
            )
            theta[i, 1] = result

        # Theta 4
        for i in np.arange(0, 8):
            t_0_1 = self._get_t_dh(1, theta[i, 0])
            t_4_5 = self._get_t_dh(5, theta[i, 4])
            t_5_6 = self._get_t_dh(6, theta[i, 5])
            t_1_6 = np.linalg.inv(t_0_1) @ t_0_6
            t_6_4 = np.linalg.inv(t_4_5 @ t_5_6)
            t_1_4 = t_1_6 @ t_6_4
            t_1_2 = self._get_t_dh(2, theta[i, 1])
            t_2_3 = self._get_t_dh(3, theta[i, 2])
            t_1_3 = t_1_2 @ t_2_3
            t_3_4 = np.linalg.inv(t_1_3) @ t_1_4
            x_3_4_y = t_3_4[1, 0]
            x_3_4_x = t_3_4[0, 0]
            result = np.arctan2((x_3_4_y), (x_3_4_x))
            theta[i, 3] = result

        theta_drop_nan = theta[~np.isnan(theta).any(axis=1)]

        return theta_drop_nan

    # Cubica
    def cubic(self, sim, qi, qf, vi, vf, ti, tf):
        dt = sim.getSimulationTimeStep()
        t = np.arange(ti, tf, dt)
        c = np.ones(t.size)
        m = np.matrix(
            [
                [1, ti, ti * ti, ti * ti * ti],
                [0, 1, 2 * ti, 3 * ti * ti * ti],
                [1, tf, tf * tf, tf * tf * tf],
                [0, 1, 2 * tf, 3 * tf * tf],
            ]
        )
        b = np.matrix([qi, vi, qf, vf]).T
        a = (m.I) * b

        pos = (
            np.multiply(a[0, 0], c)
            + np.multiply(a[1, 0], t)
            + np.multiply(a[2, 0], np.multiply(t, t))
            + np.multiply(a[3, 0], np.multiply(t, np.multiply(t, t)))
        )
        vel = (
            np.multiply(a[1, 0], c)
            + 2 * np.multiply(a[2, 0], t)
            + 3 * np.multiply(a[3, 0], np.multiply(t, t))
        )
        accel = 2 * np.multiply(a[2, 0], c) + 6 * np.multiply(a[3, 0], t)
        return pos, vel, accel
