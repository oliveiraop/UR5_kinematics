from math import sqrt
import numpy as np
from numpy import arccos, arcsin, arctan2, pi, real, sin, cos
from cmath import acos


class UR5_arm:
    def _dh_transform(self, alpha: float, a: float, d: float, theta: float):
      dh_matrix: np.matrix = np.matrix([
          [cos(theta), -sin(theta), 0, a],
          [sin(theta)*cos(alpha), cos(theta) *
              cos(alpha), -sin(alpha), -sin(alpha)*d],
          [sin(theta)*sin(alpha), cos(theta) *
              sin(alpha), cos(alpha), cos(alpha)*d],
          [0, 0, 0, 1]
      ])
      return dh_matrix

    def _t01(self):
      T01 = np.matrix([
          [1, 0, 0, -0.214],
          [0, 1, 0, -0.314],
          [0, 0, 1, 0.4246],
          [0, 0, 0, 1]
        ])
      return T01

    def _h01(self, theta1):
      d1 = 0.089159
      H01 = np.matrix([
        [cos(theta1+pi/2), 0, sin(theta1+pi/2), 0],
        [sin(theta1+pi/2), 0, -cos(theta1+pi/2), 0],
        [0, 1, 0, d1],
        [0, 0, 0, 1]
      ])
      return H01

    def _h12(self, theta2):
      a2 = -0.425
      H12 = np.matrix([
        [cos(theta2-pi/2), -sin(theta2-pi/2), 0, a2*cos(theta2-pi/2)],
        [sin(theta2-pi/2), cos(theta2-pi/2), 0, a2*sin(theta2-pi/2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
      ])
      return H12
    
    def _h23(self, theta3):
      a3 = -0.39225
      H23 = np.matrix([
        [cos(theta3), -sin(theta3), 0, a3*cos(theta3)],
        [sin(theta3), cos(theta3), 0, a3*sin(theta3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
      ])
      return H23
    
    def _h34(self, theta4):
      d4 = 0.10915
      H34 = np.matrix([
        [cos(theta4-pi/2), 0, sin(theta4-pi/2), 0],
        [sin(theta4-pi/2), 0, -cos(theta4-pi/2), 0],
        [0, 1, 0, d4],
        [0, 0, 0, 1]
      ])
      return H34
    
    def _h45(self, theta5):
      d5 = 0.09465
      H45 = np.matrix([
        [cos(theta5), 0, -sin(theta5), 0],
        [sin(theta5), 0, cos(theta5), 0],
        [0, -1, 0, d5],
        [0, 0, 0, 1]
      ])
      return H45
    
    def _h56(self, theta6):
      d6 = 0.0823
      H56 = np.matrix([
        [cos(theta6), -sin(theta6), 0, 0],
        [sin(theta6), cos(theta6), 0, 0],
        [0, 0, 1, d6],
        [0, 0, 0, 1]
      ])
      return H56
    
    def forward_kinematic(self, theta1, theta2, theta3, theta4, theta5, theta6):
      """
      join   |   theta   |      d  |      a   |     alpha |
      _____________________________________________________
      1      |   theta1  |      d1 |      0   |     pi/2  |
      2      |   theta2  |      0  |      a2  |     0     |
      3      |   theta3  |      0  |      a3  |     0     |
      4      |   theta4  |      d4 |      0   |     pi/2  |
      5      |   theta5  |      d5 |      0   |     -pi/2 |
      6      |   theta6  |      d6 |      0   |     0     |
      """

      T01 = self._t01()

      H01 = self._h01(theta1)

      H12 = self._h12(theta2)

      H23 = self._h23(theta3)

      H34 = self._h34(theta4)

      H45 = self._h45(theta5)

      H56 = self._h56(theta6)

      matrix = T01*H01*H12*H23*H34*H45*H56

      matrix[abs(matrix) < 1e-15] = 0

      return matrix

    def inverse_kinematic(self, T_matrix):

      # deslocamento do braço no cenário
      T01 = self._t01()

      d1 = 0.089159
      a2 = -0.425
      a3 = -0.39225
      d4 = 0.10915
      d5 = 0.09465
      d6 = 0.0823

      solutions = np.matrix(np.zeros((8,6)))
      print(solutions)
      # solução theta 1
      p05: np.matrix = T_matrix * np.matrix([0, 0, -d6, 1]).T
      phi1 = arctan2(p05.item(0, 0), p05.item(1, 0))
      phi2 = arccos(d4/sqrt((p05.item(0, 0)*p05.item(0, 0)) +
                    (p05.item(1, 0)*p05.item(1, 0))))
      for i in range(0, 4):
        solutions[i,0] = (phi1 + phi2 + pi/2) 
        solutions[i+4,0] = (phi1 - phi2 + pi/2)
      # solução theta 5

      sol1 = ((T_matrix.item(0, 3) * sin(solutions[0, 0])) -
        (T_matrix.item(1, 3) * cos(solutions[0, 0])) - d4) / d6
      sol2 = ((T_matrix.item(0, 3) * sin(solutions[4,0])) -
              (T_matrix.item(1, 3) * cos(solutions[4,0])) - d4) / d6
      solutions[0,4] = real(acos(sol1))
      solutions[4,4] = real(acos(sol1))
      solutions[1,4] = real(-acos(sol1))
      solutions[5,4] = real(-acos(sol1))
      solutions[2,4] = real(acos(sol2))
      solutions[6,4] = real(acos(sol2))
      solutions[3,4] = real(-acos(sol2))
      solutions[7,4] = real(-acos(sol2))

      # solução theta 6 

      for i in range(0, 7):
        H01 = self._h01(solutions[i,0])
        T61 = (((T01 * H01).I)*T_matrix).I
        Tzy61 = T61[1,2]
        Tzx61 = T61[0,2]
        theta5 = solutions[i, 4]
        solutions[i,5] = arctan2(-Tzy61/sin(theta5), Tzx61/sin(theta5))

      # solução theta 3
      for i in range(0, 7, 2):
        H01 = self._h01(solutions[i,0])
        H45 = self._h45(solutions[i,4])
        H56 = self._h56(solutions[i,5])
        H16 = ((T01 * H01).I)*T_matrix
        H14 = H16*((H45*H56).I)
        P13 = (H14 * np.matrix([0, -d4, 0, 1]).T)
        P13 = np.delete(P13, 3, 0)
        theta3 = acos(((np.linalg.norm(P13)*np.linalg.norm(P13)) - a2*a2 - a3*a3) / (2*a2*a3))
        solutions[i,2] = real(theta3)
        solutions[i+1,2] = real(-theta3)
      

      # solução theta 2 e 4
      for i in range(0, 7):
        H01 = self._h01(solutions[i,0])
        H45 = self._h45(solutions[i,4])
        H56 = self._h56(solutions[i,5])
        H16 = ((T01 * H01).I)*T_matrix
        H14 = H16*((H45*H56).I)
        P13 = (H14 * np.matrix([0, -d4, 0, 1]).T)
        P13 = np.delete(P13, 3, 0)
        solutions[i, 1] = -arctan2(P13[1,0], -P13[1,0]) + arcsin((a3*sin(solutions[i,2]))/np.linalg.norm(P13))

        # theta 4

        H23 = self._h23(solutions[i,2])
        H12 = self._h12(solutions[i,1])
        H34 = ((H12*H23).I)*H14
        solutions[i,3] = arctan2(H34[1,0], H34[0,0])
      print(solutions*180/pi)

    def cubic(self, qi, qf, vi, vf, ti, tf):
      t = np.arange(ti, tf, 0.05)
      c = np.ones(t.size)
      m = np.matrix([
        [1, ti, ti*ti, ti*ti*ti],
        [0, 1, 2*ti, 3*ti*ti*ti],
        [1, tf, tf*tf, tf*tf*tf],
        [0, 1, 2*tf, 3*tf*tf]
      ])
      b = np.matrix([qi, vi, qf, vf]).T
      a = (m.I)*b

      pos = np.multiply(a[0,0], c) + np.multiply(a[1,0], t) + np.multiply(a[2,0], np.multiply(t, t)) + np.multiply(a[3,0], np.multiply(t, np.multiply(t, t)))
      vel = np.multiply(a[1,0], c) + 2*np.multiply(a[2,0], t) + 3*np.multiply(a[3,0], np.multiply(t, t))
      accel = 2*np.multiply(a[2,0], c) + 6*np.multiply(a[3,0], t)
      return pos, vel, accel



    def get_t_table(theta1, theta2, theta3, theta4, theta5, theta6):
        matrix = np.matrix([
            [cos(pi/2), -sin(pi/2), 0, -0.214],
            [sin(pi/2), cos(pi/2), 0, -0.314],
            [0, 0, 1, -0.41],
            [0, 0, 0, 1]
        ])
        for idx, value in enumerate(alpha):
            matrix = matrix * \
                dh_transform(alpha[idx], a[idx], d[idx], theta[idx])

        matrix[abs(matrix) < 1e-15] = 0
        return matrix

    def get_pos(theta: list):
        pass

if __name__ == '__main__':
  ur5 = UR5_arm()
  matrix = ur5.forward_kinematic(pi/2, 0, pi/2, 0, 0, 0)
  matrix = ur5.forward_kinematic((300 * pi / 180), (14.78 * pi / 180) + pi/2,  63.25 * pi / 180 , -78.04 * pi / 180, (180 * pi / 180) + pi/2, 59 * pi / 180)
  print(matrix)
  matrix = np.matrix([
    [1.0, 0, 0, 0.4000004827976227], 
    [0, 1.0, 0, -0.3580000102519989], 
    [0, 0, 1.0, 0.4714893400669098],
    [0, 0, 0, 1.0]
    ])
  print(matrix)
  ur5.inverse_kinematic(matrix)

  d = [0.4992-0.41, 0, 0, -0.1038+0.2140, 0.4994-0.4047, -0.0288+0.1038]
  theta = [0, 0, 0, 0, 0, 0]
  a = [0, 0, -0.7391+0.314, -1.1312+0.7391, 0, 0]
  alpha = [0, pi/2, 0, 0, pi/2, -pi/2]


  def dh_transform(alpha: float, a: float, d: float, theta: float):
      dh_matrix: np.matrix = np.matrix([
          [cos(theta), -sin(theta), 0, a],
          [sin(theta)*cos(alpha), cos(theta) *
          cos(alpha), -sin(alpha), -sin(alpha)*d],
          [sin(theta)*sin(alpha), cos(theta) *
          sin(alpha), cos(alpha), cos(alpha)*d],
          [0, 0, 0, 1]
      ])
      return dh_matrix


  # posição inicial da peça considerando o gira em relação ao eixo de origem da simulaçao
  matrix = np.matrix([
      [cos(pi/2), -sin(pi/2), 0, -0.214],
      [sin(pi/2), cos(pi/2), 0, -0.314],
      [0, 0, 1, 0.4246],
      [0, 0, 0, 1]
  ])
  """ matrix = np.matrix([
    [-1, 0, 0, -0.214],
    [0, -1, 0, -0.314],
    [0, 0, 1, 0.4246],
    [0, 0, 0, 1]
  ]) """

  #matrix = np.identity(4)
  for idx, value in enumerate(alpha):
      matrix = matrix * dh_transform(alpha[idx], a[idx], d[idx], theta[idx])

  matrix[abs(matrix) < 1e-15] = 0

  print(matrix)


  # THETA 1
  p05: np.matrix = matrix * np.matrix([0, 0, -d[5], 1]).T
  phi1 = arctan2(p05.item(0, 0), p05.item(1, 0))
  phi2 = arccos(d[3]/sqrt((p05.item(0, 0)*p05.item(0, 0)) +
                (p05.item(1, 0)*p05.item(1, 0))))
  theta1 = phi1 + phi2 + pi/2
  theta1_sol2 = phi1 - phi2 + pi/2

  print(theta1 * 180 / pi)


  # THETA 5
  sol1 = ((matrix.item(0, 3) * sin(theta1)) -
          (matrix.item(1, 3) * cos(theta1)) - d[3])
  sol2 = ((matrix.item(0, 3) * sin(theta1_sol2)) -
          (matrix.item(1, 3) * cos(theta1_sol2)) - d[3])
  print(value)
  theta5 = arccos(value / d[5])
  print(theta5 * 180 / pi)

  # THETA 6

  theta6 = arctan2(((- matrix.item(1, 0) * sin(theta1)) + (matrix.item(1, 1) * cos(theta1))) /
                  sin(theta5), ((matrix.item(0, 0)*sin(theta1)) - (matrix.item(0, 1) * cos(theta1))) / sin(theta5))

  print(theta6 * 180 / pi)
  # THETA 3

  dh_matrix = np.identity(4)
  for i in range(1, 3):
      dh_matrix = dh_matrix * dh_transform(alpha[i], a[i], d[i], theta[i])

  #theta3 = acos()
