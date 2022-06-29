from cmath import pi
import numpy as np
import matplotlib.pyplot as plt


qi = 0
qf = pi/2
vi = 0
vf = 0
ti = 0
tf = 10


t = np.arange(ti, tf, (tf-ti)/100)
print(t)
c = np.ones(t.size)
print(c)
m = np.matrix([
  [1, ti, ti*ti, ti*ti*ti],
  [0, 1, 2*ti, 3*ti*ti*ti],
  [1, tf, tf*tf, tf*tf*tf],
  [0, 1, 2*tf, 3*tf*tf]
])
print(m)
b = np.matrix([qi, vi, qf, vf]).T
print(b)
a = (m.I)*b
print(a[1,0])

pos = np.multiply(a[0,0], c) + np.multiply(a[1,0], t) + np.multiply(a[2,0], np.multiply(t, t)) + np.multiply(a[3,0], np.multiply(t, np.multiply(t, t)))
vel = np.multiply(a[1,0], c) + 2*np.multiply(a[2,0], t) + 3*np.multiply(a[3,0], np.multiply(t, t))
accel = 2*np.multiply(a[2,0], c) + 6*np.multiply(a[3,0], t)
plt.plot(pos)
plt.figure(2)
plt.plot(vel)
plt.figure(3)
plt.plot(accel)
plt.show()