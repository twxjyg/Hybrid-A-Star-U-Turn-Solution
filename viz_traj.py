#!/usr/bin/python

import matplotlib.pyplot as plt
import math
import sys


if len(sys.argv) > 1:
    lane_data_file = sys.argv[1]
else:
    lane_data_file = "./test_data/lane_data.txt"

smoothed_path = open(lane_data_file + ".traj", 'r')
a_star_path = open(lane_data_file + ".traj.astar", 'r')

# draw smoothed path
x = []
y = []
theta = []
kappa = []
for line in smoothed_path:
    point = line.split(',')
    x.append(float(point[0]))
    y.append(float(point[1]))
    theta.append(float(point[2]))
    kappa.append(float(point[3]))


plt.subplot(2, 1, 1)

plt.axis('equal')
plt.plot(x, y, 'g-', label='smooted traj')
plt.legend(loc='upper left')
# plt.plot(x, kappa, 'g-')
u = []
v = []
for t in theta:
    u.append(0.5 * math.cos(t))
    v.append(0.5 * math.sin(t))

plt.quiver(x, y, u, v, angles='uv', color='g', width=0.002)

plt.xlabel('x')
plt.ylabel('y')

plt.subplot(2, 1, 2)

# plt.plot(x, theta, 'g-')
plt.axis('equal')
plt.plot(x, kappa, 'g-', label='smoothed kappa')
plt.legend(loc='upper left')
plt.xlabel('x')
plt.ylabel('kappa')


# draw a-star raw path
x = []
y = []
theta = []
kappa = []
for line in a_star_path:
    point = line.split(',')
    x.append(float(point[0]))
    y.append(float(point[1]))
    theta.append(float(point[2]))
    kappa.append(float(point[3]))

plt.subplot(2, 1, 1)
plt.axis('equal')
plt.plot(x, y, 'r-', label='hybrid-a-star traj')
plt.legend(loc='upper left')
# plt.plot(x, kappa, 'r--')

u = []
v = []
for t in theta:
    u.append(0.5 * math.cos(t))
    v.append(0.5 * math.sin(t))

plt.quiver(x, y, u, v, angles='uv', color='r', width=0.002)

plt.subplot(2, 1, 2)
# plt.plot(x, theta)
plt.axis('equal')
plt.plot(x, kappa, 'r-', label='hybrid-a-star kappa')
plt.legend(loc='upper left')




plt.savefig(lane_data_file+".png")

plt.show()
