import numpy as np
from numpy.core.function_base import linspace

f = open('../resources/waypoint_for_uav/waypoint_3.txt', 'w')
z = 1



t = .2
for i in range(64):
    f.write('%.6f %.6f %.6f %.6f %.6f\n'%(t, 0,0, z, 225))

t = .2
# 第三段
xs = np.linspace(-2.0 + np.sqrt(2) / 4, 0, int((4 * np.sqrt(2) - 1) / 2 * 10))
for i in range(len(xs)):
    x = xs[len(xs) - i - 1]
    print(x)
    y = (2 + np.sqrt(2) / 4) / (2 - np.sqrt(2) / 4) * x
    print(y)
    yaw = 225
    f.write('%.6f %.6f %.6f %.6f %.6f\n'%(t, x, y, z, yaw))


t = .2
# 第四段
for theta in linspace(-np.pi, -np.pi / 4, 12):
    x = -2 + 0.5 * np.cos(-theta - 1.25 * np.pi)
    y = -2 + 0.5 * np.sin(-theta - 1.25 * np.pi)
    yaw = ((np.arctan2(-x - 2, y + 2) * 180 / np.pi) + 360) % 360
    f.write('%.6f %.6f %.6f %.6f %.6f\n'%(t, x, y, z, yaw))

t = .2
# 第五段
x = -2.5
ys = np.linspace(-2, 0.9, 29)
yaw = 90.0 
for y in ys:
    f.write('%.6f %.6f %.6f %.6f %.6f\n'%(t, x, y, z, yaw))

f.close()