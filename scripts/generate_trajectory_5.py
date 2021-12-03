import numpy as np
from numpy.core.function_base import linspace

f = open('../resources/waypoint_for_uav/waypoint_5.txt', 'w')
z = 1

t = .2
for i in range(128):
    f.write('%.6f %.6f %.6f %.6f %.6f\n'%(t, -2.5,2.5, z, 225))
f.close()