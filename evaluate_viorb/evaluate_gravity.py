#!/usr/bin/env python

import numpy as np
from skinematics import quat
from math import pi

# inputs
# from body frame to world
q_wb = [0.042773,0.816907,-0.063879,0.571623] # w, x, y, z
# from random camera (r) to reference camera (c)
q_cr = [1.0, 0.0, 0.0, 0.0]
g_r = [0.155649, 9.290025, 3.147590]
T_bc = [	
			0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
			0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
			-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
			0.0, 0.0, 0.0, 1.0
		]

g_absolute = np.asarray([0, 0, -9.8])

R_wb = quat.quat2rotmat(q_wb)
R_cr = quat.quat2rotmat(q_cr)
g_r = np.asarray(g_r).transpose()
T_bc = np.asarray(T_bc).reshape(4, 4)

g_world = np.dot(R_wb, 
	np.dot(T_bc[:3, :3], 
		np.dot(R_cr, g_r)
	)
)

print g_world
print np.arccos(
	np.dot(g_world/np.linalg.norm(g_world), g_absolute/np.linalg.norm(g_absolute))
) * 180 / pi





