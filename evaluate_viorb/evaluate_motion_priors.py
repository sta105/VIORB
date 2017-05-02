#!/usr/bin/env python

import sys
import numpy as np
from skinematics import quat, rotmat
import associate

if len(sys.argv) < 4:
	print 'Usage: python evalute_motion_priors <groundtruth> <imu_prior> <cvm_prior>'
	exit(0)

def toPose(position, quaternion):
	R = quat.quat2rotmat(np.asarray(quaternion))
	T = np.eye(4)
	T[:3, :3] = R
	T[:3, 3] = np.asarray(position).transpose()
	return T

groundtruth_file = sys.argv[1]
imu_file = sys.argv[2]
cmv_file = sys.argv[3]

groundtruth = associate.read_file_list(groundtruth_file, timestamp_scale=1e-9)
imu_prior = associate.read_file_list(imu_file)
cvm_prior = associate.read_file_list(cmv_file)

# compute the relative pose
groundtruth_relative = [(i, None) for i in groundtruth]
groundtruth_relative.sort()

for i in range(1, len(groundtruth_relative)):
	p1 = groundtruth[groundtruth_relative[i-1][0]][0:3]
	q1 = groundtruth[groundtruth_relative[i-1][0]][3:7]
	T1 = toPose(p1, q1)

	p2 = groundtruth[groundtruth_relative[i][0]][0:3]
	q2 = groundtruth[groundtruth_relative[i][0]][3:7]
	T2 = toPose(p2, q2)

	relative_T = np.linalg.inv(T2) * T1
	groundtruth_relative[i] = (groundtruth_relative[i][0], relative_T)

groundtruth_relative = dict(groundtruth_relative)

imu_matches = associate.associate(groundtruth_relative, imu_prior, offset=0.0, max_difference=0.02)
cvm_matches = associate.associate(groundtruth_relative, cvm_prior, offset=0.0, max_difference=0.02)

if len(imu_matches) != len(cvm_matches):
	print 'Not exact correspondence between IMU and CVM'
	exit(0)

imu_errors = [0 for i in range(len(imu_matches))]
cvm_errors = [0 for i in range(len(cvm_matches))]

for match_idx in range(len(imu_matches)):
	ground_relative_T = groundtruth_relative[imu_matches[match_idx][0]]

	imu_prediction = imu_prior[imu_matches[match_idx][1]]
	cvm_prediction = cvm_prior[cvm_matches[match_idx][1]]

	imu_p = imu_prediction[0:3]
	imu_q = imu_prediction[3:7]
	# convention of ORB-SLAM is x, y, z, w: but we need w, x, y, z for skinematics
	imu_q[0], imu_q[3] = imu_q[3], imu_q[0]
	imu_relative_T = toPose(imu_p, imu_q)

	cvm_p = cvm_prediction[0:3]
	cvm_q = cvm_prediction[3:7]
	# convention of ORB-SLAM is x, y, z, w: but we need w, x, y, z for skinematics
	cvm_q[0], cvm_q[3] = cvm_q[3], cvm_q[0]
	cvm_relative_T = toPose(cvm_p, cvm_q)

	imu_error = np.linalg.inv(ground_relative_T) * imu_relative_T
	cvm_error = np.linalg.inv(ground_relative_T) * csv_relative_T