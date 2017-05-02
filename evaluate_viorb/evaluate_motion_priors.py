#!/usr/bin/env python

import sys
import numpy as np
from skinematics import quat, rotmat
import associate
from math import pi
import matplotlib.pyplot as plt

if len(sys.argv) < 4:
    print 'Usage: python evalute_motion_priors <groundtruth> <imu_prior> <cvm_prior>'
    exit(0)

def toPose(position, quaternion):
    R = quat.quat2rotmat(np.asarray(quaternion))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(position).transpose()
    return T

def navStateToPose(nav_states):
    nav_states_list = [(i, None) for i in nav_states]
    nav_states_list.sort()

    for i in range(len(nav_states_list)):
        p = nav_states[nav_states_list[i][0]][0:3]
        q = nav_states[nav_states_list[i][0]][3:7]
        # convention of ORB-SLAM is x, y, z, w: but we need w, x, y, z for skinematics
        q = [q[3], q[0], q[1], q[2]]
        T = toPose(p, q)

        nav_states_list[i] = (nav_states_list[i][0], T)

    return dict(nav_states_list)

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

    relative_T = np.dot(np.linalg.inv(T2), T1)
    groundtruth_relative[i] = (groundtruth_relative[i][0], relative_T)

groundtruth_relative = dict(groundtruth_relative)

imu_relative = navStateToPose(imu_prior)
cvm_relative = navStateToPose(cvm_prior)

imu_matches = associate.associate(groundtruth_relative, imu_relative, offset=0.0, max_difference=0.02)
cvm_matches = associate.associate(groundtruth_relative, cvm_relative, offset=0.0, max_difference=0.02)

if len(imu_matches) != len(cvm_matches):
    print 'Not exact correspondence between IMU and CVM'
    exit(0)

imu_errors = np.zeros( (len(imu_matches), 4) )
cvm_errors = np.zeros( (len(cvm_matches), 4) )

for match_idx in range(len(imu_matches)):
    ground_relative_T = groundtruth_relative[imu_matches[match_idx][0]]
    imu_relative_T = imu_relative[imu_matches[match_idx][1]]
    cvm_relative_T = cvm_relative[cvm_matches[match_idx][1]]

    imu_R_error = np.asarray(rotmat.rotmat2Euler(imu_relative_T[:3, :3])) - np.asarray(rotmat.rotmat2Euler(ground_relative_T[:3, :3]))
    # imu_R_error = np.dot(imu_relative_T[:3, :3].transpose(), ground_relative_T[:3, :3])
    
    cvm_R_error = np.asarray(rotmat.rotmat2Euler(cvm_relative_T[:3, :3])) - np.asarray(rotmat.rotmat2Euler(ground_relative_T[:3, :3]))
    # cvm_R_error = np.dot(cvm_relative_T[:3, :3].transpose(), ground_relative_T[:3, :3])
    
    imu_t_error = np.arccos(
        np.dot(ground_relative_T[:3, 3]/np.linalg.norm(ground_relative_T[:3, 3]), imu_relative_T[:3, 3]/np.linalg.norm(imu_relative_T[:3, 3]))
    ) * 180 / pi

    cvm_t_error = np.arccos(
        np.dot(ground_relative_T[:3, 3]/np.linalg.norm(ground_relative_T[:3, 3]), cvm_relative_T[:3, 3]/np.linalg.norm(cvm_relative_T[:3, 3]))
    ) * 180 / pi

    # imu_t_error = ground_relative_T[:3, 3] - imu_relative_T[:3, 3]
    # imu_t_error = np.linalg.norm(imu_t_error)

    # cvm_t_error = ground_relative_T[:3, 3] - cvm_relative_T[:3, 3]
    # cvm_t_error = np.linalg.norm(cvm_t_error)

    imu_errors[match_idx, :3] = imu_R_error
    imu_errors[match_idx, 3] = imu_t_error

    cvm_errors[match_idx, :3] = cvm_R_error
    cvm_errors[match_idx, 3] = cvm_t_error

    # print imu_matches[match_idx]
    # print imu_q
    # print quat.rotmat2quat(imu_relative_T[:3, :3])
    # print cvm_q
    # print quat.rotmat2quat(cvm_relative_T[:3, :3])
    # print quat.rotmat2quat(ground_relative_T[:3, :3])
    
    # print rotmat.rotmat2Euler(imu_relative_T)
    # print rotmat.rotmat2Euler(cvm_relative_T)
    # print rotmat.rotmat2Euler(ground_relative_T)
    # exit(0)

plt.subplot(231)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(imu_errors.shape[0]), imu_errors[:, 0], label='IMU error')
plt.title('Rotation X error')
plt.legend()

plt.subplot(234)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 0], label='CVM error')
plt.title('Rotation X error')
plt.legend()


plt.subplot(232)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(imu_errors.shape[0]), imu_errors[:, 1], label='IMU error')
plt.title('Rotation Y error')
plt.legend()

plt.subplot(235)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 1], label='CVM error')
plt.title('Rotation Y error')
plt.legend()

plt.subplot(233)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(imu_errors.shape[0]), imu_errors[:, 2], label='IMU error')
plt.title('Rotation Z error')
plt.legend()

plt.subplot(236)
plt.xlabel('Frame ID')
plt.ylabel('Error (degrees)')
plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 2], label='CVM error')
plt.title('Rotation Z error')
plt.legend()

######################################### Translation #########################################

plt.figure()

plt.subplot(211)
plt.xlabel('Frame ID')
plt.ylabel('Error (degree)')
plt.title('Translation error')
plt.plot(imu_errors[:, 3], label='IMU error')
plt.legend()

plt.subplot(212)
plt.xlabel('Frame ID')
plt.ylabel('Error (degree)')
plt.title('Translation error')
plt.plot(cvm_errors[:, 3], label='CVM error')   
plt.legend()

# save
groundtruth_relative_list = [
    [
        imu_matches[match_idx][0], 
        groundtruth_relative[imu_matches[match_idx][0]][0, 3], 
        groundtruth_relative[imu_matches[match_idx][0]][1, 3], 
        groundtruth_relative[imu_matches[match_idx][0]][2, 3]
    ] +
    list(
        rotmat.rotmat2Euler(groundtruth_relative[imu_matches[match_idx][0]][:3, :3])
    )
    for match_idx in range(len(imu_matches))
]
groundtruth_relative_list.sort()
groundtruth_relative_list = np.asarray(groundtruth_relative_list)
np.savetxt("gt_relative.csv", groundtruth_relative_list, fmt="%.6f")

imu_relative_list = [
    [
        imu_matches[match_idx][1], 
        imu_relative[imu_matches[match_idx][1]][0, 3], 
        imu_relative[imu_matches[match_idx][1]][1, 3], 
        imu_relative[imu_matches[match_idx][1]][2, 3]
    ] +
    list(
        rotmat.rotmat2Euler(imu_relative[imu_matches[match_idx][0]][:3, :3])
    )
    for match_idx in range(len(imu_matches))
]
imu_relative_list.sort()
imu_relative_list = np.asarray(imu_relative_list)
np.savetxt("imu_relative.csv", imu_relative_list, fmt="%.6f")

cvm_relative_list = [
    [
        cvm_matches[match_idx][1], 
        cvm_relative[cvm_matches[match_idx][1]][0, 3], 
        cvm_relative[cvm_matches[match_idx][1]][1, 3], 
        cvm_relative[cvm_matches[match_idx][1]][2, 3]
    ] +
    list(
        rotmat.rotmat2Euler(cvm_relative[cvm_matches[match_idx][0]][:3, :3])
    )
    for match_idx in range(len(cvm_matches))
]
cvm_relative_list.sort()
cvm_relative_list = np.asarray(cvm_relative_list)
np.savetxt("cvm_relative.csv", cvm_relative_list, fmt="%.6f")
    
plt.show()