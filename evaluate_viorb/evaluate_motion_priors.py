#!/usr/bin/env python

import sys
import numpy as np
from skinematics import quat, rotmat
import associate
import math
from math import pi, sqrt
import matplotlib.pyplot as plt

T_bc = np.asarray([    
            0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0
        ]).reshape(4, 4)

T_cb = np.linalg.inv(T_bc)

if len(sys.argv) < 4:
    print 'Usage: python evalute_motion_priors <groundtruth> <imu_prior> <cvm_prior>'
    exit(0)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def toPose(position, quaternion):
    quaternion = np.asarray(quaternion)
    quaternion = quaternion / np.linalg.norm(quaternion)

    R = quat.quat2rotmat(quaternion)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(position).transpose()
    return T

def navStateToPose(nav_states):
    nav_states_list = [(i, np.eye(4)) for i in nav_states]
    nav_states_list.sort()

    nav_state_reference_timestamps = [(i, None) for i in nav_states]

    for i in range(len(nav_states_list)):
        t2 = nav_states_list[i][0]
        p2 = nav_states[nav_states_list[i][0]][0:3]
        q2 = nav_states[nav_states_list[i][0]][3:7]
        # convention of ORB-SLAM is x, y, z, w: but we need w, x, y, z for skinematics
        q2 = np.asarray([q2[3], q2[0], q2[1], q2[2]])
        T2 = toPose(p2, q2)

        t1 = nav_states[nav_states_list[i][0]][7]
        p1 = nav_states[nav_states_list[i][0]][8:11]
        q1 = nav_states[nav_states_list[i][0]][11:15]
        # convention of ORB-SLAM is x, y, z, w: but we need w, x, y, z for skinematics
        q1 = np.asarray([q1[3], q1[0], q1[1], q1[2]])
        T1 = toPose(p1, q1)

        relative_T = np.dot(T2, T1)
        # relative_T = np.eye(4)
        # relative_T[:3, :3] = np.dot(T2[:3, :3].transpose(), T1[:3, :3])
        # relative_T[:3, 3] = np.dot(T2[:3, :3].transpose(), T1[:3, 3] - T2[:3, 3])
        nav_states_list[i] = (t2, relative_T)
        nav_state_reference_timestamps[i] = (t2, t1)

        
    return dict(nav_states_list), dict(nav_state_reference_timestamps)

# compute the relative pose
def absoluteToRelative(absolute):
    relative = [(i, np.eye(4)) for i in absolute]
    relative.sort()

    for i in range(10, len(relative)):
        p1 = absolute[relative[i-10][0]][0:3]
        q1 = absolute[relative[i-10][0]][3:7]
        T1 = toPose(p1, q1)

        p2 = absolute[relative[i][0]][0:3]
        q2 = absolute[relative[i][0]][3:7]
        T2 = toPose(p2, q2)

        # relative_T = np.dot(np.linalg.inv(T2), T1)
        relative_T = np.eye(4)
        relative_T[:3, :3] = np.dot(T2[:3, :3].transpose(), T1[:3, :3])
        relative_T[:3, 3] = np.dot(T2[:3, :3].transpose(), T1[:3, 3] - T2[:3, 3])
        relative[i] = (relative[i][0], relative_T)

    return dict(relative)

def parseGroundtruth(nav_states):
    nav_states_list = [(i, np.eye(4)) for i in nav_states]
    nav_states_list.sort()

    for i in range(len(nav_states_list)):
        t = nav_states_list[i][0]
        p = nav_states[nav_states_list[i][0]][0:3]
        q = nav_states[nav_states_list[i][0]][3:7]
        q = np.asarray(q)
        T = toPose(p, q)

        nav_states_list[i] = (t, T)

    return dict(nav_states_list)

def findRelativeGroundtruth(relative_pose_dict, reference_timestamp, groundtruth_poses, matches):
    relative_ground_truth = []
    # convert from list of tuples to dict (first element is ground truth timestamp, but we want the relative pose timestamp to be the key)
    matches_dict = dict([(i[1], i[0]) for i in matches])

    for pose_t2 in relative_pose_dict:
        groundtruth_T2 = groundtruth_poses[matches_dict[pose_t2]]

        pose_t1 = reference_timestamp[pose_t2]
        groundtruth_T1 = groundtruth_poses[matches_dict[pose_t1]]

        relative_T = np.dot(np.linalg.inv(groundtruth_T2), groundtruth_T1)
        # convert groundtruth to camera frame
        relative_T = np.dot(
            np.dot(T_cb, relative_T),
            T_bc
        )
        relative_ground_truth.append( (matches_dict[pose_t2], relative_T) )

    return dict(relative_ground_truth)

groundtruth_file = sys.argv[1]
imu_file = sys.argv[2]
cmv_file = sys.argv[3]

groundtruth = associate.read_file_list(groundtruth_file, timestamp_scale=1e-9)
imu_prior = associate.read_file_list(imu_file)
cvm_prior = associate.read_file_list(cmv_file)

# convert from list to pose dictionary
groundtruth_poses = parseGroundtruth(groundtruth)

# get only the ones contained in both imu and cvm
imu_cvm_matches = associate.associate(imu_prior, cvm_prior, offset=0.0, max_difference=0.005)
imu_prior = {i[0]: imu_prior[ i[0] ] for i in imu_cvm_matches}
cvm_prior = {i[0]: cvm_prior[ i[1] ] for i in imu_cvm_matches}
# get only the groundtruth contained in both imu and cvm (and the one previous to it!)
# imu_groundtruth_matches = associate.associate(imu_prior, groundtruth, offset=0.0, max_difference=0.02)
# groundtruth = {i[0]: groundtruth[ i[0] ] for i in imu_groundtruth_matches}

# to relative
# groundtruth_relative = absoluteToRelative(groundtruth)

# --------------------------------------- relative poses of imu --------------------------------------- #
imu_relative, imu_reference_timestamps = navStateToPose(imu_prior)
required_timestamps = imu_reference_timestamps.keys() + imu_reference_timestamps.values()
# the associate function required dict
imu_matches = associate.associate(groundtruth_poses, {i:None for i in required_timestamps}, offset=0.0, max_difference=0.005)
imu_groundtruth_relative = findRelativeGroundtruth(imu_relative, imu_reference_timestamps, groundtruth_poses, imu_matches)

# --------------------------------------- relative poses of cvm --------------------------------------- #
cvm_relative, cvm_reference_timestamps = navStateToPose(cvm_prior)
required_timestamps = cvm_reference_timestamps.keys() + cvm_reference_timestamps.values()
# the associate function required dict
cvm_matches = associate.associate(groundtruth_poses, {i:None for i in required_timestamps}, offset=0.0, max_difference=0.005)
cvm_relative_groundtruth = findRelativeGroundtruth(cvm_relative, cvm_reference_timestamps, groundtruth_poses, cvm_matches)


if len(imu_matches) != len(cvm_matches):
    print 'Not exact correspondence between IMU and CVM'
    exit(0)

groundtruth_relative = imu_groundtruth_relative
imu_matches = associate.associate(groundtruth_relative, imu_relative, offset=0.0, max_difference=0.005)
cvm_matches = associate.associate(groundtruth_relative, cvm_relative, offset=0.0, max_difference=0.005)

#imu_errors = np.zeros( (len(imu_matches), 4) )
#cvm_errors = np.zeros( (len(cvm_matches), 4) )

imu_predictions = np.zeros( (len(imu_matches), 6) )
cvm_predictions = np.zeros( (len(cvm_matches), 6) )
groundtruth_predictions = np.zeros( (len(imu_matches), 6) )
timestamp_at_prediction = np.zeros( (len(imu_matches)) )

for match_idx in range(len(imu_matches)):
    ground_relative_T = groundtruth_relative[imu_matches[match_idx][0]]
    imu_relative_T = imu_relative[imu_matches[match_idx][1]]
    cvm_relative_T = cvm_relative[cvm_matches[match_idx][1]]


    # ---------------------------- Errors ---------------------------- #
    # imu_R_error = np.asarray(rotationMatrixToEulerAngles(imu_relative_T[:3, :3])) - np.asarray(rotationMatrixToEulerAngles(ground_relative_T[:3, :3]))
    # # imu_R_error = np.dot(imu_relative_T[:3, :3].transpose(), ground_relative_T[:3, :3])
    
    # cvm_R_error = np.asarray(rotationMatrixToEulerAngles(cvm_relative_T[:3, :3])) - np.asarray(rotationMatrixToEulerAngles(ground_relative_T[:3, :3]))
    # # cvm_R_error = np.dot(cvm_relative_T[:3, :3].transpose(), ground_relative_T[:3, :3])
    

    # imu_t_error = np.arccos(
    #     np.dot(ground_relative_T[:3, 3]/np.linalg.norm(ground_relative_T[:3, 3]), imu_relative_T[:3, 3]/np.linalg.norm(imu_relative_T[:3, 3]))
    # ) * 180 / pi

    # cvm_t_error = np.arccos(
    #     np.dot(ground_relative_T[:3, 3]/np.linalg.norm(ground_relative_T[:3, 3]), cvm_relative_T[:3, 3]/np.linalg.norm(cvm_relative_T[:3, 3]))
    # ) * 180 / pi

    # # imu_t_error = ground_relative_T[:3, 3] - imu_relative_T[:3, 3]
    # # imu_t_error = np.linalg.norm(imu_t_error)

    # # cvm_t_error = ground_relative_T[:3, 3] - cvm_relative_T[:3, 3]
    # # cvm_t_error = np.linalg.norm(cvm_t_error)

    # imu_errors[match_idx, :3] = imu_R_error
    # imu_errors[match_idx, 3] = imu_t_error

    # cvm_errors[match_idx, :3] = cvm_R_error
    # cvm_errors[match_idx, 3] = cvm_t_error


    # ----------------------- Predictions ----------------------------- #
    imu_predictions[match_idx, :3] = np.asarray(rotationMatrixToEulerAngles(imu_relative_T[:3, :3])) * 180 / pi
    # imu_predictions[match_idx, :3] = np.asarray(imu_prior[imu_matches[match_idx][1]][3:6])/2.0 * 180 / pi
    imu_predictions[match_idx, 3:] = imu_relative_T[:3, 3]/np.linalg.norm(imu_relative_T[:3, 3])

    cvm_predictions[match_idx, :3] = np.asarray(rotationMatrixToEulerAngles(cvm_relative_T[:3, :3])) * 180 / pi
    # cvm_predictions[match_idx, :3] = np.asarray(cvm_prior[cvm_matches[match_idx][1]][3:6])/2.0 * 180 / pi
    cvm_predictions[match_idx, 3:] = cvm_relative_T[:3, 3]/np.linalg.norm(cvm_relative_T[:3, 3])

    groundtruth_predictions[match_idx, :3] = np.asarray(rotationMatrixToEulerAngles(ground_relative_T[:3, :3])) * 180 / pi
    # groundtruth_predictions[match_idx, :3] = np.asarray(groundtruth[imu_matches[match_idx][0]][4:7])/2.0 * 180 / pi
    groundtruth_predictions[match_idx, 3:] = ground_relative_T[:3, 3]/np.linalg.norm(ground_relative_T[:3, 3])

    timestamp_at_prediction[match_idx] = imu_matches[match_idx][0]

    # print imu_matches[match_idx]
    # print imu_q
    # print quat.rotmat2quat(imu_relative_T[:3, :3])
    # print cvm_q
    # print quat.rotmat2quat(cvm_relative_T[:3, :3])
    # print quat.rotmat2quat(ground_relative_T[:3, :3])
    
    # print rotationMatrixToEulerAngles(imu_relative_T)
    # print rotationMatrixToEulerAngles(cvm_relative_T)
    # print rotationMatrixToEulerAngles(ground_relative_T)
    # exit(0)

plt.subplot(231)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 0], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 0], label='Groundtruth')
plt.title('IMU Rotation X prediction')
plt.legend()

plt.subplot(232)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 1], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 1], label='Groundtruth')
plt.title('IMU Rotation Y prediction')
plt.legend()

plt.subplot(233)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 2], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 2], label='Groundtruth')
plt.title('IMU Rotation Z prediction')
plt.legend()

plt.subplot(234)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 0], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 0], label='Groundtruth')
plt.title('CVM Rotation X prediction')
plt.legend()

plt.subplot(235)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 1], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 1], label='Groundtruth')
plt.title('CVM Rotation Y prediction')
plt.legend()

plt.subplot(236)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 2], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 2], label='Groundtruth')
plt.title('CVM Rotation Z prediction')
plt.legend()

######################################### Translation #########################################

plt.figure()

plt.subplot(231)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 3], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 3], label='Groundtruth')
plt.title('IMU Translation X prediction')
plt.legend()

plt.subplot(232)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 4], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 4], label='Groundtruth')
plt.title('IMU Translation Y prediction')
plt.legend()

plt.subplot(233)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, imu_predictions[:, 5], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 5], label='Groundtruth')
plt.title('IMU Translation Z prediction')
plt.legend()

plt.subplot(234)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 3], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 3], label='Groundtruth')
plt.title('CVM Translation X prediction')
plt.legend()

plt.subplot(235)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 4], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 4], label='Groundtruth')
plt.title('CVM Translation Y prediction')
plt.legend()

plt.subplot(236)
plt.xlabel('Frame ID')
plt.ylabel('Prediction (degrees)')
plt.plot(timestamp_at_prediction, cvm_predictions[:, 5], label='Prediction')
plt.plot(timestamp_at_prediction, groundtruth_predictions[:, 5], label='Groundtruth')
plt.title('CVM Translation Z prediction')
plt.legend()


# plt.subplot(231)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(imu_errors.shape[0]), imu_errors[:, 0], label='IMU error')
# plt.title('Rotation X error')
# plt.legend()

# plt.subplot(234)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 0], label='CVM error')
# plt.title('Rotation X error')
# plt.legend()


# plt.subplot(232)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(imu_errors.shape[0]), imu_errors[:, 1], label='IMU error')
# plt.title('Rotation Y error')
# plt.legend()

# plt.subplot(235)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 1], label='CVM error')
# plt.title('Rotation Y error')
# plt.legend()

# plt.subplot(233)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(imu_errors.shape[0]), imu_errors[:, 2], label='IMU error')
# plt.title('Rotation Z error')
# plt.legend()

# plt.subplot(236)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degrees)')
# plt.plot(range(cvm_errors.shape[0]), cvm_errors[:, 2], label='CVM error')
# plt.title('Rotation Z error')
# plt.legend()

######################################### Translation #########################################

# plt.figure()

# plt.subplot(211)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degree)')
# plt.title('Translation error')
# plt.plot(imu_errors[:, 3], label='IMU error')
# plt.legend()

# plt.subplot(212)
# plt.xlabel('Frame ID')
# plt.ylabel('Error (degree)')
# plt.title('Translation error')
# plt.plot(cvm_errors[:, 3], label='CVM error')   
# plt.legend()

# save
groundtruth_relative_list = [
    [
        imu_matches[match_idx][0], 
        groundtruth_relative[imu_matches[match_idx][0]][0, 3], 
        groundtruth_relative[imu_matches[match_idx][0]][1, 3], 
        groundtruth_relative[imu_matches[match_idx][0]][2, 3]
    ] +
    list(
        # rotationMatrixToEulerAngles(groundtruth_relative[imu_matches[match_idx][0]][:3, :3])
        # quat.rotmat2quat(groundtruth_relative[imu_matches[match_idx][0]][:3, :3])[0]
        groundtruth_relative[imu_matches[match_idx][0]][:3, :3].reshape([9])

    )
    for match_idx in range(len(imu_matches))
]
groundtruth_relative_list.sort()
print groundtruth_relative_list[:5]
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
        # rotationMatrixToEulerAngles(imu_relative[imu_matches[match_idx][0]][:3, :3])
        # quat.rotmat2quat(groundtruth_relative[imu_matches[match_idx][0]][:3, :3])[0]
        imu_relative[imu_matches[match_idx][1]][:3, :3].reshape([9])
    )
    for match_idx in range(len(imu_matches))
]
imu_relative_list.sort()
imu_relative_list = np.asarray(imu_relative_list)
np.savetxt("imu_relative.csv", imu_relative_list, fmt="%.6f")

# cvm_relative_list = [
#     [
#         cvm_matches[match_idx][1], 
#         cvm_relative[cvm_matches[match_idx][1]][0, 3], 
#         cvm_relative[cvm_matches[match_idx][1]][1, 3], 
#         cvm_relative[cvm_matches[match_idx][1]][2, 3]
#     ] +
#     list(
#         rotationMatrixToEulerAngles(cvm_relative[cvm_matches[match_idx][0]][:3, :3])
#     )
#     for match_idx in range(len(cvm_matches))
# ]
# cvm_relative_list.sort()
# cvm_relative_list = np.asarray(cvm_relative_list)
# np.savetxt("cvm_relative.csv", cvm_relative_list, fmt="%.6f")
    
plt.show()