#!/usr/bin/env python

import sys
import numpy as np
import associate
import matplotlib.pyplot as plt

if len(sys.argv) < 3:
    print 'Usage: python evalute_imu_count <imu_count_file> <ground_truth>'
    exit(0)

imu_file = sys.argv[1]
groundtruth_file = sys.argv[2]

imu_count_dict = associate.read_file_list(imu_file)

imu_count_tuple_list = [tuple([i] + imu_count_dict[i]) for i in imu_count_dict]
imu_count_tuple_list.sort()

print imu_count_tuple_list[:5]

count = [0 for i in range(len(imu_count_dict))]
for idx in range(len(imu_count_tuple_list)):
	count[idx] = round(imu_count_tuple_list[idx][1] / (imu_count_tuple_list[idx][0] - imu_count_tuple_list[idx][2]) * 0.05, 3)

plt.plot(sorted(imu_count_dict.keys()), count)
plt.show()


