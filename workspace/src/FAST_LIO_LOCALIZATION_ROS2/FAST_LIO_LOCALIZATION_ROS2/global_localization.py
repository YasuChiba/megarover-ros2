#!/usr/bin/env python2
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import open3d as o3d
import ros2_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf2_ros
import rclpy
from rclpy.node import Node


# define class to do global localization
class GlobalLocalization:
    def __init__(self):
        self.global_map = None
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None

    def pose_to_mat(self, pose_msg):
        return np.matmul(
            tf2_ros.transform_listener.xyz_to_mat44(pose_msg.pose.pose.position),
            tf2_ros.transform_listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
        )

    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        pc = np.zeros([len(pc_array), 3])
        pc[:, 0] = pc_array['x']
        pc[:, 1] = pc_array['y']
        pc[:, 2] = pc_array['z']
        return pc

    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        result_icp = o3d.registration.registration_icp(
            voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
            1.0 * scale, initial,
            o3d.registration.TransformationEstimationPointToPoint(),
            o3d.registration.ICPConvergenceCriteria(max_iteration=20)
        )

        return result_icp.transformation, result_icp.fitness

    #def inverse_se3(self