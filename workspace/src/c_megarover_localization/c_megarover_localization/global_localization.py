#!/usr/bin/env python3
# coding=utf8

import os
import time
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import tf_transformations
import ros2_numpy
import copy
from rcl_interfaces.msg import ParameterDescriptor


def pose_to_mat(pose_msg: PoseWithCovarianceStamped):
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    return np.matmul(
        translation_matrix([position.x, position.y, position.z]),
        quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


def msg_to_array(pc_msg):
    pc_array = ros2_numpy.numpify(pc_msg)
    return pc_array["xyz"]


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def pose_with_covariance_stamped_to_mat(pose_msg: PoseWithCovarianceStamped):
    """
    Convert a ROS PoseWithCovarianceStamped message to a 4x4 transformation matrix.

    Parameters:
    - pose_msg: A geometry_msgs/msg/PoseWithCovarianceStamped message.

    Returns:
    - A 4x4 numpy array representing the transformation matrix.
    """

    # Access the nested Pose object
    pose = pose_msg.pose.pose

    # Extract the position (x, y, z)
    translation = [pose.position.x, pose.position.y, pose.position.z]

    # Extract the orientation (x, y, z, w)
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]

    # Use tf_transformations to convert translation and quaternion to a transformation matrix
    translation_matrix = tf_transformations.translation_matrix(translation)
    rotation_matrix = tf_transformations.quaternion_matrix(quaternion)

    # Combine the translation and rotation into a single transformation matrix
    transformation_matrix = np.matmul(translation_matrix, rotation_matrix)

    return transformation_matrix


def odom_to_mat(odom_msg: Odometry):
    """
    Convert a ROS Odometry message to a 4x4 transformation matrix.

    Parameters:
    - odom_msg: A nav_msgs/msg/Odometry message.

    Returns:
    - A 4x4 numpy array representing the transformation matrix.
    """
    # Extract the position (x, y, z) from the odometry message
    translation = [
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    ]

    # Extract the orientation (x, y, z, w) from the odometry message
    quaternion = [
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w,
    ]

    # Create translation and rotation matrices
    translation_matrix = tf_transformations.translation_matrix(translation)
    rotation_matrix = tf_transformations.quaternion_matrix(quaternion)
    # Combine the translation and rotation into a single transformation matrix
    transformation_matrix = np.matmul(translation_matrix, rotation_matrix)

    return transformation_matrix


def voxel_down_sample(pcd: o3d.geometry.PointCloud, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


class LocalizationNode(Node):

    def declare_params(self):
        self.declare_parameter("map_voxel_size", 0.01)
        self.declare_parameter("scan_voxel_size", 0.01)
        self.declare_parameter(
            "localization_th",
            0.95,
            ParameterDescriptor(
                description="The threshold of global localization."
                + "only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken"
            ),
        )
        self.declare_parameter(
            "fov",
            3.14,
            ParameterDescriptor(
                description="FOV(rad), modify this according to your LiDAR type"
            ),
        )
        self.declare_parameter(
            "fov_far",
            10.0,
            ParameterDescriptor(description="The farthest distance(meters) within FOV"),
        )
        self.declare_parameter(
            "freq_localization",
            0.5,
            ParameterDescriptor(description="Global localization frequency (HZ)"),
        )
        self.declare_parameter(
            "map_file_path",
            "/home/user/workspace/pcd/sendagi.pcd",
            ParameterDescriptor(description="The path of the map file (pcd format)"),
        )
        self.declare_parameter(
            "map_frame",
            "map",
            ParameterDescriptor(description="default: map"),
        )
        self.declare_parameter(
            "odom_frame",
            "odom",
            ParameterDescriptor(description="default: odom"),
        )

    def get_params(self):
        self.MAP_VOXEL_SIZE = (
            self.get_parameter("map_voxel_size").get_parameter_value().double_value
        )
        self.SCAN_VOXEL_SIZE = (
            self.get_parameter("scan_voxel_size").get_parameter_value().double_value
        )
        self.LOCALIZATION_TH = (
            self.get_parameter("localization_th").get_parameter_value().double_value
        )
        self.FOV = self.get_parameter("fov").get_parameter_value().double_value
        self.FOV_FAR = self.get_parameter("fov_far").get_parameter_value().double_value
        self.FREQ_LOCALIZATION = (
            self.get_parameter("freq_localization").get_parameter_value().double_value
        )
        self.MAP_FILE_PATH = (
            self.get_parameter("map_file_path").get_parameter_value().string_value
        )
        self.MAP_FRAME = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self.ODOM_FRAME = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )

    def __init__(self):
        super().__init__("fast_lio_localization")
        self.get_logger().info("LocalizationNode init")

        self.declare_params()
        self.get_params()

        self.initialized = False
        self.cur_scan: PointCloud2 = None
        self.cur_odom: Odometry = None
        self.T_map_to_odom = np.eye(4)
        self.global_map: o3d.geometry.PointCloud = None

        # Load map
        # check the exsisitence of the map file
        if not os.path.exists(self.MAP_FILE_PATH):
            self.get_logger().error("Map file not found: {}".format(self.MAP_FILE_PATH))
            raise FileNotFoundError("Map file not found: {}".format(self.MAP_FILE_PATH))
        original_map_pcd = o3d.io.read_point_cloud(self.MAP_FILE_PATH)
        self.global_map = voxel_down_sample(original_map_pcd, self.MAP_VOXEL_SIZE)

        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 1)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 1)

        self.subscription = self.create_subscription(
            PointCloud2,
            "/cloud_registered",
            self.cb_save_cur_scan,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "/Odometry", self.cb_save_cur_odom, qos_profile_sensor_data
        )

        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.cb_initial_pose,
            qos_profile_sensor_data,
        )

        # call global_localization every 1/self.FREQ_LOCALIZATION seconds
        self.localization_timer = self.create_timer(
            1 / self.FREQ_LOCALIZATION, self.global_localization
        )

    def global_localization(self):
        if not self.initialized:
            self.get_logger().info("waiting for initial pose")
            return False

        self._global_localization(self.T_map_to_odom)

    def initial_global_localization(self, initial_pose):
        return self._global_localization(initial_pose)

    def _global_localization(self, pose_estimation):
        self.get_logger().debug("Global localization by scan-to-map matching......")

        if pose_estimation is None:
            self.get_logger().error("pose_estimation is None")
            raise ValueError("pose_estimation is None")

        scan_tobe_mapped = copy.copy(self.cur_scan)
        tic = time.time()

        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)

        transformation, _ = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
        )

        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1
        )
        toc = time.time()
        self.get_logger().debug("Time: {}".format(toc - tic))
        self.get_logger().debug("")

        if fitness > self.LOCALIZATION_TH:
            self.T_map_to_odom = transformation

            map_to_odom = Odometry()
            xyz = translation_from_matrix(self.T_map_to_odom)
            quat = quaternion_from_matrix(self.T_map_to_odom)

            position = Point()
            position.x = xyz[0]
            position.y = xyz[1]
            position.z = xyz[2]
            orientation = Quaternion()
            orientation.x = quat[0]
            orientation.y = quat[1]
            orientation.z = quat[2]
            orientation.w = quat[3]
            pose = Pose()
            pose.position = position
            pose.orientation = orientation

            map_to_odom.pose.pose = pose
            map_to_odom.header.stamp = self.cur_odom.header.stamp
            map_to_odom.header.frame_id = self.MAP_FRAME
            map_to_odom.child_frame_id = self.ODOM_FRAME
            self.pub_map_to_odom.publish(map_to_odom)
            return True
        else:
            self.get_logger().warn("Not match!!!!")
            self.get_logger().warn("{}".format(transformation))
            self.get_logger().warn("fitness score:{}".format(fitness))
            return False

    def cb_initial_pose(self, pose_msg: PoseWithCovarianceStamped):
        #if self.initialized:
        #    return

        self.initialized = False

        self.initial_pose = pose_with_covariance_stamped_to_mat(pose_msg)
        if self.cur_scan:
            self.initialized = self.initial_global_localization(self.initial_pose)
        else:
            self.get_logger().warn("First scan not received!!!!!")

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg

    def cb_save_cur_scan(self, pc_msg: PointCloud2):
        pc_msg.header.frame_id = self.ODOM_FRAME
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)

        pc_msg.fields = [
            pc_msg.fields[0],
            pc_msg.fields[1],
            pc_msg.fields[2],
            pc_msg.fields[4],
            pc_msg.fields[5],
            pc_msg.fields[6],
            pc_msg.fields[3],
            pc_msg.fields[7],
        ]
        pc = msg_to_array(pc_msg)

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

    def crop_global_map_in_FOV(self, pose_estimation):
        # Convert the current odometry information to a transformation matrix
        T_odom_mat = odom_to_mat(self.cur_odom)

        # Calculate the transformation matrix from the map frame to the base_link frame
        # This is achieved by multiplying the pose estimation matrix (map to odom)
        # with the transformation from odom to base_link
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_mat)
        # Invert the transformation matrix to get from base_link to map frame
        T_base_link_to_map = inverse_se3(T_map_to_base_link)

        # Convert the global map points to homogeneous coordinates (add a column of ones)
        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack(
            [global_map_in_map, np.ones(len(global_map_in_map))]
        )
        # Transform the global map points from the map frame to the base_link frame
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 将视角内的地图点提取出来
        if self.FOV >= 3.14:
            # Simplified condition for a 360-degree FOV
            indices = np.where(
                (
                    global_map_in_base_link[:, 0] ** 2
                    + global_map_in_base_link[:, 1] ** 2
                )
                < self.FOV_FAR**2
            )
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.FOV_FAR)
                & (
                    np.abs(
                        np.arctan2(
                            global_map_in_base_link[:, 1], global_map_in_base_link[:, 0]
                        )
                    )
                    < self.FOV / 2.0
                )
            )

        global_map_in_FOV = o3d.geometry.PointCloud()
        # Extract only the XYZ coordinates (discard the homogeneous coordinate) of points within the FOV
        global_map_in_FOV.points = o3d.utility.Vector3dVector(
            np.squeeze(global_map_in_map[indices, :3])
        )

        # 发布fov内点云
        header = self.cur_odom.header
        header.frame_id = self.MAP_FRAME
        cloud_msg = pc2.create_cloud_xyz32(
            header, np.array(global_map_in_FOV.points)[::10]
        )
        # publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])
        self.pub_submap.publish(cloud_msg)

        return global_map_in_FOV

    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
            voxel_down_sample(pc_scan, self.SCAN_VOXEL_SIZE * scale),
            voxel_down_sample(pc_map, self.MAP_VOXEL_SIZE * scale),
            1.0 * scale,
            initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )

        return result_icp.transformation, result_icp.fitness


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
