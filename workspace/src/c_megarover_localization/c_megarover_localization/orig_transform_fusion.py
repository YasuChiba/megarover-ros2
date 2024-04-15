#!/usr/bin/env python3
# coding=utf8

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from nav_msgs.msg import Odometry
import copy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor


def rotation_matrix_to_quaternion(rotation_matrix):
    quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)
    return quaternion


def pose_to_mat(pose_msg):
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    return np.matmul(
        tf_transformations.translation_matrix((position.x, position.y, position.z)),
        tf_transformations.quaternion_matrix(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        ),
    )


class TransformFusionNode(Node):

    def declare_params(self):
        self.declare_parameter(
            "freq_tf_broadcast",
            20.0,
            ParameterDescriptor(description="TF broadcast frequency (HZ)"),
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
        self.FREQ_TF_BROADCAST = (
            self.get_parameter("freq_tf_broadcast").get_parameter_value().double_value
        )
        self.MAP_FRAME = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self.ODOM_FRAME = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )

    def __init__(self):
        super().__init__("transform_fusion")

        self.declare_params()
        self.get_params()

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.br = TransformBroadcaster(self)
        self.declare_parameter("freq_pub_localization", 50)

        self.sub_odometry = self.create_subscription(
            Odometry, "/Odometry", self.cb_save_cur_odom, 1
        )
        self.sub_map_to_odom = self.create_subscription(
            Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1
        )

        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        self.localization_timer = self.create_timer(
            1 / self.FREQ_TF_BROADCAST, self.transform_fusion
        )

    def transform_fusion(self):
        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if self.cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(self.cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        if cur_odom is not None:
            self.broadcast_transform(T_map_to_odom)

    def broadcast_transform(self, transformation_matrix):
        t = TransformStamped()

        # Fill header information
        t.header.stamp = self.cur_odom_to_baselink.header.stamp
        t.header.frame_id = self.MAP_FRAME
        t.child_frame_id = self.ODOM_FRAME

        # Extract translation from the transformation matrix
        t.transform.translation.x = transformation_matrix[0, 3]
        t.transform.translation.y = transformation_matrix[1, 3]
        t.transform.translation.z = transformation_matrix[2, 3]

        # Convert the rotation matrix to a quaternion
        q = rotation_matrix_to_quaternion(transformation_matrix)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transformation
        self.br.sendTransform(t)

    def cb_save_cur_odom(self, odom_msg):
        self.cur_odom_to_baselink = odom_msg

    def cb_save_map_to_odom(self, odom_msg):
        self.cur_map_to_odom = odom_msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
