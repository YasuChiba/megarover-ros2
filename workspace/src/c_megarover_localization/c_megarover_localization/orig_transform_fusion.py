#!/usr/bin/env python3
# coding=utf8

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import copy
import tf_transformations
from geometry_msgs.msg import TransformStamped

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__("transform_fusion")
        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.br = TransformBroadcaster(self)
        self.declare_parameter("freq_pub_localization", 50)
        self.freq_pub_localization = (
            self.get_parameter("freq_pub_localization")
            .get_parameter_value()
            .integer_value
        )

        self.sub_odometry = self.create_subscription(
            Odometry, "/Odometry", self.cb_save_cur_odom, 1
        )
        self.sub_map_to_odom = self.create_subscription(
            Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1
        )

        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        # Launching the transformation fusion
        # threading.Thread(target=self.transform_fusion).start()
        self.localization_timer = self.create_timer(
            1 / self.freq_pub_localization, self.transform_fusion
        )

    def pose_to_mat(self, pose_msg):
        return np.matmul(
            tf_transformations.translation_matrix(
                [
                    pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    pose_msg.pose.pose.position.z,
                ]
            ),
            tf_transformations.quaternion_matrix(
                [
                    pose_msg.pose.pose.orientation.x,
                    pose_msg.pose.pose.orientation.y,
                    pose_msg.pose.pose.orientation.z,
                    pose_msg.pose.pose.orientation.w,
                ]
            ),
        )

    def transform_fusion(self):
        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        translation = tf_transformations.translation_from_matrix(T_map_to_odom)
        quaternion = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.br.sendTransform(t)

        #self.br.sendTransform(
        #    tf_transformations.translation_from_matrix(T_map_to_odom),
        #    tf_transformations.quaternion_from_matrix(T_map_to_odom),
        #    self.get_clock().now().to_msg(),
        #    "odom",
        #    "map",
        #)

        if cur_odom is not None:
            localization = Odometry()
            T_odom_to_base_link = self.pose_to_mat(cur_odom)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf_transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link)
            
            # xyz to Point, quat to Quaternion
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

            localization.pose.pose = pose
            localization.twist = cur_odom.twist
            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = "odom"
            localization.child_frame_id = "base_footprint"
            self.pub_localization.publish(localization)

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
