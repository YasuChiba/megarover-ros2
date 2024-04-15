#!/usr/bin/env python3
# coding=utf8

import threading
import time
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
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

# global_map = None
# initialized = False
# T_map_to_odom = np.eye(4)
# cur_odom = None
# cur_scan = None


def pose_to_mat(pose_msg: PoseWithCovarianceStamped):
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    return np.matmul(
        translation_matrix([position.x, position.y, position.z]),
        quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


def msg_to_array(pc_msg):
    pc_array = ros2_numpy.numpify(pc_msg)
    #pc = np.zeros([len(pc_array), 3])
    #pc[:, 0] = pc_array["x"]
    #pc[:, 1] = pc_array["y"]
    #pc[:, 2] = pc_array["z"]
    #return pc
    return pc_array["xyz"]


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(
        len(pc),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.float32),
        ],
    )
    data["x"] = pc[:, 0]
    data["y"] = pc[:, 1]
    data["z"] = pc[:, 2]
    if pc.shape[1] == 4:
        data["intensity"] = pc[:, 3]
    
    # convert data to PointCloud2 without ros2_numpy
    pc_msg = PointCloud2()
    pc_msg.header = header
    pc_msg.height = 1
    pc_msg.width = len(pc)
    pc_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pc_msg.is_bigendian = False
    pc_msg.point_step = 16
    pc_msg.row_step = 16 * len(pc)
    pc_msg.is_dense = True
    pc_msg.data = data.tobytes()
    publisher.publish(pc_msg)

    #msg = ros2_numpy.msgify(PointCloud2, data)
    #msg.header = header
   # publisher.publish(msg)


def voxel_down_sample(pcd: o3d.geometry.PointCloud, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")

        self.get_logger().info("LocalizationNode init")

        self.MAP_VOXEL_SIZE = 0.01
        self.SCAN_VOXEL_SIZE = 0.01
        self.initialized = False
        self.cur_scan: PointCloud2 = None
        self.cur_odom: Odometry = None
        self.T_map_to_odom = np.eye(4)
        self.global_map:o3d.geometry.PointCloud = None

        # The threshold of global localization,
        # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
        self.LOCALIZATION_TH = 0.7
        # FOV(rad), modify this according to your LiDAR type
        self.FOV = 1.57
        # The farthest distance(meters) within FOV
        self.FOV_FAR = 20
        # Global localization frequency (HZ)
        self.FREQ_LOCALIZATION = 0.1


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

        # wait until global map is received
        self.map_subscription = self.create_subscription(
            PointCloud2, "/map", self.cb_globalmap, qos_profile_sensor_data
        )

        #while rclpy.ok() and self.global_map is None:
        #    self.get_logger().info("waiting for global map")
        #    time.sleep(1)
        # unsubscribe from global map
        #self.map_subscription.destroy()

        #######
        #######

        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.cb_initial_pose,
            qos_profile_sensor_data,
        )

        #while rclpy.ok() and not self.initialized:
        #    self.get_logger().info("Waiting for initial pose...")
        #    time.sleep(1)
        #self.initial_pose_subscription.destroy()

        #self.get_logger().info("")
        #self.get_logger().info("initialize successfully!")

        
        # call global_localization every 1 second.
        self.localization_timer = self.create_timer(1/self.FREQ_LOCALIZATION, lambda: self.global_localization(self.T_map_to_odom))


    def global_localization(self, pose_estimation):
        self.get_logger().info("Constant global localization......")
        if self.global_map is None:
            self.get_logger().info("waiting for global map")
            return False
        
        if not self.initialized:
            self.get_logger().info("waiting for initial pose")
            return False
        
        self._global_localization(pose_estimation)
        
    def initial_global_localization(self, initial_pose):
        self.get_logger().info("Initial global localization......")
        return self._global_localization(initial_pose)
    

    def _global_localization(self, pose_estimation):
        self.get_logger().info("Global localization by scan-to-map matching......")

        if pose_estimation is None:
            #pose_estimation = self.T_map_to_odom
            self.get_logger().error("pose_estimation is None")
            raise ValueError("pose_estimation is None")

        scan_tobe_mapped = copy.copy(self.cur_scan)
        tic = time.time()

        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)

        # 粗配准
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
    
        # 精配准
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                        scale=1)
        toc = time.time()
        self.get_logger().info('Time: {}'.format(toc - tic))
        self.get_logger().info('')

        if fitness > self.LOCALIZATION_TH:
            # T_map_to_odom = np.matmul(transformation, pose_estimation)
            self.T_map_to_odom = transformation

            # 发布map_to_odom
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
            #map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            map_to_odom.header.stamp = self.cur_odom.header.stamp
            map_to_odom.header.frame_id = 'map'
            map_to_odom.child_frame_id = 'odom'
            self.pub_map_to_odom.publish(map_to_odom)
            return True
        else:
            self.get_logger().warn('Not match!!!!')
            self.get_logger().warn('{}'.format(transformation))
            self.get_logger().warn('fitness score:{}'.format(fitness))
            return False

        

    def cb_globalmap(self, pc_msg: PointCloud2):
        if self.global_map is not None:
            return

        self.global_map = o3d.geometry.PointCloud()
        self.global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
        self.global_map = voxel_down_sample(self.global_map, self.MAP_VOXEL_SIZE)
        self.get_logger().info("Global map received")

    def cb_initial_pose(self, pose_msg: PoseWithCovarianceStamped):
        if self.initialized:
            return
        
        self.initial_pose = pose_to_mat(pose_msg)
        if self.cur_scan:
            self.initialized = self.initial_global_localization(self.initial_pose)
        else:
            self.get_logger().warn("First scan not received!!!!!")

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg

    def cb_save_cur_scan(self, pc_msg: PointCloud2):
        #self.get_logger().info("Received scan")
        # log properties of pc_msg.header
        #self.get_logger().info("header: {}".format(pc_msg.header))

        #self.get_logger().info("height: {}".format(pc_msg.height))
        #self.get_logger().info("width: {}".format(pc_msg.width))

        pc_msg.header.frame_id = "odom"
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)

        pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
        pc = msg_to_array(pc_msg)

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

    def crop_global_map_in_FOV(self, pose_estimation: PoseWithCovarianceStamped):
        # 当前scan原点的位姿
        T_odom_to_base_link = pose_to_mat(self.cur_odom)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = inverse_se3(T_map_to_base_link)


        # log T_odom_to_base_link
        #self.get_logger().info("T_odom_to_base_link: {}".format(T_odom_to_base_link))

        # 把地图转换到lidar系下
        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 将视角内的地图点提取出来
        if self.FOV > 3.14:
            # 环状lidar 仅过滤距离
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0) &
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布fov内点云
        header = self.cur_odom.header
        header.frame_id = 'map'
        publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV


    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        #result_icp = o3d.registration.registration_icp(
        #    voxel_down_sample(pc_scan, self.SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, self.MAP_VOXEL_SIZE * scale),
        #    1.0 * scale, initial,
        #    o3d.registration.TransformationEstimationPointToPoint(),
        #    o3d.registration.ICPConvergenceCriteria(max_iteration=20)
        #)

        #return result_icp.transformation, result_icp.fitness
    
        result_icp = o3d.pipelines.registration.registration_icp(
            voxel_down_sample(pc_scan, self.SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, self.MAP_VOXEL_SIZE * scale),
            1.0 * scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )

        return result_icp.transformation, result_icp.fitness

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
