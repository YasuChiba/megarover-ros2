import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped,PoseStamped, Point, Quaternion
import tf_transformations
import sensor_msgs_py.point_cloud2 as pc2
import ros2_numpy as rnp
from .library.color_print import COLOR_PRINT
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import threading
import time
import signal
import tf2_ros
import tf2_geometry_msgs  # Don't forget to import this package for do_transform_pose

class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.initialize_variables()
        self.declare_params()
        self.get_params()
        self.set_pubsub()
        self.color_print.print_in_purple("ICP node started!")
        self.color_print.print_in_blue(self.global_map)
        timer_period = 1.0 / 10  # seconds (10 Hz)
        self.pub_map()
        signal.signal(signal.SIGINT, self.sigint_handler)
        wait_for_initpose_thread = threading.Thread(target=self.wait_for_initpose)
        wait_for_initpose_thread.start()
        self.localization_timer = self.create_timer(1/self.FREQ_LOCALIZATION, self.localization)

    def initialize_variables(self):
        self.T_map_to_odom = np.eye(4)
        self.keyfram_timstamp = None
        self.finished = False
        self.keyframe_pcd_ = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.color_print = COLOR_PRINT(self)
        self.current_odom = None
        self.location_initialized = False
        self.br = TransformBroadcaster(self)
        self.map_header = Header()
        self.map_header.frame_id = "map"



    def declare_params(self):
        self.declare_parameter('map/map_location', '/')
        self.declare_parameter('map/map_name', 'test')
        self.declare_parameter("icp/map_voxel_size",0.5)
        self.declare_parameter("icp/scan_voxel_size",0.5)
        self.declare_parameter("icp/freq_localization",0.5)
        self.declare_parameter("icp/localization_threshold",0.95)
        self.declare_parameter("icp/field_of_view",3.14)
        self.declare_parameter("icp/field_of_view_far",100)
        self.declare_parameter('icp/initialpose_iteration', 20)
        self.declare_parameter('icp/executing_iteration', 10)

    def get_params(self):
        self.MAP_VOXEL_SIZE =             self.get_parameter("icp/map_voxel_size").get_parameter_value().double_value
        self.SCAN_VOXEL_SIZE =            self.get_parameter("icp/scan_voxel_size").get_parameter_value().double_value
        self.FREQ_LOCALIZATION =          self.get_parameter("icp/freq_localization").get_parameter_value().double_value
        self.LOCALIZATION_TH =            self.get_parameter("icp/localization_threshold").get_parameter_value().double_value
        self.FOV =                        self.get_parameter("icp/field_of_view").get_parameter_value().double_value
        self.FOV_FAR =                    self.get_parameter("icp/field_of_view_far").get_parameter_value().integer_value
        self.initialpose_iteration =      self.get_parameter('icp/initialpose_iteration').get_parameter_value().integer_value      
        self.executing_iteration =        self.get_parameter('icp/executing_iteration').get_parameter_value().integer_value
        map_location_ =                   self.get_parameter('map/map_location').get_parameter_value().string_value      
        map_name_ =                       self.get_parameter('map/map_name').get_parameter_value().string_value      
        map_file_path = map_location_ + "/"+ map_name_ + ".pcd"
        self.color_print.print_in_blue(map_file_path)
      
        # Load map
        original_map_pcd = o3d.io.read_point_cloud(map_file_path)  # Update the path
        self.global_map = self.voxel_down_sample(original_map_pcd, self.MAP_VOXEL_SIZE)
    
    def set_pubsub(self):
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_map_to_odom_odometry = self.create_publisher(Odometry,"/map_to_odom",  1)
        self.pub_submap = self.create_publisher(PointCloud2,'/submap',  1)
        self.map_publisher_ = self.create_publisher(PointCloud2, 'map', qos_profile)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)

        self.init_pose_received = False
        self.initial_pose = None
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'lio_odom',
            self.odom_callback,
            10

        )
        self.keyframe_sub_ = self.create_subscription(
            PointCloud2,
            'keyframe_scan',
            self.keyframe_callback,
            10)



    def sigint_handler(self, signal_received, frame):
        self.color_print.print_in_red("Received sigint")
        self.finished = True

    # ----------- related to Matrix transformation --------------- #

    def msg_to_array(self, pc_msg):
        pc_array = rnp.numpify(pc_msg)
        pc = np.zeros([len(pc_array), 3])
        pc[:, 0] = pc_array['x']
        pc[:, 1] = pc_array['y']
        pc[:, 2] = pc_array['z']
        return pc

    def pose_with_covariance_stamped_to_mat_no_orientation(self, pose_msg):
        """
        Convert a ROS PoseWithCovarianceStamped message to a 4x4 transformation matrix without orientation.

        Parameters:
        - pose_msg: A geometry_msgs/msg/PoseWithCovarianceStamped message.

        Returns:
        - A 4x4 numpy array representing the transformation matrix with no orientation.
        """
        # Access the nested Pose object
        pose = pose_msg.pose.pose

        # Extract the position (x, y, z)
        translation = [pose.position.x, pose.position.y, pose.position.z]

        # Create a 4x4 transformation matrix with no orientation
        # The matrix is initialized to an identity matrix
        transformation_matrix = np.eye(4)

        # Set the translation components
        transformation_matrix[0, 3] = translation[0]
        transformation_matrix[1, 3] = translation[1]
        transformation_matrix[2, 3] = translation[2]

        return transformation_matrix

    def pose_with_covariance_stamped_to_mat(self, pose_msg):
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
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Use tf_transformations to convert translation and quaternion to a transformation matrix
        translation_matrix = tf_transformations.translation_matrix(translation)
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)

        # Combine the translation and rotation into a single transformation matrix
        transformation_matrix = np.matmul(translation_matrix, rotation_matrix)

        return transformation_matrix

    def pose_to_mat(self,pose_msg):
        # Assuming pose_msg is a geometry_msgs/Pose
        translation = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quaternion = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]

        translation_matrix = tf_transformations.translation_matrix(translation)
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)

        return np.matmul(translation_matrix, rotation_matrix)


    def odom_to_mat(self, odom_msg):
        """
        Convert a ROS Odometry message to a 4x4 transformation matrix.

        Parameters:
        - odom_msg: A nav_msgs/msg/Odometry message.

        Returns:
        - A 4x4 numpy array representing the transformation matrix.
        """
        # Extract the position (x, y, z) from the odometry message
        translation = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]

        # Extract the orientation (x, y, z, w) from the odometry message
        quaternion = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]

        # Create translation and rotation matrices
        translation_matrix = tf_transformations.translation_matrix(translation)
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)
        # Combine the translation and rotation into a single transformation matrix
        transformation_matrix = np.matmul(translation_matrix, rotation_matrix)

        return transformation_matrix


    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    # ------------------------------------------ #



    #-------------- related to ICP --------------#

    def preprocess_point_cloud(self, point_cloud, voxel_size):
        """
        Downsamples and estimates normals for a given point cloud.
        
        Args:
            point_cloud (o3d.geometry.PointCloud): The input point cloud to preprocess.
            voxel_size (float): The voxel size for downsampling.
            
        Returns:
            o3d.geometry.PointCloud: The downsampled point cloud with normals.
        """
        # Voxel downsampling
        downsampled = point_cloud.voxel_down_sample(voxel_size)
        
        # Estimate normals. The search parameter can be adjusted.
        # Here, we're using a radius that's 2x the voxel size for the neighborhood.
        downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        
        return downsampled

         # Perform ICP and return transformation matrix and confidence (fitness)
    def registration_at_scale(self, pc_scan, pc_map,trans_init, scale):
        try:
            # Apply voxel downsampling and normal estimation to both point clouds
            pc_scan_down = self.preprocess_point_cloud(pc_scan, self.SCAN_VOXEL_SIZE*scale)
            pc_map_down = self.preprocess_point_cloud(pc_map, self.MAP_VOXEL_SIZE*scale)
            self.color_print.print_in_yellow(f"scan {pc_scan_down}")
            self.color_print.print_in_yellow(f"map {pc_map_down}")
            if self.location_initialized:
                max_iteration_ = self.initialpose_iteration
            else:
                max_iteration_ = self.executing_iteration
            icp_result = o3d.pipelines.registration.registration_generalized_icp(
                pc_scan_down, pc_map_down, 1.0*scale, trans_init,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration_))
            return icp_result.transformation, icp_result.fitness
        except Exception as e:
            self.color_print.print_in_red(e)
            return  np.eye(4), 0


    def crop_global_map_in_FOV(self, global_map, pose_estimation, cur_odom):
        # Convert the current odometry information to a transformation matrix
        T_odom_mat = self.odom_to_mat(cur_odom)
        # Calculate the transformation matrix from the map frame to the base_link frame
        # This is achieved by multiplying the pose estimation matrix (map to odom)
        # with the transformation from odom to base_link
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_mat)
        # Invert the transformation matrix to get from base_link to map frame
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # Convert the global map points to homogeneous coordinates (add a column of ones)
        global_map_in_map = np.array(global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        
        # Transform the global map points from the map frame to the base_link frame
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T


        if self.FOV >= 3.14:
            # Simplified condition for a 360-degree FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] ** 2 + global_map_in_base_link[:, 1] ** 2) < self.FOV_FAR ** 2
            )
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0) &
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )

        # Create a new point cloud for the map points within the FOV
        global_map_in_FOV = o3d.geometry.PointCloud()
        # Extract only the XYZ coordinates (discard the homogeneous coordinate) of points within the FOV
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))
        
        # Prepare a ROS PointCloud2 message to publish the cropped map
        # Here, we're downsampling by taking every 10th point for efficiency
        header = cur_odom.header
        header.frame_id = 'map'
        cloud_msg = pc2.create_cloud_xyz32(header, np.array(global_map_in_FOV.points)[::10])
        self.pub_submap.publish(cloud_msg)
        
        return global_map_in_FOV





    # ------------------------------------------ #

    # ----------------- pc modifiers ---------- #
    def voxel_down_sample(self, pcd, voxel_size):
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
        return pcd_down

    def convert_ros_pointcloud2_to_o3d(self, ros_point_cloud):
        # Convert ROS PointCloud2 message to numpy array using ros2_numpy
        np_points = rnp.numpify(ros_point_cloud)
        # Extract XYZ coordinates from the numpy array and create an Open3D point cloud
        #xyz = np_points[['x', 'y', 'z']].view((np.float32, 3))
        xyz = np.stack([np_points['x'], np_points['y'], np_points['z']], axis=-1)
        
        #xyz = np_points["xyz"]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def publish_point_cloud(self,publisher, header, points):
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        publisher.publish(cloud_msg)

    # ------------------------------------------ #

    # ------- callbacks ------------------- #
    def pub_map(self):
        self.map_header.stamp = self.get_clock().now().to_msg()  # Current time
        points = np.asarray(self.global_map.points)
        cloud_msg = pc2.create_cloud_xyz32(self.map_header, points)
        self.map_publisher_.publish(cloud_msg)

    def odom_callback(self,msg):
        self.current_odom = msg


    def initial_pose_callback(self, msg):
        self.initial_pose = self.pose_with_covariance_stamped_to_mat(msg)
        self.publish_map_odom(self.initial_pose)
        self.init_pose_received = True
        self.color_print.print_in_green('Initial pose received.')

    def keyframe_callback(self, msg):
        try:
            msg.header.frame_id = "map"
            self.keyfram_timstamp = msg.header.stamp
            self.keyframe_pcd_ = self.convert_ros_pointcloud2_to_o3d(msg)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error('Error in keyframe_callback')
    # ------------------------------------------ #



    def global_localization(self, pose_estimation):
        if self.current_odom is not None and self.keyframe_pcd_ is not None:
            self.color_print.print_in_blue("Start ICP")
            self.global_map_in_FOV = self.crop_global_map_in_FOV(self.global_map, pose_estimation, self.current_odom)
            rough_transformation, rough_fitness = self.registration_at_scale(self.keyframe_pcd_, self.global_map_in_FOV , pose_estimation, 5)
            self.color_print.print_in_orange(f"rough_fitness: {rough_fitness}")
            transformation, fitness = self.registration_at_scale(self.keyframe_pcd_, self.global_map_in_FOV, rough_transformation, 1)
            

            if fitness > self.LOCALIZATION_TH:
                self.color_print.print_in_green('Matched!!!')
                self.color_print.print_in_pink(f"fitness: {fitness}")
                self.T_map_to_odom = transformation
                self.publish_map_odom(self.T_map_to_odom)
                return True
            else:
                self.color_print.print_in_yellow('Not match...')
                self.color_print.print_in_yellow('fitness score:{}'.format(fitness))
                return False
        self.color_print.print_in_yellow("Not received odom or keyframe yet")
        return False

    def publish_map_odom(self, transformation44):
        map_to_odom = Odometry()
        xyz = tf_transformations.translation_from_matrix(transformation44)
        quat = tf_transformations.quaternion_from_matrix(transformation44)


        # Fill the Odometry message
        map_to_odom.pose.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        map_to_odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Assuming 'now' is the current time. In ROS2, get the current time using the node's clock
        now = self.get_clock().now()
        map_to_odom.header.stamp = now.to_msg()

        map_to_odom.header.frame_id = 'map'

        # Publish the map_to_odom transformation
        self.pub_map_to_odom_odometry.publish(map_to_odom)

    def localization(self):
        if self.location_initialized:
            self.color_print.print_in_yellow(f"Calculating the transform {self.T_map_to_odom}")
            self.global_localization(self.T_map_to_odom)

    def wait_for_initpose(self):
        while not self.finished and not self.location_initialized:
            if self.init_pose_received:
                # Add the point cloud to the visualizer
                self.location_initialized  = self.global_localization(self.initial_pose)
            else:
                self.color_print.print_in_orange("Init pose is not received")
            time.sleep(0.5)





def main(args=None):
    rclpy.init(args=args)
    icp_node = ICPNode()
    rclpy.spin(icp_node)
    icp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()