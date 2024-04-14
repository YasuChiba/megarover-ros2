#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/map_mode.hpp"

void filters(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_filters,
             const double &thre_low, const double &thre_high);
void pointcloud_to_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        nav_msgs::msg::OccupancyGrid &msg);

std::string pcd_file_path, output_file_path;
double thre_z_min = 0.0;
double thre_z_max = 1.0;
double map_resolution = 0.02;
double thre_radius = 0.05;
int thres_point_count = 10;

// ref: https://github.com/Hinson-A/pcd2pgm_package/blob/develop/pcd2pgm/src/pcd2pgm.cpp
int main(int argc, char **argv)
{
  // parameters
  // - pcd_file_path
  // - output_file_path (file path to save the map, default: "map")
  // - map_resolution (default: 0.02)
  // - thre_z_min (使用される点の最小高さ, default: 0.0)
  // - thre_z_max (使用される点の最大高さ,default: 1.0)
  // - thre_radius (フィルタパラメーター, default: 0.05)
  // - thres_point_count (フィルタパラメーター, default: 10)

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");

  node->declare_parameter("pcd_file_path", "");
  node->declare_parameter("output_file_path", "map");
  node->declare_parameter("map_resolution", 0.02);
  node->declare_parameter("thre_z_min", 0.0);
  node->declare_parameter("thre_z_max", 1.0);
  node->declare_parameter("thre_radius", 0.05);
  node->declare_parameter("thres_point_count", 10);
  

  node->get_parameter("pcd_file_path", pcd_file_path);
  node->get_parameter("output_file_path", output_file_path);
  node->get_parameter("map_resolution", map_resolution);
  node->get_parameter("thre_z_min", thre_z_min);
  node->get_parameter("thre_z_max", thre_z_max);
  node->get_parameter("thre_radius", thre_radius);
  node->get_parameter("thres_point_count", thres_point_count);


  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_filters(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *pcd_cloud) == -1)
  {
    RCLCPP_ERROR(node->get_logger(), "failed to open PCD file");
    return (-1);
  }

  std::cout << "num of points: " << pcd_cloud->points.size() << std::endl;
  filters(pcd_cloud, cloud_after_filters, thre_z_min, thre_z_max);

  nav_msgs::msg::OccupancyGrid map_topic_msg;
  pointcloud_to_grid(cloud_after_filters, map_topic_msg);

  //"{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
  nav2_map_server::SaveParameters save_parameters;
  save_parameters.map_file_name = output_file_path;
  save_parameters.image_format = "pgm";
  save_parameters.free_thresh = 0.25;
  save_parameters.occupied_thresh = 0.65;
  save_parameters.mode = nav2_map_server::MapMode::Trinary;

  nav2_map_server::saveMapToFile(map_topic_msg, save_parameters);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

void filters(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_filters,
             const double &thre_low, const double &thre_high)
{
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setFilterLimitsNegative(false);
  passthrough.filter(*cloud_after_filters);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  radiusoutlier.setInputCloud(cloud_after_filters);
  radiusoutlier.setRadiusSearch(thre_radius);
  radiusoutlier.setMinNeighborsInRadius(thres_point_count);
  radiusoutlier.filter(*cloud_after_filters);
}

void pointcloud_to_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        nav_msgs::msg::OccupancyGrid &msg)
{
  // msg.header.seq = 0;
  // msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  // msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  //? ? ??
  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty())
  {
    std::cout << "pcd is empty" << std::endl;
    return;
  }

  for (int i = 0; i < cloud->points.size() - 1; i++)
  {
    if (i == 0)
    {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // origin的确定
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  // 设置栅格地图大小
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  // 实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  // ROS_INFO("data size = %d\n", msg.data.size());
  std::cout << "data size = " << msg.data.size() << std::endl;

  for (int iter = 0; iter < cloud->points.size(); iter++)
  {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // 栅格地图的占有概率[0,100]，这里设置为占据
    msg.data[i + j * msg.info.width] = 100;
    //    msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z *
    //    k_line + b_line)) % 255;
  }
}
