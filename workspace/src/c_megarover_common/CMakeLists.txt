cmake_minimum_required(VERSION 3.8)
project(c_megarover_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_filter SHARED
  src/pointcloud_filter.cpp)
rclcpp_components_register_node(pointcloud_filter 
  PLUGIN "PointCloudFilter"
  EXECUTABLE pointcloud_filter_node)

ament_auto_add_library(livox_to_pointcloud2 SHARED
  src/livox_to_pointcloud2.cpp)
rclcpp_components_register_node(livox_to_pointcloud2 
  PLUGIN "LivoxToPointCloud2"
  EXECUTABLE livox_to_pointcloud2_node)


ament_auto_add_library(pointcloud_to_pcd SHARED
src/pointcloud_to_pcd.cpp)
rclcpp_components_register_node(pointcloud_to_pcd 
PLUGIN "PointCloudToPCD"
EXECUTABLE pointcloud_to_pcd_node)


ament_auto_add_library(pcd_to_pointcloud SHARED
src/pcd_to_pointcloud.cpp)
rclcpp_components_register_node(pcd_to_pointcloud 
PLUGIN "PCDPublisher"
EXECUTABLE pcd_to_pointcloud_node)


ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
