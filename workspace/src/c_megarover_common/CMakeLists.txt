cmake_minimum_required(VERSION 3.8)
project(c_megarover_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL REQUIRED)
find_package(livox_ros_driver2 REQUIRED)


include_directories(
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#====================

add_library(pointcloud_filter SHARED
  src/pointcloud_filter/pointcloud_filter.cpp
)
ament_target_dependencies(pointcloud_filter
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)
target_include_directories(pointcloud_filter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

rclcpp_components_register_nodes(pointcloud_filter "PointCloudFilter")


#=========================



add_library(livox_to_pointcloud2 SHARED
  src/livox_to_pointcloud2.cpp
)
ament_target_dependencies(livox_to_pointcloud2
  rclcpp
  rclcpp_components
  sensor_msgs
  livox_ros_driver2
)
target_include_directories(livox_to_pointcloud2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(livox_to_pointcloud2 "LivoxToPointCloud2")


#====================================================================

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  pointcloud_filter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  livox_to_pointcloud2
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
