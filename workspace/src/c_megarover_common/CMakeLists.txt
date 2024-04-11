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

include_directories(
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_library(minimal_comp_node SHARED
  src/minimal_comp_node1.cpp
)

add_library(pointcloud_filter SHARED
  src/pointcloud_filter/pointcloud_filter.cpp
)

target_compile_definitions(minimal_comp_node
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)

ament_target_dependencies(minimal_comp_node
  rclcpp
  rclcpp_components
)

ament_target_dependencies(pointcloud_filter
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)

#target_compile_options(minimal_comp_node PUBLIC -Wall)


target_include_directories(minimal_comp_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(pointcloud_filter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

rclcpp_components_register_nodes(minimal_comp_node "minimal_comp::MinimalCompNode1")

rclcpp_components_register_nodes(pointcloud_filter "pointcloud_filter::FilterNode")

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS
  minimal_comp_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  pointcloud_filter
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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
