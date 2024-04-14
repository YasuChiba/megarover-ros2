/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include <vector>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

void save_map_to_files(const nav_msgs::msg::OccupancyGrid &map, const std::string &pgm_file, const std::string &yaml_file);

// ref: https://github.com/ANYbotics/grid_map/blob/humble/grid_map_pcl/src/grid_map_pcl_loader_node.cpp
int main(int argc, char **argv)
{
  // parameters
  // - pcd_file_path
  // - parameter_path (default: "c_megarover_common/config/pcd_to_occupancygrid_tool.yaml")
  // - map_frame (default: "map")

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");
  gm::setVerbosityLevelToDebugIfFlagSet(node);

  std::string parameter_path, pcd_file_path;
  node->declare_parameter("parameter_path",
                          ament_index_cpp::get_package_share_directory("c_megarover_common") + "/config/pcd_to_occupancygrid_tool.yaml");
  node->declare_parameter("pcd_file_path", "");
  node->get_parameter("parameter_path", parameter_path);
  node->get_parameter("pcd_file_path", pcd_file_path);

  grid_map::GridMapPclLoader gridMapPclLoader(node->get_logger());
  gridMapPclLoader.loadParameters(parameter_path);
  gridMapPclLoader.loadCloudFromPcdFile(pcd_file_path);

  gm::processPointcloud(&gridMapPclLoader, node);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(node));

  //gm::saveGridMap(gridMap, node, gm::getMapRosbagTopic(node));

  // print girdMap info
  std::cout << "Grid map info: " << std::endl;
  std::cout << " - Frame id: " << gridMap.getFrameId() << std::endl;
  std::cout << " - Length: " << gridMap.getLength() << std::endl;
  std::cout << " - Position: " << gridMap.getPosition() << std::endl;
  std::cout << " - Resolution: " << gridMap.getResolution() << std::endl;
  std::cout << " - Size: " << gridMap.getSize() << std::endl;
  std::cout << " - Timestamp: " << gridMap.getTimestamp() << std::endl;
  std::cout << "Layers: " << std::endl;
  for (const auto &layer : gridMap.getLayers())
  {
    std::cout << " - " << layer << std::endl;
  }

  // convert gridMap to OccupancyGrid using  GridMapRosConverter::toOccupancyGrid
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", 0.0, 1.0, occupancyGrid);
  save_map_to_files(occupancyGrid, "map.pgm", "map.yaml");


  rclcpp::shutdown();
  return EXIT_SUCCESS;
}



void save_map_to_files(const nav_msgs::msg::OccupancyGrid &map, const std::string &pgm_file, const std::string &yaml_file)
{
  // Save PGM file
  std::ofstream pgm(pgm_file, std::ios::out | std::ios::binary);
  pgm << "P5\n"
      << map.info.width << " " << map.info.height << "\n255\n";
  for (auto value : map.data)
  {
    unsigned char pixel_value = value == -1 ? 205 : (value == 0 ? 254 : 0);
    pgm.write(reinterpret_cast<const char *>(&pixel_value), sizeof(pixel_value));
  }
  pgm.close();

  // Save YAML file
  std::ofstream yaml(yaml_file);
  yaml << "image: " << pgm_file << "\n";
  yaml << "resolution: " << map.info.resolution << "\n";
  yaml << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", " << map.info.origin.orientation.z << "]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.196\n";
  yaml.close();

  printf("Map saved to %s and %s \n", pgm_file.c_str(), yaml_file.c_str());
}
