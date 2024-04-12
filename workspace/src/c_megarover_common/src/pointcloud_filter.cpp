#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "c_megarover_common/pointcloud_filter.hpp"
#include "c_megarover_common/pointcloud_types.hpp"

PointCloudFilter::PointCloudFilter(
    const rclcpp::NodeOptions &options) : PointCloudFilter("", options)
{
}

PointCloudFilter::PointCloudFilter(
    const std::string &name_space,
    const rclcpp::NodeOptions &options) : Node("pointcloud_filter_node", name_space, options)
{
  pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_cloud", 10, std::bind(&PointCloudFilter::pcl_callback, this, std::placeholders::_1));
  pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("out_cloud", 10);
}

// filter point cloud
void PointCloudFilter::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{

  pcl::PointCloud<LivoxPointXyzitlt>::Ptr cloud(new pcl::PointCloud<LivoxPointXyzitlt>);
  pcl::PointCloud<LivoxPointXyzitlt>::Ptr cloud_filtered(new pcl::PointCloud<LivoxPointXyzitlt>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // // PassThrough Filter
  pcl::PassThrough<LivoxPointXyzitlt> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x"); // x axis
  // extract point cloud between 1.0 and 3.0 m
  pass.setFilterLimits(-0.6, 100.0);
  // pass.setFilterLimitsNegative (true);   // extract range reverse
  pass.filter(*cloud_filtered);

  // // Approximate Voxel Grid
  // pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avg;
  // avg.setInputCloud(cloud);
  // avg.setLeafSize(0.2f, 0.2f, 0.2f);
  // // avg.setDownsampleAllData(true);
  // avg.filter(*cloud_filtered);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<LivoxPointXyzitlt> voxelGrid;
  voxelGrid.setInputCloud(cloud_filtered);
  double leaf_size_ = 0.1;
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(*cloud_filtered);

  // // Voxel Grid: pattern 2
  // pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
  // pcl::toPCLPointCloud2(*cloud, *cloud_blob);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  // vg.setInputCloud(cloud_blob);
  // leaf_size_ = 0.1;
  // vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // vg.filter(*cloud_filtered_blob);
  // pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  // // Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<LivoxPointXyzitlt> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.setNegative(false);
  sor.filter(*cloud_filtered);

  // // Radius Outlier Removal
  pcl::RadiusOutlierRemoval<LivoxPointXyzitlt> outrem;
  outrem.setInputCloud(cloud_filtered);
  outrem.setRadiusSearch(0.1);
  outrem.setMinNeighborsInRadius(2);
  outrem.setKeepOrganized(false);
  outrem.filter(*cloud_filtered);

  // // Conditional Removal
  // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, 0.0)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 3.0)));
  // pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud);
  // // condrem.setKeepOrganized(true);
  // condrem.filter(*cloud_filtered);
  // // vector<int> Idx;
  // // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, Idx);

  // convert type of cloud_filtered from LivoxPointXyzitlt to LivoxPointXyzrtlt. The field "intensity" should be converted to "reflectivity"
  // this is required because of FAST_LIO. FAST_LIO only accepts LivoxPointXyzrtlt type
  pcl::PointCloud<LivoxPointXyzrtlt>::Ptr cloud_filtered_rt(new pcl::PointCloud<LivoxPointXyzrtlt>);
  for (size_t i = 0; i < cloud_filtered->points.size(); i++)
  {
    LivoxPointXyzrtlt point_rt;
    point_rt.x = cloud_filtered->points[i].x;
    point_rt.y = cloud_filtered->points[i].y;
    point_rt.z = cloud_filtered->points[i].z;
    point_rt.intensity = cloud_filtered->points[i].intensity;
    point_rt.reflectivity = cloud_filtered->points[i].intensity;
    point_rt.tag = cloud_filtered->points[i].tag;
    point_rt.line = cloud_filtered->points[i].line;
    point_rt.timestamp = cloud_filtered->points[i].timestamp;
    cloud_filtered_rt->push_back(point_rt);
  }

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered_rt, sensor_msg);
  sensor_msg.header = cloud_msg->header;
  pcl_publisher_->publish(sensor_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudFilter)
