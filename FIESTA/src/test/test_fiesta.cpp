#include "Fiesta.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "FIESTA");
  ros::NodeHandle node("~");

  // 障碍物信息
// sensor_msgs::PointCloud2::ConstPtr
// sensor_msgs::Image::ConstPtr

// 位姿估计信息
// geometry_msgs::PoseStamped::ConstPtr
// nav_msgs::Odometry::ConstPtr
// geometry_msgs::TransformStamped::ConstPtr

  // fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> esdf_map(node);
  fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::PoseStamped::ConstPtr> esdf_map(node);
  ros::spin();
  return 0;
}
