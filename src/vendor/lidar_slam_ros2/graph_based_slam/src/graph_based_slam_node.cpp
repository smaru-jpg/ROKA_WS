#include "graph_based_slam/graph_based_slam_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // Transient-local MapArray delivery lets the saver receive the latest
  // submap snapshot after startup.  These nodes run as separate processes,
  // so intra-process transport provides no benefit and is incompatible with
  // transient-local durability in ROS 2 Humble.
  options.use_intra_process_comms(false);
  rclcpp::spin(std::make_shared<graphslam::GraphBasedSlamComponent>(options));
  rclcpp::shutdown();
  return 0;
}
