#include "scanmatcher/scanmatcher_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // MapArray is transient-local so the graph process can always receive the
  // latest keyframes.  The publisher and graph are separate processes.
  options.use_intra_process_comms(false);
  rclcpp::spin(std::make_shared<graphslam::ScanMatcherComponent>(options));
  rclcpp::shutdown();
  return 0;
}
