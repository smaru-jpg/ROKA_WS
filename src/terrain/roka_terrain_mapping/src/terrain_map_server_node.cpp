#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TerrainMapServer : public rclcpp::Node
{
public:
  TerrainMapServer() : Node("terrain_map_server")
  {
    const auto bag_path = declare_parameter("bag_path", "");
    const auto bag_topic = declare_parameter("bag_topic", "/terrain/static_map");
    const auto output_topic = declare_parameter("output_topic", "/terrain/static_map");
    const auto zero_timestamp = declare_parameter("zero_timestamp", false);
    if (bag_path.empty()) {
      throw std::invalid_argument("bag_path must point to a saved terrain_map.bag");
    }
    if (!grid_map::GridMapRosConverter::loadFromBag(bag_path, bag_topic, map_)) {
      throw std::runtime_error("failed to load terrain map from " + bag_path);
    }
    publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(
      output_topic, rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(1s, [this, zero_timestamp]() {
      map_.setTimestamp(zero_timestamp ? 0 : now().nanoseconds());
      publisher_->publish(*grid_map::GridMapRosConverter::toMessage(map_));
    });
    RCLCPP_INFO(get_logger(), "loaded terrain map %s", bag_path.c_str());
  }

private:
  grid_map::GridMap map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainMapServer>());
  rclcpp::shutdown();
  return 0;
}
