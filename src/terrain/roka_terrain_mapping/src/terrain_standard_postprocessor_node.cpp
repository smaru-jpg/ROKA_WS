#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "filters/filter_chain.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roka_terrain_mapping
{

// This node intentionally contains no terrain algorithm of its own.  It is a
// one-shot ROS 2 wrapper around the upstream ANYbotics GridMapPclLoader and
// grid_map filter-chain plugins so offline processing remains deterministic.
class TerrainStandardPostprocessor : public rclcpp::Node
{
public:
  TerrainStandardPostprocessor()
  : Node("terrain_standard_postprocessor"), filter_chain_("grid_map::GridMap")
  {
    const auto input_pcd = declare_parameter<std::string>("input_pcd", "");
    const auto pcl_config = declare_parameter<std::string>("pcl_config", "");
    const auto output_bag = declare_parameter<std::string>("output_bag", "");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/terrain/static_map");
    const auto map_frame = declare_parameter<std::string>("map_frame", "map");
    const auto chain_prefix = declare_parameter<std::string>(
      "filter_chain_prefix", "grid_map_filters");

    if (!std::filesystem::is_regular_file(input_pcd)) {
      throw std::invalid_argument("input PCD does not exist: " + input_pcd);
    }
    if (!std::filesystem::is_regular_file(pcl_config)) {
      throw std::invalid_argument("grid_map_pcl config does not exist: " + pcl_config);
    }
    if (output_bag.empty() || output_topic.empty() || map_frame.empty()) {
      throw std::invalid_argument("output_bag/output_topic/map_frame must be set");
    }
    if (std::filesystem::exists(output_bag)) {
      throw std::invalid_argument("refusing to overwrite terrain bag: " + output_bag);
    }
    if (!filter_chain_.configure(
        chain_prefix, get_node_logging_interface(), get_node_parameters_interface()))
    {
      throw std::runtime_error("failed to configure upstream grid_map filter chain");
    }

    grid_map::GridMapPclLoader loader(get_logger());
    loader.loadParameters(pcl_config);
    loader.loadCloudFromPcdFile(input_pcd);
    loader.preProcessInputCloud();
    loader.initializeGridMapGeometryFromInputCloud();
    loader.addLayerFromInputCloud("elevation");

    auto elevation_map = loader.getGridMap();
    elevation_map.setFrameId(map_frame);
    elevation_map.setTimestamp(now().nanoseconds());
    grid_map::GridMap filtered_map;
    if (!filter_chain_.update(elevation_map, filtered_map)) {
      throw std::runtime_error("upstream grid_map filter chain failed");
    }
    filtered_map.setFrameId(map_frame);
    filtered_map.setTimestamp(now().nanoseconds());
    if (!grid_map::GridMapRosConverter::saveToBag(
        filtered_map, output_bag, output_topic))
    {
      throw std::runtime_error("failed to save filtered GridMap: " + output_bag);
    }

    std::size_t observed = 0;
    std::size_t possible = 0;
    std::size_t caution = 0;
    std::size_t blocked = 0;
    for (grid_map::GridMapIterator iterator(filtered_map); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      if (!filtered_map.isValid(index, "risk")) {continue;}
      const auto risk = filtered_map.at("risk", index);
      ++observed;
      if (risk <= 0.50F) {++possible;} else if (risk <= 1.00F) {++caution;} else {++blocked;}
    }
    if (observed == 0) {
      throw std::runtime_error("filtered terrain contains no observed risk cells");
    }
    RCLCPP_INFO(
      get_logger(),
      "standard terrain complete: cells=%zu possible=%zu caution=%zu blocked=%zu layers=%zu; %s",
      observed, possible, caution, blocked, filtered_map.getLayers().size(), output_bag.c_str());
  }

private:
  filters::FilterChain<grid_map::GridMap> filter_chain_;
};

}  // namespace roka_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<roka_terrain_mapping::TerrainStandardPostprocessor>();
    (void)node;
  } catch (const std::exception & error) {
    std::cerr << "terrain_standard_postprocessor: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
