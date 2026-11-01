#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace roka_terrain_mapping
{

class TerrainObstacleFilter : public rclcpp::Node
{
public:
  TerrainObstacleFilter()
  : Node("terrain_obstacle_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter("input_cloud_topic", "/velodyne_points");
    map_topic_ = declare_parameter("terrain_map_topic", "/terrain/local_map");
    output_topic_ = declare_parameter("output_obstacle_topic", "/terrain/obstacle_points");
    min_clearance_ = declare_parameter("min_obstacle_clearance_m", 0.15);
    max_clearance_ = declare_parameter("max_obstacle_clearance_m", 1.50);
    unknown_is_obstacle_ = declare_parameter("unknown_is_obstacle", false);
    unknown_level_ground_z_ = declare_parameter("unknown_level_ground_z_m", 0.0);
    tf_fallback_tolerance_sec_ = declare_parameter(
      "tf_fallback_tolerance_sec", 0.25);

    if (min_clearance_ < 0.0 || max_clearance_ <= min_clearance_ ||
      tf_fallback_tolerance_sec_ <= 0.0 || !std::isfinite(unknown_level_ground_z_))
    {
      throw std::invalid_argument("invalid obstacle clearance limits");
    }

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&TerrainObstacleFilter::mapCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TerrainObstacleFilter::cloudCallback, this, std::placeholders::_1));
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(), "terrain obstacle filter: %s + %s -> %s, clearance %.2f..%.2f m",
      cloud_topic_.c_str(), map_topic_.c_str(), output_topic_.c_str(),
      min_clearance_, max_clearance_);
  }

private:
  struct Point
  {
    float x;
    float y;
    float z;
  };

  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
  {
    grid_map::GridMap map;
    if (!grid_map::GridMapRosConverter::fromMessage(*message, map) ||
      !map.exists("elevation"))
    {
      if (!map_warning_active_) {
        RCLCPP_WARN(get_logger(), "terrain map is invalid or has no elevation layer");
        map_warning_active_ = true;
      }
      return;
    }
    map_warning_active_ = false;
    std::scoped_lock lock(mutex_);
    map_ = std::move(map);
    map_valid_ = true;
  }

  void publishCloud(
    const std_msgs::msg::Header & header, const std::vector<Point> & points)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    obstacle_pub_->publish(output);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    grid_map::GridMap map;
    {
      std::scoped_lock lock(mutex_);
      if (!map_valid_) {
        // An empty cloud prevents an unfiltered rising track surface from being
        // interpreted as an obstacle while the first elevation map is forming.
        publishCloud(input->header, {});
        return;
      }
      map = map_;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    if (input->header.frame_id == map.getFrameId()) {
      cloud = *input;
      tf_warning_active_ = false;
    } else {
      geometry_msgs::msg::TransformStamped transform;
      std::string error;
      try {
        transform = tf_buffer_.lookupTransform(
          map.getFrameId(), input->header.frame_id, input->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
      } catch (const tf2::TransformException & exact_error) {
        error = exact_error.what();
        try {
          transform = tf_buffer_.lookupTransform(
            map.getFrameId(), input->header.frame_id,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(0.02));
          const double offset = std::abs(
            (rclcpp::Time(input->header.stamp) -
            rclcpp::Time(transform.header.stamp)).seconds());
          if (offset > tf_fallback_tolerance_sec_) {
            error += "; latest TF offset " + std::to_string(offset) +
              " s exceeds fallback tolerance";
            transform.header.frame_id.clear();
          }
        } catch (const tf2::TransformException & latest_error) {
          error += "; latest TF unavailable: ";
          error += latest_error.what();
          transform.header.frame_id.clear();
        }
      }
      if (transform.header.frame_id.empty()) {
        if (!tf_warning_active_) {
          RCLCPP_WARN(
            get_logger(), "dropping cloud without terrain TF: %s", error.c_str());
          tf_warning_active_ = true;
        }
        return;
      }
      tf2::doTransform(*input, cloud, transform);
      tf_warning_active_ = false;
    }

    std::vector<Point> obstacles;
    obstacles.reserve(static_cast<std::size_t>(cloud.width * cloud.height / 8U));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        grid_map::Index index;
        const bool known = map.getIndex(grid_map::Position(*x, *y), index) &&
          map.isValid(index, "elevation");
        if (!known) {
          if (unknown_is_obstacle_) {
            const double level_clearance =
              static_cast<double>(*z) - unknown_level_ground_z_;
            if (level_clearance >= min_clearance_ && level_clearance <= max_clearance_) {
              obstacles.push_back(Point{*x, *y, *z});
            }
          }
          continue;
        }
        const double clearance = static_cast<double>(*z) - map.at("elevation", index);
        if (clearance >= min_clearance_ && clearance <= max_clearance_) {
          obstacles.push_back(Point{*x, *y, *z});
        }
      }
      fields_error_active_ = false;
    } catch (const std::runtime_error & error) {
      if (!fields_error_active_) {
        RCLCPP_ERROR(
          get_logger(), "PointCloud2 requires float x/y/z fields: %s", error.what());
        fields_error_active_ = true;
      }
      return;
    }
    publishCloud(cloud.header, obstacles);
  }

  std::string cloud_topic_;
  std::string map_topic_;
  std::string output_topic_;
  double min_clearance_;
  double max_clearance_;
  bool unknown_is_obstacle_;
  double unknown_level_ground_z_;
  double tf_fallback_tolerance_sec_;
  std::mutex mutex_;
  bool map_valid_{false};
  bool map_warning_active_{false};
  bool tf_warning_active_{false};
  bool fields_error_active_{false};
  grid_map::GridMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
};

}  // namespace roka_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roka_terrain_mapping::TerrainObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
