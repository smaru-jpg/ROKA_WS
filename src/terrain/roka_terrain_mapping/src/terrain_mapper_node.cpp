#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "roka_terrain_mapping/terrain_math.hpp"

using namespace std::chrono_literals;

namespace roka_terrain_mapping
{

class TerrainMapper : public rclcpp::Node
{
public:
  TerrainMapper()
  : Node("terrain_mapper"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter("input_cloud_topic", "/velodyne_points");
    output_topic_ = declare_parameter("output_map_topic", "/terrain/local_map");
    target_frame_ = declare_parameter("target_frame", "odom");
    tracking_frame_ = declare_parameter("tracking_frame", "base_link");
    length_x_ = declare_parameter("map_length_x", 12.0);
    length_y_ = declare_parameter("map_length_y", 12.0);
    resolution_ = declare_parameter("resolution", 0.10);
    min_height_ = declare_parameter("min_height", -1.5);
    max_height_ = declare_parameter("max_height", 2.5);
    min_points_ = declare_parameter("min_points_per_cell", 2);
    max_age_ = declare_parameter("max_cell_age_sec", 0.75);
    ground_separation_variance_ = declare_parameter(
      "ground_separation_variance_m2", 0.0025);
    tf_fallback_tolerance_sec_ = declare_parameter(
      "tf_fallback_tolerance_sec", 0.25);
    limits_.slope_deg = declare_parameter("slope_limit_deg", 35.0);
    limits_.roughness_m = declare_parameter("roughness_limit_m", 0.12);
    limits_.step_m = declare_parameter("step_limit_m", 0.20);
    limits_.variance_m2 = declare_parameter("variance_limit_m2", 0.010);
    const char * roka_root = std::getenv("ROKA_WS_ROOT");
    const std::string default_bundle_directory = roka_root ?
      (std::filesystem::path(roka_root) / "maps" / "track_v1").string() :
      "maps/track_v1";
    bundle_directory_ = declare_parameter(
      "map_bundle_directory", default_bundle_directory);
    save_topic_ = declare_parameter("save_bag_topic", "/terrain/static_map");
    const auto publish_rate = declare_parameter("publish_rate_hz", 10.0);

    if (length_x_ <= 0.0 || length_y_ <= 0.0 || resolution_ <= 0.0 ||
      min_height_ >= max_height_ || min_points_ < 1 || publish_rate <= 0.0 ||
      ground_separation_variance_ < 0.0 || tf_fallback_tolerance_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid terrain mapper geometry, height, or rate parameter");
    }

    map_.setFrameId(target_frame_);
    map_.setGeometry(
      grid_map::Length(length_x_, length_y_), resolution_, grid_map::Position::Zero());
    for (const auto & layer : layers_) {
      map_.add(layer);
    }
    map_.setBasicLayers({"elevation"});

    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
      output_topic_, rclcpp::QoS(1).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TerrainMapper::cloudCallback, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map", std::bind(
        &TerrainMapper::saveMap, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&TerrainMapper::publishMap, this));

    RCLCPP_INFO(
      get_logger(), "2.5D mapper: %s -> %s, frame=%s, %.1fx%.1f m @ %.2f m",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      length_x_, length_y_, resolution_);
  }

private:
  struct CellStats
  {
    std::size_t count{0};
    double mean{0.0};
    double m2{0.0};
    double minimum{std::numeric_limits<double>::infinity()};

    void add(double value)
    {
      ++count;
      const double delta = value - mean;
      mean += delta / static_cast<double>(count);
      m2 += delta * (value - mean);
      minimum = std::min(minimum, value);
    }
  };

  bool lookupTransformAtOrNear(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, geometry_msgs::msg::TransformStamped & transform,
    std::string & error)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException & exact_error) {
      error = exact_error.what();
    }
    try {
      transform = tf_buffer_.lookupTransform(
        target, source, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(0.02));
      const rclcpp::Time available(transform.header.stamp);
      const double offset = std::abs((stamp - available).seconds());
      if (offset > tf_fallback_tolerance_sec_) {
        error += "; latest TF offset " + std::to_string(offset) +
          " s exceeds fallback tolerance";
        return false;
      }
      ++tf_fallbacks_;
      return true;
    } catch (const tf2::TransformException & latest_error) {
      error += "; latest TF unavailable: ";
      error += latest_error.what();
      return false;
    }
  }

  bool lookupTrackingPosition(grid_map::Position & position, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    std::string error;
    if (!lookupTransformAtOrNear(target_frame_, tracking_frame_, stamp, tf, error)) {
      if (!tracking_tf_warning_active_) {
        RCLCPP_WARN(
          get_logger(), "tracking TF unavailable; keeping map center: %s", error.c_str());
        tracking_tf_warning_active_ = true;
      }
      return false;
    }
    position.x() = tf.transform.translation.x;
    position.y() = tf.transform.translation.y;
    tracking_tf_warning_active_ = false;
    return true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    if (input->header.frame_id == target_frame_) {
      cloud = *input;
      cloud_tf_warning_active_ = false;
    } else {
      geometry_msgs::msg::TransformStamped tf;
      std::string error;
      if (lookupTransformAtOrNear(
          target_frame_, input->header.frame_id, rclcpp::Time(input->header.stamp), tf, error))
      {
        tf2::doTransform(*input, cloud, tf);
        cloud_tf_warning_active_ = false;
      } else {
        ++dropped_clouds_;
        if (!cloud_tf_warning_active_) {
          RCLCPP_WARN(get_logger(), "dropping cloud without TF: %s", error.c_str());
          cloud_tf_warning_active_ = true;
        }
        return;
      }
    }

    std::scoped_lock lock(mutex_);
    grid_map::Position center = map_.getPosition();
    lookupTrackingPosition(center, input->header.stamp);
    map_.move(center);
    const auto received_at = now();
    const double update_dt = last_cloud_receive_time_.nanoseconds() == 0 ? 0.0 :
      std::max(0.0, (received_at - last_cloud_receive_time_).seconds());
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      if (!map_.isValid(index, "elevation")) {
        continue;
      }
      const float cell_age = std::isfinite(map_.at("age", index)) ?
        map_.at("age", index) + static_cast<float>(update_dt) : static_cast<float>(update_dt);
      if (cell_age > max_age_) {
        for (const auto & layer : layers_) {
          map_.at(layer, index) = std::numeric_limits<float>::quiet_NaN();
        }
      } else {
        map_.at("age", index) = cell_age;
      }
    }

    const auto size = map_.getSize();
    std::vector<CellStats> stats(static_cast<std::size_t>(size(0) * size(1)));
    std::size_t accepted = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");
      for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it) {
        const float x = *x_it;
        const float y = *y_it;
        const float z = *z_it;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          z < min_height_ || z > max_height_)
        {
          continue;
        }
        grid_map::Index index;
        if (!map_.getIndex(grid_map::Position(x, y), index)) {
          continue;
        }
        const auto linear = static_cast<std::size_t>(index(0) * size(1) + index(1));
        stats[linear].add(z);
        ++accepted;
      }
      fields_error_active_ = false;
    } catch (const std::runtime_error & error) {
      ++dropped_clouds_;
      if (!fields_error_active_) {
        RCLCPP_ERROR(
          get_logger(), "PointCloud2 requires float x/y/z fields: %s", error.what());
        fields_error_active_ = true;
      }
      return;
    }

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      const auto linear = static_cast<std::size_t>(index(0) * size(1) + index(1));
      const auto & cell = stats[linear];
      if (cell.count < static_cast<std::size_t>(min_points_)) {
        continue;
      }
      const double observed_variance =
        cell.count > 1 ? cell.m2 / static_cast<double>(cell.count - 1) : 0.0;
      // A vertical obstacle and the track surface can share one XY cell. In
      // that case use the lower return as ground instead of averaging the
      // obstacle into the pump-track elevation.
      const double observed_elevation = observed_variance >= ground_separation_variance_ ?
        cell.minimum : cell.mean;
      const bool existing = map_.isValid(index, "elevation");
      const double alpha = existing ? 0.60 : 1.0;
      const double previous_elevation = existing ? map_.at("elevation", index) : observed_elevation;
      const double previous_variance = existing && std::isfinite(map_.at("variance", index)) ?
        map_.at("variance", index) : observed_variance;
      const double previous_confidence = existing && std::isfinite(map_.at("confidence", index)) ?
        map_.at("confidence", index) : 0.0;
      map_.at("elevation", index) = static_cast<float>(
        (1.0 - alpha) * previous_elevation + alpha * observed_elevation);
      map_.at("variance", index) = static_cast<float>(
        (1.0 - alpha) * previous_variance + alpha * observed_variance);
      map_.at("age", index) = 0.0F;
      map_.at("confidence", index) = static_cast<float>(
        std::min(1.0, previous_confidence + 0.20 * static_cast<double>(cell.count)));
    }

    computeDerivedLayers();
    map_.setTimestamp(rclcpp::Time(input->header.stamp).nanoseconds());
    last_cloud_receive_time_ = received_at;
    last_cloud_source_time_ = rclcpp::Time(input->header.stamp);
    last_point_count_ = accepted;
    ++processed_clouds_;
  }

  void computeDerivedLayers()
  {
    grid_map::Index x_shift;
    grid_map::Index y_shift;
    grid_map::getIndexShiftFromPositionShift(
      x_shift, grid_map::Vector(resolution_, 0.0), resolution_);
    grid_map::getIndexShiftFromPositionShift(
      y_shift, grid_map::Vector(0.0, resolution_), resolution_);
    const auto buffer_size = map_.getSize();
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      if (!map_.isValid(index, "elevation")) {
        continue;
      }
      grid_map::Position position;
      map_.getPosition(index, position);
      const float center = map_.at("elevation", index);
      auto neighbor_at = [&](const grid_map::Index & shift, float & value) {
          grid_map::Vector position_shift;
          grid_map::getPositionShiftFromIndexShift(position_shift, shift, resolution_);
          if (!map_.isInside(position + position_shift)) {
            return false;
          }
          grid_map::Index neighbor_index = index + shift;
          grid_map::wrapIndexToRange(neighbor_index, buffer_size);
          if (!map_.isValid(neighbor_index, "elevation")) {
            return false;
          }
          value = map_.at("elevation", neighbor_index);
          return std::isfinite(value);
        };
      float x_minus = center;
      float x_plus = center;
      float y_minus = center;
      float y_plus = center;
      neighbor_at(-x_shift, x_minus);
      neighbor_at(x_shift, x_plus);
      neighbor_at(-y_shift, y_minus);
      neighbor_at(y_shift, y_plus);
      const double gradient_x = (x_plus - x_minus) / (2.0 * resolution_);
      const double gradient_y = (y_plus - y_minus) / (2.0 * resolution_);
      const double longitudinal = std::atan(gradient_x) * 180.0 / M_PI;
      const double lateral = std::atan(gradient_y) * 180.0 / M_PI;
      const double slope = std::atan(std::hypot(gradient_x, gradient_y)) * 180.0 / M_PI;

      double step = 0.0;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          float neighbor = center;
          if (neighbor_at(dx * x_shift + dy * y_shift, neighbor)) {
            step = std::max(step, std::abs(static_cast<double>(neighbor - center)));
          }
        }
      }

      const double variance = std::max(0.0F, map_.at("variance", index));
      const double roughness = std::sqrt(variance);
      const double confidence = map_.at("confidence", index);
      const double normalizer = std::sqrt(
        gradient_x * gradient_x + gradient_y * gradient_y + 1.0);

      map_.at("normal_x", index) = static_cast<float>(-gradient_x / normalizer);
      map_.at("normal_y", index) = static_cast<float>(-gradient_y / normalizer);
      map_.at("normal_z", index) = static_cast<float>(1.0 / normalizer);
      map_.at("slope", index) = static_cast<float>(slope);
      map_.at("longitudinal_slope", index) = static_cast<float>(longitudinal);
      map_.at("lateral_slope", index) = static_cast<float>(lateral);
      map_.at("roughness", index) = static_cast<float>(roughness);
      map_.at("step_height", index) = static_cast<float>(step);

      const TraversabilityInput input{
        slope, roughness, step, variance, confidence};
      const auto risk = normalizedRisk(input, limits_);
      map_.at("risk", index) = static_cast<float>(risk);
      map_.at("traversability", index) = static_cast<float>(traversabilityFromRisk(risk));
    }
  }

  void publishMap()
  {
    std::scoped_lock lock(mutex_);
    const auto stamp = now();
    const double age = last_cloud_source_time_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      (stamp - last_cloud_source_time_).seconds();
    const double receive_age = last_cloud_receive_time_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      (stamp - last_cloud_receive_time_).seconds();
    // Keep the newest input-cloud timestamp.  Consumers can then distinguish
    // a fresh 10 Hz publication timer from genuinely fresh LiDAR terrain.
    map_pub_->publish(*grid_map::GridMapRosConverter::toMessage(map_));
    publishDiagnostics(age, receive_age);
  }

  void publishDiagnostics(double age, double receive_age)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "terrain_mapping/input";
    status.hardware_id = "velodyne_pointcloud";
    if (!std::isfinite(age) || age > max_age_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "point cloud stale or not received";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "terrain map current";
    }
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    add("age_sec", std::isfinite(age) ? std::to_string(age) : "inf");
    add(
      "receive_age_sec",
      std::isfinite(receive_age) ? std::to_string(receive_age) : "inf");
    add("last_points", std::to_string(last_point_count_));
    add("processed_clouds", std::to_string(processed_clouds_));
    add("dropped_clouds", std::to_string(dropped_clouds_));
    add("tf_fallbacks", std::to_string(tf_fallbacks_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  void saveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::scoped_lock lock(mutex_);
    if (processed_clouds_ == 0) {
      response->success = false;
      response->message = "no point cloud has been processed";
      return;
    }
    const std::filesystem::path directory(bundle_directory_);
    const auto bag_path = directory / "terrain_map.bag";
    try {
      std::filesystem::create_directories(directory);
      if (std::filesystem::exists(bag_path)) {
        response->success = false;
        response->message = "refusing to overwrite existing " + bag_path.string();
        return;
      }
      map_.setTimestamp(now().nanoseconds());
      if (!grid_map::GridMapRosConverter::saveToBag(map_, bag_path.string(), save_topic_)) {
        response->success = false;
        response->message = "GridMapRosConverter::saveToBag failed";
        return;
      }
      std::ofstream metadata(directory / "terrain_metadata.yaml", std::ios::out);
      metadata << "schema_version: 1\n"
               << "frame_id: " << target_frame_ << "\n"
               << "topic: " << save_topic_ << "\n"
               << "resolution: " << resolution_ << "\n"
               << "length: [" << length_x_ << ", " << length_y_ << "]\n"
               << "layers:\n";
      for (const auto & layer : layers_) {
        metadata << "  - " << layer << "\n";
      }
      response->success = true;
      response->message = "saved terrain map to " + bag_path.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  const std::vector<std::string> layers_{
    "elevation", "variance", "age", "normal_x", "normal_y", "normal_z", "slope",
    "longitudinal_slope", "lateral_slope", "roughness", "step_height", "confidence",
    "risk", "traversability"};
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string tracking_frame_;
  std::string bundle_directory_;
  std::string save_topic_;
  double length_x_;
  double length_y_;
  double resolution_;
  double min_height_;
  double max_height_;
  double max_age_;
  double ground_separation_variance_;
  double tf_fallback_tolerance_sec_;
  int min_points_;
  TraversabilityLimits limits_;
  grid_map::GridMap map_;
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Time last_cloud_source_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_receive_time_{0, 0, RCL_ROS_TIME};
  std::size_t last_point_count_{0};
  std::size_t processed_clouds_{0};
  std::size_t dropped_clouds_{0};
  std::size_t tf_fallbacks_{0};
  bool tracking_tf_warning_active_{false};
  bool cloud_tf_warning_active_{false};
  bool fields_error_active_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roka_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roka_terrain_mapping::TerrainMapper>());
  rclcpp::shutdown();
  return 0;
}
