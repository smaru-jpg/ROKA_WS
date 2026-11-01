#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace roka_terrain_mapping
{

class TerrainReviewVisualizer : public rclcpp::Node
{
public:
  TerrainReviewVisualizer()
  : Node("terrain_review_visualizer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter("input_topic", "/terrain/static_map");
    output_topic_ = declare_parameter("output_topic", "/terrain/visual/classes");
    keepout_topic_ = declare_parameter("keepout_topic", "/terrain/manual_keepout_mask");
    caution_risk_ = declare_parameter("caution_risk", 0.50);
    blocked_risk_ = declare_parameter("blocked_risk", 1.00);
    audit_file_ = declare_parameter("audit_file", "");
    click_topic_ = declare_parameter("click_topic", "/terrain_review/clicked_point");
    click_review_enabled_ = declare_parameter("enable_click_review", false);
    if (!(caution_risk_ >= 0.0 && blocked_risk_ > caution_risk_ && blocked_risk_ <= 2.0)) {
      throw std::invalid_argument("invalid terrain review thresholds");
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, qos);
    keepout_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(keepout_topic_, qos);
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, qos,
      std::bind(&TerrainReviewVisualizer::mapCallback, this, std::placeholders::_1));
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      click_topic_, 10,
      std::bind(&TerrainReviewVisualizer::clickCallback, this, std::placeholders::_1));
    mode_possible_service_ = makeService(
      "/terrain_review/mode_possible", &TerrainReviewVisualizer::modePossible);
    mode_blocked_service_ = makeService(
      "/terrain_review/mode_blocked", &TerrainReviewVisualizer::modeBlocked);
    mode_inspect_service_ = makeService(
      "/terrain_review/mode_inspect", &TerrainReviewVisualizer::modeInspect);
    commit_service_ = makeService("/terrain_review/commit", &TerrainReviewVisualizer::commit);
    cancel_service_ = makeService("/terrain_review/cancel", &TerrainReviewVisualizer::cancel);
    undo_service_ = makeService("/terrain_review/undo", &TerrainReviewVisualizer::undo);
    reset_service_ = makeService("/terrain_review/reset", &TerrainReviewVisualizer::reset);
    save_service_ = makeService("/terrain_review/save", &TerrainReviewVisualizer::save);
    RCLCPP_INFO(
      get_logger(), "terrain review: %s -> %s; polygon annotation=%s; keepout=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      click_review_enabled_ ? "enabled" : "disabled", keepout_topic_.c_str());
  }

private:
  enum class ReviewMode {Inspect, Possible, Blocked};
  struct ReviewPolygon
  {
    ReviewMode mode;
    std::vector<geometry_msgs::msg::Point> vertices;
  };
  using Trigger = std_srvs::srv::Trigger;
  using Callback = void (TerrainReviewVisualizer::*)(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr);

  rclcpp::Service<Trigger>::SharedPtr makeService(const std::string & name, Callback callback)
  {
    return create_service<Trigger>(
      name, std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  visualization_msgs::msg::Marker makeCellMarker(
    const std::string & name, int id, float r, float g, float b, float a) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.getFrameId();
    marker.header.stamp = now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double cell = map_.getResolution();
    marker.scale.x = 0.92 * cell;
    marker.scale.y = 0.92 * cell;
    marker.scale.z = 0.025;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  }

  visualization_msgs::msg::Marker makeOutline(
    const std::string & name, int id, const std::vector<geometry_msgs::msg::Point> & vertices,
    ReviewMode mode, bool close) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.getFrameId();
    marker.header.stamp = now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = close ? 0.045 : 0.025;
    marker.color.a = 1.0;
    if (mode == ReviewMode::Possible) {
      marker.color.r = 0.0F; marker.color.g = 1.0F; marker.color.b = 0.72F;
    } else {
      marker.color.r = 0.90F; marker.color.g = 0.0F; marker.color.b = 0.80F;
    }
    for (auto point : vertices) {
      point.z += 0.12;
      marker.points.push_back(point);
    }
    if (close && !marker.points.empty()) {marker.points.push_back(marker.points.front());}
    return marker;
  }

  static bool pointInPolygon(
    double x, double y, const std::vector<geometry_msgs::msg::Point> & vertices)
  {
    if (vertices.size() < 3) {return false;}
    bool inside = false;
    std::size_t previous = vertices.size() - 1;
    for (std::size_t current = 0; current < vertices.size(); ++current) {
      const auto & a = vertices[current];
      const auto & b = vertices[previous];
      const bool crosses = ((a.y > y) != (b.y > y)) &&
        (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x);
      if (crosses) {inside = !inside;}
      previous = current;
    }
    return inside;
  }

  ReviewMode overrideAt(double x, double y, bool & overridden) const
  {
    for (auto iterator = polygons_.rbegin(); iterator != polygons_.rend(); ++iterator) {
      if (pointInPolygon(x, y, iterator->vertices)) {
        overridden = true;
        return iterator->mode;
      }
    }
    overridden = false;
    return ReviewMode::Inspect;
  }

  nav_msgs::msg::OccupancyGrid makeKeepoutMask() const
  {
    nav_msgs::msg::OccupancyGrid mask;
    mask.header.frame_id = map_.getFrameId();
    mask.header.stamp = now();
    const double resolution = map_.getResolution();
    const auto length = map_.getLength();
    const auto center = map_.getPosition();
    mask.info.resolution = resolution;
    mask.info.width = static_cast<std::uint32_t>(std::ceil(length.x() / resolution));
    mask.info.height = static_cast<std::uint32_t>(std::ceil(length.y() / resolution));
    mask.info.origin.position.x = center.x() - 0.5 * length.x();
    mask.info.origin.position.y = center.y() - 0.5 * length.y();
    mask.info.origin.orientation.w = 1.0;
    mask.data.assign(static_cast<std::size_t>(mask.info.width) * mask.info.height, 0);
    for (std::uint32_t row = 0; row < mask.info.height; ++row) {
      const double y = mask.info.origin.position.y + (row + 0.5) * resolution;
      for (std::uint32_t column = 0; column < mask.info.width; ++column) {
        const double x = mask.info.origin.position.x + (column + 0.5) * resolution;
        for (const auto & polygon : polygons_) {
          if (polygon.mode == ReviewMode::Blocked && pointInPolygon(x, y, polygon.vertices)) {
            mask.data[static_cast<std::size_t>(row) * mask.info.width + column] = 100;
            break;
          }
        }
      }
    }
    return mask;
  }

  void publish()
  {
    if (map_.getSize().prod() == 0) {return;}
    auto possible = makeCellMarker("possible_green", 0, 0.08F, 0.88F, 0.20F, 0.78F);
    auto caution = makeCellMarker("caution_yellow", 1, 1.00F, 0.72F, 0.02F, 0.82F);
    auto blocked = makeCellMarker("blocked_red", 2, 1.00F, 0.10F, 0.06F, 0.88F);
    auto unknown = makeCellMarker("unobserved_gray", 3, 0.48F, 0.50F, 0.55F, 0.12F);
    auto manual_possible = makeCellMarker(
      "manual_possible_polygon", 4, 0.00F, 1.00F, 0.72F, 0.94F);
    auto manual_blocked = makeCellMarker(
      "manual_blocked_polygon", 5, 0.85F, 0.00F, 0.80F, 0.94F);
    manual_possible.scale.z = manual_blocked.scale.z = 0.045;

    const bool has_risk = map_.exists("risk");
    const bool has_traversability = map_.exists("traversability");
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      grid_map::Position position;
      if (!map_.getPosition(index, position)) {continue;}
      geometry_msgs::msg::Point point;
      point.x = position.x();
      point.y = position.y();
      const bool elevation_valid = map_.isValid(index, "elevation");
      point.z = elevation_valid ? static_cast<double>(map_.at("elevation", index)) + 0.025 : 0.0;
      bool overridden = false;
      const auto manual_mode = overrideAt(point.x, point.y, overridden);
      if (overridden) {
        point.z += 0.035;
        (manual_mode == ReviewMode::Possible ? manual_possible : manual_blocked).points.push_back(
          point);
        continue;
      }
      if (!elevation_valid || (!has_risk && !has_traversability)) {
        unknown.points.push_back(point);
        continue;
      }
      double risk = std::numeric_limits<double>::quiet_NaN();
      if (has_risk && map_.isValid(index, "risk")) {
        risk = static_cast<double>(map_.at("risk", index));
      } else if (has_traversability && map_.isValid(index, "traversability")) {
        risk = 1.0 - static_cast<double>(map_.at("traversability", index));
      }
      if (!std::isfinite(risk)) {unknown.points.push_back(point);} else if (risk <= caution_risk_) {
        possible.points.push_back(point);
      } else if (risk <= blocked_risk_) {caution.points.push_back(point);} else {
        blocked.points.push_back(point);
      }
    }
    visualization_msgs::msg::MarkerArray result;
    result.markers = {possible, caution, blocked, unknown, manual_possible, manual_blocked};
    int marker_id = 100;
    for (const auto & polygon : polygons_) {
      result.markers.push_back(
        makeOutline(
          "committed_review_polygons", marker_id++, polygon.vertices, polygon.mode, true));
    }
    if (!draft_.empty() && mode_ != ReviewMode::Inspect) {
      result.markers.push_back(
        makeOutline(
          "draft_review_polygon", marker_id, draft_, mode_, false));
    }
    publisher_->publish(result);
    keepout_publisher_->publish(makeKeepoutMask());
  }

  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::GridMap incoming;
    if (!grid_map::GridMapRosConverter::fromMessage(*message, incoming)) {
      RCLCPP_ERROR(get_logger(), "failed to decode terrain GridMap");
      return;
    }
    map_ = std::move(incoming);
    publish();
  }

  void clickCallback(const geometry_msgs::msg::PointStamped::SharedPtr point)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!click_review_enabled_ || mode_ == ReviewMode::Inspect || map_.getFrameId().empty()) {
      return;
    }
    geometry_msgs::msg::PointStamped selected = *point;
    std::string source_frame = selected.header.frame_id;
    if (!source_frame.empty() && source_frame.front() == '/') {source_frame.erase(0, 1);}
    if (source_frame != map_.getFrameId()) {
      try {
        selected = tf_buffer_.transform(*point, map_.getFrameId(), tf2::durationFromSec(0.10));
      } catch (const tf2::TransformException & error) {
        RCLCPP_ERROR(get_logger(), "cannot transform polygon vertex: %s", error.what());
        return;
      }
    }
    draft_.push_back(selected.point);
    publish();
    RCLCPP_INFO(
      get_logger(), "review polygon vertex %zu: (%.2f, %.2f)", draft_.size(),
      selected.point.x, selected.point.y);
  }

  void setMode(ReviewMode mode, const std::string & label, Trigger::Response::SharedPtr response)
  {
    if (!click_review_enabled_) {
      response->success = false;
      response->message = "manual terrain polygons are disabled in this view";
      return;
    }
    draft_.clear();
    mode_ = mode;
    publish();
    response->success = true;
    response->message = "terrain polygon mode: " + label + "; click >=3 vertices, then commit";
  }

  void modePossible(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    setMode(ReviewMode::Possible, "possible", response);
  }
  void modeBlocked(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    setMode(ReviewMode::Blocked, "blocked", response);
  }
  void modeInspect(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    draft_.clear();
    mode_ = ReviewMode::Inspect;
    publish();
    response->success = true;
    response->message = "terrain review mode: inspect only";
  }
  void commit(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == ReviewMode::Inspect || draft_.size() < 3) {
      response->success = false;
      response->message = "polygon commit needs possible/blocked mode and at least 3 vertices";
      return;
    }
    polygons_.push_back(ReviewPolygon{mode_, draft_});
    draft_.clear();
    publish();
    response->success = true;
    response->message = "terrain review polygon committed";
  }
  void cancel(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    draft_.clear();
    mode_ = ReviewMode::Inspect;
    publish();
    response->success = true;
    response->message = "draft terrain polygon cancelled";
  }
  void undo(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!draft_.empty()) {
      draft_.pop_back();
      response->message = "last draft vertex removed";
    } else if (!polygons_.empty()) {
      polygons_.pop_back();
      response->message = "last committed polygon removed";
    } else {
      response->success = false;
      response->message = "no terrain review polygon to undo";
      return;
    }
    publish();
    response->success = true;
  }
  void reset(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    draft_.clear();
    polygons_.clear();
    mode_ = ReviewMode::Inspect;
    publish();
    response->success = true;
    response->message = "all terrain review polygons reset";
  }
  void save(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (audit_file_.empty()) {
      response->success = false;
      response->message = "audit_file is not configured";
      return;
    }
    try {
      const std::filesystem::path output(audit_file_);
      std::filesystem::create_directories(output.parent_path());
      auto temporary = output;
      temporary += ".tmp";
      std::ofstream stream(temporary);
      stream << "schema_version: 2\n"
             << "source_topic: " << input_topic_ << "\n"
             << "map_frame: " << map_.getFrameId() << "\n"
             << "review_only: true\n"
             << "autonomy_input: false\n"
             << "keepout_topic: " << keepout_topic_ << "\n"
             << "polygons:\n";
      for (const auto & polygon : polygons_) {
        stream << "  - classification: "
               << (polygon.mode == ReviewMode::Possible ? "possible" : "blocked") << "\n"
               << "    vertices:\n";
        for (const auto & vertex : polygon.vertices) {
          stream << "      - [" << vertex.x << ", " << vertex.y << ", " << vertex.z << "]\n";
        }
      }
      if (!stream.good()) {throw std::runtime_error("failed to write terrain review YAML");}
      stream.close();
      std::filesystem::rename(temporary, output);
      response->success = true;
      response->message = "saved review-only terrain polygons: " + output.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string keepout_topic_;
  std::string audit_file_;
  std::string click_topic_;
  double caution_risk_;
  double blocked_risk_;
  bool click_review_enabled_;
  ReviewMode mode_{ReviewMode::Inspect};
  grid_map::GridMap map_;
  std::vector<geometry_msgs::msg::Point> draft_;
  std::vector<ReviewPolygon> polygons_;
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_publisher_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Service<Trigger>::SharedPtr mode_possible_service_;
  rclcpp::Service<Trigger>::SharedPtr mode_blocked_service_;
  rclcpp::Service<Trigger>::SharedPtr mode_inspect_service_;
  rclcpp::Service<Trigger>::SharedPtr commit_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr undo_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
};

}  // namespace roka_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<roka_terrain_mapping::TerrainReviewVisualizer>());
  } catch (const std::exception & error) {
    std::cerr << "terrain_review_visualizer: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
