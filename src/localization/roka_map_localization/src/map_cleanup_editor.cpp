#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapCleanupEditor : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;

  MapCleanupEditor()
  : Node("map_cleanup_editor"), original_(new Cloud), working_(new Cloud), removed_(new Cloud)
  {
    input_path_ = absolutePath(declare_parameter<std::string>("input_pcd", ""));
    output_path_ = absolutePath(declare_parameter<std::string>("output_pcd", ""));
    audit_path_ = absolutePath(declare_parameter<std::string>("audit_file", ""));
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    removal_radius_m_ = declare_parameter<double>("removal_radius_m", 0.30);
    minimum_remaining_fraction_ = declare_parameter<double>(
      "minimum_remaining_fraction", 0.50);

    if (input_path_.empty() || !std::filesystem::is_regular_file(input_path_)) {
      throw std::invalid_argument("input_pcd must name an existing PCD file");
    }
    if (output_path_.empty()) {
      output_path_ = input_path_.parent_path() / "map_cleaned.pcd";
    }
    if (audit_path_.empty()) {
      audit_path_ = input_path_.parent_path() / "map_cleanup.yaml";
    }
    if (input_path_ == output_path_) {
      throw std::invalid_argument("map cleanup never overwrites the source PCD");
    }
    if (!(removal_radius_m_ > 0.02 && removal_radius_m_ <= 2.0)) {
      throw std::invalid_argument("removal_radius_m must be in (0.02, 2.0]");
    }
    if (!(minimum_remaining_fraction_ >= 0.50 && minimum_remaining_fraction_ <= 1.0)) {
      throw std::invalid_argument("minimum_remaining_fraction must be in [0.50, 1.0]");
    }

    if (pcl::io::loadPCDFile<PointT>(input_path_.string(), *original_) != 0) {
      throw std::runtime_error("failed to load input PCD");
    }
    std::vector<int> finite_indices;
    Cloud finite;
    pcl::removeNaNFromPointCloud(*original_, finite, finite_indices);
    *original_ = std::move(finite);
    if (original_->size() < 1000) {
      throw std::runtime_error("input PCD contains fewer than 1000 finite points");
    }
    *working_ = *original_;

    const auto cloud_qos = rclcpp::QoS(1).reliable().transient_local();
    original_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cleanup/original", cloud_qos);
    working_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cleanup/working", cloud_qos);
    removed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cleanup/removed", cloud_qos);
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&MapCleanupEditor::clickCallback, this, std::placeholders::_1));
    undo_service_ = create_service<std_srvs::srv::Trigger>(
      "/map_cleanup/undo",
      std::bind(
        &MapCleanupEditor::undoCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/map_cleanup/reset",
      std::bind(
        &MapCleanupEditor::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/map_cleanup/save",
      std::bind(
        &MapCleanupEditor::saveCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    publishAll();
    RCLCPP_INFO(
      get_logger(),
      "map cleanup ready: %zu points, click radius %.2fm; source remains read-only",
      original_->size(), removal_radius_m_);
  }

private:
  static std::filesystem::path absolutePath(const std::string & value)
  {
    if (value.empty()) {return {};}
    return std::filesystem::absolute(std::filesystem::path(value)).lexically_normal();
  }

  void publishCloud(
    const Cloud & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    publisher->publish(message);
  }

  void rebuildRemoved()
  {
    removed_->clear();
    for (const auto & operation : history_) {
      *removed_ += *operation;
    }
    removed_->width = static_cast<std::uint32_t>(removed_->size());
    removed_->height = 1;
    removed_->is_dense = true;
  }

  void publishAll() const
  {
    publishCloud(*original_, original_pub_);
    publishCloud(*working_, working_pub_);
    publishCloud(*removed_, removed_pub_);
  }

  void clickCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string frame = message->header.frame_id;
    if (!frame.empty() && frame.front() == '/') {frame.erase(0, 1);}
    if (frame != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "ignored click in frame '%s'; RViz Fixed Frame must be '%s'",
        frame.c_str(), map_frame_.c_str());
      return;
    }
    const double radius_squared = removal_radius_m_ * removal_radius_m_;
    Cloud::Ptr kept(new Cloud);
    Cloud::Ptr selected(new Cloud);
    kept->reserve(working_->size());
    selected->reserve(512);
    for (const auto & point : working_->points) {
      const double dx = point.x - message->point.x;
      const double dy = point.y - message->point.y;
      const double dz = point.z - message->point.z;
      if (dx * dx + dy * dy + dz * dz <= radius_squared) {
        selected->push_back(point);
      } else {
        kept->push_back(point);
      }
    }
    if (selected->empty()) {
      RCLCPP_WARN(get_logger(), "no map points found inside the clicked sphere");
      return;
    }
    const double remaining_fraction =
      static_cast<double>(kept->size()) / static_cast<double>(original_->size());
    if (remaining_fraction < minimum_remaining_fraction_) {
      RCLCPP_ERROR(
        get_logger(), "removal rejected: only %.1f%% of the original map would remain",
        100.0 * remaining_fraction);
      return;
    }
    history_.push_back(selected);
    working_ = kept;
    rebuildRemoved();
    publishCloud(*working_, working_pub_);
    publishCloud(*removed_, removed_pub_);
    RCLCPP_INFO(
      get_logger(), "preview removed %zu points; total removed=%zu, remaining=%zu",
      selected->size(), removed_->size(), working_->size());
  }

  void undoCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (history_.empty()) {
      response->success = false;
      response->message = "nothing to undo";
      return;
    }
    *working_ += *history_.back();
    history_.pop_back();
    rebuildRemoved();
    publishCloud(*working_, working_pub_);
    publishCloud(*removed_, removed_pub_);
    response->success = true;
    response->message = "last removal undone";
  }

  void resetCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *working_ = *original_;
    history_.clear();
    rebuildRemoved();
    publishCloud(*working_, working_pub_);
    publishCloud(*removed_, removed_pub_);
    response->success = true;
    response->message = "cleanup preview reset to source map";
  }

  void saveCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      std::filesystem::create_directories(output_path_.parent_path());
      auto pcd_temp = output_path_;
      pcd_temp += ".tmp";
      auto audit_temp = audit_path_;
      audit_temp += ".tmp";
      std::error_code ignored;
      std::filesystem::remove(pcd_temp, ignored);
      std::filesystem::remove(audit_temp, ignored);
      if (pcl::io::savePCDFileBinary(pcd_temp.string(), *working_) != 0) {
        throw std::runtime_error("failed to write binary cleaned PCD");
      }
      {
        std::ofstream audit(audit_temp);
        audit << "schema_version: 1\n"
              << "source_map: " << input_path_.string() << "\n"
              << "cleaned_map: " << output_path_.string() << "\n"
              << "map_frame: " << map_frame_ << "\n"
              << "removal_radius_m: " << removal_radius_m_ << "\n"
              << "manual_operations: " << history_.size() << "\n"
              << "original_points: " << original_->size() << "\n"
              << "removed_points: " << removed_->size() << "\n"
              << "remaining_points: " << working_->size() << "\n"
              << "source_preserved: true\n";
        if (!audit.good()) {throw std::runtime_error("failed to write cleanup audit");}
      }
      std::filesystem::rename(pcd_temp, output_path_);
      std::filesystem::rename(audit_temp, audit_path_);
      response->success = true;
      response->message = "saved cleaned map without modifying source: " + output_path_.string();
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "map cleanup save failed: %s", error.what());
    }
  }

  std::filesystem::path input_path_;
  std::filesystem::path output_path_;
  std::filesystem::path audit_path_;
  std::string map_frame_;
  double removal_radius_m_;
  double minimum_remaining_fraction_;
  Cloud::Ptr original_;
  Cloud::Ptr working_;
  Cloud::Ptr removed_;
  std::vector<Cloud::Ptr> history_;
  std::mutex mutex_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr working_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapCleanupEditor>());
  } catch (const std::exception & error) {
    std::cerr << "map_cleanup_editor: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
