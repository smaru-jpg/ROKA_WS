#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointcloudSectorFilter : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;

  PointcloudSectorFilter()
  : Node("pointcloud_sector_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/velodyne_points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/mapping/front_120_points");
    horizontal_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 120.0);
    minimum_range_m_ = declare_parameter<double>("minimum_range_m", 0.9);
    maximum_range_m_ = declare_parameter<double>("maximum_range_m", 30.0);
    minimum_z_m_ = declare_parameter<double>(
      "minimum_z_m", -std::numeric_limits<double>::infinity());
    maximum_z_m_ = declare_parameter<double>(
      "maximum_z_m", std::numeric_limits<double>::infinity());
    reliable_output_ = declare_parameter<bool>("reliable_output", false);

    if (!(horizontal_fov_deg_ > 0.0 && horizontal_fov_deg_ <= 360.0)) {
      throw std::invalid_argument("horizontal_fov_deg must be in (0, 360]");
    }
    if (!(minimum_range_m_ >= 0.0 && maximum_range_m_ > minimum_range_m_)) {
      throw std::invalid_argument("pointcloud sector range is invalid");
    }
    if (!(maximum_z_m_ > minimum_z_m_)) {
      throw std::invalid_argument("pointcloud sector z bounds are invalid");
    }

    const auto input_qos = rclcpp::SensorDataQoS().keep_last(2);
    auto output_qos = rclcpp::QoS(2).durability_volatile();
    if (reliable_output_) {
      output_qos.reliable();
    } else {
      output_qos.best_effort();
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      std::bind(&PointcloudSectorFilter::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "display-only sector: %s -> %s, %.1f deg, %.1f..%.1f m",
      input_topic_.c_str(), output_topic_.c_str(), horizontal_fov_deg_,
      minimum_range_m_, maximum_range_m_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    Cloud input;
    pcl::fromROSMsg(*message, input);
    Cloud output;
    output.header = input.header;
    output.reserve(input.size());

    constexpr double kPi = 3.14159265358979323846;
    const double half_fov = horizontal_fov_deg_ * kPi / 360.0;
    const double minimum_squared = minimum_range_m_ * minimum_range_m_;
    const double maximum_squared = maximum_range_m_ * maximum_range_m_;
    for (const auto & point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity))
      {
        continue;
      }
      const double squared_range =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z;
      if (squared_range < minimum_squared || squared_range > maximum_squared) {
        continue;
      }
      if (point.z < minimum_z_m_ || point.z > maximum_z_m_) {
        continue;
      }
      if (std::abs(std::atan2(point.y, point.x)) > half_fov) {
        continue;
      }
      output.push_back(point);
    }
    output.width = static_cast<std::uint32_t>(output.size());
    output.height = 1;
    output.is_dense = true;

    sensor_msgs::msg::PointCloud2 result;
    pcl::toROSMsg(output, result);
    result.header = message->header;
    publisher_->publish(result);
  }

  std::string input_topic_;
  std::string output_topic_;
  double horizontal_fov_deg_;
  double minimum_range_m_;
  double maximum_range_m_;
  double minimum_z_m_;
  double maximum_z_m_;
  bool reliable_output_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudSectorFilter>());
  rclcpp::shutdown();
  return 0;
}
