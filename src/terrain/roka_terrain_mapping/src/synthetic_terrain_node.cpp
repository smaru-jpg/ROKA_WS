#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class SyntheticTerrain : public rclcpp::Node
{
public:
  SyntheticTerrain() : Node("synthetic_terrain")
  {
    terrain_ = declare_parameter("terrain_type", "flat");
    frame_id_ = declare_parameter("frame_id", "odom");
    cloud_topic_ = declare_parameter("cloud_topic", "/velodyne_points");
    slope_deg_ = declare_parameter("slope_deg", 15.0);
    step_height_ = declare_parameter("step_height_m", 0.18);
    roughness_ = declare_parameter("roughness_m", 0.04);
    length_ = declare_parameter("length", 10.0);
    // Denser than the mapper grid so synthetic points do not alias on cell borders.
    resolution_ = declare_parameter("resolution", 0.05);
    rate_ = declare_parameter("publish_rate_hz", 10.0);
    if (length_ <= 0.0 || resolution_ <= 0.0 || rate_ <= 0.0) {
      throw std::invalid_argument("synthetic geometry and rate must be positive");
    }
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), std::bind(&SyntheticTerrain::publish, this));
    RCLCPP_INFO(get_logger(), "synthetic terrain '%s' at %.1f Hz", terrain_.c_str(), rate_);
  }

private:
  double height(double x, double y) const
  {
    const double tangent = std::tan(slope_deg_ * M_PI / 180.0);
    if (terrain_ == "flat") {
      return 0.0;
    }
    if (terrain_ == "uphill") {
      return tangent * x;
    }
    if (terrain_ == "downhill") {
      return -tangent * x;
    }
    if (terrain_ == "cross_slope") {
      return tangent * y;
    }
    if (terrain_ == "step") {
      return x >= 1.0 ? step_height_ : 0.0;
    }
    if (terrain_ == "rough") {
      return roughness_ * (std::sin(8.0 * x) + 0.7 * std::sin(11.0 * y));
    }
    if (terrain_ == "mixed") {
      return 0.5 * tangent * x + roughness_ * std::sin(9.0 * y) +
             (x > 1.5 ? 0.5 * step_height_ : 0.0);
    }
    throw std::invalid_argument("unknown terrain_type: " + terrain_);
  }

  void publish()
  {
    const auto stamp = now();
    const int width = static_cast<int>(std::floor(length_ / resolution_)) + 1;
    const std::size_t point_count = static_cast<std::size_t>(width * width);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);
    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    for (int ix = 0; ix < width; ++ix) {
      const double x = -0.5 * length_ + ix * resolution_;
      for (int iy = 0; iy < width; ++iy, ++x_it, ++y_it, ++z_it) {
        const double y = -0.5 * length_ + iy * resolution_;
        *x_it = static_cast<float>(x);
        *y_it = static_cast<float>(y);
        *z_it = static_cast<float>(height(x, y));
      }
    }
    cloud_pub_->publish(cloud);

    double roll = 0.0;
    double pitch = 0.0;
    if (terrain_ == "cross_slope") {
      roll = slope_deg_ * M_PI / 180.0;
    } else if (terrain_ == "uphill") {
      pitch = -slope_deg_ * M_PI / 180.0;
    } else if (terrain_ == "downhill") {
      pitch = slope_deg_ * M_PI / 180.0;
    }
    tf2::Quaternion q;
    q.setRPY(roll, pitch, 0.0);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
    imu.orientation_covariance[0] = 0.0025;
    imu.orientation_covariance[4] = 0.0025;
    imu.orientation_covariance[8] = 0.01;
    imu.angular_velocity_covariance[0] = 0.001;
    imu.angular_velocity_covariance[4] = 0.001;
    imu.angular_velocity_covariance[8] = 0.001;
    imu.linear_acceleration.z = 9.80665;
    imu.linear_acceleration_covariance[0] = 0.04;
    imu.linear_acceleration_covariance[4] = 0.04;
    imu.linear_acceleration_covariance[8] = 0.04;
    imu_pub_->publish(imu);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.orientation = imu.orientation;
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.01;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped base;
    base.header.stamp = stamp;
    base.header.frame_id = "odom";
    base.child_frame_id = "base_link";
    base.transform.rotation.x = q.x();
    base.transform.rotation.y = q.y();
    base.transform.rotation.z = q.z();
    base.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(base);

    geometry_msgs::msg::TransformStamped footprint;
    footprint.header.stamp = stamp;
    footprint.header.frame_id = "odom";
    footprint.child_frame_id = "base_footprint";
    footprint.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(footprint);
  }

  std::string terrain_;
  std::string frame_id_;
  std::string cloud_topic_;
  double slope_deg_;
  double step_height_;
  double roughness_;
  double length_;
  double resolution_;
  double rate_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SyntheticTerrain>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("synthetic_terrain"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
