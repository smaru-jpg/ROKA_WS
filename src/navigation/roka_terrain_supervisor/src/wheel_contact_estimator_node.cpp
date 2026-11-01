#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "roka_interfaces/msg/wheel_contact_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace roka_terrain_supervisor
{

class WheelContactEstimator : public rclcpp::Node
{
public:
  WheelContactEstimator()
  : Node("wheel_contact_estimator")
  {
    body_odom_topic_ = declare_parameter("body_odom_topic", "/odom");
    wheel_odom_topic_ = declare_parameter("wheel_odom_topic", "/wheel/odom");
    derive_body_twist_from_pose_ = declare_parameter(
      "derive_body_twist_from_pose", false);
    pose_twist_filter_alpha_ = declare_parameter("pose_twist_filter_alpha", 0.35);
    imu_topic_ = declare_parameter("imu_topic", "/imu/data");
    joint_state_topic_ = declare_parameter("joint_state_topic", "/joint_states");
    output_topic_ = declare_parameter("contact_state_topic", "/terrain/wheel_contact");
    const double rate = declare_parameter("contact_publish_rate_hz", 50.0);
    wheel_radius_ = declare_parameter("wheel_radius", 0.103);
    wheel_track_ = declare_parameter("wheel_track", 0.427149994);
    body_timeout_ = declare_parameter("body_odom_timeout_sec", 0.35);
    wheel_timeout_ = declare_parameter("wheel_odom_timeout_sec", 0.35);
    imu_timeout_ = declare_parameter("contact_imu_timeout_sec", 0.25);
    motor_timeout_ = declare_parameter("motor_feedback_timeout_sec", 0.50);
    joint_timeout_ = declare_parameter("joint_feedback_timeout_sec", 0.25);
    airborne_accel_max_ = declare_parameter("airborne_accel_norm_max_mps2", 3.0);
    landing_accel_min_ = declare_parameter("landing_accel_norm_min_mps2", 13.0);
    airborne_hold_ = declare_parameter("airborne_hold_sec", 0.05);
    partial_hold_ = declare_parameter("partial_contact_hold_sec", 0.08);
    landing_hold_ = declare_parameter("landing_hold_sec", 0.30);
    moving_speed_min_ = declare_parameter("contact_moving_speed_min_mps", 0.08);
    wheel_overspeed_ratio_ = declare_parameter("wheel_unload_speed_ratio", 1.60);
    wheel_overspeed_margin_ = declare_parameter("wheel_unload_speed_margin_mps", 0.12);
    unloaded_current_max_ = declare_parameter("unloaded_current_max_a", 0.60);
    loaded_current_min_ = declare_parameter("loaded_current_min_a", 1.20);
    feedback_signs_ = {
      declare_parameter("left_mot1_feedback_sign", 1),
      declare_parameter("left_mot2_feedback_sign", -1),
      declare_parameter("right_mot1_feedback_sign", 1),
      declare_parameter("right_mot2_feedback_sign", -1)};

    if (rate <= 0.0 || wheel_radius_ <= 0.0 || wheel_track_ <= 0.0 ||
      std::min({body_timeout_, wheel_timeout_, imu_timeout_, motor_timeout_, joint_timeout_}) <= 0.0 ||
      airborne_accel_max_ <= 0.0 || landing_accel_min_ <= airborne_accel_max_ ||
      airborne_hold_ <= 0.0 || partial_hold_ <= 0.0 || landing_hold_ <= 0.0 ||
      pose_twist_filter_alpha_ <= 0.0 || pose_twist_filter_alpha_ > 1.0 ||
      std::any_of(feedback_signs_.begin(), feedback_signs_.end(),
      [](int64_t sign) {return sign != -1 && sign != 1;}))
    {
      throw std::invalid_argument("invalid wheel-contact estimator parameter");
    }

    body_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      body_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelContactEstimator::bodyOdomCallback, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelContactEstimator::wheelOdomCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelContactEstimator::imuCallback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelContactEstimator::jointCallback, this, std::placeholders::_1));
    left_motor_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/md/left/raw_state", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        motorCallback(0, message);
      });
    right_motor_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/md/right/raw_state", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        motorCallback(2, message);
      });
    contact_pub_ = create_publisher<roka_interfaces::msg::WheelContactState>(output_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&WheelContactEstimator::update, this));
    candidate_since_ = now();
    state_since_ = now();
    RCLCPP_INFO(
      get_logger(), "wheel-contact estimator: body=%s wheel=%s imu=%s -> %s",
      body_odom_topic_.c_str(), wheel_odom_topic_.c_str(), imu_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  using Contact = roka_interfaces::msg::WheelContactState;

  static std::string stateName(uint8_t state)
  {
    switch (state) {
      case Contact::GROUNDED: return "GROUNDED";
      case Contact::PARTIAL_CONTACT: return "PARTIAL_CONTACT";
      case Contact::AIRBORNE: return "AIRBORNE";
      case Contact::LANDING: return "LANDING";
      default: return "UNKNOWN";
    }
  }

  void bodyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & twist = message->twist.twist;
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    if (derive_body_twist_from_pose_) {
      const auto & position = message->pose.pose.position;
      const auto & q = message->pose.pose.orientation;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w))
      {
        return;
      }
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const rclcpp::Time pose_stamp(message->header.stamp);
      if (previous_body_pose_valid_) {
        const double dt = (pose_stamp - previous_body_pose_stamp_).seconds();
        if (dt >= 0.005 && dt <= 0.50) {
          const double velocity_x = (position.x - previous_body_x_) / dt;
          const double velocity_y = (position.y - previous_body_y_) / dt;
          const double derived_speed =
            std::cos(yaw) * velocity_x + std::sin(yaw) * velocity_y;
          const double yaw_delta = std::atan2(
            std::sin(yaw - previous_body_yaw_),
            std::cos(yaw - previous_body_yaw_));
          const double derived_yaw_rate = yaw_delta / dt;
          if (std::isfinite(derived_speed) && std::isfinite(derived_yaw_rate)) {
            const double alpha = body_valid_ ? pose_twist_filter_alpha_ : 1.0;
            body_speed_ = alpha * derived_speed + (1.0 - alpha) * body_speed_;
            body_yaw_rate_ =
              alpha * derived_yaw_rate + (1.0 - alpha) * body_yaw_rate_;
            body_valid_ = true;
            last_body_ = now();
          }
        }
      }
      previous_body_x_ = position.x;
      previous_body_y_ = position.y;
      previous_body_yaw_ = yaw;
      previous_body_pose_stamp_ = pose_stamp;
      previous_body_pose_valid_ = true;
      return;
    }
    body_speed_ = twist.linear.x;
    body_yaw_rate_ = twist.angular.z;
    body_valid_ = true;
    last_body_ = now();
  }

  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!std::isfinite(message->twist.twist.linear.x)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    wheel_odom_speed_ = message->twist.twist.linear.x;
    wheel_odom_valid_ = true;
    last_wheel_odom_ = now();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto & a = message->linear_acceleration;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    acceleration_norm_ = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    imu_valid_ = std::isfinite(acceleration_norm_);
    last_imu_ = now();
  }

  void motorCallback(
    std::size_t offset, const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->data.size() < 6) {
      return;
    }
    std::scoped_lock lock(mutex_);
    for (std::size_t channel = 0; channel < 2; ++channel) {
      const std::size_t index = offset + channel;
      const double rpm = message->data[channel] * feedback_signs_[index];
      const double current = message->data[channel + 2];
      if (std::isfinite(rpm) && std::isfinite(current)) {
        wheel_speed_[index] = rpm * 2.0 * M_PI * wheel_radius_ / 60.0;
        wheel_current_[index] = std::abs(current);
        wheel_valid_[index] = true;
      }
    }
    last_motor_ = now();
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->name.size() != message->velocity.size()) {
      return;
    }
    static const std::array<std::string, 4> tokens = {
      "front_left_wheel", "rear_left_wheel", "front_right_wheel", "rear_right_wheel"};
    std::array<double, 4> speeds{};
    std::array<bool, 4> found{};
    for (std::size_t item = 0; item < message->name.size(); ++item) {
      if (!std::isfinite(message->velocity[item])) {
        continue;
      }
      for (std::size_t wheel = 0; wheel < tokens.size(); ++wheel) {
        if (message->name[item].find(tokens[wheel]) != std::string::npos) {
          speeds[wheel] = message->velocity[item] * wheel_radius_;
          found[wheel] = true;
        }
      }
    }
    if (std::all_of(found.begin(), found.end(), [](bool value) {return value;})) {
      std::scoped_lock lock(mutex_);
      joint_wheel_speed_ = speeds;
      joint_valid_ = true;
      last_joint_ = now();
    }
  }

  bool fresh(const rclcpp::Time & stamp, double timeout, const rclcpp::Time & now_stamp) const
  {
    return stamp.nanoseconds() != 0 && (now_stamp - stamp).seconds() <= timeout;
  }

  void update()
  {
    std::scoped_lock lock(mutex_);
    const auto stamp = now();
    const bool body_fresh = body_valid_ && fresh(last_body_, body_timeout_, stamp);
    const bool wheel_odom_fresh = wheel_odom_valid_ &&
      fresh(last_wheel_odom_, wheel_timeout_, stamp);
    const bool imu_fresh = imu_valid_ && fresh(last_imu_, imu_timeout_, stamp);
    const bool motor_fresh = fresh(last_motor_, motor_timeout_, stamp) &&
      std::all_of(wheel_valid_.begin(), wheel_valid_.end(), [](bool value) {return value;});
    const bool joint_fresh = joint_valid_ && fresh(last_joint_, joint_timeout_, stamp);

    std::array<double, 4> speeds{};
    bool wheel_feedback_fresh = false;
    bool current_available = false;
    if (motor_fresh) {
      speeds = wheel_speed_;
      wheel_feedback_fresh = true;
      current_available = true;
    } else if (joint_fresh) {
      speeds = joint_wheel_speed_;
      wheel_feedback_fresh = true;
    }
    const double mean_wheel_speed = wheel_feedback_fresh ?
      std::accumulate(speeds.begin(), speeds.end(), 0.0,
      [](double sum, double value) {return sum + std::abs(value);}) / 4.0 :
      (wheel_odom_fresh ? std::abs(wheel_odom_speed_) : 0.0);
    const double body_abs = body_fresh ? std::abs(body_speed_) : 0.0;
    const bool moving = std::max(mean_wheel_speed, body_abs) >= moving_speed_min_;

    std::array<float, 4> probability{1.0F, 1.0F, 1.0F, 1.0F};
    int unloaded_count = 0;
    if (current_available && moving) {
      const std::array<double, 4> expected = {
        std::abs(body_speed_ - 0.5 * wheel_track_ * body_yaw_rate_),
        std::abs(body_speed_ - 0.5 * wheel_track_ * body_yaw_rate_),
        std::abs(body_speed_ + 0.5 * wheel_track_ * body_yaw_rate_),
        std::abs(body_speed_ + 0.5 * wheel_track_ * body_yaw_rate_)};
      for (std::size_t wheel = 0; wheel < 4; ++wheel) {
        const std::size_t pair = wheel % 2 == 0 ? wheel + 1 : wheel - 1;
        const bool relatively_unloaded =
          wheel_current_[pair] >= loaded_current_min_ &&
          wheel_current_[wheel] < 0.35 * wheel_current_[pair];
        const bool overspeed = std::abs(speeds[wheel]) >
          wheel_overspeed_ratio_ * expected[wheel] + wheel_overspeed_margin_;
        const bool low_current = wheel_current_[wheel] <= unloaded_current_max_;
        if (relatively_unloaded || (low_current && overspeed)) {
          probability[wheel] = 0.10F;
          ++unloaded_count;
        } else if (low_current) {
          probability[wheel] = 0.60F;
        }
      }
    }

    const bool airborne_signal = imu_fresh && acceleration_norm_ <= airborne_accel_max_;
    uint8_t desired = Contact::GROUNDED;
    std::string reason = "normal loaded-wheel evidence";
    if (!body_fresh || !imu_fresh || (!wheel_feedback_fresh && !wheel_odom_fresh)) {
      desired = Contact::UNKNOWN;
      reason = "body odom, IMU, or wheel feedback is stale";
    } else if (airborne_signal) {
      desired = Contact::AIRBORNE;
      reason = "IMU proper-acceleration norm indicates free flight";
      probability.fill(0.0F);
    } else if (state_ == Contact::AIRBORNE && acceleration_norm_ >= landing_accel_min_) {
      desired = Contact::LANDING;
      reason = "post-flight acceleration impulse";
      landing_until_ = stamp + rclcpp::Duration::from_seconds(landing_hold_);
    } else if (landing_until_.nanoseconds() != 0 && stamp < landing_until_) {
      desired = Contact::LANDING;
      reason = "landing torque-ramp window";
    } else if (unloaded_count > 0 && unloaded_count < 4) {
      desired = Contact::PARTIAL_CONTACT;
      reason = "one or more wheels show low-load/overspeed evidence";
    }

    if (desired != candidate_state_) {
      candidate_state_ = desired;
      candidate_since_ = stamp;
    }
    const double required_hold = desired == Contact::AIRBORNE ? airborne_hold_ :
      (desired == Contact::PARTIAL_CONTACT ? partial_hold_ : 0.02);
    if (desired != state_ && (stamp - candidate_since_).seconds() >= required_hold) {
      state_ = desired;
      state_since_ = stamp;
    }

    Contact message;
    message.header.stamp = stamp;
    message.header.frame_id = "base_link";
    message.state = state_;
    message.state_name = stateName(state_);
    message.wheel_names = {"left_mot1", "left_mot2", "right_mot1", "right_mot2"};
    message.wheel_contact_probability.assign(probability.begin(), probability.end());
    if (state_ == Contact::AIRBORNE) {
      message.estimated_contact_count = 0;
    } else if (state_ == Contact::GROUNDED) {
      message.estimated_contact_count = 4;
    } else {
      message.estimated_contact_count = static_cast<uint8_t>(std::count_if(
        probability.begin(), probability.end(), [](float value) {return value >= 0.5F;}));
    }
    message.confidence = current_available ? 0.90F :
      (wheel_feedback_fresh ? 0.70F : (wheel_odom_fresh ? 0.55F : 0.0F));
    if (state_ == Contact::UNKNOWN) {
      message.confidence = 0.0F;
    }
    message.wheel_speed_mps = static_cast<float>(mean_wheel_speed);
    message.body_speed_mps = static_cast<float>(body_abs);
    message.slip_ratio = static_cast<float>(
      mean_wheel_speed >= moving_speed_min_ ? body_abs / mean_wheel_speed : 1.0);
    message.acceleration_norm_mps2 = static_cast<float>(acceleration_norm_);
    message.wheel_feedback_fresh = wheel_feedback_fresh;
    message.wheel_odom_fresh = wheel_odom_fresh;
    message.body_odom_fresh = body_fresh;
    message.imu_fresh = imu_fresh;
    message.reason = reason;
    contact_pub_->publish(message);
    publishDiagnostics(message);
  }

  void publishDiagnostics(const Contact & message)
  {
    if (++diagnostic_tick_ % 25 != 0) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = message.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "terrain_supervisor/wheel_contact";
    status.hardware_id = "roka_four_wheel_base";
    status.level = message.state == Contact::UNKNOWN ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = message.state_name + ": " + message.reason;
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    add("estimated_contact_count", std::to_string(message.estimated_contact_count));
    add("confidence", std::to_string(message.confidence));
    add("wheel_speed_mps", std::to_string(message.wheel_speed_mps));
    add("body_speed_mps", std::to_string(message.body_speed_mps));
    add("slip_ratio", std::to_string(message.slip_ratio));
    add("acceleration_norm_mps2", std::to_string(message.acceleration_norm_mps2));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string body_odom_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string joint_state_topic_;
  std::string output_topic_;
  double wheel_radius_;
  double wheel_track_;
  bool derive_body_twist_from_pose_;
  double pose_twist_filter_alpha_;
  double body_timeout_;
  double wheel_timeout_;
  double imu_timeout_;
  double motor_timeout_;
  double joint_timeout_;
  double airborne_accel_max_;
  double landing_accel_min_;
  double airborne_hold_;
  double partial_hold_;
  double landing_hold_;
  double moving_speed_min_;
  double wheel_overspeed_ratio_;
  double wheel_overspeed_margin_;
  double unloaded_current_max_;
  double loaded_current_min_;
  std::array<int64_t, 4> feedback_signs_{};
  std::mutex mutex_;
  double body_speed_{0.0};
  double body_yaw_rate_{0.0};
  double wheel_odom_speed_{0.0};
  double acceleration_norm_{9.80665};
  bool body_valid_{false};
  bool wheel_odom_valid_{false};
  bool imu_valid_{false};
  bool joint_valid_{false};
  bool previous_body_pose_valid_{false};
  double previous_body_x_{0.0};
  double previous_body_y_{0.0};
  double previous_body_yaw_{0.0};
  rclcpp::Time previous_body_pose_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 4> wheel_speed_{};
  std::array<double, 4> wheel_current_{};
  std::array<double, 4> joint_wheel_speed_{};
  std::array<bool, 4> wheel_valid_{};
  uint8_t state_{Contact::UNKNOWN};
  uint8_t candidate_state_{Contact::UNKNOWN};
  rclcpp::Time last_body_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motor_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_{0, 0, RCL_ROS_TIME};
  rclcpp::Time candidate_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time landing_until_{0, 0, RCL_ROS_TIME};
  int diagnostic_tick_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_motor_sub_;
  rclcpp::Publisher<Contact>::SharedPtr contact_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roka_terrain_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roka_terrain_supervisor::WheelContactEstimator>());
  rclcpp::shutdown();
  return 0;
}
