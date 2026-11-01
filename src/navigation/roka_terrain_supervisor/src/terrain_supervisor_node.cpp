#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "roka_interfaces/msg/terrain_state.hpp"
#include "roka_interfaces/msg/wheel_contact_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "roka_terrain_supervisor/supervisor_math.hpp"

namespace roka_terrain_supervisor
{

class TerrainSupervisor : public rclcpp::Node
{
public:
  TerrainSupervisor()
  : Node("terrain_supervisor"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_cmd_topic_ = declare_parameter("input_cmd_topic", "/cmd_vel_mux");
    output_cmd_topic_ = declare_parameter("output_cmd_topic", "/cmd_vel_terrain");
    terrain_topic_ = declare_parameter("terrain_topic", "/terrain/local_map");
    imu_topic_ = declare_parameter("imu_topic", "/imu/data");
    const std::string legacy_odom_topic = declare_parameter("odom_topic", "/odom");
    body_odom_topic_ = declare_parameter("body_odom_topic", legacy_odom_topic);
    wheel_odom_topic_ = declare_parameter("wheel_odom_topic", "/wheel/odom");
    contact_state_topic_ = declare_parameter("contact_state_topic", "/terrain/wheel_contact");
    track_speed_topic_ = declare_parameter("track_speed_topic", "/track/reference_speed");
    base_frame_ = declare_parameter("base_frame", "base_link");
    const double rate = declare_parameter("publish_rate_hz", 30.0);
    command_timeout_ = declare_parameter("command_timeout_sec", 0.35);
    terrain_timeout_ = declare_parameter("terrain_timeout_sec", 0.40);
    imu_timeout_ = declare_parameter("imu_timeout_sec", 0.25);
    odom_timeout_ = declare_parameter("odom_timeout_sec", 0.35);
    contact_timeout_ = declare_parameter("contact_state_timeout_sec", 0.25);
    require_terrain_ = declare_parameter("require_terrain", true);
    allow_level_fallback_ = declare_parameter("allow_level_fallback", false);
    enable_terrain_decisions_ = declare_parameter("enable_terrain_decisions", true);
    require_imu_ = declare_parameter("require_imu", false);
    require_odom_ = declare_parameter("require_odom", false);
    enable_slip_ = declare_parameter("enable_slip_detection", false);
    use_track_speed_limit_ = declare_parameter("use_track_speed_limit", true);
    diagnostics_observe_only_ = declare_parameter("diagnostics_observe_only", false);
    diagnostic_rate_hz_ = declare_parameter("diagnostic_rate_hz", 1.0);
    track_speed_fail_closed_ = declare_parameter("track_speed_fail_closed", true);
    track_speed_timeout_ = declare_parameter("track_speed_timeout_sec", 0.50);
    stale_track_speed_limit_ = declare_parameter("stale_track_speed_limit_mps", 0.10);
    max_linear_ = declare_parameter("max_linear_speed", 0.60);
    max_angular_ = declare_parameter("max_angular_speed", 0.80);
    warning_ratio_ = declare_parameter("warning_ratio", 0.80);
    critical_ratio_ = declare_parameter("critical_ratio", 0.90);
    hard_ratio_ = declare_parameter("hard_ratio", 1.00);
    state_hold_ = declare_parameter("state_hold_sec", 0.20);
    recovery_hold_ = declare_parameter("recovery_hold_sec", 0.50);
    uphill_enter_ = declare_parameter("uphill_enter_deg", 8.0);
    downhill_enter_ = declare_parameter("downhill_enter_deg", 8.0);
    cross_enter_ = declare_parameter("cross_slope_enter_deg", 8.0);
    uphill_scale_ = declare_parameter("uphill_speed_scale", 1.0);
    downhill_scale_ = declare_parameter("downhill_speed_scale", 1.0);
    cross_slope_scale_ = declare_parameter("cross_slope_speed_scale", 1.0);
    slip_scale_ = declare_parameter("slip_speed_scale", 0.40);
    roll_warning_ = declare_parameter("roll_warning_deg", 10.5);
    roll_critical_ = declare_parameter("roll_critical_deg", 26.0);
    roll_hard_ = declare_parameter("roll_hard_deg", 41.0);
    pitch_warning_ = declare_parameter("pitch_warning_deg", 15.0);
    pitch_critical_ = declare_parameter("pitch_critical_deg", 25.5);
    pitch_hard_ = declare_parameter("pitch_hard_deg", 35.5);
    absolute_roll_hard_ = declare_parameter("absolute_roll_hard_deg", 85.0);
    absolute_pitch_hard_ = declare_parameter("absolute_pitch_hard_deg", 65.0);
    attitude_prediction_horizon_ = declare_parameter("attitude_prediction_horizon_sec", 0.20);
    attitude_hard_hold_ = declare_parameter("attitude_hard_hold_sec", 0.08);
    grounded_acceleration_ = declare_parameter("grounded_acceleration_mps2", 0.50);
    partial_acceleration_ = declare_parameter("partial_contact_acceleration_mps2", 0.30);
    airborne_acceleration_ = declare_parameter("airborne_acceleration_mps2", 0.10);
    landing_acceleration_ = declare_parameter("landing_acceleration_mps2", 0.25);
    normal_deceleration_ = declare_parameter("normal_deceleration_mps2", 0.80);
    airborne_deceleration_ = declare_parameter("airborne_deceleration_mps2", 0.15);
    angular_acceleration_ = declare_parameter("angular_acceleration_radps2", 1.50);
    angular_deceleration_ = declare_parameter("angular_deceleration_radps2", 2.00);
    footprint_length_ = declare_parameter("footprint_length", 0.88);
    footprint_width_ = declare_parameter("footprint_width", 0.624);
    lookahead_ = declare_parameter("lookahead_distance", 1.00);
    sampling_mode_ = declare_parameter("terrain_sampling_mode", "wheel_corridors");
    wheel_track_ = declare_parameter("wheel_track", 0.427149994);
    wheel_corridor_half_width_ = declare_parameter("wheel_corridor_half_width", 0.08);
    minimum_valid_cells_ = declare_parameter("minimum_valid_cells", 4);
    slip_command_min_ = declare_parameter("slip_command_min_mps", 0.15);
    slip_velocity_ratio_ = declare_parameter("slip_velocity_ratio", 0.35);
    slip_hold_ = declare_parameter("slip_hold_sec", 0.80);

    if (rate <= 0.0 || command_timeout_ <= 0.0 || terrain_timeout_ <= 0.0 ||
      max_linear_ <= 0.0 || max_angular_ <= 0.0 || warning_ratio_ <= 0.0 ||
      !(warning_ratio_ < critical_ratio_ && critical_ratio_ < hard_ratio_) ||
      !(roll_warning_ < roll_critical_ && roll_critical_ < roll_hard_) ||
      !(pitch_warning_ < pitch_critical_ && pitch_critical_ < pitch_hard_) ||
      std::min({uphill_scale_, downhill_scale_, cross_slope_scale_, slip_scale_}) <= 0.0 ||
      std::max({uphill_scale_, downhill_scale_, cross_slope_scale_, slip_scale_}) > 1.0 ||
      absolute_roll_hard_ <= roll_hard_ || absolute_pitch_hard_ <= pitch_hard_ ||
      attitude_prediction_horizon_ < 0.0 || attitude_hard_hold_ <= 0.0 ||
      std::min({grounded_acceleration_, partial_acceleration_, airborne_acceleration_,
        landing_acceleration_, normal_deceleration_, airborne_deceleration_,
        angular_acceleration_, angular_deceleration_}) <= 0.0 ||
      wheel_track_ <= 0.0 || wheel_corridor_half_width_ <= 0.0 || minimum_valid_cells_ < 1 ||
      (require_terrain_ && allow_level_fallback_) ||
      (sampling_mode_ != "wheel_corridors" && sampling_mode_ != "full_footprint") ||
      diagnostic_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("invalid supervisor rate, timeout, speed, or risk thresholds");
    }

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      std::bind(&TerrainSupervisor::commandCallback, this, std::placeholders::_1));
    terrain_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&TerrainSupervisor::terrainCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainSupervisor::imuCallback, this, std::placeholders::_1));
    body_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      body_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainSupervisor::bodyOdomCallback, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainSupervisor::wheelOdomCallback, this, std::placeholders::_1));
    contact_sub_ = create_subscription<roka_interfaces::msg::WheelContactState>(
      contact_state_topic_, 10,
      std::bind(&TerrainSupervisor::contactCallback, this, std::placeholders::_1));
    track_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      track_speed_topic_, 10,
      std::bind(&TerrainSupervisor::trackSpeedCallback, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
    state_pub_ = create_publisher<roka_interfaces::msg::TerrainState>("/terrain/state", 10);
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("/terrain/speed_limit", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&TerrainSupervisor::update, this));

    state_since_ = now();
    candidate_since_ = now();
    RCLCPP_INFO(
      get_logger(),
      "terrain supervisor: %s -> %s; terrain=%s; sampling=%s; 80/90/100 enabled",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str(), terrain_topic_.c_str(),
      sampling_mode_.c_str());
  }

private:
  struct TerrainMetrics
  {
    bool valid{false};
    double risk{0.0};
    double longitudinal_deg{0.0};
    double lateral_deg{0.0};
    double roughness{0.0};
    double step{0.0};
    int valid_cells{0};
  };

  static bool finiteCommand(const geometry_msgs::msg::Twist & command)
  {
    return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
           std::isfinite(command.angular.z);
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::scoped_lock lock(mutex_);
    command_ = *message;
    command_valid_ = finiteCommand(*message);
    last_command_ = now();
  }

  void terrainCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
  {
    grid_map::GridMap map;
    if (!grid_map::GridMapRosConverter::fromMessage(*message, map)) {
      const std::string warning = "invalid GridMap message";
      if (terrain_input_warning_ != warning) {
        RCLCPP_WARN(get_logger(), "%s", warning.c_str());
        terrain_input_warning_ = warning;
      }
      return;
    }
    for (const auto & required :
      {"risk", "normal_x", "normal_y", "normal_z", "roughness", "step_height"})
    {
      if (!map.exists(required)) {
        const std::string warning = std::string("terrain map missing layer '") + required + "'";
        if (terrain_input_warning_ != warning) {
          RCLCPP_WARN(get_logger(), "%s", warning.c_str());
          terrain_input_warning_ = warning;
        }
        return;
      }
    }
    terrain_input_warning_.clear();

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        // The GridMap publisher refreshes its header at publish time; the
        // newest body TF can legitimately be a few milliseconds older.
        map.getFrameId(), base_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(0.05));
      terrain_tf_warning_active_ = false;
    } catch (const tf2::TransformException & error) {
      if (!terrain_tf_warning_active_) {
        RCLCPP_WARN(get_logger(), "terrain sampling TF unavailable: %s", error.what());
        terrain_tf_warning_active_ = true;
      }
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double bx = transform.transform.translation.x;
    const double by = transform.transform.translation.y;

    geometry_msgs::msg::Twist command_snapshot;
    {
      std::scoped_lock lock(mutex_);
      command_snapshot = command_;
    }
    const double curvature = std::abs(command_snapshot.linear.x) > 0.05 ?
      std::clamp(command_snapshot.angular.z / command_snapshot.linear.x, -2.0, 2.0) : 0.0;

    TerrainMetrics metrics;
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const auto index = *iterator;
      if (!map.isValid(index, "risk")) {
        continue;
      }
      grid_map::Position position;
      map.getPosition(index, position);
      const double dx = position.x() - bx;
      const double dy = position.y() - by;
      const double forward = cos_yaw * dx + sin_yaw * dy;
      const double lateral = -sin_yaw * dx + cos_yaw * dy;
      if (forward < -0.5 * footprint_length_ || forward > lookahead_) {
        continue;
      }
      if (sampling_mode_ == "full_footprint") {
        if (std::abs(lateral) > 0.5 * footprint_width_) {
          continue;
        }
      } else {
        // Follow the two predicted wheel lanes instead of taking the maximum
        // over the whole banked module. The quadratic term is the local
        // approximation of the commanded skid-steer arc.
        const double path_lateral = 0.5 * curvature * forward * forward;
        const double left_lane = path_lateral + 0.5 * wheel_track_;
        const double right_lane = path_lateral - 0.5 * wheel_track_;
        const double lane_distance = std::min(
          std::abs(lateral - left_lane), std::abs(lateral - right_lane));
        if (lane_distance > wheel_corridor_half_width_) {
          continue;
        }
      }
      const double nx = map.at("normal_x", index);
      const double ny = map.at("normal_y", index);
      const double nz = map.at("normal_z", index);
      if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz) || std::abs(nz) < 1e-5) {
        continue;
      }
      const double gx = -nx / nz;
      const double gy = -ny / nz;
      const double long_gradient = gx * cos_yaw + gy * sin_yaw;
      const double lateral_gradient = -gx * sin_yaw + gy * cos_yaw;
      const double longitudinal = std::atan(long_gradient) * 180.0 / M_PI;
      const double lateral_slope = std::atan(lateral_gradient) * 180.0 / M_PI;
      if (std::abs(longitudinal) > std::abs(metrics.longitudinal_deg)) {
        metrics.longitudinal_deg = longitudinal;
      }
      if (std::abs(lateral_slope) > std::abs(metrics.lateral_deg)) {
        metrics.lateral_deg = lateral_slope;
      }
      metrics.risk = std::max(metrics.risk, static_cast<double>(map.at("risk", index)));
      metrics.roughness = std::max(
        metrics.roughness, static_cast<double>(map.at("roughness", index)));
      metrics.step = std::max(metrics.step, static_cast<double>(map.at("step_height", index)));
      ++metrics.valid_cells;
    }
    metrics.valid = metrics.valid_cells >= minimum_valid_cells_;
    const auto map_timestamp = map.getTimestamp();
    const auto terrain_stamp = map_timestamp > 0U ?
      rclcpp::Time(
      static_cast<int64_t>(map_timestamp), get_clock()->get_clock_type()) : now();
    std::scoped_lock lock(mutex_);
    terrain_ = metrics;
    last_terrain_ = terrain_stamp;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    // sensor_msgs uses covariance[0] < 0 to mark orientation unavailable.
    // Never turn an intentionally unvalidated EBIMU mounting into a false
    // body-tilt stop.  Once commissioning supplies covariance, the same topic
    // becomes active without changing this node.
    if (message->orientation_covariance[0] < 0.0) {
      std::scoped_lock lock(mutex_);
      imu_valid_ = false;
      return;
    }
    const auto & orientation = message->orientation;
    if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      return;
    }
    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)yaw;
    std::scoped_lock lock(mutex_);
    imu_roll_deg_ = roll * 180.0 / M_PI;
    imu_pitch_deg_ = pitch * 180.0 / M_PI;
    imu_roll_rate_radps_ = message->angular_velocity.x;
    imu_pitch_rate_radps_ = message->angular_velocity.y;
    imu_acceleration_norm_ = std::sqrt(
      message->linear_acceleration.x * message->linear_acceleration.x +
      message->linear_acceleration.y * message->linear_acceleration.y +
      message->linear_acceleration.z * message->linear_acceleration.z);
    imu_valid_ = std::isfinite(imu_roll_deg_) && std::isfinite(imu_pitch_deg_) &&
      std::isfinite(imu_roll_rate_radps_) && std::isfinite(imu_pitch_rate_radps_) &&
      std::isfinite(imu_acceleration_norm_);
    last_imu_ = now();
  }

  void bodyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!std::isfinite(message->twist.twist.linear.x) ||
      !std::isfinite(message->twist.twist.angular.z))
    {
      return;
    }
    std::scoped_lock lock(mutex_);
    body_speed_ = message->twist.twist.linear.x;
    body_yaw_rate_ = message->twist.twist.angular.z;
    body_odom_valid_ = true;
    last_body_odom_ = now();
  }

  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!std::isfinite(message->twist.twist.linear.x)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    wheel_speed_ = message->twist.twist.linear.x;
    wheel_odom_valid_ = true;
    last_wheel_odom_ = now();
  }

  void contactCallback(const roka_interfaces::msg::WheelContactState::SharedPtr message)
  {
    std::scoped_lock lock(mutex_);
    contact_state_ = message->state;
    contact_count_ = message->estimated_contact_count;
    contact_confidence_ = message->confidence;
    contact_valid_ = true;
    last_contact_ = now();
  }

  void trackSpeedCallback(const std_msgs::msg::Float32::SharedPtr message)
  {
    if (!std::isfinite(message->data) || message->data <= 0.0F) {
      return;
    }
    std::scoped_lock lock(mutex_);
    track_speed_limit_ = std::clamp(static_cast<double>(message->data), 0.05, max_linear_);
    last_track_speed_ = now();
  }

  uint8_t classify(const TerrainMetrics & terrain, bool slip) const
  {
    using State = roka_interfaces::msg::TerrainState;
    if (slip) {
      return State::SLIP;
    }
    if (std::abs(terrain.lateral_deg) >= cross_enter_) {
      return State::CROSS_SLOPE;
    }
    if (terrain.longitudinal_deg >= uphill_enter_) {
      return State::UPHILL;
    }
    if (terrain.longitudinal_deg <= -downhill_enter_) {
      return State::DOWNHILL;
    }
    return State::NORMAL;
  }

  static std::string stateName(uint8_t state)
  {
    using State = roka_interfaces::msg::TerrainState;
    switch (state) {
      case State::NORMAL: return "NORMAL";
      case State::UPHILL: return "UPHILL";
      case State::DOWNHILL: return "DOWNHILL";
      case State::CROSS_SLOPE: return "CROSS_SLOPE";
      case State::SLIP: return "SLIP";
      case State::FAULT: return "FAULT";
      default: return "UNKNOWN";
    }
  }

  void update()
  {
    std::scoped_lock lock(mutex_);
    const auto stamp = now();
    const bool command_fresh = last_command_.nanoseconds() != 0 &&
      (stamp - last_command_).seconds() <= command_timeout_;
    const bool terrain_input_fresh = last_terrain_.nanoseconds() != 0 &&
      (stamp - last_terrain_).seconds() <= terrain_timeout_;
    const bool terrain_fresh = terrain_.valid && terrain_input_fresh;
    const bool imu_fresh = imu_valid_ && last_imu_.nanoseconds() != 0 &&
      (stamp - last_imu_).seconds() <= imu_timeout_;
    const bool body_odom_fresh = body_odom_valid_ && last_body_odom_.nanoseconds() != 0 &&
      (stamp - last_body_odom_).seconds() <= odom_timeout_;
    const bool wheel_odom_fresh = wheel_odom_valid_ && last_wheel_odom_.nanoseconds() != 0 &&
      (stamp - last_wheel_odom_).seconds() <= odom_timeout_;
    const bool contact_fresh = contact_valid_ && last_contact_.nanoseconds() != 0 &&
      (stamp - last_contact_).seconds() <= contact_timeout_;
    const bool track_speed_fresh = use_track_speed_limit_ &&
      last_track_speed_.nanoseconds() != 0 &&
      (stamp - last_track_speed_).seconds() <= track_speed_timeout_;

    const double relative_roll_deg = imu_roll_deg_ -
      (terrain_fresh ? terrain_.lateral_deg : 0.0);
    // A positive MuJoCo/EBIMU pitch aligns with a downhill (negative)
    // longitudinal surface gradient, hence the addition here.
    const double relative_pitch_deg = imu_pitch_deg_ +
      (terrain_fresh ? terrain_.longitudinal_deg : 0.0);
    const double roll_load_deg = dynamicRollLoadDeg(
      relative_roll_deg, imu_roll_rate_radps_, body_speed_, body_yaw_rate_,
      attitude_prediction_horizon_);
    const double pitch_load_deg = projectedAbsoluteAngleDeg(
      relative_pitch_deg, imu_pitch_rate_radps_, attitude_prediction_horizon_);
    last_relative_roll_deg_ = relative_roll_deg;
    last_relative_pitch_deg_ = relative_pitch_deg;
    last_roll_load_deg_ = roll_load_deg;
    last_pitch_load_deg_ = pitch_load_deg;
    last_contact_fresh_ = contact_fresh;
    last_wheel_odom_fresh_ = wheel_odom_fresh;
    last_track_speed_fresh_ = track_speed_fresh;
    const bool gross_attitude = imu_fresh &&
      (std::abs(imu_roll_deg_) >= absolute_roll_hard_ ||
      std::abs(imu_pitch_deg_) >= absolute_pitch_hard_);
    const bool attitude_hard = imu_fresh &&
      (roll_load_deg >= roll_hard_ || pitch_load_deg >= pitch_hard_ || gross_attitude);
    if (attitude_hard) {
      if (attitude_hard_since_.nanoseconds() == 0) {
        attitude_hard_since_ = stamp;
      }
    } else {
      attitude_hard_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    const bool attitude_hard_persistent = attitude_hard &&
      (stamp - attitude_hard_since_).seconds() >= attitude_hard_hold_;

    bool fault = false;
    std::string reason = "ok";
    const bool stale_motion_command = !command_fresh &&
      (std::abs(command_.linear.x) > 1.0e-4 || std::abs(command_.angular.z) > 1.0e-4);
    if (stale_motion_command) {
      fault = true;
      reason = "non-zero command timeout";
    } else if (!command_valid_) {
      fault = true;
      reason = "non-finite command";
    } else if (enable_terrain_decisions_ && require_terrain_ && !terrain_fresh) {
      fault = true;
      reason = "terrain map stale or invalid";
    } else if (
      enable_terrain_decisions_ && allow_level_fallback_ && !terrain_input_fresh)
    {
      fault = true;
      reason = "terrain input stale during controlled level fallback";
    } else if (require_imu_ && !imu_fresh) {
      fault = true;
      reason = "IMU stale or invalid";
    } else if (require_odom_ && !body_odom_fresh) {
      fault = true;
      reason = "body odometry stale or invalid";
    } else if (
      enable_terrain_decisions_ && terrain_fresh && terrain_.risk >= hard_ratio_)
    {
      fault = true;
      reason = "terrain risk exceeds 100% limit";
    } else if (attitude_hard_persistent) {
      fault = true;
      reason = roll_load_deg >= roll_hard_ ?
        "data-derived roll-over guard exceeded" : "pitch-over guard exceeded";
    } else if (!command_fresh) {
      // Nav2 stops publishing after a completed FollowPath. A stale zero (or
      // no command yet) is a safe idle state, not a hardware/safety fault.
      reason = "idle: no active motion command";
    }

    const double active_linear_limit = track_speed_fresh ?
      std::min(max_linear_, track_speed_limit_) :
      (use_track_speed_limit_ && track_speed_fail_closed_ ?
      std::min(max_linear_, stale_track_speed_limit_) : max_linear_);
    // Compare wheel motion with the command that can actually leave this
    // supervisor. Nav2 may ask for more than the mapped 0.30 m/s ceiling;
    // comparing odometry with the uncapped request
    // falsely declared persistent slip and reduced the valid command again.
    const double slip_reference_speed = std::min(
      std::abs(command_.linear.x), active_linear_limit);
    bool slip = false;
    const double slip_denominator = wheel_odom_fresh &&
      std::abs(wheel_speed_) >= slip_command_min_ ? std::abs(wheel_speed_) :
      slip_reference_speed;
    if (!fault && enable_slip_ && body_odom_fresh &&
      slip_denominator >= slip_command_min_ &&
      std::abs(body_speed_) < slip_velocity_ratio_ * slip_denominator)
    {
      if (slip_since_.nanoseconds() == 0) {
        slip_since_ = stamp;
      }
      slip = (stamp - slip_since_).seconds() >= slip_hold_;
    } else {
      slip_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    using State = roka_interfaces::msg::TerrainState;
    const uint8_t desired = fault ? State::FAULT :
      (enable_terrain_decisions_ ? classify(terrain_, slip) :
      (slip ? State::SLIP : State::NORMAL));
    if (desired == State::FAULT) {
      state_ = desired;
      candidate_state_ = desired;
      state_since_ = stamp;
      candidate_since_ = stamp;
    } else if (desired != state_) {
      if (desired != candidate_state_) {
        candidate_state_ = desired;
        candidate_since_ = stamp;
      }
      const double hold = desired == State::NORMAL ? recovery_hold_ : state_hold_;
      if ((stamp - candidate_since_).seconds() >= hold) {
        state_ = desired;
        state_since_ = stamp;
      }
    } else {
      candidate_state_ = desired;
      candidate_since_ = stamp;
    }

    if (!fault && state_ == State::SLIP) {
      reason = "wheel/body odometry slip: non-learning speed derate";
    } else if (!fault && !enable_terrain_decisions_) {
      reason = "recorded-route mode: terrain and obstacle decisions bypassed";
    } else if (!fault && !terrain_fresh && allow_level_fallback_) {
      reason = "controlled level fallback: wheel-corridor terrain cells unavailable";
    } else if (!fault && use_track_speed_limit_ && track_speed_fail_closed_ &&
      !track_speed_fresh)
    {
      reason = "track speed profile stale: fail-closed crawl limit";
    }
    const double risk = enable_terrain_decisions_ ? effectiveTerrainRisk(
      terrain_fresh, allow_level_fallback_, terrain_.risk, hard_ratio_) : 0.0;
    double scale = fault ? 0.0 : speedScaleFromRisk(
      risk, warning_ratio_, critical_ratio_, hard_ratio_);
    double attitude_scale = 1.0;
    if (imu_fresh) {
      attitude_scale = std::min(
        attitudeScale(roll_load_deg, roll_warning_, roll_critical_, roll_hard_),
        attitudeScale(pitch_load_deg, pitch_warning_, pitch_critical_, pitch_hard_));
      // During the short persistence window retain controllability instead of
      // issuing a one-sample zero command on a pump crest.
      if (attitude_hard && !attitude_hard_persistent) {
        attitude_scale = std::max(attitude_scale, 0.30);
      }
      scale = std::min(scale, attitude_scale);
      if (!fault && attitude_scale < 1.0) {
        reason = "attitude envelope derate";
      }
    }
    if (state_ == State::UPHILL) {
      scale = std::min(scale, uphill_scale_);
    } else if (state_ == State::DOWNHILL) {
      scale = std::min(scale, downhill_scale_);
    } else if (state_ == State::CROSS_SLOPE) {
      scale = std::min(scale, cross_slope_scale_);
    } else if (state_ == State::SLIP) {
      scale = std::min(scale, slip_scale_);
    } else if (state_ == State::FAULT) {
      scale = 0.0;
    }

    geometry_msgs::msg::Twist target;
    if (scale > 0.0 && command_fresh && command_valid_) {
      const double requested_linear = clampFinite(command_.linear.x, max_linear_);
      const double track_scale = std::abs(requested_linear) > active_linear_limit ?
        active_linear_limit / std::abs(requested_linear) : 1.0;
      // Scale yaw together with linear speed so the RPP curvature is preserved.
      target.linear.x = requested_linear * track_scale * scale;
      target.angular.z = clampFinite(command_.angular.z, max_angular_) * track_scale * scale;
    }
    using Contact = roka_interfaces::msg::WheelContactState;
    const uint8_t active_contact = contact_fresh ? contact_state_ : Contact::UNKNOWN;
    double linear_acceleration = grounded_acceleration_;
    double linear_deceleration = normal_deceleration_;
    if (active_contact == Contact::PARTIAL_CONTACT) {
      // A lifted wheel is normal on the track. Preserve speed and curvature;
      // only prevent a fresh torque step while load is asymmetric.
      linear_acceleration = partial_acceleration_;
    } else if (active_contact == Contact::AIRBORNE) {
      linear_acceleration = airborne_acceleration_;
      linear_deceleration = airborne_deceleration_;
      if (body_odom_fresh && std::abs(target.linear.x) > 1.0e-4) {
        const double direction = std::copysign(1.0, target.linear.x);
        const double landing_speed = std::max(0.05, std::abs(body_speed_));
        target.linear.x = direction * std::min(std::abs(target.linear.x), landing_speed);
      }
    } else if (active_contact == Contact::LANDING) {
      linear_acceleration = landing_acceleration_;
    }

    geometry_msgs::msg::Twist output;
    const double dt = last_output_stamp_.nanoseconds() == 0 ? 1.0 / 30.0 :
      std::clamp((stamp - last_output_stamp_).seconds(), 1.0e-3, 0.20);
    if (!fault) {
      output.linear.x = slewLimited(
        last_output_.linear.x, target.linear.x, linear_acceleration, linear_deceleration, dt);
      output.angular.z = slewLimited(
        last_output_.angular.z, target.angular.z,
        angular_acceleration_, angular_deceleration_, dt);
    }
    last_output_ = output;
    last_output_stamp_ = stamp;
    command_pub_->publish(output);

    std_msgs::msg::Float32 speed;
    speed.data = static_cast<float>(active_linear_limit * scale);
    speed_pub_->publish(speed);

    roka_interfaces::msg::TerrainState state;
    state.header.stamp = stamp;
    state.header.frame_id = base_frame_;
    state.state = state_;
    state.state_name = stateName(state_);
    state.speed_scale = static_cast<float>(scale);
    state.risk = static_cast<float>(risk);
    state.longitudinal_slope_deg = static_cast<float>(terrain_.longitudinal_deg);
    state.lateral_slope_deg = static_cast<float>(terrain_.lateral_deg);
    state.roughness_m = static_cast<float>(terrain_.roughness);
    state.step_height_m = static_cast<float>(terrain_.step);
    state.terrain_fresh = terrain_fresh;
    state.imu_fresh = imu_fresh;
    state.odom_fresh = body_odom_fresh;
    state.reason = reason;
    state_pub_->publish(state);
    publishDiagnostics(state);
  }

  void publishDiagnostics(const roka_interfaces::msg::TerrainState & state)
  {
    const std::string signature = state.state_name + ":" + state.reason;
    const auto steady_now = std::chrono::steady_clock::now();
    if (
      !last_diagnostic_signature_.empty() && signature == last_diagnostic_signature_ &&
      std::chrono::duration<double>(steady_now - last_diagnostic_publish_).count() <
      1.0 / diagnostic_rate_hz_)
    {
      return;
    }
    last_diagnostic_signature_ = signature;
    last_diagnostic_publish_ = steady_now;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = state.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = diagnostics_observe_only_ ?
      "terrain_mapping/monitor" : "terrain_supervisor/safety";
    status.hardware_id = "roka_base";
    if (diagnostics_observe_only_) {
      status.level = state.state == roka_interfaces::msg::TerrainState::NORMAL ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "OBSERVE_ONLY " + state.state_name + ": " + state.reason;
    } else {
      status.level = state.state == roka_interfaces::msg::TerrainState::FAULT ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        (state.speed_scale < 1.0F ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK);
      status.message = state.state_name + ": " + state.reason;
    }
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    add("risk", std::to_string(state.risk));
    add("speed_scale", std::to_string(state.speed_scale));
    add("terrain_fresh", state.terrain_fresh ? "true" : "false");
    add(
      "level_fallback_active",
      allow_level_fallback_ && !state.terrain_fresh ? "true" : "false");
    add("imu_fresh", state.imu_fresh ? "true" : "false");
    add("body_odom_fresh", state.odom_fresh ? "true" : "false");
    add("wheel_odom_fresh", last_wheel_odom_fresh_ ? "true" : "false");
    add("contact_state_fresh", last_contact_fresh_ ? "true" : "false");
    add("contact_state", std::to_string(contact_state_));
    add("estimated_contact_count", std::to_string(contact_count_));
    add("contact_confidence", std::to_string(contact_confidence_));
    add("relative_roll_deg", std::to_string(last_relative_roll_deg_));
    add("relative_pitch_deg", std::to_string(last_relative_pitch_deg_));
    add("dynamic_roll_load_deg", std::to_string(last_roll_load_deg_));
    add("dynamic_pitch_load_deg", std::to_string(last_pitch_load_deg_));
    add("track_speed_fresh", last_track_speed_fresh_ ? "true" : "false");
    add("track_speed_limit", std::to_string(track_speed_limit_));
    add("terrain_valid_cells", std::to_string(terrain_.valid_cells));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  std::string terrain_topic_;
  std::string imu_topic_;
  std::string body_odom_topic_;
  std::string wheel_odom_topic_;
  std::string contact_state_topic_;
  std::string track_speed_topic_;
  std::string base_frame_;
  double command_timeout_;
  double terrain_timeout_;
  double imu_timeout_;
  double odom_timeout_;
  double contact_timeout_;
  bool require_terrain_;
  bool allow_level_fallback_;
  bool enable_terrain_decisions_;
  bool require_imu_;
  bool require_odom_;
  bool enable_slip_;
  bool use_track_speed_limit_;
  bool diagnostics_observe_only_;
  double diagnostic_rate_hz_;
  bool track_speed_fail_closed_;
  double track_speed_timeout_;
  double stale_track_speed_limit_;
  double max_linear_;
  double max_angular_;
  double warning_ratio_;
  double critical_ratio_;
  double hard_ratio_;
  double state_hold_;
  double recovery_hold_;
  double uphill_enter_;
  double downhill_enter_;
  double cross_enter_;
  double uphill_scale_;
  double downhill_scale_;
  double cross_slope_scale_;
  double slip_scale_;
  double roll_warning_;
  double roll_critical_;
  double roll_hard_;
  double pitch_warning_;
  double pitch_critical_;
  double pitch_hard_;
  double absolute_roll_hard_;
  double absolute_pitch_hard_;
  double attitude_prediction_horizon_;
  double attitude_hard_hold_;
  double grounded_acceleration_;
  double partial_acceleration_;
  double airborne_acceleration_;
  double landing_acceleration_;
  double normal_deceleration_;
  double airborne_deceleration_;
  double angular_acceleration_;
  double angular_deceleration_;
  double footprint_length_;
  double footprint_width_;
  double lookahead_;
  std::string sampling_mode_;
  double wheel_track_;
  double wheel_corridor_half_width_;
  int minimum_valid_cells_;
  double slip_command_min_;
  double slip_velocity_ratio_;
  double slip_hold_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist command_;
  TerrainMetrics terrain_;
  bool command_valid_{false};
  bool imu_valid_{false};
  bool body_odom_valid_{false};
  bool wheel_odom_valid_{false};
  bool contact_valid_{false};
  bool terrain_tf_warning_active_{false};
  std::string terrain_input_warning_;
  double imu_roll_deg_{0.0};
  double imu_pitch_deg_{0.0};
  double imu_roll_rate_radps_{0.0};
  double imu_pitch_rate_radps_{0.0};
  double imu_acceleration_norm_{9.80665};
  double body_speed_{0.0};
  double body_yaw_rate_{0.0};
  double wheel_speed_{0.0};
  uint8_t contact_state_{roka_interfaces::msg::WheelContactState::UNKNOWN};
  uint8_t contact_count_{0};
  double contact_confidence_{0.0};
  double last_relative_roll_deg_{0.0};
  double last_relative_pitch_deg_{0.0};
  double last_roll_load_deg_{0.0};
  double last_pitch_load_deg_{0.0};
  bool last_contact_fresh_{false};
  bool last_wheel_odom_fresh_{false};
  bool last_track_speed_fresh_{false};
  double track_speed_limit_{0.60};
  geometry_msgs::msg::Twist last_output_;
  uint8_t state_{roka_interfaces::msg::TerrainState::FAULT};
  uint8_t candidate_state_{roka_interfaces::msg::TerrainState::FAULT};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_terrain_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_body_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_track_speed_{0, 0, RCL_ROS_TIME};
  rclcpp::Time slip_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time attitude_hard_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time candidate_since_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<roka_interfaces::msg::WheelContactState>::SharedPtr contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr track_speed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<roka_interfaces::msg::TerrainState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::chrono::steady_clock::time_point last_diagnostic_publish_{};
  std::string last_diagnostic_signature_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roka_terrain_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roka_terrain_supervisor::TerrainSupervisor>());
  rclcpp::shutdown();
  return 0;
}
