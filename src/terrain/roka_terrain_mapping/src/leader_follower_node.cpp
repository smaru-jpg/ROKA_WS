#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "roka_terrain_mapping/leader_follow_math.hpp"

using namespace std::chrono_literals;

namespace roka_terrain_mapping
{
namespace
{
struct Pose2
{
  Point2 position;
  double yaw{0.0};
};

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Point2 transform_from_base(const Point2 & point, const Pose2 & robot)
{
  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);
  return Point2{
    robot.position.x + cosine * point.x - sine * point.y,
    robot.position.y + sine * point.x + cosine * point.y};
}
}  // namespace

class LeaderFollower : public rclcpp::Node
{
public:
  LeaderFollower()
  : Node("leader_follower"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/velodyne_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "mission5_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    limits_.desired_distance = declare_parameter<double>("desired_distance_m", 2.0);
    limits_.gap_gain = declare_parameter<double>("gap_gain", 0.90);
    limits_.maximum_linear = declare_parameter<double>("maximum_linear_speed_mps", 1.20);
    limits_.maximum_angular = declare_parameter<double>("maximum_angular_speed_rps", 1.20);
    limits_.minimum_lookahead = declare_parameter<double>("minimum_lookahead_m", 0.45);
    limits_.maximum_lookahead = declare_parameter<double>("maximum_lookahead_m", 1.20);
    limits_.lookahead_time = declare_parameter<double>("lookahead_time_sec", 0.80);
    limits_.curvature_gain = declare_parameter<double>("curvature_speed_gain", 0.40);
    limits_.minimum_curve_speed_scale = declare_parameter<double>(
      "minimum_curve_speed_scale", 0.45);
    limits_.curve_detection_distance = declare_parameter<double>(
      "curve_detection_distance_m", 2.0);
    limits_.curve_mode_enter_curvature = declare_parameter<double>(
      "curve_mode_enter_curvature_inv_m", 0.16);
    limits_.curve_mode_exit_curvature = declare_parameter<double>(
      "curve_mode_exit_curvature_inv_m", 0.10);
    limits_.straight_lookahead = declare_parameter<double>(
      "straight_lookahead_m", 1.50);
    limits_.turn_minimum_lookahead = declare_parameter<double>(
      "turn_minimum_lookahead_m", 0.60);
    limits_.turn_maximum_lookahead = declare_parameter<double>(
      "turn_maximum_lookahead_m", 0.80);
    limits_.straight_heading_gain = declare_parameter<double>(
      "straight_heading_gain", 1.20);
    limits_.straight_cross_track_gain = declare_parameter<double>(
      "straight_cross_track_gain", 0.90);
    limits_.straight_heading_deadband = declare_parameter<double>(
      "straight_heading_deadband_deg", 3.0) * M_PI / 180.0;
    limits_.straight_cross_track_deadband = declare_parameter<double>(
      "straight_cross_track_deadband_m", 0.05);
    limits_.straight_maximum_angular = declare_parameter<double>(
      "straight_maximum_angular_rps", 0.35);
    limits_.maximum_turn_linear = declare_parameter<double>(
      "maximum_turn_linear_speed_mps", 0.45);
    limits_.rotate_to_heading_angle = declare_parameter<double>(
      "rotate_to_heading_angle_rad", 0.65);
    limits_.rotate_gain = declare_parameter<double>("rotate_gain", 1.8);
    limits_.minimum_rotate_angular = declare_parameter<double>(
      "minimum_rotate_angular_rps", 0.35);

    detection_timeout_ = declare_parameter<double>("detection_timeout_sec", 0.70);
    cloud_timeout_ = declare_parameter<double>("cloud_timeout_sec", 0.40);
    occlusion_coast_timeout_ = declare_parameter<double>("occlusion_coast_timeout_sec", 6.5);
    lost_lock_timeout_ = declare_parameter<double>("lost_lock_timeout_sec", 10.0);
    occlusion_velocity_decay_ = declare_parameter<double>(
      "occlusion_velocity_decay_per_sec", 0.15);
    maximum_prediction_uncertainty_ = declare_parameter<double>(
      "maximum_prediction_uncertainty_m", 5.0);
    process_acceleration_variance_ = declare_parameter<double>(
      "tracker_acceleration_variance", 1.2);
    measurement_variance_ = declare_parameter<double>("tracker_measurement_variance", 0.08);
    association_gate_sigma_ = declare_parameter<double>("association_gate_sigma", 7.0);
    association_gate_m_ = declare_parameter<double>("association_gate_m", 2.0);
    minimum_confirmations_ = declare_parameter<int>("minimum_confirmations", 5);
    reacquisition_confirmations_ = declare_parameter<int>("reacquisition_confirmations", 3);
    predicted_tail_repair_threshold_ = declare_parameter<double>(
      "predicted_tail_repair_threshold_m", 0.20);
    initial_confirmation_duration_ = declare_parameter<double>(
      "initial_confirmation_duration_sec", 0.50);
    initial_track_match_gate_ = declare_parameter<double>("initial_track_match_gate_m", 0.65);
    initial_minimum_speed_ = declare_parameter<double>("initial_minimum_speed_mps", 0.15);
    initial_motion_consistency_ = declare_parameter<double>(
      "initial_motion_consistency", 0.65);
    association_limits_.maximum_observed_speed = declare_parameter<double>(
      "maximum_candidate_speed_mps", 2.0);
    association_limits_.maximum_speed_increase_acceleration = declare_parameter<double>(
      "maximum_candidate_acceleration_mps2", 4.0);
    association_limits_.direction_check_minimum_speed = declare_parameter<double>(
      "direction_check_minimum_speed_mps", 0.15);
    association_limits_.minimum_direction_allowance_rad = declare_parameter<double>(
      "minimum_direction_allowance_deg", 30.0) * M_PI / 180.0;
    association_limits_.maximum_direction_rate_radps = declare_parameter<double>(
      "maximum_direction_change_rate_degps", 120.0) * M_PI / 180.0;
    stationary_unlock_time_ = declare_parameter<double>("stationary_unlock_time_sec", 0.50);

    initial_minimum_x_ = declare_parameter<double>("initial_minimum_x_m", 0.60);
    initial_maximum_x_ = declare_parameter<double>("initial_maximum_x_m", 6.0);
    initial_maximum_abs_y_ = declare_parameter<double>("initial_maximum_abs_y_m", 1.0);
    maximum_tracking_range_ = declare_parameter<double>("maximum_tracking_range_m", 12.0);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf_m", 0.08);
    ground_distance_ = declare_parameter<double>("ground_distance_m", 0.12);
    ground_maximum_angle_ = declare_parameter<double>("ground_maximum_angle_deg", 20.0) *
      M_PI / 180.0;
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance_m", 0.35);
    minimum_cluster_points_ = declare_parameter<int>("minimum_cluster_points", 5);
    maximum_cluster_points_ = declare_parameter<int>("maximum_cluster_points", 20000);
    minimum_extent_ = declare_parameter<double>("minimum_cluster_extent_m", 0.12);
    maximum_extent_x_ = declare_parameter<double>("maximum_target_extent_x_m", 1.2);
    maximum_extent_y_ = declare_parameter<double>("maximum_target_extent_y_m", 0.6);
    maximum_extent_z_ = declare_parameter<double>("maximum_target_extent_z_m", 2.0);
    planar_thickness_ratio_ = declare_parameter<double>("planar_thickness_ratio", 0.04);
    planar_minimum_span_ = declare_parameter<double>("planar_minimum_span_m", 1.0);
    wall_minimum_horizontal_span_ = declare_parameter<double>(
      "wall_minimum_horizontal_span_m", 2.0);
    emergency_point_range_ = declare_parameter<double>("emergency_point_range_m", 0.80);
    minimum_target_gap_ = declare_parameter<double>("minimum_target_gap_m", 0.90);
    footprint_front_ = declare_parameter<double>("footprint_front_m", 0.59);
    footprint_rear_ = declare_parameter<double>("footprint_rear_m", 0.29);
    footprint_half_width_ = declare_parameter<double>("footprint_half_width_m", 0.312);
    final_surface_clearance_ = declare_parameter<double>(
      "final_surface_clearance_m", 0.10);
    clearance_release_hysteresis_ = declare_parameter<double>(
      "clearance_release_hysteresis_m", 0.15);
    braking_pipeline_latency_ = declare_parameter<double>(
      "braking_pipeline_latency_sec", 0.30);
    braking_deceleration_ = declare_parameter<double>(
      "braking_deceleration_mps2", 1.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout_sec", 0.30);
    trail_spacing_ = declare_parameter<double>("trail_spacing_m", 0.08);
    maximum_leader_speed_ = declare_parameter<double>("maximum_leader_speed_mps", 2.0);

    imu_timeout_ = declare_parameter<double>("imu_timeout_sec", 0.30);
    minimum_quaternion_norm_ = declare_parameter<double>("minimum_quaternion_norm", 0.95);
    maximum_quaternion_norm_ = declare_parameter<double>("maximum_quaternion_norm", 1.05);
    maximum_roll_ = declare_parameter<double>("maximum_abs_roll_deg", 45.0) * M_PI / 180.0;
    maximum_pitch_ = declare_parameter<double>("maximum_abs_pitch_deg", 35.0) * M_PI / 180.0;
    enabled_ = declare_parameter<bool>("auto_enable", true);
    validate_parameters();

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mission5_raw", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/mission5/status", rclcpp::QoS(1).transient_local().reliable());
    gap_pub_ = create_publisher<std_msgs::msg::Float64>("/mission5/gap_m", 10);
    curvature_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/mission5/local_curvature", 10);
    heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/mission5/heading_error_rad", 10);
    cross_track_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/mission5/cross_track_error_m", 10);
    control_mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/mission5/control_mode", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/mission5/leader_marker", 10);
    trail_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/mission5/leader_trail", rclcpp::QoS(1).transient_local().reliable());
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mission5/obstacle_points", rclcpp::SensorDataQoS());
    tracking_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mission5/tracking_odom", 10);
    tracking_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    field_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/field/preflight_ready", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        field_ready_ = message->data;
        if (!field_ready_) {
          publish_zero();
          publish_status("WAIT_PREFLIGHT");
        }
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LeaderFollower::cloud_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&LeaderFollower::imu_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&LeaderFollower::odom_callback, this, std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/mission5/set_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        const bool was_enabled = enabled_;
        enabled_ = request->data;
        if (enabled_ && !was_enabled) {
          reset_tracking();
        }
        publish_zero();
        response->success = true;
        response->message = enabled_ ?
          "Section-5 follower enabled; target lock reset" : "Section-5 follower disabled";
        publish_status(enabled_ ? "WAIT_PREFLIGHT" : "DISABLED");
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/mission5/reset_target",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        reset_tracking();
        publish_zero();
        publish_status("ACQUIRE_MOVING_TARGET");
        response->success = true;
        response->message = "locked target cleared; moving-target acquisition restarted";
      });
    timer_ = create_wall_timer(50ms, std::bind(&LeaderFollower::control_tick, this));
    last_predict_time_ = std::chrono::steady_clock::now();
    publish_status(enabled_ ? "WAIT_PREFLIGHT" : "DISABLED");
    RCLCPP_INFO(
      get_logger(),
      "Section-5 locked-ID follower: target=%.2fm max=%.2fm/s gate=%.1fm",
      limits_.desired_distance, limits_.maximum_linear, association_gate_m_);
  }

  ~LeaderFollower() override {publish_zero();}

private:
  struct Candidate
  {
    Point2 base;
    Point2 fixed;
    double z{0.0};
    double extent_x{0.0};
    double extent_y{0.0};
    double extent_z{0.0};
    double score{0.0};
    double footprint_clearance{std::numeric_limits<double>::infinity()};
    bool planar_occluder{false};
    bool wall_like{false};
    bool target_size_valid{false};
    std::vector<int> indices;
  };

  struct TentativeTrack
  {
    Point2 start;
    Point2 last;
    double path_length{0.0};
    int hits{1};
    bool updated{false};
    Candidate latest;
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::time_point last_seen;
  };

  void validate_parameters() const
  {
    if (limits_.desired_distance <= 0.5 || limits_.maximum_linear <= 0.0 ||
      limits_.maximum_linear > 1.20 || limits_.maximum_angular <= 0.0 ||
      limits_.minimum_lookahead <= 0.0 ||
      limits_.maximum_lookahead < limits_.minimum_lookahead ||
      limits_.curve_detection_distance < 0.50 ||
      limits_.curve_mode_exit_curvature <= 0.0 ||
      limits_.curve_mode_enter_curvature <= limits_.curve_mode_exit_curvature ||
      limits_.straight_lookahead <= 0.0 ||
      limits_.turn_minimum_lookahead <= 0.0 ||
      limits_.turn_maximum_lookahead < limits_.turn_minimum_lookahead ||
      limits_.straight_heading_gain <= 0.0 || limits_.straight_cross_track_gain <= 0.0 ||
      limits_.straight_heading_deadband < 0.0 ||
      limits_.straight_cross_track_deadband < 0.0 ||
      limits_.straight_maximum_angular <= 0.0 ||
      limits_.straight_maximum_angular > limits_.maximum_angular ||
      limits_.maximum_turn_linear <= 0.0 ||
      limits_.rotate_to_heading_angle <= 0.0 || limits_.rotate_gain <= 0.0 ||
      limits_.minimum_rotate_angular <= 0.0 ||
      limits_.minimum_rotate_angular > limits_.maximum_angular ||
      detection_timeout_ <= 0.0 || cloud_timeout_ <= 0.0 ||
      occlusion_coast_timeout_ <= detection_timeout_ ||
      lost_lock_timeout_ <= occlusion_coast_timeout_ ||
      occlusion_velocity_decay_ < 0.0 || measurement_variance_ <= 0.0 ||
      association_gate_sigma_ <= 1.0 || association_gate_m_ <= 0.0 ||
      minimum_confirmations_ < 2 || reacquisition_confirmations_ < 2 ||
      predicted_tail_repair_threshold_ <= 0.0 ||
      initial_confirmation_duration_ <= 0.0 || initial_minimum_speed_ <= 0.0 ||
      association_limits_.maximum_observed_speed <= initial_minimum_speed_ ||
      initial_maximum_x_ <= initial_minimum_x_ ||
      initial_maximum_abs_y_ <= 0.0 ||
      maximum_tracking_range_ <= initial_maximum_x_ ||
      voxel_leaf_ <= 0.0 || cluster_tolerance_ <= voxel_leaf_ ||
      minimum_cluster_points_ < 3 || maximum_cluster_points_ <= minimum_cluster_points_ ||
      minimum_extent_ <= 0.0 || maximum_extent_x_ < minimum_extent_ ||
      maximum_extent_y_ < minimum_extent_ || maximum_extent_z_ < minimum_extent_ ||
      planar_thickness_ratio_ <= 0.0 || wall_minimum_horizontal_span_ <= 0.0 ||
      emergency_point_range_ <= 0.0 || minimum_target_gap_ <= 0.0 ||
      footprint_front_ <= 0.0 || footprint_rear_ <= 0.0 ||
      footprint_half_width_ <= 0.0 || final_surface_clearance_ < 0.0 ||
      clearance_release_hysteresis_ <= 0.0 || braking_pipeline_latency_ < 0.0 ||
      braking_deceleration_ <= 0.0 || odom_timeout_ <= 0.0 ||
      trail_spacing_ <= 0.0 || maximum_leader_speed_ < limits_.maximum_linear ||
      imu_timeout_ <= 0.0 || minimum_quaternion_norm_ <= 0.0 ||
      maximum_quaternion_norm_ <= minimum_quaternion_norm_)
    {
      throw std::invalid_argument("invalid Section-5 locked-target parameters");
    }
    if (fixed_frame_.empty() || odom_frame_.empty() || fixed_frame_ == odom_frame_) {
      throw std::invalid_argument(
              "Section-5 tracking frame must be non-empty and distinct from odom");
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto & q = message->orientation;
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(q_norm) || q_norm < minimum_quaternion_norm_ ||
      q_norm > maximum_quaternion_norm_)
    {
      attitude_ok_ = false;
      last_imu_time_ = std::chrono::steady_clock::now();
      return;
    }
    const double x = q.x / q_norm;
    const double y = q.y / q_norm;
    const double z = q.z / q_norm;
    const double w = q.w / q_norm;
    const double roll = std::atan2(
      2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double raw_yaw = std::atan2(
      2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    attitude_ok_ = std::isfinite(roll) && std::isfinite(pitch) &&
      std::abs(roll) <= maximum_roll_ && std::abs(pitch) <= maximum_pitch_;
    if (attitude_ok_ && std::isfinite(raw_yaw)) {
      if (!imu_heading_initialized_) {
        last_raw_imu_yaw_ = raw_yaw;
        relative_imu_yaw_ = 0.0;
        imu_heading_initialized_ = true;
      } else {
        relative_imu_yaw_ += normalize_angle(raw_yaw - last_raw_imu_yaw_);
        last_raw_imu_yaw_ = raw_yaw;
      }
      tracking_pose_.yaw = relative_imu_yaw_;
    }
    last_imu_time_ = std::chrono::steady_clock::now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    measured_linear_speed_ = std::abs(message->twist.twist.linear.x);
    last_odom_time_ = steady_now;
    if (!imu_heading_initialized_) {
      last_pose_update_time_ = steady_now;
      return;
    }
    double dt = 0.0;
    if (last_pose_update_time_.time_since_epoch().count() != 0) {
      dt = clamp(
        std::chrono::duration<double>(steady_now - last_pose_update_time_).count(),
        0.0, 0.20);
    }
    last_pose_update_time_ = steady_now;
    tracking_pose_.yaw = relative_imu_yaw_;
    const double linear = message->twist.twist.linear.x;
    tracking_pose_.position.x += linear * std::cos(tracking_pose_.yaw) * dt;
    tracking_pose_.position.y += linear * std::sin(tracking_pose_.yaw) * dt;
    tracking_pose_ready_ = true;

    nav_msgs::msg::Odometry tracking = *message;
    tracking.header.frame_id = fixed_frame_;
    tracking.child_frame_id = base_frame_;
    tracking.pose.pose.position.x = tracking_pose_.position.x;
    tracking.pose.pose.position.y = tracking_pose_.position.y;
    tracking.pose.pose.position.z = 0.0;
    tracking.pose.pose.orientation.x = 0.0;
    tracking.pose.pose.orientation.y = 0.0;
    tracking.pose.pose.orientation.z = std::sin(0.5 * tracking_pose_.yaw);
    tracking.pose.pose.orientation.w = std::cos(0.5 * tracking_pose_.yaw);
    tracking_odom_pub_->publish(tracking);

    // Keep the existing single-parent TF tree.  odom->base_footprint remains
    // owned by the MD driver; this sibling transform makes the base pose seen
    // from mission5_odom equal to the IMU-heading/wheel-distance pose above.
    const Point2 wheel_position{
      message->pose.pose.position.x, message->pose.pose.position.y};
    const double wheel_yaw = yaw_from_quaternion(message->pose.pose.orientation);
    const double odom_to_tracking_yaw = normalize_angle(wheel_yaw - tracking_pose_.yaw);
    const double cosine = std::cos(odom_to_tracking_yaw);
    const double sine = std::sin(odom_to_tracking_yaw);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message->header;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = fixed_frame_;
    transform.transform.translation.x = wheel_position.x -
      (cosine * tracking_pose_.position.x - sine * tracking_pose_.position.y);
    transform.transform.translation.y = wheel_position.y -
      (sine * tracking_pose_.position.x + cosine * tracking_pose_.position.y);
    transform.transform.rotation.z = std::sin(0.5 * odom_to_tracking_yaw);
    transform.transform.rotation.w = std::cos(0.5 * odom_to_tracking_yaw);
    tracking_tf_broadcaster_->sendTransform(transform);
  }

  std::optional<Eigen::Affine3f> lookup_affine(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(target, source, stamp, 100ms);
      const auto & qm = transform.transform.rotation;
      Eigen::Quaternionf q(
        static_cast<float>(qm.w), static_cast<float>(qm.x),
        static_cast<float>(qm.y), static_cast<float>(qm.z));
      if (q.norm() < 1.0e-6F) {
        return std::nullopt;
      }
      Eigen::Affine3f affine = Eigen::Affine3f::Identity();
      affine.linear() = q.normalized().toRotationMatrix();
      affine.translation() = Eigen::Vector3f(
        static_cast<float>(transform.transform.translation.x),
        static_cast<float>(transform.transform.translation.y),
        static_cast<float>(transform.transform.translation.z));
      return affine;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Section-5 cloud TF unavailable: %s", error.what());
      return std::nullopt;
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!field_ready_ || message->width * message->height == 0U) {
      return;
    }
    const rclcpp::Time latest(0, 0, get_clock()->get_clock_type());
    const auto sensor_to_base = lookup_affine(base_frame_, message->header.frame_id, latest);
    if (!sensor_to_base || !tracking_pose_ready_) {
      return;
    }
    const Pose2 robot_pose = tracking_pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*message, *source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *body, *sensor_to_base);
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    roi->reserve(body->size());
    for (const auto & point : body->points) {
      const double range = std::hypot(point.x, point.y);
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        range >= 0.35 && range <= maximum_tracking_range_ &&
        point.z >= -1.0 && point.z <= 2.5)
      {
        roi->push_back(point);
      }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    if (roi->size() >= static_cast<std::size_t>(minimum_cluster_points_)) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(roi);
      voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
      voxel.filter(*filtered);
      obstacles = remove_ground(filtered);
    }
    const auto candidates = extract_candidates(obstacles, robot_pose);
    publish_safety_obstacles(obstacles, candidates, message->header.stamp);
    last_cloud_time_ = std::chrono::steady_clock::now();
    // B disables only autonomous target following.  Keep the LiDAR obstacle
    // stream alive so the same smoother/collision-monitor chain can protect
    // subsequent LB manual driving.
    if (!enabled_) {
      return;
    }
    if (candidates.empty()) {
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (!tracker_.initialized()) {
      update_initial_tracks(candidates, robot_pose, steady_now);
      return;
    }
    const auto selected = select_locked_candidate(candidates, steady_now);
    if (!selected) {
      return;
    }
    const double target_age = std::chrono::duration<double>(
      steady_now - last_measurement_time_).count();
    if (target_age > detection_timeout_) {
      update_reacquisition(*selected, steady_now);
    } else {
      accept_locked_measurement(*selected, steady_now);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_ground(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered)
  {
    auto obstacles = filtered;
    if (filtered->size() < 20U) {
      return obstacles;
    }
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(static_cast<float>(ground_maximum_angle_));
    segmentation.setDistanceThreshold(ground_distance_);
    segmentation.setMaxIterations(80);
    segmentation.setInputCloud(filtered);
    segmentation.segment(*ground_indices, *coefficients);
    if (ground_indices->indices.size() >= 10U) {
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      obstacles.reset(new pcl::PointCloud<pcl::PointXYZ>);
      extract.setInputCloud(filtered);
      extract.setIndices(ground_indices);
      extract.setNegative(true);
      extract.filter(*obstacles);
    }
    return obstacles;
  }

  std::vector<Candidate> extract_candidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacles,
    const Pose2 & robot_pose)
  {
    std::vector<Candidate> candidates;
    if (obstacles->size() < static_cast<std::size_t>(minimum_cluster_points_)) {
      return candidates;
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(obstacles);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(cluster_tolerance_);
    extraction.setMinClusterSize(minimum_cluster_points_);
    extraction.setMaxClusterSize(maximum_cluster_points_);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(obstacles);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    for (const auto & cluster : clusters) {
      pcl::PointCloud<pcl::PointXYZ> points;
      points.reserve(cluster.indices.size());
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (const auto index : cluster.indices) {
        const auto & point = obstacles->points.at(static_cast<std::size_t>(index));
        points.push_back(point);
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
      }
      centroid /= static_cast<double>(cluster.indices.size());
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto & point : points.points) {
        const Eigen::Vector3d delta(
          point.x - centroid.x(), point.y - centroid.y(), point.z - centroid.z());
        covariance += delta * delta.transpose();
      }
      covariance /= static_cast<double>(cluster.indices.size());
      const auto eigenvalues = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(
        covariance).eigenvalues();
      Eigen::Vector4f minimum;
      Eigen::Vector4f maximum;
      pcl::getMinMax3D(points, minimum, maximum);
      const double ex = maximum.x() - minimum.x();
      const double ey = maximum.y() - minimum.y();
      const double ez = maximum.z() - minimum.z();
      if (std::max({ex, ey, ez}) < minimum_extent_) {
        continue;
      }
      const double major = std::max(1.0e-9, eigenvalues[2]);
      const bool planar =
        eigenvalues[0] / major <= planar_thickness_ratio_ &&
        eigenvalues[1] / major >= 0.08 &&
        std::max({ex, ey, ez}) >= planar_minimum_span_;
      Candidate c;
      c.base = Point2{centroid.x(), centroid.y()};
      c.fixed = transform_from_base(c.base, robot_pose);
      c.z = centroid.z();
      std::vector<Point2> horizontal_points;
      horizontal_points.reserve(points.size());
      for (const auto & point : points.points) {
        horizontal_points.push_back(Point2{point.x, point.y});
      }
      const auto horizontal_extents = oriented_horizontal_extents(horizontal_points);
      c.extent_x = horizontal_extents.length;
      c.extent_y = horizontal_extents.width;
      c.extent_z = ez;
      for (const auto & point : points.points) {
        c.footprint_clearance = std::min(
          c.footprint_clearance,
          rectangular_footprint_clearance(
            Point2{point.x, point.y}, footprint_front_, footprint_rear_,
            footprint_half_width_));
      }
      c.planar_occluder = planar;
      c.wall_like = planar && std::max(ex, ey) >= wall_minimum_horizontal_span_;
      c.target_size_valid = oriented_target_size_is_valid(
        horizontal_extents, ez, minimum_extent_, maximum_extent_x_,
        maximum_extent_y_, maximum_extent_z_);
      c.indices = cluster.indices;
      candidates.push_back(std::move(c));
    }
    return candidates;
  }

  bool inside_initial_roi(const Candidate & c) const
  {
    return c.target_size_valid && !c.planar_occluder &&
      c.base.x >= initial_minimum_x_ && c.base.x <= initial_maximum_x_ &&
      std::abs(c.base.y) <= initial_maximum_abs_y_;
  }

  void update_initial_tracks(
    const std::vector<Candidate> & candidates, const Pose2 & robot_pose,
    const std::chrono::steady_clock::time_point & steady_now)
  {
    for (auto & track : initial_tracks_) {
      track.updated = false;
    }
    for (const auto & candidate : candidates) {
      if (!inside_initial_roi(candidate)) {
        continue;
      }
      TentativeTrack * matched = nullptr;
      double best = initial_track_match_gate_;
      for (auto & track : initial_tracks_) {
        const double d = distance(track.last, candidate.fixed);
        if (!track.updated && d < best) {
          best = d;
          matched = &track;
        }
      }
      if (matched == nullptr) {
        TentativeTrack track;
        track.start = candidate.fixed;
        track.last = candidate.fixed;
        track.latest = candidate;
        track.started = steady_now;
        track.last_seen = steady_now;
        track.updated = true;
        initial_tracks_.push_back(std::move(track));
      } else {
        matched->path_length += distance(matched->last, candidate.fixed);
        matched->last = candidate.fixed;
        matched->latest = candidate;
        matched->last_seen = steady_now;
        matched->hits += 1;
        matched->updated = true;
      }
    }
    initial_tracks_.erase(
      std::remove_if(
        initial_tracks_.begin(), initial_tracks_.end(),
        [&](const TentativeTrack & track) {
          return std::chrono::duration<double>(steady_now - track.last_seen).count() >
                 detection_timeout_;
        }),
      initial_tracks_.end());
    TentativeTrack * selected = nullptr;
    double selected_score = std::numeric_limits<double>::infinity();
    for (auto & track : initial_tracks_) {
      const double duration = std::chrono::duration<double>(
        track.last_seen - track.started).count();
      const double displacement = distance(track.start, track.last);
      const double speed = duration > 0.0 ? displacement / duration : 0.0;
      const double consistency = track.path_length > 1.0e-6 ?
        displacement / track.path_length : 0.0;
      if (track.hits < minimum_confirmations_ || duration < initial_confirmation_duration_ ||
        speed < initial_minimum_speed_ ||
        speed > association_limits_.maximum_observed_speed ||
        consistency < initial_motion_consistency_)
      {
        continue;
      }
      const double score = track.latest.base.x + 0.6 * std::abs(track.latest.base.y);
      if (score < selected_score) {
        selected_score = score;
        selected = &track;
      }
    }
    if (selected == nullptr) {
      publish_status("ACQUIRE_MOVING_TARGET");
      return;
    }
    const double duration = std::chrono::duration<double>(
      selected->last_seen - selected->started).count();
    const Point2 velocity = (selected->last - selected->start) / std::max(0.05, duration);
    tracker_.initialize(selected->last, 0.10, 0.50, velocity);
    target_extent_x_ = selected->latest.extent_x;
    target_extent_y_ = selected->latest.extent_y;
    target_extent_z_ = selected->latest.extent_z;
    last_target_surface_clearance_ = selected->latest.footprint_clearance;
    last_target_surface_time_ = steady_now;
    last_measurement_position_ = selected->last;
    last_measurement_time_ = steady_now;
    last_predict_time_ = steady_now;
    trail_.seed(robot_pose.position, selected->last, trail_spacing_);
    last_confirmed_trail_station_ = trail_.end_station();
    initial_tracks_.clear();
    publish_tracking_visualization(current_stamp());
    publish_status("TRACKING_LOCKED");
  }

  bool shape_matches(const Candidate & c) const
  {
    return c.target_size_valid && !c.planar_occluder &&
      shape_ratio_is_plausible(c.extent_x, target_extent_x_) &&
      shape_ratio_is_plausible(c.extent_y, target_extent_y_) &&
      shape_ratio_is_plausible(c.extent_z, target_extent_z_);
  }

  std::optional<Candidate> select_locked_candidate(
    std::vector<Candidate> candidates,
    const std::chrono::steady_clock::time_point & steady_now)
  {
    std::vector<Candidate> valid;
    const double dt = std::chrono::duration<double>(
      steady_now - last_measurement_time_).count();
    const bool reversal_unlocked = stationary_since_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - stationary_since_).count() >=
      stationary_unlock_time_;
    for (auto candidate : candidates) {
      if (!shape_matches(candidate)) {
        continue;
      }
      const double euclidean = distance(candidate.fixed, tracker_.position());
      const double mahalanobis = tracker_.innovation_distance_squared(
        candidate.fixed, measurement_variance_);
      if (euclidean > association_gate_m_ ||
        mahalanobis > association_gate_sigma_ * association_gate_sigma_ ||
        !kinematic_candidate_is_consistent(
          last_measurement_position_, candidate.fixed, dt, tracker_.velocity(),
          reversal_unlocked, association_limits_))
      {
        continue;
      }
      const double shape_error =
        std::abs(candidate.extent_x / target_extent_x_ - 1.0) +
        std::abs(candidate.extent_y / target_extent_y_ - 1.0) +
        std::abs(candidate.extent_z / target_extent_z_ - 1.0);
      candidate.score = mahalanobis + 0.25 * shape_error;
      valid.push_back(std::move(candidate));
    }
    if (valid.empty()) {
      return std::nullopt;
    }
    return *std::min_element(
      valid.begin(), valid.end(),
      [](const Candidate & a, const Candidate & b) {return a.score < b.score;});
  }

  void update_reacquisition(
    const Candidate & candidate, const std::chrono::steady_clock::time_point & steady_now)
  {
    if (!reacquisition_track_ ||
      distance(reacquisition_track_->last, candidate.fixed) > initial_track_match_gate_)
    {
      TentativeTrack track;
      track.start = candidate.fixed;
      track.last = candidate.fixed;
      track.latest = candidate;
      track.started = steady_now;
      track.last_seen = steady_now;
      reacquisition_track_ = std::move(track);
      return;
    }
    auto & track = *reacquisition_track_;
    track.path_length += distance(track.last, candidate.fixed);
    track.last = candidate.fixed;
    track.latest = candidate;
    track.last_seen = steady_now;
    track.hits += 1;
    if (track.hits >= reacquisition_confirmations_) {
      accept_locked_measurement(track.latest, steady_now);
      reacquisition_track_.reset();
    }
  }

  void accept_locked_measurement(
    const Candidate & selected, const std::chrono::steady_clock::time_point & steady_now)
  {
    const double measurement_age = std::chrono::duration<double>(
      steady_now - last_measurement_time_).count();
    const double prediction_correction = distance(tracker_.position(), selected.fixed);
    if (measurement_age > detection_timeout_ &&
      prediction_correction > predicted_tail_repair_threshold_)
    {
      trail_.truncate_after(last_confirmed_trail_station_);
      RCLCPP_INFO(
        get_logger(),
        "[MISSION5] repaired occlusion trail tail: correction=%.3fm confirmed_station=%.2fm",
        prediction_correction, last_confirmed_trail_station_);
    }
    const double dt = std::max(
      0.02, std::chrono::duration<double>(steady_now - last_measurement_time_).count());
    const double observed_speed = distance(last_measurement_position_, selected.fixed) / dt;
    if (observed_speed < association_limits_.direction_check_minimum_speed) {
      if (stationary_since_.time_since_epoch().count() == 0) {
        stationary_since_ = steady_now;
      }
    } else {
      stationary_since_ = {};
    }
    tracker_.update(selected.fixed, measurement_variance_);
    target_extent_x_ = 0.85 * target_extent_x_ + 0.15 * selected.extent_x;
    target_extent_y_ = 0.85 * target_extent_y_ + 0.15 * selected.extent_y;
    target_extent_z_ = 0.85 * target_extent_z_ + 0.15 * selected.extent_z;
    last_target_surface_clearance_ = selected.footprint_clearance;
    last_target_surface_time_ = steady_now;
    last_measurement_position_ = selected.fixed;
    last_measurement_time_ = steady_now;
    if (tracker_.speed() >= 0.05) {
      trail_.append(tracker_.position(), trail_spacing_);
    }
    last_confirmed_trail_station_ = trail_.end_station();
    publish_tracking_visualization(current_stamp());
    publish_status(tracker_.speed() < 0.15 ? "STOPPED_LOCKED" : "TRACKING_LOCKED");
  }

  void control_tick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double prediction_dt = std::chrono::duration<double>(
      steady_now - last_predict_time_).count();
    last_predict_time_ = steady_now;
    if (tracker_.initialized()) {
      tracker_.predict(prediction_dt, process_acceleration_variance_);
    }
    if (!enabled_ || !field_ready_) {
      publish_zero();
      publish_status(!enabled_ ? "DISABLED" : "WAIT_PREFLIGHT");
      return;
    }
    const bool imu_fresh = last_imu_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_imu_time_).count() <= imu_timeout_;
    if (!imu_fresh || !attitude_ok_) {
      publish_zero();
      publish_status(imu_fresh ? "ATTITUDE_STOP" : "WAIT_IMU");
      return;
    }
    const bool cloud_fresh = last_cloud_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_cloud_time_).count() <= cloud_timeout_;
    if (!cloud_fresh) {
      publish_zero();
      publish_status("SENSOR_FAULT");
      return;
    }
    const bool odom_fresh = last_odom_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_odom_time_).count() <= odom_timeout_;
    if (!odom_fresh) {
      publish_zero();
      publish_status("WAIT_ODOM");
      return;
    }
    if (!tracker_.initialized()) {
      publish_zero();
      publish_status("ACQUIRE_MOVING_TARGET");
      return;
    }
    const double target_age = std::chrono::duration<double>(
      steady_now - last_measurement_time_).count();
    if (!tracking_pose_ready_) {
      publish_zero();
      publish_status("WAIT_TRACKING_POSE");
      return;
    }
    const Pose2 robot_pose = tracking_pose_;
    if (target_age > occlusion_coast_timeout_ ||
      target_age > lost_lock_timeout_ ||
      tracker_.position_uncertainty() > maximum_prediction_uncertainty_)
    {
      publish_zero();
      publish_status("LOST_LOCKED");
      return;
    }
    const bool occluded = target_age > detection_timeout_;
    if (occluded) {
      tracker_.damp_velocity(std::exp(-occlusion_velocity_decay_ * prediction_dt));
      if (tracker_.speed() >= 0.05) {
        trail_.append(tracker_.position(), trail_spacing_);
        publish_tracking_visualization(current_stamp());
      }
    }
    auto command = trail_command(
      trail_, robot_pose.position, robot_pose.yaw,
      std::min(tracker_.speed(), maximum_leader_speed_), current_linear_, limits_,
      std::max(0.0, last_robot_station_ - 0.30), last_robot_station_ + 1.0,
      curve_mode_);
    if (!command.valid) {
      publish_zero();
      publish_status("WAIT_TRAIL");
      return;
    }
    if (command.curve_mode != curve_mode_) {
      curve_mode_ = command.curve_mode;
      RCLCPP_INFO(
        get_logger(), "[MISSION5] CONTROL_MODE=%s curvature=%+.3f 1/m",
        curve_mode_ ? "TURN" : "STRAIGHT", command.local_path_curvature);
    }
    last_robot_station_ = std::max(last_robot_station_, command.robot_station);
    if (command.gap <= minimum_target_gap_) {
      command.linear = 0.0;
    }
    if (occluded) {
      const double progress = clamp(
        (target_age - detection_timeout_) /
        std::max(0.1, occlusion_coast_timeout_ - detection_timeout_), 0.0, 1.0);
      const double scale = 1.0 - 0.20 * progress;
      command.linear *= scale;
      command.angular *= scale;
    }
    const bool surface_fresh = last_target_surface_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_target_surface_time_).count() <=
      detection_timeout_;
    const double braking_speed = std::max(
      std::abs(current_linear_), odom_fresh ? measured_linear_speed_ : 0.0);
    const double stop_at_clearance = required_stopping_clearance(
      braking_speed, final_surface_clearance_, braking_pipeline_latency_,
      braking_deceleration_);
    if (surface_fresh && last_target_surface_clearance_ <= stop_at_clearance) {
      proximity_stop_latched_ = true;
    }
    // Keep the stop latched while the chassis is physically decelerating.  It
    // may be released only after odometry confirms near-zero speed and the
    // target has opened a clear hysteresis gap.  This prevents a 20 Hz
    // stop/go oscillation while nav2_velocity_smoother is still braking.
    if (proximity_stop_latched_) {
      const bool stopped = odom_fresh && measured_linear_speed_ <= 0.05 &&
        std::abs(current_linear_) <= 0.05;
      const bool target_moved_away = surface_fresh &&
        last_target_surface_clearance_ >=
        final_surface_clearance_ + clearance_release_hysteresis_;
      if (stopped && target_moved_away) {
        proximity_stop_latched_ = false;
      } else {
        command.linear = 0.0;
        command.angular = 0.0;
      }
    }
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    command_pub_->publish(message);
    current_linear_ = command.linear;
    std_msgs::msg::Float64 gap;
    gap.data = command.gap;
    gap_pub_->publish(gap);
    std_msgs::msg::Float64 curvature;
    curvature.data = command.local_path_curvature;
    curvature_pub_->publish(curvature);
    std_msgs::msg::Float64 heading_error;
    heading_error.data = command.heading_error;
    heading_error_pub_->publish(heading_error);
    std_msgs::msg::Float64 cross_track_error;
    cross_track_error.data = command.cross_track_error;
    cross_track_error_pub_->publish(cross_track_error);
    std_msgs::msg::String control_mode;
    control_mode.data = curve_mode_ ? "TURN" : "STRAIGHT";
    control_mode_pub_->publish(control_mode);
    publish_status(
      proximity_stop_latched_ ? "PROXIMITY_STOP_LOCKED" :
      (occluded ? "OCCLUDED_LOCKED" :
      (tracker_.speed() < 0.15 ? "STOPPED_LOCKED" : "TRACKING_LOCKED")));
  }

  void publish_safety_obstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacles,
    const std::vector<Candidate> & candidates, const builtin_interfaces::msg::Time & stamp)
  {
    pcl::PointCloud<pcl::PointXYZ> safety;
    std::vector<bool> selected(obstacles->size(), false);
    for (const auto & candidate : candidates) {
      if (candidate.wall_like) {
        for (const int index : candidate.indices) {
          if (index >= 0 && static_cast<std::size_t>(index) < selected.size()) {
            selected[static_cast<std::size_t>(index)] = true;
          }
        }
      }
    }
    safety.reserve(obstacles->size());
    const double maximum_braking_clearance = required_stopping_clearance(
      limits_.maximum_linear, final_surface_clearance_, braking_pipeline_latency_,
      braking_deceleration_);
    for (std::size_t index = 0; index < obstacles->size(); ++index) {
      const auto & point = obstacles->points[index];
      const double footprint_clearance = rectangular_footprint_clearance(
        Point2{point.x, point.y}, footprint_front_, footprint_rear_,
        footprint_half_width_);
      if (selected[index] || footprint_clearance <= maximum_braking_clearance ||
        std::hypot(point.x, point.y) <= emergency_point_range_)
      {
        safety.push_back(point);
      }
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(safety, message);
    message.header.frame_id = base_frame_;
    message.header.stamp = stamp;
    obstacle_pub_->publish(message);
  }

  void publish_tracking_visualization(const builtin_interfaces::msg::Time & stamp)
  {
    if (!tracker_.initialized()) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = stamp;
    marker.ns = "mission5_leader";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = tracker_.position().x;
    marker.pose.position.y = tracker_.position().y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.40;
    marker.color.r = 0.1F;
    marker.color.g = 1.0F;
    marker.color.b = 0.2F;
    marker.color.a = 1.0F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);
    nav_msgs::msg::Path path;
    path.header.frame_id = fixed_frame_;
    path.header.stamp = stamp;
    path.poses.reserve(trail_.size());
    for (const auto & point : trail_.points()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.position.x;
      pose.pose.position.y = point.position.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    trail_pub_->publish(path);
  }

  builtin_interfaces::msg::Time current_stamp() {return now();}

  void publish_zero()
  {
    current_linear_ = 0.0;
    command_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_status(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "[MISSION5] %s", status.c_str());
  }

  void reset_tracking()
  {
    tracker_.reset();
    trail_.clear();
    initial_tracks_.clear();
    reacquisition_track_.reset();
    stationary_since_ = {};
    current_linear_ = 0.0;
    last_robot_station_ = 0.0;
    last_confirmed_trail_station_ = 0.0;
    last_measurement_time_ = {};
    last_measurement_position_ = {};
    last_target_surface_time_ = {};
    last_target_surface_clearance_ = std::numeric_limits<double>::infinity();
    proximity_stop_latched_ = false;
    curve_mode_ = false;
  }

  std::string input_topic_;
  std::string base_frame_;
  std::string fixed_frame_;
  std::string odom_frame_;
  TrailControlLimits limits_;
  TargetAssociationLimits association_limits_;
  double detection_timeout_, cloud_timeout_, occlusion_coast_timeout_, lost_lock_timeout_;
  double occlusion_velocity_decay_, maximum_prediction_uncertainty_;
  double process_acceleration_variance_, measurement_variance_;
  double association_gate_sigma_, association_gate_m_;
  double predicted_tail_repair_threshold_;
  int minimum_confirmations_, reacquisition_confirmations_;
  double initial_confirmation_duration_, initial_track_match_gate_;
  double initial_minimum_speed_, initial_motion_consistency_, stationary_unlock_time_;
  double initial_minimum_x_, initial_maximum_x_, initial_maximum_abs_y_;
  double maximum_tracking_range_;
  float voxel_leaf_;
  double ground_distance_, ground_maximum_angle_, cluster_tolerance_;
  int minimum_cluster_points_, maximum_cluster_points_;
  double minimum_extent_, maximum_extent_x_, maximum_extent_y_, maximum_extent_z_;
  double planar_thickness_ratio_, planar_minimum_span_, wall_minimum_horizontal_span_;
  double emergency_point_range_, minimum_target_gap_, trail_spacing_, maximum_leader_speed_;
  double footprint_front_, footprint_rear_, footprint_half_width_;
  double final_surface_clearance_, clearance_release_hysteresis_;
  double braking_pipeline_latency_, braking_deceleration_, odom_timeout_;
  double imu_timeout_, minimum_quaternion_norm_, maximum_quaternion_norm_;
  double maximum_roll_, maximum_pitch_;
  bool enabled_{true}, field_ready_{false}, attitude_ok_{false};
  bool imu_heading_initialized_{false}, tracking_pose_ready_{false};
  double target_extent_x_{0.5}, target_extent_y_{0.5}, target_extent_z_{0.5};
  double current_linear_{0.0}, last_robot_station_{0.0};
  double last_confirmed_trail_station_{0.0};
  double measured_linear_speed_{0.0};
  double last_target_surface_clearance_{std::numeric_limits<double>::infinity()};
  bool proximity_stop_latched_{false};
  bool curve_mode_{false};
  double last_raw_imu_yaw_{0.0}, relative_imu_yaw_{0.0};
  Pose2 tracking_pose_;
  Point2 last_measurement_position_;
  std::string last_status_;
  ConstantVelocityTracker tracker_;
  LeaderTrail trail_;
  std::vector<TentativeTrack> initial_tracks_;
  std::optional<TentativeTrack> reacquisition_track_;
  std::chrono::steady_clock::time_point last_measurement_time_{}, last_cloud_time_{};
  std::chrono::steady_clock::time_point last_imu_time_{}, last_predict_time_{};
  std::chrono::steady_clock::time_point last_odom_time_{}, last_target_surface_time_{};
  std::chrono::steady_clock::time_point last_pose_update_time_{};
  std::chrono::steady_clock::time_point stationary_since_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tracking_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gap_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curvature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tracking_odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr field_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace roka_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roka_terrain_mapping::LeaderFollower>());
  rclcpp::shutdown();
  return 0;
}
