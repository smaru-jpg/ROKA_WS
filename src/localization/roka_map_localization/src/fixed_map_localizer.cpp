#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1.0e-6) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond q(transform.rotation());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

Eigen::Isometry3d interpolate(
  const Eigen::Isometry3d & current, const Eigen::Isometry3d & target, double alpha)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - alpha) * current.translation() + alpha * target.translation();
  Eigen::Quaterniond first(current.rotation());
  Eigen::Quaterniond second(target.rotation());
  result.linear() = first.slerp(alpha, second).normalized().toRotationMatrix();
  return result;
}
}  // namespace

class FixedMapLocalizer : public rclcpp::Node
{
public:
  FixedMapLocalizer()
  : Node("fixed_map_localizer"), map_(new Cloud), filtered_map_(new Cloud),
    broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_file_ = declare_parameter<std::string>("map_pcd", "");
    initial_pose_file_ = declare_parameter<std::string>("initial_pose_file", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    local_frame_ = declare_parameter<std::string>("local_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lio/odom");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/lio/cloud_body");
    map_leaf_ = declare_parameter<double>("map_leaf_size_m", 0.20);
    source_leaf_ = declare_parameter<double>("source_leaf_size_m", 0.20);
    registration_period_ = declare_parameter<double>("registration_period_sec", 0.35);
    max_sync_delta_ = declare_parameter<double>("maximum_odom_cloud_delta_sec", 0.05);
    local_radius_ = declare_parameter<double>("local_map_radius_m", 12.0);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution_m", 0.50);
    ndt_step_ = declare_parameter<double>("ndt_step_size_m", 0.10);
    ndt_epsilon_ = declare_parameter<double>("ndt_transformation_epsilon", 0.005);
    ndt_iterations_ = declare_parameter<int>("ndt_maximum_iterations", 30);
    max_fitness_ = declare_parameter<double>("maximum_fitness_score", 0.20);
    max_translation_ = declare_parameter<double>("maximum_translation_correction_m", 0.50);
    max_z_ = declare_parameter<double>("maximum_z_correction_m", 0.30);
    max_rotation_ = declare_parameter<double>("maximum_rotation_correction_rad", 0.35);
    stable_required_ = declare_parameter<int>("required_stable_matches", 2);
    smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.35);
    search_xy_ = declare_parameter<double>("initial_search_xy_m", 0.75);
    search_xy_step_ = declare_parameter<double>("initial_search_xy_step_m", 0.75);
    search_yaw_ = declare_parameter<double>("initial_search_yaw_rad", M_PI_2);
    search_yaw_step_ = declare_parameter<double>("initial_search_yaw_step_rad", M_PI_4);
    initial_ndt_resolution_ = declare_parameter<double>("initial_ndt_resolution_m", 1.0);
    initial_ndt_iterations_ = declare_parameter<int>("initial_ndt_maximum_iterations", 15);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    validateParameters();
    loadMap();
    seed_ = loadInitialPose(initial_pose_file_);
    have_seed_ = true;

    auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&FixedMapLocalizer::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos,
      std::bind(&FixedMapLocalizer::cloudCallback, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&FixedMapLocalizer::initialPoseCallback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/map_localization/pose", 10);
    amcl_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/map_localization/ready", rclcpp::QoS(1).transient_local().reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/map_localization/status", rclcpp::QoS(1).transient_local().reliable());
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_localization/map", rclcpp::QoS(1).transient_local().reliable());
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/map_localization/reset",
      std::bind(&FixedMapLocalizer::reset, this, std::placeholders::_1,
        std::placeholders::_2));
    publishMap();
    publishReady(false, "waiting for FAST-LIO odometry and cloud");
    RCLCPP_INFO(get_logger(), "Loaded fixed map %s: %zu filtered points",
      map_file_.c_str(), filtered_map_->size());
  }

private:
  struct TimedPose
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d pose;
  };

  void validateParameters()
  {
    if (map_file_.empty()) {
      throw std::runtime_error("map_pcd is required");
    }
    if (map_leaf_ <= 0.0 || source_leaf_ <= 0.0 || registration_period_ <= 0.0 ||
      max_sync_delta_ <= 0.0 || local_radius_ <= 1.0 || ndt_resolution_ <= 0.0 ||
      stable_required_ < 1 || smoothing_alpha_ <= 0.0 || smoothing_alpha_ > 1.0 ||
      search_xy_step_ <= 0.0 || search_yaw_step_ <= 0.0)
    {
      throw std::runtime_error("invalid fixed-map localization parameter");
    }
  }

  void loadMap()
  {
    if (pcl::io::loadPCDFile<PointT>(map_file_, *map_) != 0 || map_->empty()) {
      throw std::runtime_error("cannot load non-empty PCD: " + map_file_);
    }
    Cloud::Ptr finite_map(new Cloud);
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*map_, *finite_map, finite_indices);
    pcl::VoxelGrid<PointT> filter;
    filter.setLeafSize(map_leaf_, map_leaf_, map_leaf_);
    filter.setInputCloud(finite_map);
    filter.filter(*filtered_map_);
    if (filtered_map_->size() < 100U) {
      throw std::runtime_error("fixed PCD has fewer than 100 filtered points");
    }
  }

  Eigen::Isometry3d loadInitialPose(const std::string & path) const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (path.empty()) {
      return pose;
    }
    std::ifstream input(path);
    std::string line;
    if (!input.good() || !std::getline(input, line) || !std::getline(input, line)) {
      RCLCPP_WARN(get_logger(), "Cannot read initial pose CSV %s; using identity", path.c_str());
      return pose;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    double stamp = 0.0;
    double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (!(values >> stamp >> x >> y >> z >> roll >> pitch >> yaw)) {
      RCLCPP_WARN(get_logger(), "Invalid initial pose CSV %s; using identity", path.c_str());
      return pose;
    }
    pose.translation() = Eigen::Vector3d(x, y, z);
    pose.linear() = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return pose;
  }

  void publishMap()
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*filtered_map_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    map_pub_->publish(message);
  }

  void publishReady(bool ready, const std::string & status)
  {
    std_msgs::msg::Bool state;
    state.data = ready;
    ready_pub_->publish(state);
    std_msgs::msg::String detail;
    detail.data = status;
    status_pub_->publish(detail);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != local_frame_ || message->child_frame_id != body_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting FAST-LIO odometry with frames %s -> %s (expected %s -> %s)",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        local_frame_.c_str(), body_frame_.c_str());
      return;
    }
    odom_.push_back({rclcpp::Time(message->header.stamp), poseToEigen(message->pose.pose)});
    while (odom_.size() > 200U) {
      odom_.pop_front();
    }
  }

  bool nearestOdom(const rclcpp::Time & stamp, Eigen::Isometry3d & pose)
  {
    if (odom_.empty()) {
      return false;
    }
    auto best = odom_.begin();
    int64_t best_delta = std::numeric_limits<int64_t>::max();
    for (auto it = odom_.begin(); it != odom_.end(); ++it) {
      const int64_t delta = std::llabs((it->stamp - stamp).nanoseconds());
      if (delta < best_delta) {
        best = it;
        best_delta = delta;
      }
    }
    if (static_cast<double>(best_delta) * 1.0e-9 > max_sync_delta_) {
      return false;
    }
    pose = best->pose;
    return true;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(get_logger(), "Rejecting initial pose in frame %s; expected %s",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    seed_ = poseToEigen(message->pose.pose);
    have_seed_ = true;
    initialized_ = false;
    stable_matches_ = 0;
    publishReady(false, "new initial pose received; relocalizing");
    RCLCPP_WARN(get_logger(), "Initial pose updated; global localization reset");
  }

  Cloud::Ptr localTarget(const Eigen::Vector3d & center) const
  {
    Cloud::Ptr target(new Cloud);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(filtered_map_);
    crop.setMin(Eigen::Vector4f(
      center.x() - local_radius_, center.y() - local_radius_,
      center.z() - local_radius_ * 0.5, 1.0F));
    crop.setMax(Eigen::Vector4f(
      center.x() + local_radius_, center.y() + local_radius_,
      center.z() + local_radius_ * 0.5, 1.0F));
    crop.filter(*target);
    return target;
  }

  bool align(
    const Cloud::Ptr & source, const Cloud::Ptr & target,
    const Eigen::Isometry3d & guess, double resolution, int iterations,
    Eigen::Isometry3d & result, double & fitness) const
  {
    if (source->size() < 50U || target->size() < 100U) {
      return false;
    }
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(iterations);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    Cloud aligned;
    ndt.align(aligned, guess.matrix().cast<float>());
    if (!ndt.hasConverged()) {
      return false;
    }
    fitness = ndt.getFitnessScore(1.5);
    if (!std::isfinite(fitness)) {
      return false;
    }
    result.matrix() = ndt.getFinalTransformation().cast<double>();
    return result.matrix().allFinite();
  }

  bool initialAlignment(
    const Cloud::Ptr & source, const Eigen::Isometry3d & local_pose,
    Eigen::Isometry3d & matched, double & fitness)
  {
    if (!have_seed_) {
      return false;
    }
    const Eigen::Isometry3d nominal_correction = seed_ * local_pose.inverse();
    const Eigen::Isometry3d nominal_pose = nominal_correction * local_pose;
    fitness = std::numeric_limits<double>::infinity();
    bool found = false;
    for (double dx = -search_xy_; dx <= search_xy_ + 1.0e-6; dx += search_xy_step_) {
      for (double dy = -search_xy_; dy <= search_xy_ + 1.0e-6; dy += search_xy_step_) {
        for (double dyaw = -search_yaw_; dyaw <= search_yaw_ + 1.0e-6;
          dyaw += search_yaw_step_)
        {
          Eigen::Isometry3d guess = nominal_pose;
          guess.translation().x() += dx;
          guess.translation().y() += dy;
          guess.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            guess.rotation();
          Eigen::Isometry3d candidate;
          double candidate_fitness = std::numeric_limits<double>::infinity();
          if (align(source, filtered_map_, guess, initial_ndt_resolution_,
            initial_ndt_iterations_, candidate, candidate_fitness) &&
            candidate_fitness < fitness)
          {
            matched = candidate;
            fitness = candidate_fitness;
            found = true;
          }
        }
      }
    }
    if (!found || fitness > max_fitness_) {
      return false;
    }
    auto target = localTarget(matched.translation());
    Eigen::Isometry3d refined;
    double refined_fitness = fitness;
    if (align(source, target, matched, ndt_resolution_, ndt_iterations_,
      refined, refined_fitness) && refined_fitness <= fitness)
    {
      matched = refined;
      fitness = refined_fitness;
    }
    return fitness <= max_fitness_;
  }

  bool trackingAlignment(
    const Cloud::Ptr & source, const Eigen::Isometry3d & local_pose,
    Eigen::Isometry3d & matched, double & fitness)
  {
    const Eigen::Isometry3d predicted = map_to_local_ * local_pose;
    auto target = localTarget(predicted.translation());
    if (!align(source, target, predicted, ndt_resolution_, ndt_iterations_, matched, fitness) ||
      fitness > max_fitness_)
    {
      return false;
    }
    const Eigen::Isometry3d delta = predicted.inverse() * matched;
    return delta.translation().norm() <= max_translation_ &&
      std::abs(delta.translation().z()) <= max_z_ &&
      rotationAngle(delta.rotation()) <= max_rotation_;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    if (last_registration_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(wall_now - last_registration_).count() < registration_period_)
    {
      return;
    }
    last_registration_ = wall_now;
    if (message->header.frame_id != body_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting FAST-LIO body cloud in frame %s; expected %s",
        message->header.frame_id.c_str(), body_frame_.c_str());
      return;
    }
    Eigen::Isometry3d local_pose;
    if (!nearestOdom(rclcpp::Time(message->header.stamp), local_pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No time-aligned FAST-LIO odometry for body cloud");
      return;
    }
    Cloud::Ptr raw(new Cloud);
    pcl::fromROSMsg(*message, *raw);
    Cloud::Ptr finite(new Cloud);
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*raw, *finite, finite_indices);
    Cloud::Ptr source(new Cloud);
    pcl::VoxelGrid<PointT> filter;
    filter.setLeafSize(source_leaf_, source_leaf_, source_leaf_);
    filter.setInputCloud(finite);
    filter.filter(*source);
    Eigen::Isometry3d matched;
    double fitness = std::numeric_limits<double>::infinity();
    const bool accepted = initialized_ ?
      trackingAlignment(source, local_pose, matched, fitness) :
      initialAlignment(source, local_pose, matched, fitness);
    if (!accepted) {
      stable_matches_ = 0;
      publishReady(false, initialized_ ? "registration rejected" : "searching fixed map");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Fixed-map registration rejected (fitness=%.4f)", fitness);
      return;
    }
    const Eigen::Isometry3d correction = matched * local_pose.inverse();
    if (!initialized_) {
      ++stable_matches_;
      if (stable_matches_ < stable_required_) {
        pending_correction_ = correction;
        publishReady(false, "fixed-map candidate requires another consistent match");
        return;
      }
      const Eigen::Isometry3d consistency = pending_correction_.inverse() * correction;
      if (consistency.translation().norm() > max_translation_ ||
        rotationAngle(consistency.rotation()) > max_rotation_)
      {
        stable_matches_ = 0;
        publishReady(false, "initial fixed-map matches were inconsistent");
        return;
      }
      map_to_local_ = correction;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Fixed-map localization READY (fitness %.4f)", fitness);
    } else {
      map_to_local_ = interpolate(map_to_local_, correction, smoothing_alpha_);
    }
    publishPose(message->header.stamp, local_pose, fitness);
    publishReady(true, "localized; fitness=" + std::to_string(fitness));
  }

  void publishPose(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Isometry3d & local_pose,
    double fitness)
  {
    const Eigen::Isometry3d map_pose = map_to_local_ * local_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose = eigenToPose(map_pose);
    const double position_variance = std::clamp(fitness, 0.0025, 0.25);
    const double yaw_variance = std::clamp(fitness * 0.5, 0.0025, 0.25);
    pose.pose.covariance[0] = position_variance;
    pose.pose.covariance[7] = position_variance;
    pose.pose.covariance[14] = position_variance;
    pose.pose.covariance[21] = yaw_variance;
    pose.pose.covariance[28] = yaw_variance;
    pose.pose.covariance[35] = yaw_variance;
    pose_pub_->publish(pose);
    amcl_pose_pub_->publish(pose);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = local_frame_;
      transform.transform.translation.x = map_to_local_.translation().x();
      transform.transform.translation.y = map_to_local_.translation().y();
      transform.transform.translation.z = map_to_local_.translation().z();
      const Eigen::Quaterniond q(map_to_local_.rotation());
      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();
      broadcaster_->sendTransform(transform);
    }
  }

  void reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    initialized_ = false;
    stable_matches_ = 0;
    map_to_local_.setIdentity();
    publishReady(false, "localization reset");
    response->success = true;
    response->message = "fixed-map localization reset; waiting for a stable match";
  }

  std::string map_file_;
  std::string initial_pose_file_;
  std::string map_frame_;
  std::string local_frame_;
  std::string body_frame_;
  std::string odom_topic_;
  std::string cloud_topic_;
  double map_leaf_;
  double source_leaf_;
  double registration_period_;
  double max_sync_delta_;
  double local_radius_;
  double ndt_resolution_;
  double ndt_step_;
  double ndt_epsilon_;
  int ndt_iterations_;
  double max_fitness_;
  double max_translation_;
  double max_z_;
  double max_rotation_;
  int stable_required_;
  double smoothing_alpha_;
  double search_xy_;
  double search_xy_step_;
  double search_yaw_;
  double search_yaw_step_;
  double initial_ndt_resolution_;
  int initial_ndt_iterations_;
  bool publish_tf_;
  bool have_seed_{false};
  bool initialized_{false};
  int stable_matches_{0};
  Cloud::Ptr map_;
  Cloud::Ptr filtered_map_;
  Eigen::Isometry3d seed_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_to_local_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d pending_correction_{Eigen::Isometry3d::Identity()};
  std::deque<TimedPose> odom_;
  std::chrono::steady_clock::time_point last_registration_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FixedMapLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fixed_map_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
