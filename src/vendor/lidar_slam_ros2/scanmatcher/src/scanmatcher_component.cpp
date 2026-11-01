#include "scanmatcher/scanmatcher_component.h"
#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>

#include <pcl/filters/filter.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
: Node("scan_matcher", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  double ndt_resolution;
  int ndt_num_threads;
  double gicp_corr_dist_threshold;

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method_);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("gicp_corr_dist_threshold", 5.0);
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
  declare_parameter("trans_for_mapupdate", 1.5);
  get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
  declare_parameter("rotation_for_mapupdate", 0.15);
  get_parameter("rotation_for_mapupdate", rotation_for_mapupdate_);
  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);
  declare_parameter("use_min_max_filter", false);
  get_parameter("use_min_max_filter", use_min_max_filter_);
  declare_parameter("scan_min_range", 0.1);
  get_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", 100.0);
  get_parameter("scan_max_range", scan_max_range_);
  declare_parameter("scan_period", 0.1);
  get_parameter("scan_period", scan_period_);
  declare_parameter("map_publish_period", 15.0);
  get_parameter("map_publish_period", map_publish_period_);  
  declare_parameter("num_targeted_cloud", 10);
  get_parameter("num_targeted_cloud", num_targeted_cloud_);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_tareged_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }
  declare_parameter("minimum_registration_points", 50);
  get_parameter("minimum_registration_points", minimum_registration_points_);
  if (minimum_registration_points_ < 3) {
    RCLCPP_WARN(
      get_logger(), "minimum_registration_points must be at least 3; using 3");
    minimum_registration_points_ = 3;
  }
  declare_parameter("max_registration_translation_correction", 0.75);
  get_parameter(
    "max_registration_translation_correction",
    max_registration_translation_correction_);
  declare_parameter("max_registration_rotation_correction", 0.60);
  get_parameter(
    "max_registration_rotation_correction",
    max_registration_rotation_correction_);

  declare_parameter("initial_pose_x", 0.0);
  get_parameter("initial_pose_x", initial_pose_x_);
  declare_parameter("initial_pose_y", 0.0);
  get_parameter("initial_pose_y", initial_pose_y_);
  declare_parameter("initial_pose_z", 0.0);
  get_parameter("initial_pose_z", initial_pose_z_);
  declare_parameter("initial_pose_qx", 0.0);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  declare_parameter("initial_pose_qy", 0.0);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  declare_parameter("initial_pose_qz", 0.0);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  declare_parameter("initial_pose_qw", 1.0);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  declare_parameter("set_initial_pose", false);
  get_parameter("set_initial_pose", set_initial_pose_);
  declare_parameter("publish_tf", true);
  get_parameter("publish_tf", publish_tf_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);
  declare_parameter("use_imu", false);
  get_parameter("use_imu", use_imu_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);

  RCLCPP_INFO(
    get_logger(),
    "3D mapper configured: backend=%s, odom_prior=%s, imu_deskew=%s, "
    "keyframe=%.2fm/%.2frad, input/map_voxel=%.2f/%.2fm",
    registration_method_.c_str(), use_odom_ ? "on" : "off", use_imu_ ? "on" : "off",
    trans_for_mapupdate_, rotation_for_mapupdate_, vg_size_for_input_, vg_size_for_map_);

  if (registration_method_ == "NDT") {

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt_omp
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}

    registration_ = ndt;

  } else {
	  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;
  }

  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;

  path_.header.frame_id = global_frame_id_;

  lidar_undistortion_.setScanPeriod(scan_period_);

  initializePubSub();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;
    corrent_pose_stamped_ = *msg;
    previous_position_ = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    previous_orientation_ = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z).normalized();
    pose_pub_->publish(corrent_pose_stamped_);
    initial_pose_received_ = true;

    path_.poses.push_back(*msg);
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

ScanMatcherComponent::~ScanMatcherComponent()
{
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

void ScanMatcherComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
  // sub
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      if (msg->header.frame_id != global_frame_id_) {
        RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
        return;
      }
      RCLCPP_INFO(get_logger(), "initial_pose is received");

      corrent_pose_stamped_ = *msg;
      previous_position_.x() = corrent_pose_stamped_.pose.position.x;
      previous_position_.y() = corrent_pose_stamped_.pose.position.y;
      previous_position_.z() = corrent_pose_stamped_.pose.position.z;
      previous_orientation_ = Eigen::Quaterniond(
        corrent_pose_stamped_.pose.orientation.w,
        corrent_pose_stamped_.pose.orientation.x,
        corrent_pose_stamped_.pose.orientation.y,
        corrent_pose_stamped_.pose.orientation.z).normalized();
      initial_pose_received_ = true;

      pose_pub_->publish(corrent_pose_stamped_);
    };

  auto cloud_callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {
        sensor_msgs::msg::PointCloud2 transformed_msg;
        try {
          tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(msg->header.stamp.sec) +
            std::chrono::nanoseconds(msg->header.stamp.nanosec));
          const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
            robot_frame_id_, msg->header.frame_id, time_point);
          tf2::doTransform(*msg, transformed_msg, transform); // TODO:slow now(https://github.com/ros/geometry2/pull/432)
        } catch (tf2::TransformException & e) {
          RCLCPP_ERROR(this->get_logger(), "%s", e.what());
          return;
        }

        std::vector<float> raw_point_times;
        const auto time_field = std::find_if(
          transformed_msg.fields.begin(), transformed_msg.fields.end(),
          [](const sensor_msgs::msg::PointField & field) {
            return field.name == "time" &&
                   field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1;
          });
        if (time_field != transformed_msg.fields.end()) {
          try {
            sensor_msgs::PointCloud2ConstIterator<float> time_it(transformed_msg, "time");
            const std::size_t point_count =
              static_cast<std::size_t>(transformed_msg.width) * transformed_msg.height;
            raw_point_times.reserve(point_count);
            for (std::size_t i = 0; i < point_count; ++i, ++time_it) {
              raw_point_times.push_back(*time_it);
            }
          } catch (const std::runtime_error & error) {
            raw_point_times.clear();
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 30000,
              "Cannot read VLP-16 point time field; using azimuth fallback: %s", error.what());
          }
        } else if (use_imu_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 30000,
            "VLP-16 cloud has no FLOAT32 time field; IMU deskew is using azimuth fallback");
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(transformed_msg, *tmp_ptr);

        // Registration backends do not safely accept NaN/Inf points.  Keep
        // malformed packets out before range and voxel filtering.
        pcl::PointCloud<pcl::PointXYZI>::Ptr finite_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<int> finite_indices;
        pcl::removeNaNFromPointCloud(*tmp_ptr, *finite_ptr, finite_indices);
        tmp_ptr = finite_ptr;

        std::vector<float> finite_point_times;
        if (raw_point_times.size() ==
          static_cast<std::size_t>(transformed_msg.width) * transformed_msg.height)
        {
          finite_point_times.reserve(finite_indices.size());
          for (const int index : finite_indices) {
            if (index >= 0 && static_cast<std::size_t>(index) < raw_point_times.size()) {
              finite_point_times.push_back(raw_point_times[static_cast<std::size_t>(index)]);
            }
          }
        }

        if (use_imu_) {
          double scan_time = msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;
          lidar_undistortion_.adjustDistortion(tmp_ptr, scan_time, finite_point_times);
        }

        if (use_min_max_filter_) {
          double r;
          pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr2(new pcl::PointCloud<pcl::PointXYZI>());
          for (const auto & p : tmp_ptr->points) {
            r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
            if (scan_min_range_ < r && r < scan_max_range_) {tmp_ptr2->points.push_back(p);}
          }
          tmp_ptr = tmp_ptr2;
        }


        if (!initial_cloud_received_) {
          RCLCPP_INFO(get_logger(), "create a first map");
          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
          pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
          voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
          voxel_grid.setInputCloud(tmp_ptr);
          voxel_grid.filter(*cloud_ptr);

          if (cloud_ptr->size() < static_cast<std::size_t>(minimum_registration_points_)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Waiting for a usable first LiDAR cloud: %zu/%d filtered points",
              cloud_ptr->size(), minimum_registration_points_);
            return;
          }

          initial_cloud_received_ = true;

          Eigen::Matrix4f sim_trans = getTransformation(corrent_pose_stamped_.pose);
          pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
          pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
          registration_->setInputTarget(transformed_cloud_ptr);

          // map
          sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
          pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);
          map_msg_ptr->header.frame_id = global_frame_id_;
          map_msg_ptr->header.stamp = msg->header.stamp;

          // map array
          sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
          pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
          lidarslam_msgs::msg::SubMap submap;
          submap.header.frame_id = global_frame_id_;
          submap.header.stamp = msg->header.stamp;
          submap.distance = 0;
          submap.pose = corrent_pose_stamped_.pose;
          submap.cloud = *cloud_msg_ptr;
          submap.cloud.header.frame_id = robot_frame_id_;
          submap.cloud.header.stamp = msg->header.stamp;
          map_array_msg_.header.frame_id = global_frame_id_;
          map_array_msg_.header.stamp = msg->header.stamp;
          map_array_msg_.submaps.push_back(submap);

          previous_position_ = Eigen::Vector3d(
            submap.pose.position.x, submap.pose.position.y, submap.pose.position.z);
          previous_orientation_ = Eigen::Quaterniond(
            submap.pose.orientation.w, submap.pose.orientation.x,
            submap.pose.orientation.y, submap.pose.orientation.z).normalized();

          map_pub_->publish(*map_msg_ptr);
          // Publish the first submap immediately.  The graph saver must never
          // see an empty graph merely because the robot has not yet crossed
          // the next map-update threshold.
          map_array_pub_->publish(map_array_msg_);

          last_map_time_ = clock_.now();

        }

        if (initial_cloud_received_) {receiveCloud(tmp_ptr, msg->header.stamp);}
      }

    };

  auto imu_callback =
    [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {receiveImu(*msg);}
    };

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_pose", rclcpp::QoS(10), initial_pose_callback);

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(), imu_callback);

  input_cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);

  // pub
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose",
    rclcpp::QoS(10));
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(10));
  map_array_pub_ =
    create_publisher<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable().transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));
}

void ScanMatcherComponent::receiveCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const rclcpp::Time stamp)
{
  if (mapping_flag_ && mapping_future_.valid()) {
    auto status = mapping_future_.wait_for(0s);
    if (status == std::future_status::ready) {
      try {
        mapping_future_.get();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Map update failed safely: %s", error.what());
        is_map_updated_ = false;
      }
      if (mapping_thread_.joinable()) {
        mapping_thread_.join();
      }
      if (is_map_updated_ == true) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(
            targeted_cloud_));
        if (targeted_cloud_ptr->size() <
          static_cast<std::size_t>(minimum_registration_points_))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Keeping the previous registration target: %zu/%d points",
            targeted_cloud_ptr->size(), minimum_registration_points_);
          is_map_updated_ = false;
          mapping_flag_ = false;
        } else {
          if (registration_method_ == "NDT") {
            registration_->setInputTarget(targeted_cloud_ptr);
          } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(
              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
            voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
            voxel_grid.setInputCloud(targeted_cloud_ptr);
            voxel_grid.filter(*filtered_targeted_cloud_ptr);
            registration_->setInputTarget(filtered_targeted_cloud_ptr);
          }
          is_map_updated_ = false;
        }
      }
      mapping_flag_ = false;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);
  if (filtered_cloud_ptr->size() <
    static_cast<std::size_t>(minimum_registration_points_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping sparse LiDAR cloud: %zu/%d filtered points",
      filtered_cloud_ptr->size(), minimum_registration_points_);
    return;
  }
  registration_->setInputSource(filtered_cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(corrent_pose_stamped_.pose);
  if (!sim_trans.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping LiDAR registration because the initial pose is not finite");
    return;
  }

  if (use_odom_) {
    geometry_msgs::msg::TransformStamped odom_trans;
    try {
      odom_trans = tfbuffer_.lookupTransform(
        odom_frame_id_, robot_frame_id_, tf2_ros::fromMsg(
          stamp));
    } catch (tf2::TransformException & e) {
      // Point clouds can arrive a few milliseconds ahead of the newest odom
      // transform.  For this slow mapping profile, the newest valid transform
      // is a safer motion prior than dropping nearly every cloud.
      try {
        odom_trans = tfbuffer_.lookupTransform(
          odom_frame_id_, robot_frame_id_, tf2::TimePointZero);
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Using latest odom TF for LiDAR cloud: %s", e.what());
      } catch (tf2::TransformException & latest_error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping LiDAR cloud until odom TF is ready: %s", latest_error.what());
        return;
      }
    }
    Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
    Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();
    if (previous_odom_received_) {
      sim_trans = sim_trans * previous_odom_mat_.inverse() * odom_mat;
    }
    previous_odom_mat_ = odom_mat;
    previous_odom_received_ = true;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  try {
    registration_->align(*output_cloud, sim_trans);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "LiDAR registration failed safely: %s", error.what());
    return;
  }
  rclcpp::Time time_align_end = system_clock.now();

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  const double fitness_score = registration_->getFitnessScore();
  if (!registration_->hasConverged() || !final_transformation.allFinite() ||
    !std::isfinite(fitness_score))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping unconverged or non-finite LiDAR registration result (fitness=%.3f)",
      fitness_score);
    return;
  }

  const Eigen::Matrix4f correction = sim_trans.inverse() * final_transformation;
  const double correction_translation = correction.block<3, 1>(0, 3).norm();
  Eigen::Quaternionf correction_quaternion(correction.block<3, 3>(0, 0));
  const double correction_rotation = correction_quaternion.norm() > 1.0e-6f ?
    Eigen::AngleAxisf(correction_quaternion.normalized()).angle() :
    std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(correction_translation) || !std::isfinite(correction_rotation) ||
    (max_registration_translation_correction_ > 0.0 &&
    correction_translation > max_registration_translation_correction_) ||
    (max_registration_rotation_correction_ > 0.0 &&
    correction_rotation > max_registration_rotation_correction_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected implausible LiDAR correction: translation=%.3fm, rotation=%.3frad, "
      "fitness=%.3f",
      correction_translation, correction_rotation, fitness_score);
    return;
  }

  publishMapAndPose(cloud_ptr, final_transformation, stamp);

  if (!debug_flag_) {return;}

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "3D registration: points=%zu, fitness=%.3f, correction=%.3fm/%.3frad, "
    "align=%.3fs, submaps=%zu",
    filtered_cloud_ptr->size(), fitness_score, correction_translation, correction_rotation,
    time_align_end.seconds() - time_align_start.seconds(), map_array_msg_.submaps.size());
}

void ScanMatcherComponent::publishMapAndPose(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
{

  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  if(publish_tf_){
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = global_frame_id_;
    transform_stamped.child_frame_id = robot_frame_id_;
    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();
    transform_stamped.transform.rotation = quat_msg;
    broadcaster_.sendTransform(transform_stamped);
  }

  corrent_pose_stamped_.header.stamp = stamp;
  corrent_pose_stamped_.pose.position.x = position.x();
  corrent_pose_stamped_.pose.position.y = position.y();
  corrent_pose_stamped_.pose.position.z = position.z();
  corrent_pose_stamped_.pose.orientation = quat_msg;
  pose_pub_->publish(corrent_pose_stamped_);

  path_.poses.push_back(corrent_pose_stamped_);
  path_pub_->publish(path_);

  trans_ = (position - previous_position_).norm();
  Eigen::Quaterniond current_orientation(quat_eig.normalized());
  const double orientation_dot = std::min(
    1.0, std::abs(previous_orientation_.dot(current_orientation)));
  const double rotation = 2.0 * std::acos(orientation_dot);
  if ((trans_ >= trans_for_mapupdate_ || rotation >= rotation_for_mapupdate_) &&
    !mapping_flag_)
  {
    geometry_msgs::msg::PoseStamped corrent_pose_stamped;
    corrent_pose_stamped = corrent_pose_stamped_;
    previous_position_ = position;
    previous_orientation_ = current_orientation;
    mapping_task_ =
      std::packaged_task<void()>(
      std::bind(
        &ScanMatcherComponent::updateMap, this, cloud_ptr,
        final_transformation, corrent_pose_stamped));
    mapping_future_ = mapping_task_.get_future();
    mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
    mapping_flag_ = true;
  }
}

void ScanMatcherComponent::updateMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
  const Eigen::Matrix4f final_transformation,
  const geometry_msgs::msg::PoseStamped corrent_pose_stamped)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);
  if (filtered_cloud_ptr->size() <
    static_cast<std::size_t>(minimum_registration_points_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping sparse map update: %zu/%d filtered points",
      filtered_cloud_ptr->size(), minimum_registration_points_);
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

  targeted_cloud_.clear();
  targeted_cloud_ += *transformed_cloud_ptr;
  int num_submaps = map_array_msg_.submaps.size();
  for (int i = 0; i < num_targeted_cloud_ - 1; i++) {
    if (num_submaps - 1 - i < 0) {continue;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    targeted_cloud_ += *transformed_tmp_ptr;
  }

  /* map array */
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

  lidarslam_msgs::msg::SubMap submap;
  submap.header.frame_id = global_frame_id_;
  submap.header.stamp = corrent_pose_stamped.header.stamp;
  latest_distance_ += trans_;
  submap.distance = latest_distance_;
  submap.pose = corrent_pose_stamped.pose;
  submap.cloud = *cloud_msg_ptr;
  // Submap points are stored in the robot-local coordinates indicated by
  // map_array_msg_.cloud_coordinate; submap.pose places them in the map.
  submap.cloud.header.frame_id = robot_frame_id_;
  submap.cloud.header.stamp = corrent_pose_stamped.header.stamp;
  map_array_msg_.header.stamp = corrent_pose_stamped.header.stamp;
  map_array_msg_.submaps.push_back(submap);
  map_array_pub_->publish(map_array_msg_);

  is_map_updated_ = true;

  rclcpp::Time map_time = clock_.now();
  double dt = map_time.seconds() - last_map_time_.seconds();
  if (dt > map_publish_period_) {
    publishMap();
    last_map_time_ = map_time;
  }
}

Eigen::Matrix4f ScanMatcherComponent::getTransformation(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();
  return sim_trans;
}

void ScanMatcherComponent::receiveImu(const sensor_msgs::msg::Imu msg)
{
  if (!use_imu_) {return;}

  double roll, pitch, yaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(msg.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = static_cast<float>(msg.linear_acceleration.x) + sin(pitch) * 9.81;
  float acc_y = static_cast<float>(msg.linear_acceleration.y) - cos(pitch) * sin(roll) * 9.81;
  float acc_z = static_cast<float>(msg.linear_acceleration.z) - cos(pitch) * cos(roll) * 9.81;

  Eigen::Vector3f angular_velo{
    static_cast<float>(msg.angular_velocity.x),
    static_cast<float>(msg.angular_velocity.y),
    static_cast<float>(msg.angular_velocity.z)};
  Eigen::Vector3f acc{acc_x, acc_y, acc_z};
  Eigen::Quaternionf quat{
    static_cast<float>(msg.orientation.w),
    static_cast<float>(msg.orientation.x),
    static_cast<float>(msg.orientation.y),
    static_cast<float>(msg.orientation.z)};
  double imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void ScanMatcherComponent::publishMap()
{
  RCLCPP_INFO(get_logger(), "publish a map");

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto & submap : map_array_msg_.submaps) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);
    
    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(
      *submap_cloud_ptr, *transformed_submap_cloud_ptr,
      affine.matrix().cast<float>());

    *map_ptr += *transformed_submap_cloud_ptr;
  }
  RCLCPP_INFO(get_logger(), "Published accumulated 3D map: %zu points", map_ptr->size());

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = global_frame_id_;
  map_pub_->publish(*map_msg_ptr);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
