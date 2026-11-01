// Copyright 2024 SC_PGO_ROS2 Contributors
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>  // Fix: Required for savePCDFileBinary
#include <pcl/filters/filter.h>  // Fix: Required for removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>
// #include <pcl/octree/octree_pointcloud_voxelcentroid.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <sensor_msgs/Imu.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

// #include <eigen3/Eigen/Dense>

// #include <ceres/ceres.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "scancontext/Scancontext.h"
#include "sc_pgo/loop_constraint.hpp"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0;
double rotaionAccumulated = 1000000.0;

bool isNowKeyFrame = false;

Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init
Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init pose is zero

std::queue<std::shared_ptr<nav_msgs::msg::Odometry>> odometryBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> fullResBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::NavSatFix>> gpsBuf;
std::queue<std::pair<int, int>> scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(
  new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
std::vector<double> keyframeArcLengths;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::NonlinearFactorGraph archivedGraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
std::unique_ptr<gtsam::ISAM2> isam;  // Fix: Use smart pointer instead of raw pointer
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

bool publish_tf = false;
bool use_current_stamp_for_aft_pgo_odom = false;
bool persist_keyframe_pcd = false;
std::string map_frame_id = "map";
double maximum_sync_error_sec = 0.03;
double minimum_loop_arc_length = 8.0;
int stable_loop_candidate_count = 2;
int stable_loop_candidate_tolerance = 2;
double icp_max_correspondence_distance = 1.5;
double icp_fitness_threshold = 0.20;
double icp_minimum_overlap = 0.45;
double maximum_loop_translation_correction = 2.0;
double maximum_loop_z_correction = 0.75;
double maximum_loop_rotation_correction = 0.78539816339;
double maximum_adjacent_translation_deformation = 0.25;
double maximum_adjacent_rotation_deformation = 0.15;
int loop_history_keyframes = 10;
std::atomic<bool> graph_dirty{false};
std::atomic<int> accepted_loop_count{0};
std::atomic<int> rejected_loop_count{0};
// Manual mapping launches FAST-LIO/SC-PGO before the operator presses Xbox A
// so the estimator can warm up.  Only keyframes belonging to the A..B mapping
// session may be committed to disk; otherwise the stationary warm-up drift is
// incorrectly counted as driven track length.
std::atomic<std::size_t> mapping_session_start_index{0};
std::atomic<bool> mapping_session_boundary_set{false};
std::atomic<int> mapping_session_accepted_loop_baseline{0};
std::atomic<int> mapping_session_rejected_loop_baseline{0};

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(
  new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
std::atomic<bool> laserCloudMapPGORedraw{true};

bool useGPS = true;
// bool useGPS = false;
sensor_msgs::msg::NavSatFix::SharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;

std::shared_ptr<rclcpp::Node> nh;

// Fix: Shutdown flag for graceful thread termination
std::atomic<bool> shutdown_requested{false};

std::string padZeros(int val, int num_digits = 6)
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
    gtsam::Point3(p.x, p.y, p.z));
}  // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);
  for (const auto & _pose6d : keyframePoses) {
    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void saveOptimizedVerticesKITTIformat(
  gtsam::Values _estimates,
  std::string _filename)
{
  using namespace gtsam;

  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);

  for (const auto & key_value : _estimates) {
    auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
    if (!p) {continue;}

    const Pose3 & pose = p->value();

    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void laserOdometryHandler(
  const nav_msgs::msg::Odometry::SharedPtr _laserOdometry)
{
  std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
  odometryBuf.push(_laserOdometry);
}  // laserOdometryHandler

void laserCloudFullResHandler(
  sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes)
{
  std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
  fullResBuf.push(_laserCloudFullRes);
}  // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps)
{
  if (useGPS) {
    std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
    gpsBuf.push(_gps);
  }
}  // gpsHandler

void initNoises(void)
{
  gtsam::Vector priorNoiseVector6(6);
  priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

  gtsam::Vector odomNoiseVector6(6);
  // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
  odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

  double loopNoiseScore = 0.5;  // constant is ok...
  gtsam::Vector robustNoiseVector6(
    6);    // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
    loopNoiseScore, loopNoiseScore, loopNoiseScore;
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(
      1),      // optional: replacing Cauchy by DCS or GemanMcClure is okay but
               // Cauchy is empirically good.
    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  double bigNoiseTolerentToXY = 1000000000.0;  // 1e9
  double gpsAltitudeNoiseScore = 250.0;  // if height is misaligned after loop
                                         // clsosing, use this value bigger
  gtsam::Vector robustNoiseVector3(3);   // gps factor has 3 elements (xyz)
  robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
    gpsAltitudeNoiseScore;    // means only caring altitude here. (because
                              // LOAM-like-methods tends to be asymptotically
                              // flyging)
  robustGPSNoise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(
      1),      // optional: replacing Cauchy by DCS or GemanMcClure is okay but
               // Cauchy is empirically good.
    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}  // initNoises

Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr & _odom)
{
  auto tx = _odom->pose.pose.position.x;
  auto ty = _odom->pose.pose.position.y;
  auto tz = _odom->pose.pose.position.z;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  return Pose6D{tx, ty, tz, roll, pitch, yaw};
}  // getOdom

Pose6D diffTransformation(const Pose6D & _p1, const Pose6D & _p2)
{
  Eigen::Affine3f SE3_p1 =
    pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
  Eigen::Affine3f SE3_p2 =
    pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
  Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
  Eigen::Affine3f SE3_delta;
  SE3_delta.matrix() = SE3_delta0;
  float dx, dy, dz, droll, dpitch, dyaw;
  pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
  // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll
  // << ", " << dpitch << ", " << dyaw << std::endl;

  return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)),
    double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
}  // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(
  const pcl::PointCloud<PointType>::Ptr & cloudIn, const Pose6D & tf)
{
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur =
    pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

  int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto & pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
      transCur(0, 1) * pointFrom.y +
      transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
      transCur(1, 1) * pointFrom.y +
      transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
      transCur(2, 1) * pointFrom.y +
      transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }

  return cloudOut;
}

void pubPath(void)
{
  // Publish odom and path
  nav_msgs::msg::Odometry odomAftPGO;
  nav_msgs::msg::Path pathAftPGO;
  pathAftPGO.header.frame_id = map_frame_id;
  mKF.lock();
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    const Pose6D & pose_est =
      keyframePosesUpdated.at(node_idx);    // Updated poses

    nav_msgs::msg::Odometry odomAftPGOthis;
    odomAftPGOthis.header.frame_id = map_frame_id;
    odomAftPGOthis.child_frame_id = "aft_pgo";
    odomAftPGOthis.header.stamp =
      rclcpp::Time(keyframeTimes.at(node_idx) * 1e9);
    odomAftPGOthis.pose.pose.position.x = pose_est.x;
    odomAftPGOthis.pose.pose.position.y = pose_est.y;
    odomAftPGOthis.pose.pose.position.z = pose_est.z;

    tf2::Quaternion q;
    q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
    odomAftPGOthis.pose.pose.orientation.x = q.x();
    odomAftPGOthis.pose.pose.orientation.y = q.y();
    odomAftPGOthis.pose.pose.orientation.z = q.z();
    odomAftPGOthis.pose.pose.orientation.w = q.w();
    odomAftPGO = odomAftPGOthis;

    geometry_msgs::msg::PoseStamped poseStampAftPGO;
    poseStampAftPGO.header = odomAftPGOthis.header;
    poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

    pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
    pathAftPGO.header.frame_id = map_frame_id;
    pathAftPGO.poses.push_back(poseStampAftPGO);
  }
  mKF.unlock();
  if (use_current_stamp_for_aft_pgo_odom) {
    odomAftPGO.header.stamp = nh->get_clock()->now();
  }
  pubOdomAftPGO->publish(odomAftPGO);  // Last pose
  pubPathAftPGO->publish(pathAftPGO);  // Poses

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = odomAftPGO.header.stamp;
  transformStamped.header.frame_id = map_frame_id;
  transformStamped.child_frame_id = "aft_pgo";
  transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
  transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
  transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
  transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;

  if (publish_tf) {
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br = nullptr;
    if (!br) {
      br = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
    }
    br->sendTransform(transformStamped);
  }
}  // pubPath

void updatePoses(void)
{
  mKF.lock();
  for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size());
    node_idx++)
  {
    Pose6D & p = keyframePosesUpdated[node_idx];
    p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
    p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
    p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
    p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
    p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
    p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
  }
  mKF.unlock();

  mtxRecentPose.lock();
  const gtsam::Pose3 & lastOptimizedPose =
    isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size()) - 1);
  recentOptimizedX = lastOptimizedPose.translation().x();
  recentOptimizedY = lastOptimizedPose.translation().y();

  // Fix: Set to size() instead of size()-1 so loops with < recentIdxUpdated include all keyframes
  recentIdxUpdated = int(keyframePosesUpdated.size());
  laserCloudMapPGORedraw = true;

  mtxRecentPose.unlock();
}  // updatePoses

void runISAM2opt(void)
{
  // called when a variable added
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  isamCurrentEstimate = isam->calculateEstimate();
  updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(
  pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType * pointFrom;

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(
    transformIn.translation().x(), transformIn.translation().y(),
    transformIn.translation().z(), transformIn.rotation().roll(),
    transformIn.rotation().pitch(), transformIn.rotation().yaw());

  int numberOfCores = 8;  // TODO move to yaml
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom->x +
      transCur(0, 1) * pointFrom->y +
      transCur(0, 2) * pointFrom->z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom->x +
      transCur(1, 1) * pointFrom->y +
      transCur(1, 2) * pointFrom->z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom->x +
      transCur(2, 1) * pointFrom->y +
      transCur(2, 2) * pointFrom->z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom->intensity;
  }
  return cloudOut;
}  // transformPointCloud

void loopFindNearKeyframesCloud(
  pcl::PointCloud<PointType>::Ptr & nearKeyframes,
  const int & key, const int & submap_size)
{
  // extract and stacking near keyframes (in global coord)
  nearKeyframes->clear();
  for (int i = -submap_size; i <= submap_size; ++i) {
    int keyNear = key + i;
    if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size())) {continue;}

    mKF.lock();
    // Fix: Use pose corresponding to each keyNear, not root_idx
    *nearKeyframes += *local2global(
      keyframeLaserClouds[keyNear],
      keyframePosesUpdated[keyNear]);
    mKF.unlock();
  }

  if (nearKeyframes->empty()) {return;}

  // downsample near keyframes
  pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
  downSizeFilterICP.setInputCloud(nearKeyframes);
  downSizeFilterICP.filter(*cloud_temp);
  *nearKeyframes = *cloud_temp;
}  // loopFindNearKeyframesCloud

std::optional<gtsam::Pose3> doICPVirtualRelative(
  int _loop_kf_idx,
  int _curr_kf_idx)
{
  // parse pointclouds
  const int historyKeyframeSearchNum = loop_history_keyframes;
  pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
    new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(
    new pcl::PointCloud<PointType>());
  loopFindNearKeyframesCloud(
    cureKeyframeCloud, _curr_kf_idx, 0);
  loopFindNearKeyframesCloud(
    targetKeyframeCloud, _loop_kf_idx,
    historyKeyframeSearchNum);

  // loop verification
  sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
  pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
  cureKeyframeCloudMsg.header.frame_id = map_frame_id;
  pubLoopScanLocal->publish(cureKeyframeCloudMsg);

  sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
  pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
  targetKeyframeCloudMsg.header.frame_id = map_frame_id;
  pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // Align pointclouds
  icp.setInputSource(cureKeyframeCloud);
  icp.setInputTarget(targetKeyframeCloud);
  pcl::PointCloud<PointType>::Ptr unused_result(
    new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  const double fitness = icp.getFitnessScore(icp_max_correspondence_distance);
  if (icp.hasConverged() == false || !std::isfinite(fitness) ||
    fitness >= std::numeric_limits<double>::max())
  {
    RCLCPP_WARN_THROTTLE(
      nh->get_logger(), *nh->get_clock(), 5000,
      "Reject loop %d->%d: ICP found no valid correspondence; "
      "waiting for overlapping keyframes",
      _loop_kf_idx, _curr_kf_idx);
    rejected_loop_count++;
    return std::nullopt;
  }
  if (fitness > icp_fitness_threshold) {
    RCLCPP_WARN_THROTTLE(
      nh->get_logger(), *nh->get_clock(), 5000,
      "Reject loop %d->%d: ICP fitness %.4f (limit %.4f)",
      _loop_kf_idx, _curr_kf_idx, fitness, icp_fitness_threshold);
    rejected_loop_count++;
    return std::nullopt;
  }

  pcl::KdTreeFLANN<PointType> target_tree;
  target_tree.setInputCloud(targetKeyframeCloud);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_distance(1);
  std::size_t overlap_count = 0;
  const double max_distance_sq =
    icp_max_correspondence_distance * icp_max_correspondence_distance;
  for (const auto & point : unused_result->points) {
    if (target_tree.nearestKSearch(point, 1, nearest_index, nearest_distance) > 0 &&
      nearest_distance.front() <= max_distance_sq)
    {
      ++overlap_count;
    }
  }
  const double overlap = unused_result->empty() ? 0.0 :
    static_cast<double>(overlap_count) / static_cast<double>(unused_result->size());
  if (overlap < icp_minimum_overlap) {
    RCLCPP_WARN(
      nh->get_logger(), "Reject loop %d->%d: overlap %.3f (limit %.3f)",
      _loop_kf_idx, _curr_kf_idx, overlap, icp_minimum_overlap);
    rejected_loop_count++;
    return std::nullopt;
  }

  // Get pose transformation
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionLidarFrame;
  correctionLidarFrame = icp.getFinalTransformation();
  pcl::getTranslationAndEulerAngles(
    correctionLidarFrame, x, y, z, roll, pitch,
    yaw);
  const double translation = std::sqrt(x * x + y * y + z * z);
  const double maximum_rotation = std::max(
    {std::abs(static_cast<double>(roll)), std::abs(static_cast<double>(pitch)),
      std::abs(static_cast<double>(yaw))});
  if (translation > maximum_loop_translation_correction ||
    std::abs(static_cast<double>(z)) > maximum_loop_z_correction ||
    maximum_rotation > maximum_loop_rotation_correction)
  {
    RCLCPP_WARN(
      nh->get_logger(),
      "Reject loop %d->%d: correction xyz=%.3f/%.3f/%.3f rpy=%.3f/%.3f/%.3f",
      _loop_kf_idx, _curr_kf_idx, x, y, z, roll, pitch, yaw);
    rejected_loop_count++;
    return std::nullopt;
  }

  RCLCPP_INFO(
    nh->get_logger(), "Validated loop %d->%d: fitness=%.4f overlap=%.3f",
    _loop_kf_idx, _curr_kf_idx, fitness, overlap);
  const gtsam::Pose3 global_correction =
    Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 previous_global_pose;
  gtsam::Pose3 current_global_pose;
  {
    std::lock_guard<std::mutex> lock(mKF);
    previous_global_pose =
      Pose6DtoGTSAMPose3(keyframePosesUpdated.at(_loop_kf_idx));
    current_global_pose =
      Pose6DtoGTSAMPose3(keyframePosesUpdated.at(_curr_kf_idx));
  }
  return sc_pgo::correctedRelativePose(
    previous_global_pose, current_global_pose, global_correction);
}  // doICPVirtualRelative

template<typename PointT>
void removeNaNAndInfiniteInPlace(typename pcl::PointCloud<PointT>::Ptr & cloud)
{
  if (!cloud || cloud->empty()) {return;}

  // First pass: remove NaNs using PCL’s built-in
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Second pass: remove infinities in-place
  size_t write_idx = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      cloud->points[write_idx++] = pt;
    }
  }

  cloud->points.resize(write_idx);
  cloud->width = static_cast<uint32_t>(write_idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

void process_pg()
{
  while (!shutdown_requested && rclcpp::ok()) {  // Fix: Check shutdown flag
    while (true) {
      //
      // pop and check keyframe is or not
      //
      std::unique_lock<std::mutex> buffer_lock(mBuf);
      if (odometryBuf.empty() || fullResBuf.empty()) {
        break;
      }

      const double candidate_odom_time =
        rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
      timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
      const double synchronization_error = candidate_odom_time - timeLaser;
      if (synchronization_error < -maximum_sync_error_sec) {
        odometryBuf.pop();
        continue;
      }
      if (synchronization_error > maximum_sync_error_sec) {
        fullResBuf.pop();
        continue;
      }
      timeLaserOdometry = candidate_odom_time;

      laserCloudFullRes->clear();
      pcl::PointCloud<PointType>::Ptr thisKeyFrame(
        new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
      fullResBuf.pop();

      Pose6D pose_curr = getOdom(odometryBuf.front());
      odometryBuf.pop();

      // find nearest gps
      double eps = 0.1;  // find a gps topioc arrived within eps second
      while (!gpsBuf.empty()) {
        auto thisGPS = gpsBuf.front();
        auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
        if (abs(thisGPSTime - timeLaserOdometry) < eps) {
          currGPS = thisGPS;
          hasGPSforThisKF = true;
          break;
        } else {
          hasGPSforThisKF = false;
        }
        gpsBuf.pop();
      }
      buffer_lock.unlock();

      //
      // Early reject by counting local delta movement (for equi-spereated kf
      // drop)
      //
      odom_pose_prev = odom_pose_curr;
      odom_pose_curr = pose_curr;
      Pose6D dtf = diffTransformation(
        odom_pose_prev, odom_pose_curr);    // dtf means delta_transform

      double delta_translation = sqrt(
        dtf.x * dtf.x + dtf.y * dtf.y +
        dtf.z * dtf.z);                                // note: absolute value.
      translationAccumulated += delta_translation;
      rotaionAccumulated +=
        (dtf.roll + dtf.pitch + dtf.yaw);    // sum just naive approach.

      if (translationAccumulated > keyframeMeterGap ||
        rotaionAccumulated > keyframeRadGap)
      {
        isNowKeyFrame = true;
        translationAccumulated = 0.0;  // reset
        rotaionAccumulated = 0.0;      // reset
      } else {
        isNowKeyFrame = false;
      }

      if (!isNowKeyFrame) {continue;}

      if (!gpsOffsetInitialized) {
        if (hasGPSforThisKF) {  // if the very first frame
          gpsAltitudeInitOffset = currGPS->altitude;
          gpsOffsetInitialized = true;
        }
      }

      //
      // Save data and Add consecutive node
      //
      pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(
        new pcl::PointCloud<PointType>());
      downSizeFilterScancontext.setInputCloud(thisKeyFrame);
      downSizeFilterScancontext.filter(*thisKeyFrameDS);
      removeNaNAndInfiniteInPlace<PointType>(thisKeyFrameDS);

      mKF.lock();
      keyframeLaserClouds.push_back(thisKeyFrameDS);
      keyframePoses.push_back(pose_curr);
      keyframePosesUpdated.push_back(pose_curr);  // init
      keyframeTimes.push_back(timeLaserOdometry);
      double arc_length = 0.0;
      if (!keyframeArcLengths.empty()) {
        const Pose6D & previous_pose = keyframePoses.at(keyframePoses.size() - 2);
        const double dx = pose_curr.x - previous_pose.x;
        const double dy = pose_curr.y - previous_pose.y;
        const double dz = pose_curr.z - previous_pose.z;
        arc_length = keyframeArcLengths.back() + std::sqrt(dx * dx + dy * dy + dz * dz);
      }
      keyframeArcLengths.push_back(arc_length);

      scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

      laserCloudMapPGORedraw = true;
      mKF.unlock();

      const int prev_node_idx = keyframePoses.size() - 2;
      const int curr_node_idx =
        keyframePoses.size() -
        1;    // becuase cpp starts with 0 (actually this index could be any
              // number, but for simple implementation, we follow sequential
              // indexing)
      if (!gtSAMgraphMade /* prior node */) {
        const int init_node_idx = 0;
        gtsam::Pose3 poseOrigin =
          Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
        // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
        // gtsam::Point3(0.0, 0.0, 0.0));

        mtxPosegraph.lock();
        {
          // prior factor
          const gtsam::PriorFactor<gtsam::Pose3> factor(
            init_node_idx, poseOrigin, priorNoise);
          gtSAMgraph.add(factor);
          archivedGraph.add(factor);
          initialEstimate.insert(init_node_idx, poseOrigin);
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

        gtSAMgraphMade = true;
        graph_dirty = true;

        RCLCPP_DEBUG(nh->get_logger(), "posegraph prior node %d added", init_node_idx);
      } else { /* consecutive node (and odom factor) after the prior added */
               // == keyframePoses.size() > 1
        gtsam::Pose3 poseFrom =
          Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
        gtsam::Pose3 poseTo =
          Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

        mtxPosegraph.lock();
        {
          // odom factor
          const gtsam::BetweenFactor<gtsam::Pose3> odometry_factor(
            prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise);
          gtSAMgraph.add(odometry_factor);
          archivedGraph.add(odometry_factor);

          // gps factor
          if (hasGPSforThisKF) {
            double curr_altitude_offseted =
              currGPS->altitude - gpsAltitudeInitOffset;
            mtxRecentPose.lock();
            gtsam::Point3 gpsConstraint(
              recentOptimizedX, recentOptimizedY,
              curr_altitude_offseted);    // in this example, only adjusting
                                          // altitude (for x and y, very big
                                          // noises are set)
            mtxRecentPose.unlock();
            const gtsam::GPSFactor gps_factor(
              curr_node_idx, gpsConstraint, robustGPSNoise);
            gtSAMgraph.add(gps_factor);
            archivedGraph.add(gps_factor);
            RCLCPP_DEBUG(nh->get_logger(), "GPS factor added at node %d", curr_node_idx);
          }
          initialEstimate.insert(curr_node_idx, poseTo);
          graph_dirty = true;
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

      }
      // if want to print the current graph, use gtSAMgraph.print("\nFactor
      // Graph:\n");

      // save utility
      if (persist_keyframe_pcd) {
        const std::string curr_node_idx_str = padZeros(curr_node_idx);
        pcl::io::savePCDFileBinaryCompressed(
          pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrameDS);
        pgTimeSaveStream << timeLaser << std::endl;
      }
    }

    // ps.
    // scan context detector is running in another thread (in constant Hz, e.g.,
    // 1 Hz) pub path and point cloud in another thread

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_pg

void performSCLoopClosure(void)
{
  static int last_evaluated_current = -1;
  static int stable_candidate = -1;
  static int stable_count = 0;
  if (int(keyframePoses.size()) <
    scManager.NUM_EXCLUDE_RECENT)    // do not try too early
  {
    return;
  }

  const int curr_node_idx = keyframePoses.size() - 1;
  // process_lcd runs at 10 Hz while keyframes arrive much more slowly.  Do
  // not repeat Scan Context search and its operator log for the same frame.
  if (curr_node_idx == last_evaluated_current) {
    return;
  }
  last_evaluated_current = curr_node_idx;

  auto detectResult =
    scManager.detectLoopClosureID();    // first: nn index, second: yaw diff
  int SCclosestHistoryFrameID = detectResult.first;
  if (SCclosestHistoryFrameID != -1) {
    const int prev_node_idx = SCclosestHistoryFrameID;
    if (prev_node_idx < 0 || prev_node_idx >= curr_node_idx ||
      keyframeArcLengths.size() <= static_cast<std::size_t>(curr_node_idx))
    {
      return;
    }
    const double separated_arc =
      keyframeArcLengths[curr_node_idx] - keyframeArcLengths[prev_node_idx];
    if (separated_arc < minimum_loop_arc_length) {
      RCLCPP_DEBUG(
        nh->get_logger(), "Reject Scan Context candidate: arc %.2fm < %.2fm",
        separated_arc, minimum_loop_arc_length);
      return;
    }

    if (stable_candidate >= 0 &&
      std::abs(stable_candidate - prev_node_idx) <= stable_loop_candidate_tolerance)
    {
      ++stable_count;
    } else {
      stable_candidate = prev_node_idx;
      stable_count = 1;
    }
    if (stable_count < stable_loop_candidate_count) {
      RCLCPP_INFO(
        nh->get_logger(), "Loop candidate %d->%d awaiting confirmation %d/%d",
        prev_node_idx, curr_node_idx, stable_count, stable_loop_candidate_count);
      return;
    }

    RCLCPP_INFO(
      nh->get_logger(), "Loop candidate confirmed %d->%d after %.2fm",
      prev_node_idx, curr_node_idx, separated_arc);

    std::lock_guard<std::mutex> lock(mBuf);
    scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    // addding actual 6D constraints in the other thread, icp_calculation.
  } else {
    stable_candidate = -1;
    stable_count = 0;
  }
}  // performSCLoopClosure

void process_lcd()
{
  while (!shutdown_requested && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (shutdown_requested || !rclcpp::ok()) {break;}
    performSCLoopClosure();
    // performRSLoopClosure(); // TODO
  }
}

void process_icp(void)
{
  while (!shutdown_requested && rclcpp::ok()) {  // Fix: Check shutdown flag
    while (!scLoopICPBuf.empty()) {
      if (scLoopICPBuf.size() > 30) {
        RCLCPP_WARN(
          nh->get_logger(),
          "Too many loop closure candidates to be ICPed is waiting ... "
          "Do process_lcd less frequently (adjust loopClosureFrequency)");
      }

      mBuf.lock();
      std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
      scLoopICPBuf.pop();
      mBuf.unlock();

      const int prev_node_idx = loop_idx_pair.first;
      const int curr_node_idx = loop_idx_pair.second;
      auto relative_pose_optional =
        doICPVirtualRelative(prev_node_idx, curr_node_idx);

      if (relative_pose_optional) {
        gtsam::Pose3 relative_pose = relative_pose_optional.value();
        mtxPosegraph.lock();
        const gtsam::BetweenFactor<gtsam::Pose3> loop_factor(
          prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise);
        gtSAMgraph.add(loop_factor);
        archivedGraph.add(loop_factor);
        graph_dirty = true;
        accepted_loop_count++;
        // runISAM2opt();
        mtxPosegraph.unlock();
      }
    }

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_icp

void process_viz_path()
{
  while (!shutdown_requested && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (shutdown_requested || !rclcpp::ok()) {break;}
    if (recentIdxUpdated > 1) {
      pubPath();
    }
  }
}

void process_isam()
{
  while (!shutdown_requested && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (shutdown_requested || !rclcpp::ok()) {break;}
    if (gtSAMgraphMade && graph_dirty.exchange(false)) {
      mtxPosegraph.lock();
      runISAM2opt();
      mtxPosegraph.unlock();
      RCLCPP_DEBUG(nh->get_logger(), "iSAM2 graph update complete");
    }
  }
}

void pubMap(void)
{
  int SKIP_FRAMES = 2;  // sparse map visulalization to save computations
  int counter = 0;

  laserCloudMapPGO->clear();

  mKF.lock();
  // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size());
  // node_idx++) {
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    if (counter % SKIP_FRAMES == 0) {
      *laserCloudMapPGO += *local2global(
        keyframeLaserClouds[node_idx],
        keyframePosesUpdated[node_idx]);
    }
    counter++;
  }
  mKF.unlock();

  downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
  downSizeFilterMapPGO.filter(*laserCloudMapPGO);

  sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
  pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
  laserCloudMapPGOMsg.header.frame_id = map_frame_id;
  pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

void process_viz_map()
{
  while (!shutdown_requested && rclcpp::ok()) {
    // Wall-clock sleep avoids ROS clock teardown races after bag playback.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (shutdown_requested || !rclcpp::ok()) {break;}
    if (recentIdxUpdated > 1 && laserCloudMapPGORedraw.exchange(false)) {
      pubMap();
    }
  }
}

bool saveOptimizedBundle(std::string & detail)
{
  try {
    if (!gtSAMgraphMade || keyframePoses.size() < 2) {
      detail = "SC-PGO needs at least two keyframes";
      return false;
    }

    {
      std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
      if (graph_dirty.exchange(false)) {
        runISAM2opt();
      }
    }

    std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
    std::vector<Pose6D> raw_poses;
    std::vector<Pose6D> optimized_poses;
    std::vector<double> times;
    std::vector<Eigen::MatrixXd> scan_contexts;
    std::size_t first_session_keyframe = 0;
    bool source_vectors_consistent = true;
    {
      std::lock_guard<std::mutex> keyframe_lock(mKF);
      source_vectors_consistent =
        keyframeLaserClouds.size() == keyframePoses.size() &&
        keyframePosesUpdated.size() == keyframePoses.size() &&
        keyframeTimes.size() == keyframePoses.size() &&
        scManager.polarcontexts_.size() == keyframePoses.size();
      if (!source_vectors_consistent) {
        first_session_keyframe = 0;
      } else if (mapping_session_boundary_set.load()) {
        first_session_keyframe = std::min(
          mapping_session_start_index.load(), keyframePoses.size());
      }
      clouds.assign(
        keyframeLaserClouds.begin() + first_session_keyframe,
        keyframeLaserClouds.end());
      raw_poses.assign(
        keyframePoses.begin() + first_session_keyframe,
        keyframePoses.end());
      optimized_poses.assign(
        keyframePosesUpdated.begin() + first_session_keyframe,
        keyframePosesUpdated.end());
      times.assign(
        keyframeTimes.begin() + first_session_keyframe,
        keyframeTimes.end());
      scan_contexts.assign(
        scManager.polarcontexts_.begin() + first_session_keyframe,
        scManager.polarcontexts_.end());
    }
    if (!source_vectors_consistent) {
      detail = "SC-PGO source vectors are inconsistent";
      return false;
    }
    if (optimized_poses.size() < 2) {
      detail = "SC-PGO mapping session needs at least two post-A keyframes";
      return false;
    }
    if (optimized_poses.size() != raw_poses.size() || clouds.size() != raw_poses.size()) {
      detail = "SC-PGO keyframe vectors are inconsistent";
      return false;
    }
    if (scan_contexts.size() != raw_poses.size()) {
      detail = "SC-PGO scan-context and keyframe counts are inconsistent";
      return false;
    }

    double maximum_translation_deformation = 0.0;
    double maximum_rotation_deformation = 0.0;
    for (std::size_t index = 1; index < raw_poses.size(); ++index) {
      const gtsam::Pose3 raw_relative =
        Pose6DtoGTSAMPose3(raw_poses[index - 1]).between(
        Pose6DtoGTSAMPose3(raw_poses[index]));
      const gtsam::Pose3 optimized_relative =
        Pose6DtoGTSAMPose3(optimized_poses[index - 1]).between(
        Pose6DtoGTSAMPose3(optimized_poses[index]));
      const gtsam::Pose3 deformation = raw_relative.between(optimized_relative);
      maximum_translation_deformation = std::max(
        maximum_translation_deformation, deformation.translation().norm());
      maximum_rotation_deformation = std::max(
        maximum_rotation_deformation,
        std::max(
          {std::abs(deformation.rotation().roll()),
            std::abs(deformation.rotation().pitch()),
            std::abs(deformation.rotation().yaw())}));
    }
    if (maximum_translation_deformation > maximum_adjacent_translation_deformation ||
      maximum_rotation_deformation > maximum_adjacent_rotation_deformation)
    {
      std::ostringstream message;
      message << "optimized graph has implausible adjacent deformation: translation="
              << maximum_translation_deformation << "m rotation="
              << maximum_rotation_deformation << "rad";
      detail = message.str();
      return false;
    }

    pcl::PointCloud<PointType>::Ptr assembled(new pcl::PointCloud<PointType>());
    for (std::size_t index = 0; index < clouds.size(); ++index) {
      *assembled += *local2global(clouds[index], optimized_poses[index]);
    }
    pcl::PointCloud<PointType> filtered;
    downSizeFilterMapPGO.setInputCloud(assembled);
    downSizeFilterMapPGO.filter(filtered);
    if (filtered.empty()) {
      detail = "optimized map is empty";
      return false;
    }

    const std::filesystem::path directory(save_directory);
    std::filesystem::create_directories(directory);
    const auto dense_map_temp = directory / "optimized_map_dense.pcd.tmp";
    const auto dense_map_final = directory / "map_dense.pcd";
    const auto map_temp = directory / "optimized_map.pcd.tmp";
    const auto map_final = directory / "map.pcd";
    const auto path_temp = directory / "optimized_path.csv.tmp";
    const auto path_final = directory / "optimized_path.csv";
    const auto graph_temp = directory / "pose_graph.g2o.tmp";
    const auto graph_final = directory / "pose_graph.g2o";
    const auto manifest_temp = directory / "pgo_metrics.yaml.tmp";
    const auto manifest_final = directory / "pgo_metrics.yaml";
    const auto poses_temp = directory / "keyframe_poses.bin.tmp";
    const auto poses_final = directory / "keyframe_poses.bin";
    const auto contexts_temp = directory / "scan_context.bin.tmp";
    const auto contexts_final = directory / "scan_context.bin";

    // Binary PCD keeps the map compact enough for field use while remaining
    // directly readable by the deterministic offline Python map processors.
    // PCL's binary_compressed payload requires a PCL-only decompressor and
    // previously made the reviewed map impossible to use for 2D derivation.
    // Keep both products, following the scan/submap/global-map separation
    // used by modern LiDAR mapping viewers such as GLIM.  map_dense.pcd is
    // the assembled optimized keyframe cloud for detailed review, while
    // map.pcd is the lightweight voxelized product used by routine tools.
    if (pcl::io::savePCDFileBinary(dense_map_temp.string(), *assembled) != 0) {
      detail = "failed to write binary dense optimized PCD";
      return false;
    }
    if (pcl::io::savePCDFileBinary(map_temp.string(), filtered) != 0) {
      detail = "failed to write binary optimized PCD";
      return false;
    }
    {
      std::ofstream stream(path_temp);
      stream << "stamp,x,y,z,roll,pitch,yaw\n";
      stream << std::setprecision(17);
      for (std::size_t index = 0; index < optimized_poses.size(); ++index) {
        const auto & pose = optimized_poses[index];
        stream << times[index] << ',' << pose.x << ',' << pose.y << ',' << pose.z
               << ',' << pose.roll << ',' << pose.pitch << ',' << pose.yaw << '\n';
      }
    }
    {
      // Persist a self-contained graph for the selected A..B session.  The
      // estimator may retain its warm-up graph in memory, but the navigation
      // bundle must have the same epoch as the recorded rosbag and centerline.
      gtsam::NonlinearFactorGraph session_graph;
      gtsam::Values session_values;
      for (std::size_t index = 0; index < optimized_poses.size(); ++index) {
        session_values.insert(index, Pose6DtoGTSAMPose3(optimized_poses[index]));
        if (index == 0) {
          session_graph.add(
            gtsam::PriorFactor<gtsam::Pose3>(
              0, Pose6DtoGTSAMPose3(optimized_poses[index]), priorNoise));
        } else {
          const gtsam::Pose3 raw_relative =
            Pose6DtoGTSAMPose3(raw_poses[index - 1]).between(
            Pose6DtoGTSAMPose3(raw_poses[index]));
          session_graph.add(
            gtsam::BetweenFactor<gtsam::Pose3>(
              index - 1, index, raw_relative, odomNoise));
        }
      }
      gtsam::writeG2o(session_graph, session_values, graph_temp.string());
    }
    {
      std::ofstream stream(poses_temp, std::ios::binary);
      // The archive name is part of the on-disk map contract.  The workspace
      // was renamed from EXTREME to ROKA, but the original writer kept the old
      // magic while every validator expected ROKA_PGO1.  That made a complete
      // map fail only after the expensive mapping run had finished.
      const char magic[12] = {'R', 'O', 'K', 'A', '_', 'P', 'G', 'O', '1', '\0', '\0', '\0'};
      const std::uint32_t version = 1;
      const std::uint64_t count = optimized_poses.size();
      stream.write(magic, sizeof(magic));
      stream.write(reinterpret_cast<const char *>(&version), sizeof(version));
      stream.write(reinterpret_cast<const char *>(&count), sizeof(count));
      for (std::size_t index = 0; index < optimized_poses.size(); ++index) {
        const auto & pose = optimized_poses[index];
        const double values[7] = {
          times[index], pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw};
        stream.write(reinterpret_cast<const char *>(values), sizeof(values));
      }
      if (!stream.good()) {
        detail = "failed to write optimized keyframe pose archive";
        return false;
      }
    }
    {
      std::ofstream stream(contexts_temp, std::ios::binary);
      const char magic[12] = {'R', 'O', 'K', 'A', '_', 'S', 'C', '0', '1', '\0', '\0', '\0'};
      const std::uint32_t version = 1;
      const std::uint64_t count = scan_contexts.size();
      stream.write(magic, sizeof(magic));
      stream.write(reinterpret_cast<const char *>(&version), sizeof(version));
      stream.write(reinterpret_cast<const char *>(&count), sizeof(count));
      for (const auto & descriptor : scan_contexts) {
        const std::uint32_t rows = descriptor.rows();
        const std::uint32_t columns = descriptor.cols();
        stream.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
        stream.write(reinterpret_cast<const char *>(&columns), sizeof(columns));
        for (std::uint32_t row = 0; row < rows; ++row) {
          for (std::uint32_t column = 0; column < columns; ++column) {
            const double value = descriptor(row, column);
            stream.write(reinterpret_cast<const char *>(&value), sizeof(value));
          }
        }
      }
      if (!stream.good()) {
        detail = "failed to write scan-context descriptor archive";
        return false;
      }
    }

    const gtsam::Pose3 raw_closure =
      Pose6DtoGTSAMPose3(raw_poses.front()).between(Pose6DtoGTSAMPose3(raw_poses.back()));
    const gtsam::Pose3 optimized_closure =
      Pose6DtoGTSAMPose3(optimized_poses.front()).between(
      Pose6DtoGTSAMPose3(optimized_poses.back()));
    {
      std::ofstream stream(manifest_temp);
      stream << std::setprecision(10)
             << "schema_version: 1\n"
             << "keyframes: " << optimized_poses.size() << "\n"
             << "dense_map_points: " << assembled->size() << "\n"
             << "map_points: " << filtered.size() << "\n"
             << "session_start_keyframe: " << first_session_keyframe << "\n"
             << "discarded_warmup_keyframes: " << first_session_keyframe << "\n"
             << "accepted_loops: "
             << std::max(
        0, accepted_loop_count.load() -
        mapping_session_accepted_loop_baseline.load()) << "\n"
             << "rejected_loops: "
             << std::max(
        0, rejected_loop_count.load() -
        mapping_session_rejected_loop_baseline.load()) << "\n"
             << "raw_closure_translation_m: " << raw_closure.translation().norm() << "\n"
             << "optimized_closure_translation_m: "
             << optimized_closure.translation().norm() << "\n"
             << "maximum_adjacent_translation_deformation_m: "
             << maximum_translation_deformation << "\n"
             << "maximum_adjacent_rotation_deformation_rad: "
             << maximum_rotation_deformation << "\n";
    }

    std::filesystem::rename(dense_map_temp, dense_map_final);
    std::filesystem::rename(map_temp, map_final);
    std::filesystem::rename(path_temp, path_final);
    std::filesystem::rename(graph_temp, graph_final);
    std::filesystem::rename(poses_temp, poses_final);
    std::filesystem::rename(contexts_temp, contexts_final);
    // Publish metrics last: its presence then means every required archive
    // has already been committed in the staging directory.
    std::filesystem::rename(manifest_temp, manifest_final);
    detail = "saved " + std::to_string(assembled->size()) + " dense / " +
      std::to_string(filtered.size()) + " lightweight optimized points and " +
      std::to_string(optimized_poses.size()) + " keyframes";
    return true;
  } catch (const std::exception & error) {
    detail = error.what();
    return false;
  }
}

void startMappingSessionService(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::size_t start_index = 0;
  {
    std::lock_guard<std::mutex> keyframe_lock(mKF);
    // Retain the most recent warm-up keyframe as the A-point anchor.  Every
    // earlier keyframe is excluded from map, path, graph and localization
    // archives when B saves the bundle.
    start_index = keyframePoses.empty() ? 0 : keyframePoses.size() - 1;
  }
  mapping_session_start_index.store(start_index);
  mapping_session_accepted_loop_baseline.store(accepted_loop_count.load());
  mapping_session_rejected_loop_baseline.store(rejected_loop_count.load());
  mapping_session_boundary_set.store(true);
  response->success = true;
  response->message =
    "SC-PGO A-point accepted; warm-up keyframes before index " +
    std::to_string(start_index) + " will not be saved";
  RCLCPP_INFO(nh->get_logger(), "%s", response->message.c_str());
}

void saveOptimizedBundleService(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = saveOptimizedBundle(response->message);
  if (response->success) {
    RCLCPP_INFO(nh->get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_ERROR(nh->get_logger(), "SC-PGO save rejected: %s", response->message.c_str());
  }
}

template<typename Worker>
void runWorkerSafely(const char * name, Worker worker)
{
  try {
    worker();
  } catch (const std::exception & error) {
    if (!shutdown_requested && rclcpp::ok()) {
      RCLCPP_ERROR(nh->get_logger(), "SC-PGO worker %s failed: %s", name, error.what());
    }
  } catch (...) {
    if (!shutdown_requested && rclcpp::ok()) {
      RCLCPP_ERROR(nh->get_logger(), "SC-PGO worker %s failed with an unknown exception", name);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Set the worker stop flag from rclcpp's signal path as well as after spin().
  // This prevents worker joins from depending on the exact executor shutdown
  // order when rosbag playback has already stopped.
  rclcpp::on_shutdown([]() {shutdown_requested = true;});
  nh = rclcpp::Node::make_shared("laserPGO");

  nh->declare_parameter<std::string>("save_directory", "/");
  save_directory = nh->get_parameter("save_directory").as_string();

  nh->declare_parameter<std::string>("map_frame", "map");
  map_frame_id = nh->get_parameter("map_frame").as_string();

  nh->declare_parameter<double>("keyframe_meter_gap", 2.0);
  keyframeMeterGap = nh->get_parameter("keyframe_meter_gap").as_double();

  nh->declare_parameter<double>("keyframe_deg_gap", 10.0);
  keyframeDegGap = nh->get_parameter("keyframe_deg_gap").as_double();

  nh->declare_parameter<bool>("publish_tf", false);
  publish_tf = nh->get_parameter("publish_tf").as_bool();
  nh->declare_parameter<bool>("use_current_stamp_for_aft_pgo_odom", false);
  use_current_stamp_for_aft_pgo_odom =
    nh->get_parameter("use_current_stamp_for_aft_pgo_odom").as_bool();

  nh->declare_parameter<bool>("persist_keyframe_pcd", false);
  persist_keyframe_pcd = nh->get_parameter("persist_keyframe_pcd").as_bool();
  nh->declare_parameter<bool>("use_gps", false);
  useGPS = nh->get_parameter("use_gps").as_bool();
  nh->declare_parameter<double>("maximum_sync_error_sec", 0.03);
  maximum_sync_error_sec = nh->get_parameter("maximum_sync_error_sec").as_double();
  nh->declare_parameter<double>("minimum_loop_arc_length", 8.0);
  minimum_loop_arc_length = nh->get_parameter("minimum_loop_arc_length").as_double();
  nh->declare_parameter<int>("stable_loop_candidate_count", 2);
  stable_loop_candidate_count =
    nh->get_parameter("stable_loop_candidate_count").as_int();
  nh->declare_parameter<int>("stable_loop_candidate_tolerance", 2);
  stable_loop_candidate_tolerance =
    nh->get_parameter("stable_loop_candidate_tolerance").as_int();
  nh->declare_parameter<double>("icp_max_correspondence_distance", 1.5);
  icp_max_correspondence_distance =
    nh->get_parameter("icp_max_correspondence_distance").as_double();
  nh->declare_parameter<double>("icp_fitness_threshold", 0.20);
  icp_fitness_threshold = nh->get_parameter("icp_fitness_threshold").as_double();
  nh->declare_parameter<double>("icp_minimum_overlap", 0.45);
  icp_minimum_overlap = nh->get_parameter("icp_minimum_overlap").as_double();
  nh->declare_parameter<double>("maximum_loop_translation_correction", 2.0);
  maximum_loop_translation_correction =
    nh->get_parameter("maximum_loop_translation_correction").as_double();
  nh->declare_parameter<double>("maximum_loop_z_correction", 0.75);
  maximum_loop_z_correction =
    nh->get_parameter("maximum_loop_z_correction").as_double();
  nh->declare_parameter<double>("maximum_loop_rotation_correction", 0.78539816339);
  maximum_loop_rotation_correction =
    nh->get_parameter("maximum_loop_rotation_correction").as_double();
  nh->declare_parameter<int>("loop_history_keyframes", 10);
  loop_history_keyframes = nh->get_parameter("loop_history_keyframes").as_int();
  nh->declare_parameter<double>("maximum_adjacent_translation_deformation", 0.25);
  maximum_adjacent_translation_deformation =
    nh->get_parameter("maximum_adjacent_translation_deformation").as_double();
  nh->declare_parameter<double>("maximum_adjacent_rotation_deformation", 0.15);
  maximum_adjacent_rotation_deformation =
    nh->get_parameter("maximum_adjacent_rotation_deformation").as_double();

  const std::filesystem::path output_directory(save_directory);
  try {
    std::filesystem::create_directories(output_directory);
    if (persist_keyframe_pcd) {
      pgScansDirectory = (output_directory / "keyframes").string() + "/";
      std::filesystem::create_directories(pgScansDirectory);
      pgTimeSaveStream = std::fstream(
        (output_directory / "keyframe_times.txt").string(), std::fstream::out);
      pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    }
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(
      nh->get_logger(), "Failed to setup directory %s: %s",
      save_directory.c_str(), e.what());
    throw;
  }

  keyframeRadGap = deg2rad(keyframeDegGap);

  nh->declare_parameter<double>("sc_dist_thres", 0.2);
  scDistThres = nh->get_parameter("sc_dist_thres").as_double();

  nh->declare_parameter<double>(
    "sc_max_radius",
    20.0);    // 80 is recommended for outdoor, and lower (e.g., 20, 40) values
              // are recommended for indoor
  scMaximumRadius = nh->get_parameter("sc_max_radius").as_double();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = std::make_unique<ISAM2>(parameters);  // Fix: Use make_unique instead of raw new
  initNoises();

  scManager.setSCdistThres(scDistThres);
  scManager.setMaximumRadius(scMaximumRadius);

  float filter_size = 0.4;
  downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
  downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

  double mapVizFilterSize;
  nh->declare_parameter<double>("mapviz_filter_size", 0.4);
  mapVizFilterSize = nh->get_parameter("mapviz_filter_size")
    .as_double();                       // pose assignment every k frames
  downSizeFilterMapPGO.setLeafSize(
    mapVizFilterSize, mapVizFilterSize,
    mapVizFilterSize);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    subLaserCloudFullRes;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;

  subLaserCloudFullRes = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

  subLaserOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", 100, laserOdometryHandler);

  subGPS = nh->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 100,
    gpsHandler);

  pubOdomAftPGO = nh->create_publisher<nav_msgs::msg::Odometry>(
    "/aft_pgo_odom", rclcpp::QoS(100));
  pubOdomRepubVerifier = nh->create_publisher<nav_msgs::msg::Odometry>(
    "/repub_odom", rclcpp::QoS(100));
  pubPathAftPGO = nh->create_publisher<nav_msgs::msg::Path>(
    "/aft_pgo_path",
    rclcpp::QoS(100));
  pubMapAftPGO = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/aft_pgo_map", rclcpp::QoS(100));

  pubLoopScanLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/loop_scan_local", rclcpp::QoS(100));
  pubLoopSubmapLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/loop_submap_local", rclcpp::QoS(100));

  auto save_service = nh->create_service<std_srvs::srv::Trigger>(
    "/pgo/save_map", saveOptimizedBundleService);
  auto start_session_service = nh->create_service<std_srvs::srv::Trigger>(
    "/pgo/start_session", startMappingSessionService);

  std::thread posegraph_slam{
    []() {runWorkerSafely("posegraph", process_pg);}};
  std::thread lc_detection{
    []() {runWorkerSafely("scan-context", process_lcd);}};
  std::thread icp_calculation{
    []() {runWorkerSafely("loop-icp", process_icp);}};
  std::thread isam_update{
    []() {runWorkerSafely("isam", process_isam);}};    // if you want to call less isam2 run (for saving
  // redundant computations and no real-time visulization is
  // required), uncommment this and comment all the above
  // runisam2opt when node is added.

  std::thread viz_map{
    []() {runWorkerSafely("map-visualization", process_viz_map);}};  // visualization - map (low frequency
  // because it is heavy)
  std::thread viz_path{
    []() {runWorkerSafely("path-visualization", process_viz_path);}};    // visualization - path (high frequency)

  rclcpp::spin(nh);

  // Fix: Graceful shutdown - signal threads and wait for completion
  RCLCPP_INFO(nh->get_logger(), "Shutting down threads...");
  shutdown_requested = true;

  if (posegraph_slam.joinable()) {
    posegraph_slam.join();
  }
  if (lc_detection.joinable()) {
    lc_detection.join();
  }
  if (icp_calculation.joinable()) {
    icp_calculation.join();
  }
  if (isam_update.joinable()) {
    isam_update.join();
  }
  if (viz_map.joinable()) {
    viz_map.join();
  }
  if (viz_path.joinable()) {
    viz_path.join();
  }

  RCLCPP_INFO(nh->get_logger(), "All threads terminated. Exiting.");

  // Most ROS entities in the upstream node were global.  Destroy them in
  // dependency order here; otherwise static destruction can release the node
  // before its publishers and Fast-DDS may crash while deleting data readers.
  save_service.reset();
  subGPS.reset();
  subLaserOdometry.reset();
  subLaserCloudFullRes.reset();
  pubLoopSubmapLocal.reset();
  pubLoopScanLocal.reset();
  pubMapAftPGO.reset();
  pubPathAftPGO.reset();
  pubOdomRepubVerifier.reset();
  pubOdomAftPGO.reset();
  currGPS.reset();
  nh.reset();

  return 0;
}
