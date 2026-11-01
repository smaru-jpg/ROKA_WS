// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#ifndef LIDAR_UNDISTORTION_HPP_
#define LIDAR_UNDISTORTION_HPP_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

class LidarUndistortion
{
public:
  LidarUndistortion() {}

  // Ref:LeGO-LOAM(BSD-3 LICENSE)
  // https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L431-L459
  void getImu(
    Eigen::Vector3f angular_velo, Eigen::Vector3f acc, const Eigen::Quaternionf quat,
    const double imu_time /*[sec]*/)
  {
    if (!std::isfinite(imu_time) || !angular_velo.allFinite() ||
      !acc.allFinite() || !quat.coeffs().allFinite() || quat.norm() < 1.0e-6f)
    {
      return;
    }
    const Eigen::Quaternionf normalized_quat = quat.normalized();
    float roll, pitch, yaw;
    Eigen::Affine3f affine(normalized_quat);
    pcl::getEulerAngles(affine, roll, pitch, yaw);

    const int imu_ptr_back = imu_ptr_last_;
    imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_que_length_;

    if (imu_initialized_ && imu_ptr_last_ == imu_ptr_oldest_) {
      imu_ptr_oldest_ = (imu_ptr_oldest_ + 1) % imu_que_length_;
      if (imu_ptr_last_iter_ == imu_ptr_last_) {
        imu_ptr_last_iter_ = imu_ptr_oldest_;
      }
    }

    imu_time_[imu_ptr_last_] = imu_time;
    imu_roll_[imu_ptr_last_] = roll;
    imu_pitch_[imu_ptr_last_] = pitch;
    imu_yaw_[imu_ptr_last_] = yaw;
    imu_acc_x_[imu_ptr_last_] = acc.x();
    imu_acc_y_[imu_ptr_last_] = acc.y();
    imu_acc_z_[imu_ptr_last_] = acc.z();
    imu_angular_velo_x_[imu_ptr_last_] = angular_velo.x();
    imu_angular_velo_y_[imu_ptr_last_] = angular_velo.y();
    imu_angular_velo_z_[imu_ptr_last_] = angular_velo.z();

    if (!imu_initialized_) {
      imu_shift_x_[imu_ptr_last_] = 0.0f;
      imu_shift_y_[imu_ptr_last_] = 0.0f;
      imu_shift_z_[imu_ptr_last_] = 0.0f;
      imu_velo_x_[imu_ptr_last_] = 0.0f;
      imu_velo_y_[imu_ptr_last_] = 0.0f;
      imu_velo_z_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_x_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_y_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_z_[imu_ptr_last_] = 0.0f;
      imu_ptr_oldest_ = imu_ptr_last_;
      imu_ptr_last_iter_ = imu_ptr_last_;
      imu_initialized_ = true;
      return;
    }

    Eigen::Matrix3f rot = normalized_quat.toRotationMatrix();
    acc = rot * acc;
    // angular_velo = rot * angular_velo;

    double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
    if (time_diff > 0.0 && time_diff < 2.0 * scan_period_) {
      imu_shift_x_[imu_ptr_last_] =
        imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff *
        time_diff * 0.5;
      imu_shift_y_[imu_ptr_last_] =
        imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff *
        time_diff * 0.5;
      imu_shift_z_[imu_ptr_last_] =
        imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff *
        time_diff * 0.5;

      imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
      imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
      imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

      imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) *
        time_diff;
      imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) *
        time_diff;
      imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) *
        time_diff;
    } else {
      // A reconnect, clock jump, or delayed sample must not propagate stale
      // integrated motion into the next LiDAR revolution.
      imu_shift_x_[imu_ptr_last_] = 0.0f;
      imu_shift_y_[imu_ptr_last_] = 0.0f;
      imu_shift_z_[imu_ptr_last_] = 0.0f;
      imu_velo_x_[imu_ptr_last_] = 0.0f;
      imu_velo_y_[imu_ptr_last_] = 0.0f;
      imu_velo_z_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_x_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_y_[imu_ptr_last_] = 0.0f;
      imu_angular_rot_z_[imu_ptr_last_] = 0.0f;
    }
  }

  // Ref:LeGO-LOAM(BSD-3 LICENSE)
  // https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L491-L619
  void adjustDistortion(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const double scan_time /*[sec]*/,
    const std::vector<float> & point_relative_times = {})
  {
    int cloud_size = cloud->points.size();
    if (!imu_initialized_ || cloud_size < 2) {return;}
    const bool use_point_times = point_relative_times.size() == cloud->points.size() &&
      std::all_of(
      point_relative_times.begin(), point_relative_times.end(),
      [this](const float value) {
        return std::isfinite(value) && std::abs(value) <= 2.0 * scan_period_;
      });

    bool half_passed = false;
    float start_ori = 0.0f;
    float end_ori = 0.0f;
    float ori_diff = 0.0f;
    if (!use_point_times) {
      start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
      end_ori = -std::atan2(
        cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
      if (end_ori - start_ori > 3 * M_PI) {
        end_ori -= 2 * M_PI;
      } else if (end_ori - start_ori < M_PI) {
        end_ori += 2 * M_PI;
      }
      ori_diff = end_ori - start_ori;
      if (!std::isfinite(ori_diff) || std::abs(ori_diff) < 1.0e-6f) {return;}
    }

    Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
    Eigen::Vector3f shift_from_start;
    Eigen::Matrix3f r_s_i, r_c;
    Eigen::Vector3f adjusted_p;
    bool start_initialized = false;
    for (int i = 0; i < cloud_size; ++i) {
      pcl::PointXYZI & p = cloud->points[i];
      float rel_time = point_relative_times.empty() ? 0.0f : point_relative_times[i];
      if (!use_point_times) {
        float ori_h = -std::atan2(p.y, p.x);
        if (!half_passed) {
          if (ori_h < start_ori - M_PI * 0.5) {
            ori_h += 2 * M_PI;
          } else if (ori_h > start_ori + M_PI * 1.5) {
            ori_h -= 2 * M_PI;
          }

          if (ori_h - start_ori > M_PI) {
            half_passed = true;
          }
        } else {
          ori_h += 2 * M_PI;
          if (ori_h < end_ori - 1.5 * M_PI) {
            ori_h += 2 * M_PI;
          } else if (ori_h > end_ori + 0.5 * M_PI) {
            ori_h -= 2 * M_PI;
          }
        }
        rel_time = (ori_h - start_ori) / ori_diff * scan_period_;
      }

      int imu_ptr_front = imu_ptr_last_iter_;
      if (imu_initialized_) {
        while (imu_ptr_front != imu_ptr_last_) {
          if (scan_time + rel_time < imu_time_[imu_ptr_front]) {
            break;
          }
          imu_ptr_front = (imu_ptr_front + 1) % imu_que_length_;
        }

        if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front]) > scan_period_) {
          continue;
        }

        if (scan_time + rel_time > imu_time_[imu_ptr_front]) {
          rpy_cur(0) = imu_roll_[imu_ptr_front];
          rpy_cur(1) = imu_pitch_[imu_ptr_front];
          rpy_cur(2) = imu_yaw_[imu_ptr_front];
          shift_cur(0) = imu_shift_x_[imu_ptr_front];
          shift_cur(1) = imu_shift_y_[imu_ptr_front];
          shift_cur(2) = imu_shift_z_[imu_ptr_front];
          velo_cur(0) = imu_velo_x_[imu_ptr_front];
          velo_cur(1) = imu_velo_y_[imu_ptr_front];
          velo_cur(2) = imu_velo_z_[imu_ptr_front];
        } else {
          int imu_ptr_back = (imu_ptr_front - 1 + imu_que_length_) % imu_que_length_;
          const double interpolation_dt = imu_time_[imu_ptr_front] - imu_time_[imu_ptr_back];
          if (interpolation_dt <= 0.0) {continue;}
          float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) /
            interpolation_dt;
          float ratio_back = 1.0 - ratio_front;
          auto interpolate_angle = [ratio_front](float back, float front) {
              const float delta = std::atan2(std::sin(front - back), std::cos(front - back));
              return back + ratio_front * delta;
            };
          rpy_cur(0) = interpolate_angle(imu_roll_[imu_ptr_back], imu_roll_[imu_ptr_front]);
          rpy_cur(1) = interpolate_angle(imu_pitch_[imu_ptr_back], imu_pitch_[imu_ptr_front]);
          rpy_cur(2) = interpolate_angle(imu_yaw_[imu_ptr_back], imu_yaw_[imu_ptr_front]);
          shift_cur(0) = imu_shift_x_[imu_ptr_front] * ratio_front + imu_shift_x_[imu_ptr_back] *
            ratio_back;
          shift_cur(1) = imu_shift_y_[imu_ptr_front] * ratio_front + imu_shift_y_[imu_ptr_back] *
            ratio_back;
          shift_cur(2) = imu_shift_z_[imu_ptr_front] * ratio_front + imu_shift_z_[imu_ptr_back] *
            ratio_back;
          velo_cur(0) = imu_velo_x_[imu_ptr_front] * ratio_front + imu_velo_x_[imu_ptr_back] *
            ratio_back;
          velo_cur(1) = imu_velo_y_[imu_ptr_front] * ratio_front + imu_velo_y_[imu_ptr_back] *
            ratio_back;
          velo_cur(2) = imu_velo_z_[imu_ptr_front] * ratio_front + imu_velo_z_[imu_ptr_back] *
            ratio_back;
        }

        r_c =
          (Eigen::AngleAxisf(
            rpy_cur(2),
            Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(
            rpy_cur(1),
            Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

        if (!start_initialized) {
          rpy_start = rpy_cur;
          shift_start = shift_cur;
          velo_start = velo_cur;
          r_s_i = r_c.inverse();
          start_initialized = true;
        } else {
          shift_from_start = shift_cur - shift_start - velo_start * rel_time;
          adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
          p.x = adjusted_p.x();
          p.y = adjusted_p.y();
          p.z = adjusted_p.z();
        }
      }
      imu_ptr_last_iter_ = imu_ptr_front;
    }
  }


  void setScanPeriod(const double scan_period /*[sec]*/)
  {
    scan_period_ = scan_period;
  }

private:
  double scan_period_{0.1};
  static const int imu_que_length_{200};
  int imu_ptr_oldest_{0}, imu_ptr_last_{-1}, imu_ptr_last_iter_{0};
  bool imu_initialized_{false};

  std::array<double, imu_que_length_> imu_time_ {};
  std::array<float, imu_que_length_> imu_roll_ {};
  std::array<float, imu_que_length_> imu_pitch_ {};
  std::array<float, imu_que_length_> imu_yaw_ {};

  std::array<float, imu_que_length_> imu_acc_x_ {};
  std::array<float, imu_que_length_> imu_acc_y_ {};
  std::array<float, imu_que_length_> imu_acc_z_ {};
  std::array<float, imu_que_length_> imu_velo_x_ {};
  std::array<float, imu_que_length_> imu_velo_y_ {};
  std::array<float, imu_que_length_> imu_velo_z_ {};
  std::array<float, imu_que_length_> imu_shift_x_ {};
  std::array<float, imu_que_length_> imu_shift_y_ {};
  std::array<float, imu_que_length_> imu_shift_z_ {};

  std::array<float, imu_que_length_> imu_angular_velo_x_ {};
  std::array<float, imu_que_length_> imu_angular_velo_y_ {};
  std::array<float, imu_que_length_> imu_angular_velo_z_ {};
  std::array<float, imu_que_length_> imu_angular_rot_x_ {};
  std::array<float, imu_que_length_> imu_angular_rot_y_ {};
  std::array<float, imu_que_length_> imu_angular_rot_z_ {};
};

#endif  // LIDAR_UNDISTORTION_HPP_
