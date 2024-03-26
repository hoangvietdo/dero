// This file is part of DeRO: Dead Reckoning Based on Radar Odometry
// With Accelerometers Aided for Robot Localization.
// Copyright (C) 2024 Hoang Viet Do  <hoangvietdo@sju.ac.kr>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef RADAR_ESTIMATOR_HPP
#define RADAR_ESTIMATOR_HPP

#include <random>

#include <dero/nav_convert.hpp>
#include <dero/odr.h>
#include <dero/radar_point_cloud.hpp>
#include <dero/variable_define.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <pcl_conversions/pcl_conversions.h>

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(incsl::RadarPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, snr_db, snr_db)
                                  (float, noise_db, noise_db)
                                  (float, v_doppler_mps, v_doppler_mps))

POINT_CLOUD_REGISTER_POINT_STRUCT(incsl::SmartMicroRadarPointCloudType,
                                  (float, number_of_objects, Number_Of_Objects)
                                  (float, cycle_duration, Cycle_Duration)
                                  (float, range, Range)
                                  (float, azimuth, Azimuth)
                                  (float, speed_radial, Speed_Radial)
                                  (float, rcs, RCS)
                                  (float, power, Power)
                                  (float, noise, Noise)
                                  (float, elevation, Elevation))
// clang-format on

namespace incsl {

class RadarEstimator {
  public:
    RadarEstimator();

    bool Process(const sensor_msgs::msg::PointCloud2 &radar_msg, const RadarVelocityEstimatorParam param,
                 const RadarPositionEstimatorParam radar_position_estimator_param, const Mat4d &init_guess_pose,
                 const bool &use_dr_structure);

    ICPTransform solveICP(const pcl::PointCloud<incsl::RadarPointCloudType> &prev_pcl_msg,
                          const pcl::PointCloud<incsl::RadarPointCloudType> &curr_pcl_msg,
                          const RadarPositionEstimatorParam                 &radar_position_estimator_param,
                          const Mat4d                                       &init_guess_pose);

    Vec3d getEgoVelocity();
    Mat3d getEgoVelocityCovariance();

    ICPTransform                         getIcpTransform();
    pcl::PointCloud<RadarPointCloudType> getRadarScanInlier();

    std::string getRadarInfo();

    sensor_msgs::msg::PointCloud2 getInlierRadarRos2PCL2();
    std::vector<Vec3d>            getInlierRadarPcl();
    std::vector<Vec3d>            pcl_vec_;

  private:
    bool first_scan   = true;
    bool zupt_trigger = true;

    Mat4d prev_trans = Mat4d::Identity();
    Mat4d trans_;

    ICPTransform trans;

    Vec3d ego_velocity_;
    Vec3d v_r;

    Mat3d ego_velocity_covariance_;
    Mat3d P_v_r;

    uint ransac_iter_;

    std::vector<Vec11d> valid_targets;
    std::vector<Vec3d>  inlier_pcl_vec;

    pcl::PointCloud<RadarPointCloudType> prev_radar_scan_raw;
    pcl::PointCloud<RadarPointCloudType> radar_scan_inlier;
    pcl::PointCloud<RadarPointCloudType> foo_radar_scan_inlier;
    pcl::PointCloud<RadarPointCloudType> prev_radar_scan_inlier;

    sensor_msgs::msg::PointCloud2 inlier_radar_msg;
    sensor_msgs::msg::PointCloud2 inlier_radar_msg_;

    std::string radar_info_;

    RadarIndex idx_;

    RadarPointCloudType toRadarPointCloudType(const Vec11d &item, const RadarIndex &idx);
    bool                solve3DLsq(const MatXd &radar_data, Vec3d &v_r, Mat3d &P_v_r, bool estimate_sigma,
                                   const RadarVelocityEstimatorParam param);
    void solve3DODR(const MatXd &radar_data, Vec3d &v_r, Mat3d &P_v_r, const RadarVelocityEstimatorParam param_);
    void solve3DLsqRansac(const MatXd &radar_data, Vec3d &v_r, Mat3d &P_v_r, std::vector<uint> &inlier_idx_best,
                          const RadarVelocityEstimatorParam param);
    void pclToPcl2Msg(pcl::PointCloud<RadarPointCloudType>     radar_scan_inlier,
                      sensor_msgs::msg::PointCloud2::SharedPtr inlier_radar_msg);
    void setInlierRadarRos2PCL2(sensor_msgs::msg::PointCloud2 radar_msg);
    void setInlierRadarPcl(const std::vector<Vec3d> &pcl_vec);
    void setEgoVelocity(Vec3d v_r);
    void setEgoVelocityCovariance(Mat3d P_v_r);
    void Colorize(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
                  const std::vector<int> &color);
    pcl::PointCloud<incsl::RadarPointCloudType>
    normalizedPointCloud(const pcl::PointCloud<incsl::RadarPointCloudType> &raw_pcl_msg);
};
} // namespace incsl

#endif // RADAR_VELOCITY_ESTIMATOR_HPP
