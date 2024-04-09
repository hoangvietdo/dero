// This file is part of DeRO: Dead Reckoning Based on Radar Odometry
// With Accelerometers Aided for Robot Localization.
// Copyright (C) 2024 Hoang Viet Do  <hoangvietdo@sju.ac.kr>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// Reference:
// https://github.com/christopherdoer/reve/blob/master/radar_ego_velocity_estimator/src/radar_point_cloud.cpp

#ifndef RADAR_POINT_CLOUD_HPP
#define RADAR_POINT_CLOUD_HPP

#include <pcl/point_types.h>

namespace incsl {

struct RadarPointCloudType {
    PCL_ADD_POINT4D;     // position in [m]
    float snr_db;        // CFAR cell to side noise ratio in [dB]
    float v_doppler_mps; // Doppler velocity in [m/s]
    float noise_db;      // CFAR noise level of the side of the detected cell in [dB]
    float range;         // range in [m]
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Reference:
// https://github.com/christopherdoer/reve/blob/bcc5f6cae00ab6529774c2c2df10a170a9eb01b7/radar_ego_velocity_estimator/include/radar_ego_velocity_estimator/data_types.h
struct SmartMicroRadarPointCloudType {
    float number_of_objects;
    float cycle_duration;
    float range;
    float azimuth;
    float speed_radial;
    float rcs;
    float power;
    float noise;
    float elevation;
    PCL_ADD_POINT4D; // position in [m]
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct RadarIndex {
    uint azimuth   = 0;
    uint elevation = 1;
    uint x_r       = 2;
    uint y_r       = 3;
    uint z_r       = 4;
    uint peak_db   = 5;
    uint r_x       = 6;
    uint r_y       = 7;
    uint r_z       = 8;
    uint v_d       = 9;
    uint noise_db  = 10;
};

} // namespace incsl

#endif // !RADAR_POINT_CLOUD_HPP
