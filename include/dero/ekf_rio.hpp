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

#ifndef EKF_RIO_HPP
#define EKF_RIO_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <dero/math_tools.hpp>
#include <dero/nav_convert.hpp>

namespace incsl {

class EkfRio {
  public:
    EkfRio();

    void InitializeState(const std::vector<sensor_msgs::msg::Imu> &imu_buff);
    void ImuMechanization(const sensor_msgs::msg::Imu &imu_msg);
    void TimeUpdate(const Vec3d &f_b);
    void MeasurementUpdateRadar(const Vec3d &r, const MatXd &H);
    void ErrorCorrection(const Vec15d &error_state_);

    double getGravityValue();
    double getImuDt();
    double getImuPreviousTime();
    double getImuCurrentTime();
    double getRadarDt();
    double getRadarPreviousTime();
    double getRadarCurrentTime();

    ErrorState           getErrorState();
    State                getState();
    CoarseAlignmentState getCoarseAlignmentState();
    CovarianceMatrix     getCovarianceMatrix();

    void setImuPreviousTime(double t);
    void setImuCurrentTime(double t);
    void setImuDt();
    void setRadarPreviousTime(double t);
    void setRadarCurrentTime(double t);
    void setRadarDt();
    void setGravityValue(double &gravity_constant);

    void setQ(const Noise &noise);
    void setR_Radar(const Mat3d &R_radar);
    void setCovarianceMatrix(const Init &init);
    void setState(const State &state_);

  private:
    Vec3d gravity;
    Vec3d euler_0;

    Mat4d prevOmega = Mat4d::Identity();

    double previous_time_imu;
    double dt_imu;
    double current_time_imu;
    double previous_time_radar;
    double dt_radar;
    double current_time_radar;

    CoarseAlignmentState  ca_state;
    ErrorState            error_state;
    State                 state;
    CovarianceMatrix      covariance_matrix;
    NoiseCovarianceMatrix noise_covariance_matrix;

}; // class
} // namespace incsl

#endif // ifndef
