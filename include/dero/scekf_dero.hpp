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

#ifndef SC_EKF_DERO_HPP
#define SC_EKF_DERO_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <dero/math_tools.hpp>
#include <dero/nav_convert.hpp>
#include <dero/radar_estimator.hpp>

#include <boost/math/distributions/chi_squared.hpp>

namespace incsl {

class ScEkfDero {
  public:
    ScEkfDero();

    void InitializeState(const std::vector<sensor_msgs::msg::Imu> &imu_buff);

    void GyroscopeMechanization(const sensor_msgs::msg::Imu &imu_msg);
    void RadarMechanization(const Vec3d &v_r, IMURadarCalibrationParam &imu_radar_calibration_);

    void RadarTimeUpdate(const Vec3d &v_r, const Mat3d &P_v_r, IMURadarCalibrationParam &imu_radar_calibration_,
                         const bool &use_cloning, const int &window_size);

    bool RadarMeasurementUpdate(IMURadarCalibrationParam &imu_radar_calibration_, const ICPTransform &icp_meas,
                                const bool &outlier_reject, const bool &zupt_trigger, State &first_state);

    bool MeasurementUpdateAccel(const Vec2d &r_accel, const MatXd &H_accel, const bool &outlier_reject);

    void   ErrorCorrection(const VecXd &error_state_);
    double getGravityValue();
    double getImuDt();
    double getImuPreviousTime();
    double getImuCurrentTime();
    double getRadarDt();
    double getRadarPreviousTime();
    double getRadarCurrentTime();

    Mat2d getR_Accel();

    Vec3d getVbn();

    ErrorState              getErrorState();
    State                   getState();
    CoarseAlignmentState    getCoarseAlignmentState();
    CovarianceMatrixCloning getCovarianceMatrix();

    void setImuPreviousTime(double t);
    void setImuCurrentTime(double t);
    void setImuDt();
    void setRadarPreviousTime(double t);
    void setRadarCurrentTime(double t);
    void setRadarDt();

    void setQ(const Noise &noise);
    void setCovarianceMatrix(const Init &init);
    void setState(const State &state_);
    void setGravityValue(double &gravity_constant);
    void setR_Accel(const Vec2d &sigma_accel);

  private:
    int window_count = 0;

    Vec3d gravity;
    Vec3d euler_0;
    Vec3d w_b_hat;
    Vec3d f_b_hat;
    Vec3d v_n_;

    Mat4d  prevOmega;
    Mat12d Phi_accumulation;

    double previous_time_imu;
    double dt_imu;
    double current_time_imu;
    double previous_time_radar;
    double dt_radar;
    double current_time_radar;
    double zero_factor;

    CoarseAlignmentState    ca_state;
    ErrorState              error_state;
    State                   state;
    ErrorStateCloning       error_state_cloning;
    StateCloning            state_cloning;
    CovarianceMatrixDr      covariance_matrix;
    CovarianceMatrixCloning covariance_matrix_cloning;
    NoiseCovarianceMatrixDr noise_covariance_matrix;
    RadarEstimator          radar_estimator_;

}; // class
} // namespace incsl

#endif // ifndef
