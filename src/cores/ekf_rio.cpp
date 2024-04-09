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

#include "dero/ekf_rio.hpp"

namespace incsl {

EkfRio::EkfRio() {} // EkfRio

void EkfRio::InitializeState(const std::vector<sensor_msgs::msg::Imu> &imu_buff) {
  // Refererence: https://github.com/lastflowers/envio/blob/master/src/core/sl_iekf.cpp#L818
  // Coarse alignment

  Vec3d f_b_mean = Vec3d::Zero();
  Vec3d w_b_mean = Vec3d::Zero();

  for (long unsigned int i = 0; i < imu_buff.size(); ++i) {
    Vec3d f_b_foo, w_b_foo;

    // clang-format off
    f_b_foo << imu_buff[i].linear_acceleration.x,
               imu_buff[i].linear_acceleration.y,
               imu_buff[i].linear_acceleration.z;

    w_b_foo << imu_buff[i].angular_velocity.x,
               imu_buff[i].angular_velocity.y,
               imu_buff[i].angular_velocity.z;
    // clang-format on

    f_b_mean += f_b_foo;
    w_b_mean += w_b_foo;
  }

  f_b_mean /= imu_buff.size();
  w_b_mean /= imu_buff.size();

  double phi_0 = std::atan2(-f_b_mean(1, 0), -f_b_mean(2, 0));
  double theta_0 =
      std::atan2(f_b_mean(0, 0), std::sqrt(f_b_mean(1, 0) * f_b_mean(1, 0) + f_b_mean(2, 0) * f_b_mean(2, 0)));
  double psi_0 = 0.0;

  euler_0 << phi_0, theta_0, psi_0;
  const Vec3d foo_grav = euler2dcm(euler_0).transpose() * gravity;

  ca_state.gyro_bias   = w_b_mean;
  ca_state.position    = state.position;
  ca_state.velocity    = state.velocity;
  ca_state.euler       = euler_0;
  ca_state.accel_bias  = f_b_mean + foo_grav;
  ca_state.radar_scale = state.radar_scale;

  state.quaternion = euler2quat(ca_state.euler);
  state.accel_bias = ca_state.accel_bias;
  state.gyro_bias  = ca_state.gyro_bias;
} // void InitializeState

void EkfRio::ImuMechanization(const sensor_msgs::msg::Imu &imu_msg) {
  Vec3d f_b, w_b;

  // clang-format off
  f_b << imu_msg.linear_acceleration.x,
         imu_msg.linear_acceleration.y,
         imu_msg.linear_acceleration.z;

  w_b << imu_msg.angular_velocity.x,
         imu_msg.angular_velocity.y,
         imu_msg.angular_velocity.z;
  // clang-format on

  f_b -= state.accel_bias;
  w_b -= state.gyro_bias;

  // Simpson Integration - fourth order runge kutta
  const Vec4d      zero_omega(0.0, w_b(0, 0), w_b(1, 0), w_b(2, 0));
  Vec4d            prevQuaternion = state.quaternion;
  const Quaternion prevQuaternion_(state.quaternion(0, 0), state.quaternion(1, 0), state.quaternion(2, 0),
                                   state.quaternion(3, 0));

  const Vec4d k_1(0.5 * calculateLeftOmega(prevQuaternion) * zero_omega);
  const Vec4d k_2(0.5 * calculateLeftOmega(prevQuaternion + k_1 * dt_imu / 2) * zero_omega);
  const Vec4d k_3(0.5 * calculateLeftOmega(prevQuaternion + k_2 * dt_imu / 2) * zero_omega);
  const Vec4d k_4(0.5 * calculateLeftOmega(prevQuaternion + k_3 * dt_imu) * zero_omega);

  state.quaternion = prevQuaternion + (k_1 + 2 * k_2 + 2 * k_3 + k_4) * dt_imu / 6;
  state.quaternion = quatNormalize(state.quaternion);
  const Quaternion newQuaternion(state.quaternion(0, 0), state.quaternion(1, 0), state.quaternion(2, 0),
                                 state.quaternion(3, 0));

  const Quaternion simpson_quaternion = prevQuaternion_ * newQuaternion.inverse();
  const Mat3d      simpson_Rbn        = simpson_quaternion.toRotationMatrix();
  Mat3d            prev_Rbn           = quat2dcm(prevQuaternion);

  const Vec3d a_ = (f_b + 4.0 * (f_b + 0.5 * (simpson_Rbn - Mat3d::Identity()) * f_b) +
                    (f_b + (simpson_Rbn - Mat3d::Identity()) * f_b)) *
                   dt_imu / 6.0;

  const Vec3d prevVelocity = state.velocity;
  state.velocity           = state.velocity + prev_Rbn * a_ + gravity * dt_imu;

  const Vec3d v_ = (f_b + 4.0 * (f_b + 0.25 * (simpson_Rbn - Mat3d::Identity()) * f_b) +
                    (f_b + 0.5 * (simpson_Rbn - Mat3d::Identity()) * f_b)) *
                   dt_imu / 12.0;

  const Vec3d foo = (4.0 * v_ + a_) * dt_imu / 6.0;
  const Vec3d p_  = prevVelocity * dt_imu + prev_Rbn * foo + 0.5 * gravity * dt_imu * dt_imu;
  state.position  = state.position + p_;

  EkfRio::TimeUpdate(f_b);
} // void ImuMechanization

void EkfRio::TimeUpdate(const Vec3d &f_b) {
  Mat15d F = Mat15d::Zero();
  MatXd  G = MatXd::Zero(15, 12);
  Vec3d  foo;
  Mat3d  Rbn;

  Rbn = quat2dcm(state.quaternion);
  foo = Rbn * f_b;

  F.block<3, 3>(0, 3)  = Mat3d::Identity();
  F.block<3, 3>(3, 6)  = skewMatrix(foo);
  F.block<3, 3>(3, 9)  = -Rbn;
  F.block<3, 3>(6, 12) = Rbn;

  G.block<3, 3>(3, 0)  = Rbn;
  G.block<3, 3>(6, 3)  = -Rbn;
  G.block<3, 3>(9, 6)  = Mat3d::Identity();
  G.block<3, 3>(12, 9) = Mat3d::Identity();

  Mat12d Q_            = noise_covariance_matrix.Q;
  Q_.block<6, 6>(0, 0) = noise_covariance_matrix.Q.block<6, 6>(0, 0) * dt_imu;
  Q_.block<6, 6>(6, 6) = noise_covariance_matrix.Q.block<6, 6>(6, 6) / dt_imu;

  Mat15d Phi                   = Mat15d::Identity() + F * dt_imu + 0.5 * dt_imu * dt_imu * F * F;
  covariance_matrix.priori     = Phi * covariance_matrix.posteriori * Phi.transpose() + G * Q_ * G.transpose();
  covariance_matrix.priori     = EnsurePSD(covariance_matrix.priori);
  covariance_matrix.posteriori = covariance_matrix.priori;
} // void TimeUpdate

bool EkfRio::MeasurementUpdateRadar(const Vec3d &r_radar, const MatXd &H_radar, const Vec3d &v_r_r,
                                    const RadarVelocityEstimatorParam radar_velocity_estimator_param_,
                                    const bool                        outlier_reject) {
  const Mat3d R_discrete_radar = noise_covariance_matrix.R_radar;
  const Mat3d S_radar          = H_radar * covariance_matrix.priori * H_radar.transpose() + R_discrete_radar;

  const MatXd K_radar            = covariance_matrix.priori * H_radar.transpose() * S_radar.inverse();
  VecXd       error_state_radar_ = VecXd::Zero(15, 1);
  error_state_radar_             = K_radar * r_radar;

  if (outlier_reject) {
    const double gamma = r_radar.transpose() *
                         (H_radar * covariance_matrix.posteriori * H_radar.transpose() + R_discrete_radar).inverse() *
                         r_radar;

    boost::math::chi_squared chiSquaredDist(3.0);

    //  NOTE: Adaptive threshold for very low ZUPTs
    const double outlier_percentil_radar_ = (v_r_r.norm() > 0.05)
                                                ? radar_velocity_estimator_param_.outlier_percentil_radar
                                                : radar_velocity_estimator_param_.outlier_percentil_radar / 100.0;

    const double gamma_thresh = boost::math::quantile(chiSquaredDist, 1 - outlier_percentil_radar_);

    if (gamma < gamma_thresh) {
      covariance_matrix.posteriori = (Mat15d::Identity() - K_radar * H_radar) * covariance_matrix.priori *
                                         (Mat15d::Identity() - K_radar * H_radar).transpose() +
                                     K_radar * R_discrete_radar * K_radar.transpose();
      covariance_matrix.posteriori = EnsurePSD(covariance_matrix.posteriori);

      ErrorCorrection(error_state_radar_);
      return true;
    } else
      return false;
  } else {
    covariance_matrix.posteriori = (Mat15d::Identity() - K_radar * H_radar) * covariance_matrix.priori *
                                       (Mat15d::Identity() - K_radar * H_radar).transpose() +
                                   K_radar * R_discrete_radar * K_radar.transpose();
    covariance_matrix.posteriori = EnsurePSD(covariance_matrix.posteriori);

    ErrorCorrection(error_state_radar_);
    return true;
  }
} // void MeasurementUpdateRadar

void EkfRio::ErrorCorrection(const Vec15d &error_state_) {
  error_state.position     = error_state_.segment(0, 3);
  error_state.velocity     = error_state_.segment(3, 3);
  error_state.misalignment = error_state_.segment(6, 3);
  error_state.accel_bias   = error_state_.segment(9, 3);
  error_state.gyro_bias    = error_state_.segment(12, 3);

  state.position   -= error_state.position;
  state.velocity   -= error_state.velocity;
  state.accel_bias -= error_state.accel_bias;
  state.gyro_bias  -= error_state.gyro_bias;

  Vec4d error_quaternion;
  error_quaternion << 1.0, 0.5 * error_state.misalignment(0, 0), 0.5 * error_state.misalignment(1, 0),
      0.5 * error_state.misalignment(2, 0);

  state.quaternion = quatMultiplication(error_quaternion, state.quaternion);
} // void ErrorCorrection

double EkfRio::getGravityValue() { return gravity(2, 0); } // getGravityValue

void EkfRio::setGravityValue(double &gravity_constant) { gravity << 0.0, 0.0, gravity_constant; } // setGravityValue

CoarseAlignmentState EkfRio::getCoarseAlignmentState() { return ca_state; } // getCoarseAlignmentState

ErrorState EkfRio::getErrorState() { return error_state; } // getErrorState

State EkfRio::getState() { return state; } // getState

void EkfRio::setState(const State &state_) { state = state_; } // setState

CovarianceMatrix EkfRio::getCovarianceMatrix() { return covariance_matrix; } // getCovarianceMatrix

///// IMU /////
double EkfRio::getImuDt() { return dt_imu; } // getImuDt

void EkfRio::setImuDt() { dt_imu = current_time_imu - previous_time_imu; } // setImuDt

double EkfRio::getImuPreviousTime() { return previous_time_imu; } // getImuPreviousTime

void EkfRio::setImuPreviousTime(double t) { previous_time_imu = t; } // setImuPreviousTime

double EkfRio::getImuCurrentTime() { return current_time_imu; } // getImuCurrentTime

void EkfRio::setImuCurrentTime(double t) { current_time_imu = t; } // setImuCurrentTime

///// Radar /////
double EkfRio::getRadarDt() { return dt_radar; } // getRadarDt

void EkfRio::setRadarDt() { dt_radar = current_time_radar - previous_time_radar; } // setRadarDt

double EkfRio::getRadarPreviousTime() { return previous_time_radar; } // getRadarPreviousTime

void EkfRio::setRadarPreviousTime(double t) { previous_time_radar = t; } // setRadarPreviousTime

double EkfRio::getRadarCurrentTime() { return current_time_radar; } // getRadarCurrentTime

void EkfRio::setRadarCurrentTime(double t) { current_time_radar = t; } // setRadarCurrentTime

void EkfRio::setQ(const Noise &noise) {
  noise_covariance_matrix.Q = Mat12d::Zero();
  noise_covariance_matrix.Q.block<3, 3>(0, 0) =
      noise.velocity_random_walk * noise.velocity_random_walk * Mat3d::Identity();
  noise_covariance_matrix.Q.block<3, 3>(3, 3) =
      noise.angular_random_walk * noise.angular_random_walk * Mat3d::Identity();
  noise_covariance_matrix.Q.block<3, 3>(6, 6) =
      noise.accel_bias_random_walk * noise.accel_bias_random_walk * Mat3d::Identity();
  noise_covariance_matrix.Q.block<3, 3>(9, 9) =
      noise.gyro_bias_random_walk * noise.gyro_bias_random_walk * Mat3d::Identity();
} // setQ

void EkfRio::setR_Radar(const Mat3d &R_radar) {
  noise_covariance_matrix.R_radar = Mat3d::Zero();
  noise_covariance_matrix.R_radar = R_radar;
} // setR_Radar

void EkfRio::setCovarianceMatrix(const Init &init) {
  covariance_matrix.posteriori                     = Mat15d::Zero();
  covariance_matrix.posteriori.block<3, 3>(0, 0)   = init.position * init.position * Mat3d::Identity();
  covariance_matrix.posteriori.block<3, 3>(3, 3)   = init.velocity * init.velocity * Mat3d::Identity();
  covariance_matrix.posteriori.block<2, 2>(6, 6)   = init.euler_XY * init.euler_XY * Mat2d::Identity();
  covariance_matrix.posteriori(8, 8)               = init.euler_Z * init.euler_Z;
  covariance_matrix.posteriori.block<3, 3>(9, 9)   = init.accel_bias * init.accel_bias * Mat3d::Identity();
  covariance_matrix.posteriori.block<3, 3>(12, 12) = init.gyro_bias * init.gyro_bias * Mat3d::Identity();
} // setCovarianceMatrix

} // namespace incsl
