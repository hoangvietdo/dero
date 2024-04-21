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

#include "dero/scekf_dero.hpp"

namespace incsl {

ScEkfDero::ScEkfDero() {} // ScEkfDero

void ScEkfDero::InitializeState(const std::vector<sensor_msgs::msg::Imu> &imu_buff) {
  // Refererence:
  // https://github.com/lastflowers/envio/blob/master/src/core/sl_iekf.cpp#L818
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

  // clang-format off
  double phi_0 = std::atan2(-f_b_mean(1, 0),
                            -f_b_mean(2, 0));
  double theta_0 = std::atan2(f_b_mean(0, 0),
                              std::sqrt(f_b_mean(1, 0) * f_b_mean(1, 0) + f_b_mean(2, 0) * f_b_mean(2, 0)));
  // clang-format on

  ca_state.gyro_bias   = w_b_mean;
  ca_state.euler       = quat2euler(state.quaternion);
  ca_state.euler(0, 0) = phi_0;
  ca_state.euler(1, 0) = theta_0;
  ca_state.position    = state.position;
  ca_state.velocity    = state.velocity;
  ca_state.radar_scale = state.radar_scale;
  Vec3d foo_grav       = euler2dcm(ca_state.euler).transpose() * gravity;
  ca_state.accel_bias  = f_b_mean + foo_grav;

  state.quaternion = euler2quat(ca_state.euler);
  state.accel_bias = ca_state.accel_bias;
  state.gyro_bias  = ca_state.gyro_bias;
} // void InitializeState

void ScEkfDero::GyroscopeMechanization(const sensor_msgs::msg::Imu &imu_msg, const bool &accel_trigger) {
  // clang-format off
  w_b_hat << imu_msg.angular_velocity.x,
             imu_msg.angular_velocity.y,
             imu_msg.angular_velocity.z;
  // clang-format on

  w_b_hat -= state.gyro_bias;

  // clang-format off
  // 4th runge-kutta quaternion integration
  const Vec4d zero_omega(0.0,
                         w_b_hat(0, 0),
                         w_b_hat(1, 0),
                         w_b_hat(2, 0));

  Vec4d            prevQuaternion = state.quaternion;
  const Quaternion prevQuaternion_(state.quaternion(0, 0),
                                   state.quaternion(1, 0),
                                   state.quaternion(2, 0),
                                   state.quaternion(3, 0));
  // clang-format on
  const Vec4d k_1(0.5 * calculateLeftOmega(prevQuaternion) * zero_omega);
  const Vec4d k_2(0.5 * calculateLeftOmega(prevQuaternion + k_1 * dt_imu / 2) * zero_omega);
  const Vec4d k_3(0.5 * calculateLeftOmega(prevQuaternion + k_2 * dt_imu / 2) * zero_omega);
  const Vec4d k_4(0.5 * calculateLeftOmega(prevQuaternion + k_3 * dt_imu) * zero_omega);
  state.quaternion = prevQuaternion + (k_1 + 2 * k_2 + 2 * k_3 + k_4) * dt_imu / 6;
  state.quaternion = quatNormalize(state.quaternion);

  // if radar msg is messed up then use accel to do odometry
  if (accel_trigger) {
    Vec3d f_b;
    // clang-format off
    f_b << imu_msg.linear_acceleration.x,
           imu_msg.linear_acceleration.y,
           imu_msg.linear_acceleration.z;

    f_b -= state.accel_bias;
    const Quaternion newQuaternion(state.quaternion(0, 0),
                                   state.quaternion(1, 0),
                                   state.quaternion(2, 0),
                                   state.quaternion(3, 0));
    // clang-format on

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
  }
} // void ImuMechanization

void ScEkfDero::RadarMechanization(const Vec3d &v_r, IMURadarCalibrationParam &imu_radar_calibration_) {
  // clang-format off
  const Vec3d v_r_hat(v_r(0, 0) / state.radar_scale(0, 0),
                      v_r(1, 0) / state.radar_scale(1, 0),
                      v_r(2, 0) / state.radar_scale(2, 0));
  // clang-format on

  const Mat3d Rbn = quat2dcm(state.quaternion);

  Vec3d v_n =
      Rbn * (imu_radar_calibration_.rotation_matrix * v_r_hat - skewMatrix(w_b_hat) * imu_radar_calibration_.position);

  state.position += v_n * dt_radar;
  v_n_            = v_n;
  state.velocity  = v_n;
} // void RadarMechanization

void ScEkfDero::RadarTimeUpdate(const Vec3d &v_r, const Mat3d &P_v_r, IMURadarCalibrationParam &imu_radar_calibration_,
                                const bool &use_cloning, const int &window_size) {

  // Scale vector correction
  // clang-format off
  const Vec3d v_r_hat(v_r(0, 0) / state.radar_scale(0, 0),
                      v_r(1, 0) / state.radar_scale(1, 0),
                      v_r(2, 0) / state.radar_scale(2, 0));
  // clang-format on

  //  NOTE: Time Update
  Mat12d      F   = Mat12d::Zero();
  Mat12d      G   = Mat12d::Identity();
  const Mat3d Rbn = quat2dcm(state.quaternion);

  Vec3d foo_1         = Rbn * imu_radar_calibration_.rotation_matrix * v_r_hat;
  Vec3d foo_2         = Rbn * skewMatrix(w_b_hat) * imu_radar_calibration_.position;
  Vec3d foo           = foo_2 - foo_1;
  F.block<3, 3>(0, 3) = -skewMatrix(foo);
  F.block<3, 3>(0, 6) = -Rbn * skewMatrix(imu_radar_calibration_.position);
  F.block<3, 3>(0, 9) =
      -Rbn * imu_radar_calibration_.rotation_matrix * v_r.asDiagonal() * state.radar_scale.asDiagonal().inverse();

  const double tau_gyro  = 1000;
  const double tau_radar = 1000;

  F.block<3, 3>(3, 6) = Rbn;
  F.block<3, 3>(6, 6) = (-1.0 / tau_gyro) * Mat3d::Identity();
  F.block<3, 3>(9, 9) = (-1.0 / tau_radar) * Mat3d::Identity();

  G.block<3, 3>(0, 0) = Rbn * imu_radar_calibration_.rotation_matrix;
  G.block<3, 3>(0, 3) = Rbn * skewMatrix(imu_radar_calibration_.position);
  G.block<3, 3>(3, 3) = Rbn;

  Mat12d Q_ = Mat12d::Zero();
  Q_        = noise_covariance_matrix.Q;

  Q_(0, 0)             = P_v_r(0, 0) * dt_radar;
  Q_(1, 1)             = P_v_r(1, 1) * dt_radar;
  Q_(2, 2)             = P_v_r(2, 2) * dt_radar;
  Q_.block<3, 3>(3, 3) = noise_covariance_matrix.Q.block<3, 3>(3, 3) * dt_radar;
  Q_.block<3, 3>(6, 6) = noise_covariance_matrix.Q.block<3, 3>(6, 6) / dt_radar;
  Q_.block<3, 3>(9, 9) = noise_covariance_matrix.Q.block<3, 3>(9, 9) / dt_radar;

  Mat12d Phi = Mat12d::Zero();
  Phi        = Mat12d::Identity() + F * dt_radar + 0.5 * dt_imu * dt_imu * F * F;

  covariance_matrix.priori     = Phi * covariance_matrix.posteriori * Phi.transpose() + G * Q_ * G.transpose();
  covariance_matrix.priori     = EnsurePSDDr(covariance_matrix.priori);
  covariance_matrix.posteriori = covariance_matrix.priori;

  if (use_cloning) {
    window_count += 1;

    if (window_count == 1) {
      Phi_accumulation = Mat12d::Zero();
      Phi_accumulation = Phi;
    } else {
      Phi_accumulation = Phi * Phi_accumulation;
    }

    if (window_count == window_size) {
      if (window_size == 1) {
        //  NOTE: Special calculation for window_size = 1;
        MatXd F_cloning               = MatXd::Identity(18, 18);
        F_cloning.block<12, 12>(0, 0) = Phi_accumulation;

        MatXd G_cloning               = MatXd::Zero(18, 12);
        G_cloning.block<12, 12>(0, 0) = G;

        covariance_matrix_cloning.priori = Mat18d::Zero();
        covariance_matrix_cloning.priori = F_cloning * covariance_matrix_cloning.posteriori * F_cloning.transpose() +
                                           G_cloning * Q_ * G_cloning.transpose();

        covariance_matrix.priori = covariance_matrix_cloning.priori.block<12, 12>(0, 0);
        covariance_matrix.priori = EnsurePSDDr(covariance_matrix.priori);

      } else {
        //  NOTE: General calculation for window_size > 1
        covariance_matrix.priori = Phi * covariance_matrix.posteriori * Phi.transpose() + G * Q_ * G.transpose();
        covariance_matrix.priori = EnsurePSDDr(covariance_matrix.priori);

        covariance_matrix_cloning.priori = Mat18d::Zero();

        // Upper left
        covariance_matrix_cloning.priori.block<12, 12>(0, 0) = covariance_matrix.priori;

        // Upper Right
        covariance_matrix_cloning.priori.block<12, 6>(0, 12) =
            Phi_accumulation.block<6, 12>(0, 0).transpose() * covariance_matrix_cloning.posteriori.block<6, 6>(12, 12);

        // Lower left
        covariance_matrix_cloning.priori.block<6, 12>(12, 0) =
            covariance_matrix_cloning.posteriori.block<6, 6>(12, 12) * Phi_accumulation.block<6, 12>(0, 0);

        // Lower Right (No need to update)
        covariance_matrix_cloning.priori.block<6, 6>(12, 12) = covariance_matrix_cloning.posteriori.block<6, 6>(12, 12);
      }

      covariance_matrix_cloning.priori     = EnsurePSDCloning(covariance_matrix_cloning.priori);
      covariance_matrix_cloning.posteriori = covariance_matrix_cloning.priori;

      window_count = 0;
    }
  } // if use_cloning
} // void TimeUpdate

bool ScEkfDero::RadarMeasurementUpdate(IMURadarCalibrationParam &imu_radar_calibration_, const ICPTransform &icp_meas,
                                       const bool &outlier_reject, State &first_state, const Vec2d &r_accel,
                                       const MatXd &H_accel) {
  //  NOTE: Measurement Update
  MatXd       H_cloning = MatXd::Zero(3, 18);
  const Mat3d foo_1     = imu_radar_calibration_.rotation_matrix.transpose();
  const Mat3d foo_2     = quat2dcm(first_state.quaternion);
  const Vec3d foo_3     = state.position - first_state.position;
  const Mat3d foo_4     = foo_1 * foo_2.transpose();
  Vec3d       foo_5     = quat2dcm(state.quaternion) * imu_radar_calibration_.position;

  H_cloning.block<3, 3>(0, 0)  = foo_4;
  H_cloning.block<3, 3>(0, 3)  = foo_4 * skewMatrix(foo_5);
  H_cloning.block<3, 3>(0, 12) = -foo_4;
  H_cloning.block<3, 3>(0, 15) =
      foo_4 * (skewMatrix(first_state.position) - skewMatrix(state.position) - skewMatrix(foo_5));

  const Vec3d icp_hat        = foo_4 * foo_3;
  const Vec3d r_icp          = icp_hat - icp_meas.translation;
  const Mat3d R_discrete_icp = icp_meas.rotation * icp_meas.P_vec.asDiagonal() * icp_meas.rotation.transpose();

  Vec5d r                 = Vec5d::Zero();
  Mat5d R                 = Mat5d::Zero();
  MatXd H                 = MatXd::Zero(5, 18);
  MatXd H_accel_          = MatXd::Zero(2, 18);
  MatXd K_radar           = MatXd::Zero(18, 5);
  VecXd error_state_radar = VecXd::Zero(18, 1);

  r << r_icp, r_accel;

  H_accel_.block<2, 12>(0, 0) = H_accel;

  H.block<3, 18>(0, 0) = H_cloning;
  H.block<2, 18>(3, 0) = H_accel_;
  R.block<3, 3>(0, 0)  = R_discrete_icp;
  R.block<2, 2>(3, 3)  = noise_covariance_matrix.R_accel;
  const Mat5d S_radar  = H * covariance_matrix_cloning.priori * H.transpose() + R;
  K_radar              = covariance_matrix_cloning.priori * H.transpose() * S_radar.inverse();
  error_state_radar    = K_radar * r;

  if (outlier_reject) {
    const double gamma = r.transpose() * (H * covariance_matrix_cloning.posteriori * H.transpose() + R).inverse() * r;

    boost::math::chi_squared chiSquaredDist(5.0);
    const double             gamma_thresh = boost::math::quantile(chiSquaredDist, 1 - 0.05);

    if (gamma < gamma_thresh) {
      covariance_matrix_cloning.posteriori = Mat18d::Zero();
      covariance_matrix_cloning.posteriori = (Mat18d::Identity() - K_radar * H) * covariance_matrix_cloning.priori *
                                                 (Mat18d::Identity() - K_radar * H).transpose() +
                                             K_radar * R * K_radar.transpose();

      covariance_matrix_cloning.posteriori = EnsurePSDCloning(covariance_matrix_cloning.posteriori);

      covariance_matrix.posteriori = covariance_matrix_cloning.posteriori.block<12, 12>(0, 0);
      covariance_matrix.posteriori = EnsurePSDDr(covariance_matrix.posteriori);

      ErrorCorrection(error_state_radar);
      return true;
    } else
      return false;
  } else {
    covariance_matrix_cloning.posteriori = Mat18d::Zero();
    covariance_matrix_cloning.posteriori = (Mat18d::Identity() - K_radar * H) * covariance_matrix_cloning.priori *
                                               (Mat18d::Identity() - K_radar * H).transpose() +
                                           K_radar * R * K_radar.transpose();

    covariance_matrix_cloning.posteriori = EnsurePSDCloning(covariance_matrix_cloning.posteriori);

    covariance_matrix.posteriori = covariance_matrix_cloning.posteriori.block<12, 12>(0, 0);
    covariance_matrix.posteriori = EnsurePSDDr(covariance_matrix.posteriori);

    ErrorCorrection(error_state_radar);
    return true;
  }
} // void RadarMeasurementUpdate

bool ScEkfDero::MeasurementUpdateAccel(const Vec2d &r_accel, const MatXd &H_accel, const bool &outlier_reject) {
  const Mat2d R_discrete_accel = noise_covariance_matrix.R_accel;

  const Mat2d S_accel      = H_accel * covariance_matrix.priori * H_accel.transpose() + R_discrete_accel;
  MatXd       K_accel      = MatXd::Zero(12, 2);
  K_accel                  = covariance_matrix.priori * H_accel.transpose() * S_accel.inverse();
  VecXd error_state_accel_ = VecXd::Zero(12, 1);
  error_state_accel_       = K_accel * r_accel;

  if (outlier_reject) {
    const double gamma = r_accel.transpose() *
                         (H_accel * covariance_matrix.posteriori * H_accel.transpose() + R_discrete_accel).inverse() *
                         r_accel;

    boost::math::chi_squared chiSquaredDist(2.0);
    const double             gamma_thresh = boost::math::quantile(chiSquaredDist, 1 - 0.05);

    if (gamma < gamma_thresh) {
      covariance_matrix.posteriori = (Mat12d::Identity() - K_accel * H_accel) * covariance_matrix.priori *
                                         (Mat12d::Identity() - K_accel * H_accel).transpose() +
                                     K_accel * R_discrete_accel * K_accel.transpose();
      covariance_matrix.posteriori                             = EnsurePSDDr(covariance_matrix.posteriori);
      covariance_matrix_cloning.posteriori.block<12, 12>(0, 0) = covariance_matrix.posteriori;

      ErrorCorrection(error_state_accel_);
      return true;
    } else
      return false;
  } else {
    covariance_matrix.posteriori = (Mat12d::Identity() - K_accel * H_accel) * covariance_matrix.priori *
                                       (Mat12d::Identity() - K_accel * H_accel).transpose() +
                                   K_accel * R_discrete_accel * K_accel.transpose();
    covariance_matrix.posteriori                             = EnsurePSDDr(covariance_matrix.posteriori);
    covariance_matrix_cloning.posteriori.block<12, 12>(0, 0) = covariance_matrix.posteriori;

    ErrorCorrection(error_state_accel_);
    return true;
  }
} // void MeasurementUpdateAccel

void ScEkfDero::ErrorCorrection(const VecXd &error_state_) {
  error_state.position     = error_state_.segment(0, 3);
  error_state.misalignment = error_state_.segment(3, 3);
  error_state.gyro_bias    = error_state_.segment(6, 3);
  error_state.radar_scale  = error_state_.segment(9, 3);

  state.position    -= error_state.position;
  state.gyro_bias   -= error_state.gyro_bias;
  state.radar_scale -= error_state.radar_scale;

  Vec4d error_quaternion;
  // clang-format off
  error_quaternion << 1.0, 0.5 * error_state.misalignment(0, 0),
                           0.5 * error_state.misalignment(1, 0),
                           0.5 * error_state.misalignment(2, 0);
  // clang-format on

  state.quaternion = quatMultiplication(error_quaternion, state.quaternion);
} // void ErrorCorrection

Mat2d ScEkfDero::getR_Accel() { return noise_covariance_matrix.R_accel; } // getR_Accel

double ScEkfDero::getGravityValue() { return gravity(2, 0); } // getGravityValue

void ScEkfDero::setGravityValue(double &gravity_constant) { gravity << 0.0, 0.0, gravity_constant; } // setGravityValue

CoarseAlignmentState ScEkfDero::getCoarseAlignmentState() { return ca_state; } // getCoarseAlignmentState

ErrorState ScEkfDero::getErrorState() { return error_state; } // getErrorState

State ScEkfDero::getState() { return state; } // getState

void ScEkfDero::setState(const State &state_) {
  state                    = state_;
  state_cloning.position   = state_.position;
  state_cloning.quaternion = state_.quaternion;
} // setState

CovarianceMatrixCloning ScEkfDero::getCovarianceMatrix() { return covariance_matrix_cloning; } // getCovarianceMatrix

///// IMU /////
double ScEkfDero::getImuDt() { return dt_imu; } // getImuDt

void ScEkfDero::setImuDt() { dt_imu = current_time_imu - previous_time_imu; } // setImuDt

double ScEkfDero::getImuPreviousTime() { return previous_time_imu; } // getImuPreviousTime

void ScEkfDero::setImuPreviousTime(double t) { previous_time_imu = t; } // setImuPreviousTime

double ScEkfDero::getImuCurrentTime() { return current_time_imu; } // getImuCurrentTime

void ScEkfDero::setImuCurrentTime(double t) { current_time_imu = t; } // setImuCurrentTime

///// Radar /////
double ScEkfDero::getRadarDt() { return dt_radar; } // getRadarDt

void ScEkfDero::setRadarDt() { dt_radar = current_time_radar - previous_time_radar; } // setRadarDt

double ScEkfDero::getRadarPreviousTime() { return previous_time_radar; } // getRadarPreviousTime

void ScEkfDero::setRadarPreviousTime(double t) { previous_time_radar = t; } // setRadarPreviousTime

double ScEkfDero::getRadarCurrentTime() { return current_time_radar; } // getRadarCurrentTime

void ScEkfDero::setRadarCurrentTime(double t) { current_time_radar = t; } // setRadarCurrentTime

void ScEkfDero::setQ(const Noise &noise) {
  noise_covariance_matrix.Q = Mat12d::Zero();
  noise_covariance_matrix.Q.block<3, 3>(3, 3) =
      noise.angular_random_walk * noise.angular_random_walk * Mat3d::Identity();
  noise_covariance_matrix.Q.block<3, 3>(6, 6) =
      noise.gyro_bias_random_walk * noise.gyro_bias_random_walk * Mat3d::Identity();
  noise_covariance_matrix.Q.block<3, 3>(9, 9) =
      noise.radar_scale_random_walk * noise.radar_scale_random_walk * Mat3d::Identity();
} // setQ

void ScEkfDero::setR_Accel(const Vec2d &sigma_accel) {
  noise_covariance_matrix.R_accel       = Mat2d::Zero();
  noise_covariance_matrix.R_accel(0, 0) = sigma_accel(0, 0) * sigma_accel(0, 0);
  noise_covariance_matrix.R_accel(1, 1) = sigma_accel(1, 0) * sigma_accel(1, 0);
} // setR_Accel

void ScEkfDero::setCovarianceMatrix(const Init &init) {
  covariance_matrix.posteriori                   = Mat12d::Zero();
  covariance_matrix.posteriori.block<3, 3>(0, 0) = init.position * init.position * Mat3d::Identity();
  covariance_matrix.posteriori.block<2, 2>(3, 3) = init.euler_XY * init.euler_XY * Mat2d::Identity();
  covariance_matrix.posteriori(5, 5)             = init.euler_Z * init.euler_Z;
  covariance_matrix.posteriori.block<3, 3>(6, 6) = init.gyro_bias * init.gyro_bias * Mat3d::Identity();
  covariance_matrix.posteriori.block<3, 3>(9, 9) = init.radar_scale * init.radar_scale * Mat3d::Identity();

  covariance_matrix_cloning.posteriori                     = Mat18d::Zero();
  covariance_matrix_cloning.posteriori.block<12, 12>(0, 0) = covariance_matrix.posteriori;
  covariance_matrix_cloning.posteriori.block<6, 6>(12, 12) = covariance_matrix.posteriori.block<6, 6>(0, 0);
} // setCovarianceMatrix

Vec3d ScEkfDero::getVbn() { return v_n_; } // getVbn
} // namespace incsl
