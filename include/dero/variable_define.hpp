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

#ifndef VARIABLE_DEFINE_HPP
#define VARIABLE_DEFINE_HPP

#include <Eigen/Eigen>

namespace incsl {

typedef Eigen::Matrix<float, 4, 4> Mat4f;
typedef Eigen::Matrix<float, 3, 3> Mat3f;

typedef Eigen::Matrix<float, 3, 1> Vec3f;

typedef Eigen::Matrix<double, 1, 1>   Mat1d;
typedef Eigen::Matrix<double, 2, 2>   Mat2d;
typedef Eigen::Matrix<double, 3, 3>   Mat3d;
typedef Eigen::Matrix<double, 4, 4>   Mat4d;
typedef Eigen::Matrix<double, 5, 5>   Mat5d;
typedef Eigen::Matrix<double, 10, 10> Mat10d;
typedef Eigen::Matrix<double, 12, 12> Mat12d;
typedef Eigen::Matrix<double, 13, 13> Mat13d;
typedef Eigen::Matrix<double, 15, 15> Mat15d;
typedef Eigen::Matrix<double, 16, 16> Mat16d;
typedef Eigen::Matrix<double, 18, 18> Mat18d;
typedef Eigen::Matrix<double, 19, 19> Mat19d;
typedef Eigen::MatrixXd               MatXd;

typedef Eigen::Matrix<double, 1, 1>  Vec1d;
typedef Eigen::Matrix<double, 2, 1>  Vec2d;
typedef Eigen::Matrix<double, 3, 1>  Vec3d;
typedef Eigen::Matrix<double, 4, 1>  Vec4d;
typedef Eigen::Matrix<double, 5, 1>  Vec5d;
typedef Eigen::Matrix<double, 10, 1> Vec10d;
typedef Eigen::Matrix<double, 11, 1> Vec11d;
typedef Eigen::Matrix<double, 12, 1> Vec12d;
typedef Eigen::Matrix<double, 13, 1> Vec13d;
typedef Eigen::Matrix<double, 15, 1> Vec15d;
typedef Eigen::Matrix<double, 16, 1> Vec16d;
typedef Eigen::Matrix<double, 18, 1> Vec18d;
typedef Eigen::Matrix<double, 19, 1> Vec19d;
typedef Eigen::VectorXd              VecXd;

typedef Eigen::Quaterniond Quaternion;

typedef struct {
    Vec3d position;
    Vec3d velocity;
    Vec4d quaternion;
    Vec3d accel_bias;
    Vec3d gyro_bias;
    Vec3d radar_scale;
} State;

typedef struct {
    Vec3d position;
    Vec3d velocity;
    Vec4d quaternion;
} StateCloning;

typedef struct {
    double position;
    double velocity;
    double euler_XY;
    double euler_Z;
    double accel_bias;
    double gyro_bias;
    double radar_scale;
} Init;

typedef struct {
    Vec3d position;
    Vec3d velocity;
    Vec3d misalignment;
    Vec3d accel_bias;
    Vec3d gyro_bias;
    Vec3d radar_scale;
} ErrorState;

typedef struct {
    Vec3d position;
    Vec3d velocity;
    Vec3d misalignment;
} ErrorStateCloning;

typedef struct {
    Vec3d euler;
    Vec3d position;
    Vec3d velocity;
    Vec3d accel_bias;
    Vec3d gyro_bias;
    Vec3d radar_scale;
} CoarseAlignmentState;

typedef struct {
    Mat15d priori;
    Mat15d posteriori;
} CovarianceMatrix;

typedef struct {
    Mat12d priori;
    Mat12d posteriori;
} CovarianceMatrixDr;

typedef struct {
    Mat18d priori;
    Mat18d posteriori;
} CovarianceMatrixCloning;

typedef struct {
    Mat12d Q;
    Mat3d  R_radar;
} NoiseCovarianceMatrix;

typedef struct {
    Mat12d Q;
    Mat2d  R_accel;
} NoiseCovarianceMatrixDr;

typedef struct {
    double velocity_random_walk;
    double angular_random_walk;
    double accel_bias_random_walk;
    double gyro_bias_random_walk;
    double radar_scale_random_walk;
} Noise;

typedef struct {
    Vec3d position;
    Mat3d rotation_matrix;
    Vec4d quaternion;
} IMURadarCalibrationParam;

typedef struct {
    double min_distance;
    double max_distance;
    double min_db;
    double elevation_threshold;
    double azimuth_threshold;
    double velocity_correction_factor;
    double filter_min_z;
    double filter_max_z;
    double zero_velocity_threshold;
    double sigma_zero_velocity_x;
    double sigma_zero_velocity_y;
    double sigma_zero_velocity_z;
    double allowed_outlier_percentage;
    double max_sigma_x;
    double max_sigma_y;
    double max_sigma_z;
    double max_r_cond;
    double outlier_prob;
    double success_prob;
    double inlier_threshold;
    double sigma_offset_radar_x;
    double sigma_offset_radar_y;
    double sigma_offset_radar_z;
    double outlier_percentil_radar;
    double min_speed_odr;
    double sigma_v_d;
    double model_noise_offset_deg;
    double model_noise_scale_deg;

    bool use_ransac;
    bool use_odr;
    uint N_ransac_points;

    long unsigned int odr_inlier_threshold;
} RadarVelocityEstimatorParam;

typedef struct {
    int    max_iter;
    double max_corres_dis;
    double transform_eps;
    double euclidean_fit_eps;
    double ransac_outlier_reject_threshold;
    double icp_std_x;
    double icp_std_y;
    double icp_std_z;
} RadarPositionEstimatorParam;

typedef struct {
    Mat4d  homo;
    Mat3d  rotation;
    Vec3d  translation;
    double score;
    size_t number_of_points;
    Vec3d  P_vec;
} ICPTransform;

} // namespace incsl

#endif // !VARIABLE_DEFINE_HPP
