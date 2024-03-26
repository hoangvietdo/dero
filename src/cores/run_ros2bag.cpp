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

#include "dero/run_ros2bag.hpp"

using namespace std::chrono_literals;

namespace incsl {

RunRos2Bag::RunRos2Bag(std::string node_name) : rclcpp::Node(node_name) {
  RCLCPP_INFO(this->get_logger(), "Create a ROS2 Node!");

  this->declare_parameter("imu_topic", "");
  this->declare_parameter("radar_topic", "");
  this->declare_parameter("groundtruth_topic", "");
  this->declare_parameter("est_save_dir", "");
  this->declare_parameter("ros2_pub_rate", 1);
  this->declare_parameter("cloning_window_size", 1);
  this->declare_parameter("coarse_alignment_window_size", 0);
  this->declare_parameter("imu_body_rotation_offset_x", 0.0);
  this->declare_parameter("imu_body_rotation_offset_y", 0.0);
  this->declare_parameter("imu_body_rotation_offset_z", 0.0);
  this->declare_parameter("imu_radar_position_offset_x", 0.0);
  this->declare_parameter("imu_radar_position_offset_y", 0.0);
  this->declare_parameter("imu_radar_position_offset_z", 0.0);
  this->declare_parameter("imu_radar_quaternion_offset_w", 0.0);
  this->declare_parameter("imu_radar_quaternion_offset_x", 0.0);
  this->declare_parameter("imu_radar_quaternion_offset_y", 0.0);
  this->declare_parameter("imu_radar_quaternion_offset_z", 0.0);
  this->declare_parameter("velocity_random_walk", 0.0);
  this->declare_parameter("angular_random_walk", 0.0);
  this->declare_parameter("accel_bias_random_walk", 0.0);
  this->declare_parameter("gyro_bias_random_walk", 0.0);
  this->declare_parameter("radar_scale_random_walk", 0.0);
  this->declare_parameter("P_init_position", 0.0);
  this->declare_parameter("P_init_velocity", 0.0);
  this->declare_parameter("P_init_attitude_XY", 0.0);
  this->declare_parameter("P_init_attitude_Z", 0.0);
  this->declare_parameter("P_init_accel_bias", 0.0);
  this->declare_parameter("P_init_gyro_bias", 0.0);
  this->declare_parameter("P_init_radar_scale", 0.0);
  this->declare_parameter("use_radar", true);
  this->declare_parameter("imu_only", false);
  this->declare_parameter("use_cloning", true);
  this->declare_parameter("groundtruth_included", false);
  this->declare_parameter("max_pos_acc_threshold", 0.0);
  this->declare_parameter("max_vel_acc_threshold", 0.0);
  this->declare_parameter("sigma_pos_scaling", 0.0);
  this->declare_parameter("sigma_vel_scaling", 0.0);
  this->declare_parameter("min_number_sat", 0);
  this->declare_parameter("min_distance", 0.0);
  this->declare_parameter("max_distance", 0.0);
  this->declare_parameter("min_db", 0.0);
  this->declare_parameter("elevation_threshold", 0.0);
  this->declare_parameter("azimuth_threshold", 0.0);
  this->declare_parameter("velocity_correction_factor", 0.0);
  this->declare_parameter("filter_min_z", 0.0);
  this->declare_parameter("filter_max_z", 0.0);
  this->declare_parameter("zero_velocity_threshold", 0.0);
  this->declare_parameter("sigma_zero_velocity_x", 0.0);
  this->declare_parameter("sigma_zero_velocity_y", 0.0);
  this->declare_parameter("sigma_zero_velocity_z", 0.0);
  this->declare_parameter("allowed_outlier_percentage", 0.0);
  this->declare_parameter("max_sigma_x", 0.0);
  this->declare_parameter("max_sigma_y", 0.0);
  this->declare_parameter("max_sigma_z", 0.0);
  this->declare_parameter("max_r_cond", 0.0);
  this->declare_parameter("use_ransac", true);
  this->declare_parameter("outlier_prob", 0.0);
  this->declare_parameter("success_prob", 0.0);
  this->declare_parameter("N_ransac_points", 0);
  this->declare_parameter("inlier_threshold", 0.0);
  this->declare_parameter("sigma_offset_radar_x", 0.0);
  this->declare_parameter("sigma_offset_radar_y", 0.0);
  this->declare_parameter("sigma_offset_radar_z", 0.0);
  this->declare_parameter("outlier_percentil_radar", 0.0);
  this->declare_parameter("use_odr", false);
  this->declare_parameter("min_speed_odr", 0.0);
  this->declare_parameter("sigma_v_d", 0.0);
  this->declare_parameter("model_noise_offset_deg", 0.0);
  this->declare_parameter("model_noise_scale_deg", 0.0);
  this->declare_parameter("odr_inlier_threshold", 0);
  this->declare_parameter("gravity", 0.0);
  this->declare_parameter("use_dr_structure", false);
  this->declare_parameter("radar_outlier_reject", false);
  this->declare_parameter("bag_dir", "");
  this->declare_parameter("max_corres_dis", 0.0);
  this->declare_parameter("transform_eps", 0.0);
  this->declare_parameter("max_iter", 0);
  this->declare_parameter("euclidean_fit_eps", 0.0);
  this->declare_parameter("window_slicing", false);
  this->declare_parameter("ransac_outlier_reject_thres", 0.0);

  LoadParameters();

  pub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_broadcaster_pose_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadcaster_radar_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_period_ = 1000 / ros2_pub_rate_;

  viet_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_),
                                        std::bind(&RunRos2Bag::MsgPublish, this), pub_callback_group_);

  callback_timer_process_ = this->create_wall_timer(1ms, std::bind(&RunRos2Bag::Run, this), process_callback_group_);

  radar_publisher_        = this->create_publisher<sensor_msgs::msg::PointCloud2>("incsl/radar_filtered_scan", 10);
  radar_raw_publisher_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("incsl/radar_raw_scan", 10);
  pose_path_publisher_    = this->create_publisher<nav_msgs::msg::Path>("incsl/pose_path", 10);
  pose_path_gt_publisher_ = this->create_publisher<nav_msgs::msg::Path>("incsl/gt_path", 10);

  est_save.open(est_save_dir_);
} // RunRos2Bag

void RunRos2Bag::LoadParameters() {
  RCLCPP_INFO(this->get_logger(), "Loading Parameters...");

  this->get_parameter("imu_topic", imu_topic_name_);
  this->get_parameter("radar_topic", radar_topic_name_);
  this->get_parameter("groundtruth_topic", groundtruth_topic_name_);
  this->get_parameter("est_save_dir", est_save_dir_);
  this->get_parameter("ros2_pub_rate", ros2_pub_rate_);
  this->get_parameter("cloning_window_size", cloning_window_size);
  this->get_parameter("coarse_alignment_window_size", ca_wind_);
  this->get_parameter("gravity", grav_);
  this->get_parameter("imu_body_rotation_offset_x", imu_body_rotation_offset_x_);
  this->get_parameter("imu_body_rotation_offset_y", imu_body_rotation_offset_y_);
  this->get_parameter("imu_body_rotation_offset_z", imu_body_rotation_offset_z_);
  this->get_parameter("imu_radar_position_offset_x", imu_radar_position_offset_x_);
  this->get_parameter("imu_radar_position_offset_y", imu_radar_position_offset_y_);
  this->get_parameter("imu_radar_position_offset_z", imu_radar_position_offset_z_);
  this->get_parameter("imu_radar_quaternion_offset_w", imu_radar_quaternion_offset_w_);
  this->get_parameter("imu_radar_quaternion_offset_x", imu_radar_quaternion_offset_x_);
  this->get_parameter("imu_radar_quaternion_offset_y", imu_radar_quaternion_offset_y_);
  this->get_parameter("imu_radar_quaternion_offset_z", imu_radar_quaternion_offset_z_);
  this->get_parameter("velocity_random_walk", vel_rw_);
  this->get_parameter("angular_random_walk", ang_rw_);
  this->get_parameter("accel_bias_random_walk", accel_bias_rw_);
  this->get_parameter("gyro_bias_random_walk", gyro_bias_rw_);
  this->get_parameter("radar_scale_random_walk", radar_scale_rw_);
  this->get_parameter("P_init_position", P0_pos);
  this->get_parameter("P_init_velocity", P0_vel);
  this->get_parameter("P_init_attitude_XY", P0_att_XY);
  this->get_parameter("P_init_attitude_Z", P0_att_Z);
  this->get_parameter("P_init_accel_bias", P0_ba);
  this->get_parameter("P_init_gyro_bias", P0_bg);
  this->get_parameter("P_init_radar_scale", P0_br);
  this->get_parameter("use_cloning", use_cloning);
  this->get_parameter("use_radar", use_radar);
  this->get_parameter("imu_only", imu_only);
  this->get_parameter("groundtruth_included", groundtruth_included);
  this->get_parameter("max_pos_acc_threshold", max_pos_acc_threshold_);
  this->get_parameter("max_vel_acc_threshold", max_vel_acc_threshold_);
  this->get_parameter("sigma_pos_scaling", sigma_pos_scaling_);
  this->get_parameter("sigma_vel_scaling", sigma_vel_scaling_);
  this->get_parameter("min_number_sat", min_number_sat_);
  this->get_parameter("min_distance", min_distance_);
  this->get_parameter("max_distance", max_distance_);
  this->get_parameter("min_db", min_db_);
  this->get_parameter("elevation_threshold", elevation_threshold_);
  this->get_parameter("azimuth_threshold", azimuth_threshold_);
  this->get_parameter("velocity_correction_factor", velocity_correction_factor_);
  this->get_parameter("filter_min_z", filter_min_z_);
  this->get_parameter("filter_max_z", filter_max_z_);
  this->get_parameter("zero_velocity_threshold", zero_velocity_threshold_);
  this->get_parameter("sigma_zero_velocity_x", sigma_zero_velocity_x_);
  this->get_parameter("sigma_zero_velocity_y", sigma_zero_velocity_y_);
  this->get_parameter("sigma_zero_velocity_z", sigma_zero_velocity_z_);
  this->get_parameter("allowed_outlier_percentage", allowed_outlier_percentage_);
  this->get_parameter("max_sigma_x", max_sigma_x_);
  this->get_parameter("max_sigma_y", max_sigma_y_);
  this->get_parameter("max_sigma_z", max_sigma_z_);
  this->get_parameter("max_r_cond", max_r_cond_);
  this->get_parameter("use_ransac", use_ransac_);
  this->get_parameter("outlier_prob", outlier_prob_);
  this->get_parameter("success_prob", success_prob_);
  this->get_parameter("N_ransac_points", N_ransac_points_);
  this->get_parameter("inlier_threshold", inlier_threshold_);
  this->get_parameter("sigma_offset_radar_x", sigma_offset_radar_x_);
  this->get_parameter("sigma_offset_radar_y", sigma_offset_radar_y_);
  this->get_parameter("sigma_offset_radar_z", sigma_offset_radar_z_);
  this->get_parameter("outlier_percentil_radar", outlier_percentil_radar_);
  this->get_parameter("use_odr", use_odr_);
  this->get_parameter("min_speed_odr", min_speed_odr_);
  this->get_parameter("sigma_v_d", sigma_v_d_);
  this->get_parameter("model_noise_offset_deg", model_noise_offset_deg_);
  this->get_parameter("model_noise_scale_deg", model_noise_scale_deg_);
  this->get_parameter("odr_inlier_threshold", odr_inlier_threshold_);
  this->get_parameter("use_dr_structure", use_dr_structure);
  this->get_parameter("radar_outlier_reject", radar_outlier_reject);
  this->get_parameter("max_corres_dis", max_corres_dis_);
  this->get_parameter("transform_eps", transform_eps_);
  this->get_parameter("max_iter", max_iter_);
  this->get_parameter("euclidean_fit_eps", euclidean_fit_eps_);
  this->get_parameter("window_slicing", window_slicing);
  this->get_parameter("ransac_outlier_reject_thres", ransac_outlier_reject_thres);
  this->get_parameter("bag_dir", bag_dir);

  radar_velocity_estimator_param_.min_distance               = min_distance_;
  radar_velocity_estimator_param_.max_distance               = max_distance_;
  radar_velocity_estimator_param_.min_db                     = min_db_;
  radar_velocity_estimator_param_.elevation_threshold        = elevation_threshold_;
  radar_velocity_estimator_param_.azimuth_threshold          = azimuth_threshold_;
  radar_velocity_estimator_param_.velocity_correction_factor = velocity_correction_factor_;
  radar_velocity_estimator_param_.filter_min_z               = filter_min_z_;
  radar_velocity_estimator_param_.filter_max_z               = filter_max_z_;
  radar_velocity_estimator_param_.zero_velocity_threshold    = zero_velocity_threshold_;
  radar_velocity_estimator_param_.sigma_zero_velocity_x      = sigma_zero_velocity_x_;
  radar_velocity_estimator_param_.sigma_zero_velocity_y      = sigma_zero_velocity_y_;
  radar_velocity_estimator_param_.sigma_zero_velocity_z      = sigma_zero_velocity_z_;
  radar_velocity_estimator_param_.allowed_outlier_percentage = allowed_outlier_percentage_;
  radar_velocity_estimator_param_.max_sigma_x                = max_sigma_x_;
  radar_velocity_estimator_param_.max_sigma_y                = max_sigma_y_;
  radar_velocity_estimator_param_.max_sigma_z                = max_sigma_z_;
  radar_velocity_estimator_param_.max_r_cond                 = max_r_cond_;
  radar_velocity_estimator_param_.use_ransac                 = use_ransac_;
  radar_velocity_estimator_param_.outlier_prob               = outlier_prob_;
  radar_velocity_estimator_param_.success_prob               = success_prob_;
  radar_velocity_estimator_param_.N_ransac_points            = N_ransac_points_;
  radar_velocity_estimator_param_.inlier_threshold           = inlier_threshold_;
  radar_velocity_estimator_param_.sigma_offset_radar_x       = sigma_offset_radar_x_;
  radar_velocity_estimator_param_.sigma_offset_radar_y       = sigma_offset_radar_y_;
  radar_velocity_estimator_param_.sigma_offset_radar_z       = sigma_offset_radar_z_;
  radar_velocity_estimator_param_.outlier_percentil_radar    = outlier_percentil_radar_;
  radar_velocity_estimator_param_.use_odr                    = use_odr_;
  radar_velocity_estimator_param_.min_speed_odr              = min_speed_odr_;
  radar_velocity_estimator_param_.sigma_v_d                  = sigma_v_d_;
  radar_velocity_estimator_param_.model_noise_offset_deg     = model_noise_offset_deg_;
  radar_velocity_estimator_param_.model_noise_scale_deg      = model_noise_scale_deg_;
  radar_velocity_estimator_param_.odr_inlier_threshold       = odr_inlier_threshold_;

  radar_position_estimator_param_.max_corres_dis                  = max_corres_dis_;
  radar_position_estimator_param_.max_iter                        = max_iter_;
  radar_position_estimator_param_.transform_eps                   = transform_eps_;
  radar_position_estimator_param_.euclidean_fit_eps               = euclidean_fit_eps_;
  radar_position_estimator_param_.ransac_outlier_reject_threshold = ransac_outlier_reject_thres;

  RCLCPP_INFO_ONCE(this->get_logger(), "The IMU topic name is %s", imu_topic_name_.c_str());
  RCLCPP_INFO_ONCE(this->get_logger(), "The Radar topic name is %s", radar_topic_name_.c_str());

  if (groundtruth_included)
    RCLCPP_INFO_ONCE(this->get_logger(), "The ground truth topic name is %s", groundtruth_topic_name_.c_str());
  else {
    RCLCPP_INFO_ONCE(this->get_logger(), "No ground truth data is provided");
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing rate is %d Hz", ros2_pub_rate_);
    RCLCPP_INFO_ONCE(this->get_logger(), "Coarse alignment window size is %d", ca_wind_);
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "EKF estimation will be saved in %s", est_save_dir_.c_str());

  if (use_dr_structure)
    RCLCPP_INFO_ONCE(this->get_logger(), "Use dead reckoning approach: True");
  else
    RCLCPP_INFO_ONCE(this->get_logger(), "Use dead reckoning approach: False");

  if (radar_outlier_reject)
    RCLCPP_INFO_ONCE(this->get_logger(), "Using Chi squared test to reject radar outliers: True");
  else
    RCLCPP_INFO_ONCE(this->get_logger(), "Using Chi squared test to reject radar outliers: False");

  RCLCPP_INFO_ONCE(this->get_logger(), "IMU and vehicle's body frame rotation offset is [%f %f %f] (degree)",
                   imu_body_rotation_offset_x_, imu_body_rotation_offset_y_, imu_body_rotation_offset_z_);
  RCLCPP_INFO_ONCE(this->get_logger(), "IMU velocity random walk is %f", vel_rw_);
  RCLCPP_INFO_ONCE(this->get_logger(), "IMU angular random walk is %f", ang_rw_);
  RCLCPP_INFO_ONCE(this->get_logger(), "IMU accel bias random walk is %f", accel_bias_rw_);
  RCLCPP_INFO_ONCE(this->get_logger(), "IMU gyro bias random walk is %e", gyro_bias_rw_);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std position is %e", P0_pos);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std velocity is %e", P0_vel);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std attitude XY is %e", P0_att_XY);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std attitude Z is %e", P0_att_Z);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std accel bias is %f", P0_ba);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std gyro bias is %e", P0_bg);
  RCLCPP_INFO_ONCE(this->get_logger(), "EKF's P initial std radar scale is %f", P0_br);

  RCLCPP_INFO_ONCE(this->get_logger(), "Radar scale factor random walk std is %f", radar_scale_rw_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: min_distance is %.2f", min_distance_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: max_distance is %.2f", max_distance_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: min_db is %.2f", min_db_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: elevation threshold is %.2f", elevation_threshold_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: azimuth threshold is %.2f", azimuth_threshold_);
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar filter: zero velocity threshold is %.2f", zero_velocity_threshold_);

  RCLCPP_INFO_ONCE(this->get_logger(), "RANSAC: nuber of points is %d", N_ransac_points_);
  RCLCPP_INFO_ONCE(this->get_logger(), "RANSAC: outlier probability is %.2f", outlier_prob_);
  RCLCPP_INFO_ONCE(this->get_logger(), "RANSAC: success probability is %f", success_prob_);

  noise_.velocity_random_walk    = vel_rw_;
  noise_.angular_random_walk     = ang_rw_;
  noise_.accel_bias_random_walk  = accel_bias_rw_;
  noise_.gyro_bias_random_walk   = gyro_bias_rw_;
  noise_.radar_scale_random_walk = radar_scale_rw_;

  init_.position    = P0_pos;
  init_.velocity    = P0_vel;
  init_.euler_XY    = P0_att_XY;
  init_.euler_Z     = P0_att_Z;
  init_.accel_bias  = P0_ba;
  init_.gyro_bias   = P0_bg;
  init_.radar_scale = P0_br;

  state_.position   = Vec3d::Zero();
  state_.velocity   = Vec3d::Zero();
  state_.accel_bias = Vec3d::Zero();
  state_.gyro_bias  = Vec3d::Zero();
  state_.quaternion << 1.0, 0.0, 0.0, 0.0;
  state_.radar_scale << 1.0, 1.0, 1.0;

  accel_roll_pitch_noise_ << 2.0 * D2R, 2.0 * D2R;

  if (!use_dr_structure) {
    ekf_rio_.setQ(noise_);
    ekf_rio_.setState(state_);
    ekf_rio_.setCovarianceMatrix(init_);
    ekf_rio_.setGravityValue(grav_);

  } else {
    scekf_dero_.setQ(noise_);
    scekf_dero_.setR_Accel(accel_roll_pitch_noise_);
    scekf_dero_.setState(state_);
    scekf_dero_.setCovarianceMatrix(init_);
    scekf_dero_.setGravityValue(grav_);
  }

  Vec3d offset_body_euler(imu_body_rotation_offset_x_ * D2R, imu_body_rotation_offset_y_ * D2R,
                          imu_body_rotation_offset_z_ * D2R);

  const Mat3d offset_body_rot = euler2dcm(offset_body_euler);

  imu_radar_calibration_.position << imu_radar_position_offset_x_, imu_radar_position_offset_y_,
      imu_radar_position_offset_z_;

  imu_radar_calibration_.position = offset_body_rot * imu_radar_calibration_.position;

  Vec4d calib_quat_foo;
  calib_quat_foo << imu_radar_quaternion_offset_w_, imu_radar_quaternion_offset_x_, imu_radar_quaternion_offset_y_,
      imu_radar_quaternion_offset_z_;

  imu_radar_calibration_.rotation_matrix = offset_body_rot * quat2dcm(calib_quat_foo);
  imu_radar_calibration_.quaternion      = dcm2quat(imu_radar_calibration_.rotation_matrix);

  icp_init_pose = Mat4d::Identity();

  RCLCPP_INFO_ONCE(this->get_logger(), "IMU-Radar position offset is [%.2f %.2f %.2f] (m)",
                   imu_radar_calibration_.position(0, 0), imu_radar_calibration_.position(1, 0),
                   imu_radar_calibration_.position(2, 0));

  Vec3d calib_euler_foo = quat2euler(imu_radar_calibration_.quaternion);

  RCLCPP_INFO_ONCE(this->get_logger(), "IMU-Radar attitude offset is [%.2f %.2f %.2f] (deg)",
                   R2D * calib_euler_foo(0, 0), R2D * calib_euler_foo(1, 0), R2D * calib_euler_foo(2, 0));
  RCLCPP_INFO_ONCE(this->get_logger(), "Finished loading!");

  if (use_dr_structure)
    RCLCPP_INFO(this->get_logger(), "Running 3D Stochastic Cloning EKF Dead Reckoning Radar Odometry (SCEKF-DeRO)...");
  else
    RCLCPP_INFO(this->get_logger(), "Running 3D EKF Radar Inertial Odometry (EKF-RIO)...");

} // void LoadParameters

void RunRos2Bag::ImuCallback(const sensor_msgs::msg::Imu imu_msg) {
  imu_data_received_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "IMU: Data received!");

  if (!use_dr_structure) {
    ekf_rio_.setImuCurrentTime(imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9);

    if ((ekf_rio_.getImuCurrentTime() <= ekf_rio_.getImuPreviousTime()) && ekf_rio_init_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Imu messages in distorder!");
      return;
    }

    ekf_rio_.setImuDt();
    ekf_rio_.setImuPreviousTime(ekf_rio_.getImuCurrentTime());

  } else {
    scekf_dero_.setImuCurrentTime(imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9);

    if ((scekf_dero_.getImuCurrentTime() <= scekf_dero_.getImuPreviousTime()) && ekf_rio_init_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Imu messages in distorder!");
      return;
    }

    scekf_dero_.setImuDt();
    scekf_dero_.setImuPreviousTime(scekf_dero_.getImuCurrentTime());
  }
  // Transform IMU frame to vehicle's body frame
  Vec3d f_b_raw, w_b_raw, angle_offset, f_b_transformed, w_b_transformed;
  // clang-format off
  f_b_raw << imu_msg.linear_acceleration.x,
             imu_msg.linear_acceleration.y,
             imu_msg.linear_acceleration.z;

  w_b_raw << imu_msg.angular_velocity.x,
             imu_msg.angular_velocity.y,
             imu_msg.angular_velocity.z;

  angle_offset << imu_body_rotation_offset_x_,
                  imu_body_rotation_offset_y_,
                  imu_body_rotation_offset_z_;
  // clang-format on

  angle_offset *= D2R;

  R_imu_body = euler2dcm(angle_offset);

  f_b_transformed = R_imu_body * f_b_raw;
  w_b_transformed = R_imu_body * w_b_raw;

  sensor_msgs::msg::Imu transformed_msg;

  transformed_msg.header = imu_msg.header;

  transformed_msg.orientation = imu_msg.orientation;

  transformed_msg.linear_acceleration.x = f_b_transformed(0, 0);
  transformed_msg.linear_acceleration.y = f_b_transformed(1, 0);
  transformed_msg.linear_acceleration.z = f_b_transformed(2, 0);
  transformed_msg.angular_velocity.x    = w_b_transformed(0, 0);
  transformed_msg.angular_velocity.y    = w_b_transformed(1, 0);
  transformed_msg.angular_velocity.z    = w_b_transformed(2, 0);

  queue_imu_buff.push(transformed_msg);

} // void ImuCallback

void RunRos2Bag::RadarCallback(const sensor_msgs::msg::PointCloud2 radar_msg) {
  radar_data_received_ = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "Radar: Data received!");

  if (!use_dr_structure) {
    ekf_rio_.setRadarCurrentTime(radar_msg.header.stamp.sec + radar_msg.header.stamp.nanosec * 1e-9);

    if ((ekf_rio_.getRadarCurrentTime() <= ekf_rio_.getRadarPreviousTime()) && ekf_rio_init_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Radar messages in distorder!");
      return;
    }

    ekf_rio_.setRadarDt();
    ekf_rio_.setRadarPreviousTime(ekf_rio_.getRadarCurrentTime());

  } else {
    scekf_dero_.setRadarCurrentTime(radar_msg.header.stamp.sec + radar_msg.header.stamp.nanosec * 1e-9);

    if ((scekf_dero_.getRadarCurrentTime() <= scekf_dero_.getRadarPreviousTime()) && ekf_rio_init_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Radar messages in distorder!");
      return;
    }

    scekf_dero_.setRadarDt();
    scekf_dero_.setRadarPreviousTime(scekf_dero_.getRadarCurrentTime());
  }

  sensor_msgs::msg::PointCloud2 radar_data_;
  radar_data_.header       = radar_msg.header;
  radar_data_.height       = radar_msg.height;
  radar_data_.width        = radar_msg.width;
  radar_data_.fields       = radar_msg.fields;
  radar_data_.is_bigendian = radar_msg.is_bigendian;
  radar_data_.point_step   = radar_msg.point_step;
  radar_data_.row_step     = radar_msg.row_step;
  radar_data_.data         = radar_msg.data;
  radar_data_.is_dense     = radar_msg.is_dense;

  radar_data_pub_ = radar_data_;

  queue_radar_buff.push(radar_data_);
} // void RadarCallback

void RunRos2Bag::MsgPublish() {
  if (imu_data_received_ && radar_data_received_ && ekf_rio_init_) {

    rclcpp::Time stamp_now_;

    if (!groundtruth_included)
      stamp_now_ = this->get_clock()->now();
    else {
      stamp_now_ = gt_msg.header.stamp;
      geometry_msgs::msg::PoseStamped pose_gt_;

      pose_gt_.pose.position.x = -gt_msg.pose.position.y;
      pose_gt_.pose.position.y = -gt_msg.pose.position.x;
      pose_gt_.pose.position.z = -gt_msg.pose.position.z;

      pose_gt_.pose.orientation.w = gt_msg.pose.orientation.w;
      pose_gt_.pose.orientation.x = gt_msg.pose.orientation.x;
      pose_gt_.pose.orientation.y = gt_msg.pose.orientation.y;
      pose_gt_.pose.orientation.z = gt_msg.pose.orientation.z;

      pose_gt_.header.stamp    = stamp_now_;
      pose_gt_.header.frame_id = "world";

      pose_path_gt_.header.stamp    = stamp_now_;
      pose_path_gt_.header.frame_id = "world";
      pose_path_gt_.poses.push_back(pose_gt_);

      pose_path_gt_publisher_->publish(pose_path_gt_);
    }

    if (!use_dr_structure)
      state_ = ekf_rio_.getState();
    else
      state_ = scekf_dero_.getState();

    geometry_msgs::msg::TransformStamped pose_transform_;
    pose_transform_.header.stamp    = stamp_now_;
    pose_transform_.header.frame_id = "world";
    pose_transform_.child_frame_id  = "incsl_robot";

    pose_transform_.transform.translation.x = state_.position(0, 0);
    pose_transform_.transform.translation.y = state_.position(1, 0);
    pose_transform_.transform.translation.z = state_.position(2, 0);

    pose_transform_.transform.rotation.w = state_.quaternion(0, 0);
    pose_transform_.transform.rotation.x = state_.quaternion(1, 0);
    pose_transform_.transform.rotation.y = state_.quaternion(2, 0);
    pose_transform_.transform.rotation.z = state_.quaternion(3, 0);

    tf_broadcaster_pose_->sendTransform(pose_transform_);

    geometry_msgs::msg::TransformStamped radar_transform_;
    radar_transform_.header.stamp    = stamp_now_;
    radar_transform_.header.frame_id = "incsl_robot";
    radar_transform_.child_frame_id  = "incsl_radar";

    radar_transform_.transform.translation.x = imu_radar_calibration_.position(0, 0);
    radar_transform_.transform.translation.y = imu_radar_calibration_.position(1, 0);
    radar_transform_.transform.translation.z = imu_radar_calibration_.position(2, 0);

    radar_transform_.transform.rotation.w = imu_radar_calibration_.quaternion(0, 0);
    radar_transform_.transform.rotation.x = imu_radar_calibration_.quaternion(1, 0);
    radar_transform_.transform.rotation.y = imu_radar_calibration_.quaternion(2, 0);
    radar_transform_.transform.rotation.z = imu_radar_calibration_.quaternion(3, 0);

    tf_broadcaster_radar_->sendTransform(radar_transform_);

    radar_data_pub_.header.frame_id = "incsl_radar";
    radar_data_pub_.header.stamp    = stamp_now_;
    radar_raw_publisher_->publish(radar_data_pub_);

    sensor_msgs::msg::PointCloud2 radar_filtered_pcl2_ = radar_estimator_.getInlierRadarRos2PCL2();

    if (radar_filtered_pcl2_.header.frame_id.empty()) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Radar PointCloud2 has empty frame_id, setting default frame_id: incsl_radar!");
    }

    radar_filtered_pcl2_.header.frame_id = "incsl_radar";
    radar_filtered_pcl2_.header.stamp    = stamp_now_;
    radar_publisher_->publish(radar_filtered_pcl2_);

    geometry_msgs::msg::PoseStamped pose_;

    pose_.pose.position.x = state_.position(0, 0);
    pose_.pose.position.y = state_.position(1, 0);
    pose_.pose.position.z = state_.position(2, 0);

    pose_.pose.orientation.w = state_.quaternion(0, 0);
    pose_.pose.orientation.x = state_.quaternion(1, 0);
    pose_.pose.orientation.y = state_.quaternion(2, 0);
    pose_.pose.orientation.z = state_.quaternion(3, 0);

    pose_.header.stamp    = stamp_now_;
    pose_.header.frame_id = "world";

    pose_path_.header.stamp    = stamp_now_;
    pose_path_.header.frame_id = "world";
    pose_path_.poses.push_back(pose_);

    pose_path_publisher_->publish(pose_path_);
  } // if
} // void MsgPublish

void RunRos2Bag::ShutdownHandler() {
  est_save.close();
  RCLCPP_INFO_ONCE(this->get_logger(), "Skipping %d radar data due to initialization process", radar_skip);
  RCLCPP_INFO_ONCE(this->get_logger(), "Finished simulation!");
  RCLCPP_INFO_ONCE(this->get_logger(), "Radar: Rejected %d / %d", radar_reject, radar_valid);
  RCLCPP_INFO_ONCE(this->get_logger(), "Accel: Rejected %d / %d", accel_reject, accel_valid);
  RCLCPP_INFO_ONCE(this->get_logger(), "ICP: Rejected %d / %d", icp_reject, icp_valid);

  if (!use_dr_structure) {
    state_ = ekf_rio_.getState();
  } else {
    state_ = scekf_dero_.getState();
  }

  // clang-format off
  RCLCPP_INFO_ONCE(this->get_logger(), "Final Position [X, Y, Z] is [%.2f, %.2f, %.2f] (m)", state_.position(0, 0),
                                                                                             state_.position(1, 0),
                                                                                             state_.position(2, 0));
  RCLCPP_INFO_ONCE(this->get_logger(), "Final distance error w.r.t the origin is %.2f (m)",
                   std::sqrt(state_.position(0, 0) * state_.position(0, 0) +
                             state_.position(1, 0) * state_.position(1, 0) +
                             state_.position(2, 0) * state_.position(2, 0)));
  RCLCPP_INFO_ONCE(this->get_logger(), "Final Gyro Bias [X, Y, Z] is [%.2f, %.2f, %.2f] (deg/s)",
                   state_.gyro_bias(0, 0) * R2D,
                   state_.gyro_bias(1, 0) * R2D,
                   state_.gyro_bias(2, 0) * R2D);
  RCLCPP_INFO_ONCE(this->get_logger(), "Final Radar Scale [X, Y, Z] is [%.2f, %.2f, %.2f]", state_.radar_scale(0, 0),
                                                                                            state_.radar_scale(1, 0),
                                                                                            state_.radar_scale(2, 0));
  // clang-format on

  rclcpp::shutdown();
} // void ShutdownHandler

void RunRos2Bag::Run() {
  storage_options.uri                           = bag_dir;
  storage_options.storage_id                    = "sqlite3";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  reader.open(storage_options, converter_options);

  rclcpp::Serialization<sensor_msgs::msg::Imu>           serialization_imu;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>   serialization_radar;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization_gt;

  sensor_msgs::msg::Imu           extracted_imu_msg;
  sensor_msgs::msg::PointCloud2   extracted_radar_msg;
  geometry_msgs::msg::PoseStamped extracted_gt_msg;

  while (reader.has_next()) {
    auto                      bag_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);

    if (bag_message->topic_name == imu_topic_name_) {
      serialization_imu.deserialize_message(&extracted_serialized_msg, &extracted_imu_msg);
      RunRos2Bag::ImuCallback(extracted_imu_msg);

    } else if (bag_message->topic_name == radar_topic_name_) {
      serialization_radar.deserialize_message(&extracted_serialized_msg, &extracted_radar_msg);
      RunRos2Bag::RadarCallback(extracted_radar_msg);

    } else if (bag_message->topic_name == groundtruth_topic_name_) {
      serialization_gt.deserialize_message(&extracted_serialized_msg, &extracted_gt_msg);
      gt_msg                     = extracted_gt_msg;
      groundtruth_data_received_ = true;
    }

    if (!queue_imu_buff.empty()) {
      imu_buff.emplace_back(queue_imu_buff.front());

      // clang-format off
      w_b_raw << imu_buff.front().angular_velocity.x,
                 imu_buff.front().angular_velocity.y,
                 imu_buff.front().angular_velocity.z;

      f_b_raw << imu_buff.front().linear_acceleration.x,
                 imu_buff.front().linear_acceleration.y,
                 imu_buff.front().linear_acceleration.z;
      // clang-format on

      if (!ekf_rio_init_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Gathering IMU data for coarse alingment algorithm and initialization...");

        if (groundtruth_data_received_ || imu_buff.size() >= ca_wind_) {

          if (!use_dr_structure) {
            ekf_rio_.InitializeState(imu_buff);
            ca_state_      = ekf_rio_.getCoarseAlignmentState();
            info_grav_     = ekf_rio_.getGravityValue();
            info_imu_dt_   = ekf_rio_.getImuDt();
            info_radar_dt_ = ekf_rio_.getRadarDt();

          } else {
            scekf_dero_.InitializeState(imu_buff);
            ca_state_      = scekf_dero_.getCoarseAlignmentState();
            info_grav_     = scekf_dero_.getGravityValue();
            info_imu_dt_   = scekf_dero_.getImuDt();
            info_radar_dt_ = scekf_dero_.getRadarDt();
          }

          RCLCPP_INFO_ONCE(this->get_logger(), "Coarse alignment: using %llu IMU samples", imu_buff.size());

          RCLCPP_INFO_ONCE(this->get_logger(),
                           "Coarse alignment: initial Euler angle [roll, pitch, yaw] is [%.2f, %.2f, %.2f] (degree)",
                           ca_state_.euler(0, 0) * R2D, ca_state_.euler(1, 0) * R2D, ca_state_.euler(2, 0) * R2D);

          RCLCPP_INFO_ONCE(this->get_logger(), "Initialization: gravitational constant is %.2f (m/s^2)", info_grav_);

          RCLCPP_INFO_ONCE(this->get_logger(), "Initialization: initial position [X, Y, Z] is [%.2f, %.2f, %.2f] (m)",
                           ca_state_.position(0, 0), ca_state_.position(1, 0), ca_state_.position(2, 0));

          RCLCPP_INFO_ONCE(this->get_logger(),
                           "Initialization: initial Euler angle [X, Y, Z] is [%.2f, %.2f, %.2f] (deg)",
                           ca_state_.euler(0, 0) * R2D, ca_state_.euler(1, 0) * R2D, ca_state_.euler(2, 0) * R2D);

          RCLCPP_INFO_ONCE(this->get_logger(), "Initialization: initial velocity [X, Y, Z] is [%.2f, %.2f, %.2f] (m/s)",
                           ca_state_.velocity(0, 0), ca_state_.velocity(1, 0), ca_state_.velocity(2, 0));

          RCLCPP_INFO_ONCE(
              this->get_logger(), "Initialization: initial gyroscope bias [X, Y, Z] is [%.2f, %.2f, %.2f] (degree/s)",
              ca_state_.gyro_bias(0, 0) * R2D, ca_state_.gyro_bias(1, 0) * R2D, ca_state_.gyro_bias(2, 0) * R2D);

          RCLCPP_INFO_ONCE(this->get_logger(),
                           "Initialization: initial accelerometer bias [X, Y, Z] is [%.2f, %.2f, %.2f] (m/s^2)",
                           ca_state_.accel_bias(0, 0), ca_state_.accel_bias(1, 0), ca_state_.accel_bias(2, 0));

          RCLCPP_INFO_ONCE(this->get_logger(),
                           "Initialization: initial radar scale vector [X, Y, Z] is [%.2f, %.2f, %.2f]",
                           ca_state_.radar_scale(0, 0), ca_state_.radar_scale(1, 0), ca_state_.radar_scale(2, 0));

          RCLCPP_INFO_ONCE(this->get_logger(), "Initialization: imu dt is %f (second)", info_imu_dt_);

          if (use_radar)
            RCLCPP_INFO_ONCE(this->get_logger(), "Initialization: radar dt is %f (second)", info_radar_dt_);

          ekf_rio_init_ = true;
          save_init_    = true;
          imu_buff.clear();
        } // if imu_buff.size() > ca_wind_

      } else {
        if (!use_dr_structure)
          dt_imu_ = ekf_rio_.getImuDt();
        else
          dt_imu_ = scekf_dero_.getImuDt();

        if ((dt_imu_ > 0.05) && ekf_rio_init_)
          RCLCPP_WARN_ONCE(this->get_logger(), "IMU dt is large: dt = %f", dt_imu_);

        if (imu_buff.size() >= 2 && ekf_rio_init_)
          RCLCPP_WARN(this->get_logger(), "IMU: Data leaked! Skipped %llu data!", imu_buff.size() - 1);

        if (!use_dr_structure) {
          ekf_rio_.ImuMechanization(imu_buff.front());
          time_update_trigger = true;
        } else
          scekf_dero_.GyroscopeMechanization(imu_buff.front());

        imu_buff.clear();

      } // if-else !ekf_rio_init_

      queue_imu_buff.pop();

    } else
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for imu msg!"); // if !queue_imu_buff.empty()

    if (!queue_radar_buff.empty() && !imu_only && use_radar) {
      radar_buff.emplace_back(queue_radar_buff.front());

      if (!ekf_rio_init_) {
        radar_skip += 1;
        RCLCPP_INFO_ONCE(this->get_logger(), "Radar: Data received but rejected due to initialization process!");
        radar_buff.clear();

      } else {
        radar_valid += 1;
        const bool zupt_trigger =
            radar_estimator_.Process(radar_buff.front(), radar_velocity_estimator_param_,
                                     radar_position_estimator_param_, icp_init_pose, use_dr_structure);

        if (!use_dr_structure) {
          RCLCPP_INFO_ONCE(this->get_logger(), "Radar: Used as an aided sensor!");
          ekf_rio_.setR_Radar(radar_estimator_.getEgoVelocityCovariance());

          Vec3d r_radar = Vec3d::Zero();
          MatXd H_radar = MatXd::Zero(3, 15);

          state_       = ekf_rio_.getState();
          wb_corrected = w_b_raw - state_.gyro_bias;

          Mat3d Rbn       = quat2dcm(state_.quaternion);
          Mat3d Rrb       = imu_radar_calibration_.rotation_matrix;
          Vec3d p_r_b     = imu_radar_calibration_.position;
          Vec3d v_w       = skewMatrix(wb_corrected) * p_r_b;
          Vec3d v_b_n     = state_.velocity;
          Vec3d v_b_b     = Rbn.transpose() * v_b_n;
          Mat3d H_v       = Rrb.transpose() * Rbn.transpose();
          Mat3d H_q       = -Rrb.transpose() * Rbn.transpose() * skewMatrix(v_b_n);
          Mat3d H_bg      = -Rrb.transpose() * skewMatrix(p_r_b);
          Vec3d v_r_r_est = Rrb.transpose() * (v_w + v_b_b);

          H_radar.block<3, 3>(0, 3)  = H_v;
          H_radar.block<3, 3>(0, 6)  = H_q;
          H_radar.block<3, 3>(0, 12) = H_bg;

          Vec3d v_r_r = radar_estimator_.getEgoVelocity();
          r_radar     = v_r_r_est - v_r_r;

          if (ekf_rio_.MeasurementUpdateRadar(r_radar, H_radar, v_r_r, radar_velocity_estimator_param_,
                                              radar_outlier_reject))
            measurement_update_radar_trigger = true;
          else {
            // RCLCPP_WARN(this->get_logger(),
            // "EKF: Measurement update step is rejected due to radar failed Chi-squared test!");
            radar_reject += 1;
          }
        } else {
          RCLCPP_INFO_ONCE(this->get_logger(), "Radar: Used as a core sensor!");

          const Vec3d v_r_r = radar_estimator_.getEgoVelocity();

          scekf_dero_.RadarMechanization(v_r_r, imu_radar_calibration_);
          time_update_trigger = true;

          curr_radar_velocity = scekf_dero_.getVbn();

          if (!accel_init) {
            prev_radar_velocity = curr_radar_velocity;
            accel_init          = true;
          }

          estimated_accel     = (curr_radar_velocity - prev_radar_velocity) / scekf_dero_.getRadarDt();
          prev_radar_velocity = curr_radar_velocity;

          scekf_dero_.RadarTimeUpdate(radar_estimator_.getEgoVelocity(), radar_estimator_.getEgoVelocityCovariance(),
                                      imu_radar_calibration_, use_cloning, cloning_window_size);

          if (use_cloning) {
            if (window_count == 1 && !icp_init) {
              first_radar_scan_inlier = radar_estimator_.getRadarScanInlier();
              icp_init                = true;

            } else if (window_count == cloning_window_size && icp_init) {
              icp_valid             += 1;
              end_radar_scan_inlier  = radar_estimator_.getRadarScanInlier();

              ICPTransform icp_meas = radar_estimator_.solveICP(first_radar_scan_inlier, end_radar_scan_inlier,
                                                                radar_position_estimator_param_, icp_init_pose);

              if (scekf_dero_.RadarMeasurementUpdate(imu_radar_calibration_, icp_meas, radar_outlier_reject,
                                                     zupt_trigger))
                measurement_update_radar_trigger = true;
              else {
                // RCLCPP_WARN(this->get_logger(),
                // "EKF: Measurement update step is rejected due to ICP failed Chi-squared test!");
                icp_reject += 1;
              }
              window_count = 1;
            }
            window_count += 1;
          }

          bool accel_update = true;
          if (use_dr_structure) {
            if (accel_update && accel_init) {
              accel_valid += 1;
              ca_state_    = scekf_dero_.getCoarseAlignmentState();
              state_       = scekf_dero_.getState();

              const double accel_norm_residual =
                  std::sqrt(f_b_raw(0, 0) * f_b_raw(0, 0) + f_b_raw(1, 0) * f_b_raw(1, 0) +
                            f_b_raw(2, 0) * f_b_raw(2, 0)) -
                  scekf_dero_.getGravityValue();

              if (accel_norm_residual < 0.059)
                scekf_dero_.setR_Accel(accel_roll_pitch_noise_);
              else
                scekf_dero_.setR_Accel(3.0 * accel_roll_pitch_noise_);

              Vec2d r_accel = Vec2d::Zero();
              MatXd H_accel = MatXd::Zero(2, 12);

              Vec3d f_b_hat = f_b_raw - ca_state_.accel_bias;

              Mat3d Cbn = quat2dcm(state_.quaternion);
              f_b_hat   = f_b_hat - Cbn.transpose() * estimated_accel;

              const double f_x    = f_b_hat(0, 0);
              const double f_y    = f_b_hat(1, 0);
              const double f_z    = f_b_hat(2, 0);
              const double roll_  = std::atan2(-f_y, -f_z);
              const double foo    = std::sqrt(f_y * f_y + f_z * f_z);
              const double pitch_ = std::atan2(f_x, foo);

              const Vec3d  euler = quat2euler(state_.quaternion);
              const double phi   = euler(0, 0);
              const double theta = euler(1, 0);
              const double psi   = euler(2, 0);

              r_accel(0, 0) = phi - roll_;
              r_accel(1, 0) = theta - pitch_;

              H_accel.block<2, 2>(0, 3) << -std::cos(psi) / std::cos(theta), -std::sin(psi) / std::cos(theta),
                  std::sin(psi), -std::cos(psi);

              if (scekf_dero_.MeasurementUpdateAccel(r_accel, H_accel, true))
                measurement_update_accel_trigger = true;
              else {
                // RCLCPP_WARN(this->get_logger(),
                // "EKF: Measurement update step is rejected due to accelerometer failed Chi-squared test!");
                accel_reject += 1;
              }
            }
          }
        }

        if (radar_buff.size() >= 2 && ekf_rio_init_)
          RCLCPP_WARN(this->get_logger(), "Radar: Data leaked! Skipped %llu data!", radar_buff.size() - 1);

        radar_buff.clear();
      } // ekf_rio_init_

      queue_radar_buff.pop();
    } else
      RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for radar msg!"); // if !queue_radar_buff.empty()

    if ((time_update_trigger == true) || (save_init_ == true) || (measurement_update_radar_trigger == true) ||
        (measurement_update_accel_trigger == true)) {

      if (!use_dr_structure) {
        covariance_matrix_ = ekf_rio_.getCovarianceMatrix();
        error_state_       = ekf_rio_.getErrorState();
        state_             = ekf_rio_.getState();
      } else {
        covariance_matrix_cloning_ = scekf_dero_.getCovarianceMatrix();
        error_state_               = scekf_dero_.getErrorState();
        state_                     = scekf_dero_.getState();
      }

      Vec3d v_r_est = radar_estimator_.getEgoVelocity();

      est_save.precision(19);
      if (time_update_trigger == true)
        if (!use_dr_structure)
          est_save << ekf_rio_.getImuCurrentTime() << " ";
        else
          est_save << scekf_dero_.getImuCurrentTime() << " ";

      else if (measurement_update_radar_trigger == true) {
        if (!use_dr_structure)
          est_save << ekf_rio_.getRadarCurrentTime() << " ";
        else
          est_save << scekf_dero_.getRadarCurrentTime() << " ";
      } else if (save_init_ == true)
        if (!use_dr_structure)
          est_save << ekf_rio_.getImuCurrentTime() << " ";
        else
          est_save << scekf_dero_.getImuCurrentTime() << " ";
      else if (measurement_update_accel_trigger == true)
        est_save << scekf_dero_.getRadarCurrentTime() << " ";

      if (time_update_trigger == true || save_init_ == true) {
        error_state_.position     *= 0;
        error_state_.velocity     *= 0;
        error_state_.misalignment *= 0;
        error_state_.accel_bias   *= 0;
        error_state_.gyro_bias    *= 0;
        error_state_.radar_scale  *= 0;
      }

      for (int i = 0; i < 3; ++i)
        est_save << state_.position(i, 0) << " ";

      const bool rpg_save = true;

      if (!use_dr_structure && !rpg_save)
        for (int i = 0; i < 3; ++i)
          est_save << state_.velocity(i, 0) << " ";

      if (!rpg_save) {
        for (int i = 0; i < 4; ++i)
          est_save << state_.quaternion(i, 0) << " ";
      } else {
        // clang-format off
        est_save << state_.quaternion(1, 0) << " "
                 << state_.quaternion(2, 0) << " "
                 << state_.quaternion(3, 0) << " "
                 << state_.quaternion(0, 0) << std::endl;
        // clang-format on
      }

      if (!rpg_save) {
        if (!use_dr_structure)
          for (int i = 0; i < 3; ++i)
            est_save << state_.accel_bias(i, 0) << " ";

        for (int i = 0; i < 3; ++i)
          est_save << state_.gyro_bias(i, 0) << " ";

        if (use_dr_structure)
          for (int i = 0; i < 3; ++i)
            est_save << state_.radar_scale(i, 0) << " ";

        for (int i = 0; i < 3; ++i)
          est_save << v_r_est(i, 0) << " ";

        // error
        for (int i = 0; i < 3; ++i)
          est_save << error_state_.position(i, 0) << " ";

        if (!use_dr_structure)
          for (int i = 0; i < 3; ++i)
            est_save << error_state_.velocity(i, 0) << " ";

        for (int i = 0; i < 3; ++i)
          est_save << error_state_.misalignment(i, 0) << " ";

        if (!use_dr_structure)
          for (int i = 0; i < 3; ++i)
            est_save << error_state_.accel_bias(i, 0) << " ";

        for (int i = 0; i < 3; ++i)
          est_save << error_state_.gyro_bias(i, 0) << " ";

        if (use_dr_structure)
          for (int i = 0; i < 3; ++i)
            est_save << error_state_.radar_scale(i, 0) << " ";

        // covariance
        if (!use_dr_structure)
          for (int i = 0; i < 15; ++i)
            est_save << covariance_matrix_.posteriori(i, i) << " ";
        else
          for (int i = 0; i < 12; ++i)
            est_save << covariance_matrix_cloning_.posteriori(i, i) << " ";
        est_save << std::endl;
      }

      time_update_trigger              = false;
      save_init_                       = false;
      measurement_update_radar_trigger = false;
      measurement_update_accel_trigger = false;
    } // if save
  }
  RunRos2Bag::ShutdownHandler();
} // void Run
} // namespace incsl
