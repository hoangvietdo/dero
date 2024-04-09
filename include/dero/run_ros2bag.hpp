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

#ifndef RUN_ROS2BAG_HPP
#define RUN_ROS2BAG_HPP

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <std_msgs/msg/string.hpp>

#include <nav_msgs/msg/path.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <dero/ekf_rio.hpp>
#include <dero/nav_convert.hpp>
#include <dero/radar_estimator.hpp>
#include <dero/scekf_dero.hpp>
#include <dero/variable_define.hpp>

namespace incsl {

class RunRos2Bag : public rclcpp::Node {
  public:
    RunRos2Bag(std::string node_name);
    bool use_dr_structure;

  private:
    std::string imu_topic_name_;
    std::string radar_topic_name_;
    std::string groundtruth_topic_name_;
    std::string est_save_dir_;
    std::string bag_dir;

    std::ofstream est_save;

    std::mutex mutex_;

    sensor_msgs::msg::PointCloud2   radar_data_pub_;
    nav_msgs::msg::Path             pose_path_;
    nav_msgs::msg::Path             pose_path_gt_;
    geometry_msgs::msg::PoseStamped gt_msg;

    std::queue<sensor_msgs::msg::Imu>         queue_imu_buff;
    std::queue<sensor_msgs::msg::PointCloud2> queue_radar_buff;
    std::queue<sensor_msgs::msg::PointCloud2> queue_radar_pub_buff;

    std::vector<sensor_msgs::msg::Imu>         imu_buff;
    std::vector<sensor_msgs::msg::PointCloud2> radar_buff;
    std::vector<std::string>                   topics;

    int ros2_pub_rate_;
    int cloning_window_size;
    int timer_period_;
    int min_number_sat_;
    int num_points;
    int max_iter_;
    int accel_reject = 0;
    int accel_valid  = 0;
    int icp_reject   = 0;
    int icp_valid    = 0;
    int viet         = 0;
    int radar_valid  = 0;
    int radar_reject = 0;
    int radar_skip   = 0;
    int window_count = 0;

    uint N_ransac_points_;

    size_t count_;

    long unsigned int ca_wind_;
    long unsigned int odr_inlier_threshold_;

    double imu_body_rotation_offset_x_;
    double imu_body_rotation_offset_y_;
    double imu_body_rotation_offset_z_;
    double vel_rw_;
    double ang_rw_;
    double accel_bias_rw_;
    double gyro_bias_rw_;
    double radar_scale_rw_;
    double P0_pos;
    double P0_vel;
    double P0_att_XY;
    double P0_att_Z;
    double P0_ba;
    double P0_bg;
    double P0_bb;
    double P0_br;
    double imu_radar_position_offset_x_;
    double imu_radar_position_offset_y_;
    double imu_radar_position_offset_z_;
    double imu_radar_quaternion_offset_w_;
    double imu_radar_quaternion_offset_x_;
    double imu_radar_quaternion_offset_y_;
    double imu_radar_quaternion_offset_z_;
    double info_grav_;
    double info_imu_dt_;
    double info_radar_dt_;
    double dt_imu_;
    double max_pos_acc_threshold_;
    double max_vel_acc_threshold_;
    double sigma_pos_scaling_;
    double sigma_vel_scaling_;
    double min_distance_;
    double max_distance_;
    double min_db_;
    double elevation_threshold_;
    double azimuth_threshold_;
    double velocity_correction_factor_;
    double filter_min_z_;
    double filter_max_z_;
    double zero_velocity_threshold_;
    double sigma_zero_velocity_x_;
    double sigma_zero_velocity_y_;
    double sigma_zero_velocity_z_;
    double allowed_outlier_percentage_;
    double max_sigma_x_;
    double max_sigma_y_;
    double max_sigma_z_;
    double max_r_cond_;
    double outlier_prob_;
    double success_prob_;
    double inlier_threshold_;
    double sigma_offset_radar_x_;
    double sigma_offset_radar_y_;
    double sigma_offset_radar_z_;
    double outlier_percentil_radar_;
    double min_speed_odr_;
    double sigma_v_d_;
    double model_noise_offset_deg_;
    double model_noise_scale_deg_;
    double grav_;
    double max_corres_dis_;
    double transform_eps_;
    double euclidean_fit_eps_;
    double foo_score;
    double ransac_outlier_reject_thres;
    double height_mean;
    double scale_factor_mean;
    double scale_factor_std;

    bool ekf_rio_init_                    = false;
    bool imu_data_received_               = false;
    bool radar_data_received_             = false;
    bool groundtruth_data_received_       = false;
    bool time_update_trigger              = false;
    bool measurement_update_radar_trigger = false;
    bool measurement_update_accel_trigger = false;
    bool save_init_                       = false;
    bool radar_outlier_reject             = false;
    bool accel_outlier_reject             = true;
    bool window_slicing                   = false;
    bool accel_init                       = false;
    bool use_cloning;
    bool imu_only;
    bool use_radar;
    bool groundtruth_included;
    bool use_ransac_;
    bool use_odr_;
    bool icp_init;

    Vec2d accel_roll_pitch_noise_;
    Vec3d wb_corrected;
    Vec3d w_b_raw;
    Vec3d f_b_raw;
    Vec3d f_b_mean = Vec3d::Zero();
    Vec3d w_b_mean = Vec3d::Zero();
    Vec3d local_origin_llh_;
    Vec3d local_origin_ecef_;
    Vec3d true_scale;
    Vec3d prev_radar_velocity;
    Vec3d curr_radar_velocity;
    Vec3d estimated_accel;

    Mat3d R_imu_body;
    Mat4d icp_init_pose;

    Init                        init_;
    Noise                       noise_;
    State                       state_;
    State                       first_window;
    EkfRio                      ekf_rio_;
    ErrorState                  error_state_;
    ScEkfDero                   scekf_dero_;
    ICPTransform                trans;
    ICPTransform                first_icp;
    ICPTransform                end_icp;
    ICPTransform                prev_icp;
    CovarianceMatrix            covariance_matrix_;
    CovarianceMatrixDr          covariance_matrix_dr_;
    CovarianceMatrixCloning     covariance_matrix_cloning_;
    CoarseAlignmentState        ca_state_;
    RadarEstimator              radar_estimator_;
    RadarVelocityEstimatorParam radar_velocity_estimator_param_;
    RadarPositionEstimatorParam radar_position_estimator_param_;
    IMURadarCalibrationParam    imu_radar_calibration_;

    pcl::PointCloud<RadarPointCloudType> first_window_radar_scan_inlier;
    pcl::PointCloud<RadarPointCloudType> end_radar_scan_inlier;

    rclcpp::TimerBase::SharedPtr                                callback_timer_process_;
    rclcpp::TimerBase::SharedPtr                                viet_timer_;
    rclcpp::CallbackGroup::SharedPtr                            pub_callback_group_;
    rclcpp::CallbackGroup::SharedPtr                            process_callback_group_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_raw_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           pose_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           pose_path_gt_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         viet_publisher_;

    rosbag2_cpp::StorageOptions   storage_options;
    rosbag2_cpp::ConverterOptions converter_options;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_radar_;

    void LoadParameters();
    void Run();
    void MsgPublish();
    void ImuCallback(const sensor_msgs::msg::Imu imu_msg);
    void RadarCallback(const sensor_msgs::msg::PointCloud2 radar_msg);
    void ShutdownHandler();
}; // class RunRos2Bag
} // namespace incsl

#endif // ifndef
