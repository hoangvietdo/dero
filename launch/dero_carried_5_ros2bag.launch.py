# This file is part of DeRO: Dead Reckoning Based on Radar Odometry
# With Accelerometers Aided for Robot Localization.
# Copyright (C) 2024 Hoang Viet Do  <hoangvietdo@sju.ac.kr>

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https:##www.gnu.org#licenses#>.

import os
import launch
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ROS parameter
    imu_topic_arg                     = DeclareLaunchArgument('imu_topic',                     default_value = '/sensor_platform/imu',                              description = '')
    radar_topic_arg                   = DeclareLaunchArgument('radar_topic',                   default_value = '/sensor_platform/radar/scan',                       description = '')
    groundtruth_topic_arg             = DeclareLaunchArgument('groundtruth_topic',             default_value = '/ground_truth/pose',                                description = '')
    est_save_dir_arg                  = DeclareLaunchArgument('est_save_dir',                  default_value = '/home/vietdo/Desktop/viet_est/carried_5/scekf_dero_carried_5_est.txt',  description = '')

    imu_body_rotation_offset_x_arg    = DeclareLaunchArgument('imu_body_rotation_offset_x',    default_value = '0.0',        description = 'degree')
    imu_body_rotation_offset_y_arg    = DeclareLaunchArgument('imu_body_rotation_offset_y',    default_value = '0.0',        description = 'degree')
    imu_body_rotation_offset_z_arg    = DeclareLaunchArgument('imu_body_rotation_offset_z',    default_value = '0.0',        description = 'degree')

    imu_radar_position_offset_x_arg   = DeclareLaunchArgument('imu_radar_position_offset_x',   default_value = '0.0526368',     description = '')
    imu_radar_position_offset_y_arg   = DeclareLaunchArgument('imu_radar_position_offset_y',   default_value = '0.0824799',     description = '')
    imu_radar_position_offset_z_arg   = DeclareLaunchArgument('imu_radar_position_offset_z',   default_value = '0.0725244',     description = '')

    imu_radar_quaternion_offset_w_arg = DeclareLaunchArgument('imu_radar_quaternion_offset_w', default_value = '0.931373',  description = '')
    imu_radar_quaternion_offset_x_arg = DeclareLaunchArgument('imu_radar_quaternion_offset_x', default_value = '-0.00565244', description = '')
    imu_radar_quaternion_offset_y_arg = DeclareLaunchArgument('imu_radar_quaternion_offset_y', default_value = '0.00910475',  description = '')
    imu_radar_quaternion_offset_z_arg = DeclareLaunchArgument('imu_radar_quaternion_offset_z', default_value = '-0.363908', description = '')
    gravity_arg                       = DeclareLaunchArgument('gravity',                       default_value = '9.81',     description = '')

    # EKF parameter
    P_init_position_arg               = DeclareLaunchArgument('P_init_position',               default_value = '1.0e-8',   description = 'standard deviation')
    P_init_velocity_arg               = DeclareLaunchArgument('P_init_velocity',               default_value = '1.0e-8',   description = 'standard deviation')
    P_init_attitude_XY_arg            = DeclareLaunchArgument('P_init_attitude_XY',            default_value = '1.0e-8',   description = 'standard deviation')
    P_init_attitude_Z_arg             = DeclareLaunchArgument('P_init_attitude_Z',             default_value = '1.0e-8',   description = 'standard deviation')
    P_init_accel_bias_arg             = DeclareLaunchArgument('P_init_accel_bias',             default_value = '0.02',     description = 'standard deviation')
    P_init_gyro_bias_arg              = DeclareLaunchArgument('P_init_gyro_bias',              default_value = '5.2360e-8',description = 'standard deviation')
    P_init_radar_scale_arg            = DeclareLaunchArgument('P_init_radar_scale',            default_value = '1.0e-2',   description = 'standard deviation')

    velocity_random_walk_arg          = DeclareLaunchArgument('velocity_random_walk',          default_value = '0.03',     description = 'standard deviation')
    angular_random_walk_arg           = DeclareLaunchArgument('angular_random_walk',           default_value = '0.0035',   description = 'standard deviation')
    accel_bias_random_walk_arg        = DeclareLaunchArgument('accel_bias_random_walk',        default_value = '0.00001',  description = 'standard deviation')
    gyro_bias_random_walk_arg         = DeclareLaunchArgument('gyro_bias_random_walk',         default_value = '1.7453e-7',description = 'standard deviation')
    radar_scale_random_walk_arg       = DeclareLaunchArgument('radar_scale_random_walk',       default_value = '0.0000001',description = 'standard deviation')

    # Simulation scenario
    use_radar_arg                     = DeclareLaunchArgument('use_radar',                     default_value = 'true',    description = '')
    use_dr_structure_arg              = DeclareLaunchArgument('use_dr_structure',              default_value = 'true',    description = '')
    use_cloining_arg                  = DeclareLaunchArgument('use_cloning',                   default_value = 'true',    description = '')
    imu_only_arg                      = DeclareLaunchArgument('imu_only',                      default_value = 'false',   description = '')
    groundtruth_included_arg          = DeclareLaunchArgument('groundtruth_included',          default_value = 'false',   description = '')
    radar_outlier_reject_arg          = DeclareLaunchArgument('radar_outlier_reject',          default_value = 'false',    description = '')
    ros2_pub_rate_arg                 = DeclareLaunchArgument('ros2_pub_rate',                 default_value = '30',      description = 'Hz')
    coarse_alignment_window_size_arg  = DeclareLaunchArgument('coarse_alignment_window_size',  default_value = '17500',    description = '')

    # Radar parameter
    min_distance_arg                  = DeclareLaunchArgument('min_distance',                  default_value = '0.25',     description = '')
    max_distance_arg                  = DeclareLaunchArgument('max_distance',                  default_value = '100.0',    description = '')
    min_db_arg                        = DeclareLaunchArgument('min_db',                        default_value = '3.0',      description = '')
    elevation_threshold_arg           = DeclareLaunchArgument('elevation_threshold',           default_value = '60.0',     description = '')
    azimuth_threshold_arg             = DeclareLaunchArgument('azimuth_threshold',             default_value = '60.0',     description = '')
    velocity_correction_factor_arg    = DeclareLaunchArgument('velocity_correction_factor',    default_value = '1.0',      description = '')
    filter_min_z_arg                  = DeclareLaunchArgument('filter_min_z',                  default_value = '-100.0',   description = '')
    filter_max_z_arg                  = DeclareLaunchArgument('filter_max_z',                  default_value = '100.0',    description = '')
    zero_velocity_threshold_arg       = DeclareLaunchArgument('zero_velocity_threshold',       default_value = '0.05',     description = '')
    sigma_zero_velocity_x_arg         = DeclareLaunchArgument('sigma_zero_velocity_x',         default_value = '0.025',    description = '')
    sigma_zero_velocity_y_arg         = DeclareLaunchArgument('sigma_zero_velocity_y',         default_value = '0.025',    description = '')
    sigma_zero_velocity_z_arg         = DeclareLaunchArgument('sigma_zero_velocity_z',         default_value = '0.025',    description = '')
    allowed_outlier_percentage_arg    = DeclareLaunchArgument('allowed_outlier_percentage',    default_value = '0.25',     description = '')

    max_sigma_x_arg                   = DeclareLaunchArgument('max_sigma_x',                   default_value = '0.2',      description = '')
    max_sigma_y_arg                   = DeclareLaunchArgument('max_sigma_y',                   default_value = '0.15',     description = '')
    max_sigma_z_arg                   = DeclareLaunchArgument('max_sigma_z',                   default_value = '0.2',      description = '')
    max_r_cond_arg                    = DeclareLaunchArgument('max_r_cond',                    default_value = '1.0e3',    description = '')

    use_ransac_arg                    = DeclareLaunchArgument('use_ransac',                    default_value = 'true',     description = '')
    outlier_prob_arg                  = DeclareLaunchArgument('outlier_prob',                  default_value = '0.5',      description = '')
    success_prob_arg                  = DeclareLaunchArgument('success_prob',                  default_value = '0.99999',  description = '')
    N_ransac_points_arg               = DeclareLaunchArgument('N_ransac_points',               default_value = '3',        description = '')
    inlier_threshold_arg              = DeclareLaunchArgument('inlier_threshold',              default_value = '0.15',     description = '')

    sigma_offset_radar_x_arg          = DeclareLaunchArgument('sigma_offset_radar_x',          default_value = '0.075',    description = '')
    sigma_offset_radar_y_arg          = DeclareLaunchArgument('sigma_offset_radar_y',          default_value = '0.05',     description = '')
    sigma_offset_radar_z_arg          = DeclareLaunchArgument('sigma_offset_radar_z',          default_value = '0.075',    description = '')
    outlier_percentil_radar_arg       = DeclareLaunchArgument('outlier_percentil_radar',       default_value = '0.1',      description = '')

    use_odr_arg                       = DeclareLaunchArgument('use_odr',                       default_value = 'true',     description = '')
    min_speed_odr_arg                 = DeclareLaunchArgument('min_speed_odr',                 default_value = '4.0',      description = '')
    sigma_v_d_arg                     = DeclareLaunchArgument('sigma_v_d',                     default_value = '0.125',    description = '')
    model_noise_offset_deg_arg        = DeclareLaunchArgument('model_noise_offset_deg',        default_value = '2.0',      description = 'degree')
    model_noise_scale_deg_arg         = DeclareLaunchArgument('model_noise_scale_deg',         default_value = '10.0',     description = 'degree')
    odr_inlier_threshold_arg          = DeclareLaunchArgument('odr_inlier_threshold',          default_value = '10',       description = '')

    max_corres_dis_arg                = DeclareLaunchArgument('max_corres_dis',                default_value = '4.0',          description = '')
    max_iter_arg                      = DeclareLaunchArgument('max_iter',                      default_value = '500',          description = '')
    transform_eps_arg                 = DeclareLaunchArgument('transform_eps',                 default_value = '0.000000001',    description = '')
    euclidean_fit_eps_arg             = DeclareLaunchArgument('euclidean_fit_eps',             default_value = '0.000000001',    description = '')
    cloning_window_size_arg           = DeclareLaunchArgument('cloning_window_size',           default_value = '3',            description = '')
    window_slicing_arg                = DeclareLaunchArgument('window_slicing',                default_value = 'false',        description = '')
    ransac_outlier_reject_thres_arg   = DeclareLaunchArgument('ransac_outlier_reject_thres',   default_value = '0.001',          description = '')  

    bag_dir_arg                       = DeclareLaunchArgument('bag_dir',
    default_value = '/home/vietdo/Downloads/radar_inertial_datasets_icins_2021/carried_datasets/carried_5/carried_5.db3', description = '');

    scekf_dero_ros2bag_node = Node(
        package='dero',
        executable='scekf_dero_ros2bag_node',
        output='screen',
        parameters=[{
            'imu_topic':                     launch.substitutions.LaunchConfiguration('imu_topic'),
            'radar_topic':                   launch.substitutions.LaunchConfiguration('radar_topic'),
            'groundtruth_topic':             launch.substitutions.LaunchConfiguration('groundtruth_topic'),
            'est_save_dir':                  launch.substitutions.LaunchConfiguration('est_save_dir'),
            'ros2_pub_rate':                 launch.substitutions.LaunchConfiguration('ros2_pub_rate'),
            'cloning_window_size':           launch.substitutions.LaunchConfiguration('cloning_window_size'),
            'use_cloning':                   launch.substitutions.LaunchConfiguration('use_cloning'),
            'gravity':                       launch.substitutions.LaunchConfiguration('gravity'),
            'coarse_alignment_window_size':  launch.substitutions.LaunchConfiguration('coarse_alignment_window_size'),
            'imu_body_rotation_offset_x':    launch.substitutions.LaunchConfiguration('imu_body_rotation_offset_x'),
            'imu_body_rotation_offset_y':    launch.substitutions.LaunchConfiguration('imu_body_rotation_offset_y'),
            'imu_body_rotation_offset_z':    launch.substitutions.LaunchConfiguration('imu_body_rotation_offset_z'),
            'imu_radar_position_offset_x':   launch.substitutions.LaunchConfiguration('imu_radar_position_offset_x'),
            'imu_radar_position_offset_y':   launch.substitutions.LaunchConfiguration('imu_radar_position_offset_y'),
            'imu_radar_position_offset_z':   launch.substitutions.LaunchConfiguration('imu_radar_position_offset_z'),
            'imu_radar_quaternion_offset_w': launch.substitutions.LaunchConfiguration('imu_radar_quaternion_offset_w'),
            'imu_radar_quaternion_offset_x': launch.substitutions.LaunchConfiguration('imu_radar_quaternion_offset_x'),
            'imu_radar_quaternion_offset_y': launch.substitutions.LaunchConfiguration('imu_radar_quaternion_offset_y'),
            'imu_radar_quaternion_offset_z': launch.substitutions.LaunchConfiguration('imu_radar_quaternion_offset_z'),
            'velocity_random_walk':          launch.substitutions.LaunchConfiguration('velocity_random_walk'),
            'angular_random_walk':           launch.substitutions.LaunchConfiguration('angular_random_walk'),
            'accel_bias_random_walk':        launch.substitutions.LaunchConfiguration('accel_bias_random_walk'),
            'gyro_bias_random_walk':         launch.substitutions.LaunchConfiguration('gyro_bias_random_walk'),
            'radar_scale_random_walk':       launch.substitutions.LaunchConfiguration('radar_scale_random_walk'),
            'P_init_position':               launch.substitutions.LaunchConfiguration('P_init_position'),
            'P_init_velocity':               launch.substitutions.LaunchConfiguration('P_init_velocity'),
            'P_init_attitude_XY':            launch.substitutions.LaunchConfiguration('P_init_attitude_XY'),
            'P_init_attitude_Z':             launch.substitutions.LaunchConfiguration('P_init_attitude_Z'),
            'P_init_accel_bias':             launch.substitutions.LaunchConfiguration('P_init_accel_bias'),
            'P_init_gyro_bias':              launch.substitutions.LaunchConfiguration('P_init_gyro_bias'),
            'P_init_radar_scale':            launch.substitutions.LaunchConfiguration('P_init_radar_scale'),
            'use_radar':                     launch.substitutions.LaunchConfiguration('use_radar'),
            'use_dr_structure':              launch.substitutions.LaunchConfiguration('use_dr_structure'),
            'imu_only':                      launch.substitutions.LaunchConfiguration('imu_only'),
            'groundtruth_included':          launch.substitutions.LaunchConfiguration('groundtruth_included'),
            'min_distance':                  launch.substitutions.LaunchConfiguration('min_distance'),
            'max_distance':                  launch.substitutions.LaunchConfiguration('max_distance'),
            'min_db':                        launch.substitutions.LaunchConfiguration('min_db'),
            'elevation_threshold':           launch.substitutions.LaunchConfiguration('elevation_threshold'),
            'azimuth_threshold':             launch.substitutions.LaunchConfiguration('azimuth_threshold'),
            'velocity_correction_factor':    launch.substitutions.LaunchConfiguration('velocity_correction_factor'),
            'filter_min_z':                  launch.substitutions.LaunchConfiguration('filter_min_z'),
            'filter_max_z':                  launch.substitutions.LaunchConfiguration('filter_max_z'),
            'zero_velocity_threshold':       launch.substitutions.LaunchConfiguration('zero_velocity_threshold'),
            'sigma_zero_velocity_x':         launch.substitutions.LaunchConfiguration('sigma_zero_velocity_x'),
            'sigma_zero_velocity_y':         launch.substitutions.LaunchConfiguration('sigma_zero_velocity_y'),
            'sigma_zero_velocity_z':         launch.substitutions.LaunchConfiguration('sigma_zero_velocity_z'),
            'allowed_outlier_percentage':    launch.substitutions.LaunchConfiguration('allowed_outlier_percentage'),
            'max_sigma_x':                   launch.substitutions.LaunchConfiguration('max_sigma_x'),
            'max_sigma_y':                   launch.substitutions.LaunchConfiguration('max_sigma_y'),
            'max_sigma_z':                   launch.substitutions.LaunchConfiguration('max_sigma_z'),
            'max_r_cond':                    launch.substitutions.LaunchConfiguration('max_r_cond'),
            'use_ransac':                    launch.substitutions.LaunchConfiguration('use_ransac'),
            'outlier_prob':                  launch.substitutions.LaunchConfiguration('outlier_prob'),
            'success_prob':                  launch.substitutions.LaunchConfiguration('success_prob'),
            'N_ransac_points':               launch.substitutions.LaunchConfiguration('N_ransac_points'),
            'inlier_threshold':              launch.substitutions.LaunchConfiguration('inlier_threshold'),
            'sigma_offset_radar_x':          launch.substitutions.LaunchConfiguration('sigma_offset_radar_x'),
            'sigma_offset_radar_y':          launch.substitutions.LaunchConfiguration('sigma_offset_radar_y'),
            'sigma_offset_radar_z':          launch.substitutions.LaunchConfiguration('sigma_offset_radar_z'),
            'outlier_percentil_radar':       launch.substitutions.LaunchConfiguration('outlier_percentil_radar'),
            'use_odr':                       launch.substitutions.LaunchConfiguration('use_odr'),
            'min_speed_odr':                 launch.substitutions.LaunchConfiguration('min_speed_odr'),
            'sigma_v_d':                     launch.substitutions.LaunchConfiguration('sigma_v_d'),
            'model_noise_offset_deg':        launch.substitutions.LaunchConfiguration('model_noise_offset_deg'),
            'model_noise_scale_deg':         launch.substitutions.LaunchConfiguration('model_noise_scale_deg'),
            'odr_inlier_threshold':          launch.substitutions.LaunchConfiguration('odr_inlier_threshold'),
            'radar_outlier_reject':          launch.substitutions.LaunchConfiguration('radar_outlier_reject'),
            'ransac_outlier_reject_thres':   launch.substitutions.LaunchConfiguration('ransac_outlier_reject_thres'),
            'max_corres_dis':                launch.substitutions.LaunchConfiguration('max_corres_dis'),
            'max_iter':                      launch.substitutions.LaunchConfiguration('max_iter'),
            'transform_eps':                 launch.substitutions.LaunchConfiguration('transform_eps'),
            'euclidean_fit_eps':             launch.substitutions.LaunchConfiguration('euclidean_fit_eps'),
            'window_slicing':                launch.substitutions.LaunchConfiguration('window_slicing'),
            'bag_dir':                       launch.substitutions.LaunchConfiguration('bag_dir'),
        }])

    return LaunchDescription([
        bag_dir_arg,
        imu_topic_arg,
        radar_topic_arg,
        groundtruth_topic_arg,
        est_save_dir_arg,
        ros2_pub_rate_arg,
        cloning_window_size_arg,
        coarse_alignment_window_size_arg,
        gravity_arg,
        use_cloining_arg,
        imu_body_rotation_offset_x_arg,
        imu_body_rotation_offset_y_arg,
        imu_body_rotation_offset_z_arg,
        imu_radar_position_offset_x_arg,  
        imu_radar_position_offset_y_arg,  
        imu_radar_position_offset_z_arg,  
        imu_radar_quaternion_offset_w_arg,
        imu_radar_quaternion_offset_x_arg,
        imu_radar_quaternion_offset_y_arg,
        imu_radar_quaternion_offset_z_arg,
        velocity_random_walk_arg,
        angular_random_walk_arg,
        gyro_bias_random_walk_arg,
        radar_scale_random_walk_arg,
        accel_bias_random_walk_arg,
        P_init_position_arg,
        P_init_attitude_XY_arg,
        P_init_attitude_Z_arg,
        P_init_velocity_arg,
        P_init_gyro_bias_arg,
        P_init_accel_bias_arg,
        P_init_radar_scale_arg,
        use_radar_arg,
        use_dr_structure_arg,
        imu_only_arg,
        groundtruth_included_arg,
        min_distance_arg,
        max_distance_arg,
        min_db_arg,
        elevation_threshold_arg,
        azimuth_threshold_arg,
        velocity_correction_factor_arg,
        filter_min_z_arg,
        filter_max_z_arg,
        zero_velocity_threshold_arg,
        sigma_zero_velocity_x_arg,
        sigma_zero_velocity_y_arg,
        sigma_zero_velocity_z_arg,
        allowed_outlier_percentage_arg,
        max_sigma_x_arg,
        max_sigma_y_arg,
        max_sigma_z_arg,
        max_r_cond_arg,
        max_corres_dis_arg,
        max_iter_arg,
        euclidean_fit_eps_arg,
        transform_eps_arg,
        use_ransac_arg,
        outlier_prob_arg,
        success_prob_arg,
        N_ransac_points_arg,
        inlier_threshold_arg,
        sigma_offset_radar_x_arg,
        sigma_offset_radar_y_arg,
        sigma_offset_radar_z_arg,
        outlier_percentil_radar_arg,
        use_odr_arg,
        radar_outlier_reject_arg,
        min_speed_odr_arg,
        sigma_v_d_arg,
        window_slicing_arg,
        ransac_outlier_reject_thres_arg,
        model_noise_offset_deg_arg,
        model_noise_scale_deg_arg,
        odr_inlier_threshold_arg,
        scekf_dero_ros2bag_node,
    ])
