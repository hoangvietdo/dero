# This file is part of DeRO: Dead Reckoning Based on Radar Odometry
# With Accelerometers Aided for Robot Localization.
# Copyright (C) 2024 Hoang Viet Do  <hoangvietdo@sju.ac.kr>

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https:##www.gnu.org#licenses#>.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dero'),
        'config',
        'rio_default.yaml'
    )

    est_save_dir_arg = DeclareLaunchArgument('est_save_dir',
                                              default_value = '/home/vietdo/Desktop/viet_est/carried_2/ekf_rio_carried_2_est.txt',
                                              description = '')
    bag_dir_arg = DeclareLaunchArgument('bag_dir',
                                         default_value = '/home/vietdo/Downloads/radar_inertial_datasets_icins_2021/carried_datasets/carried_2_fixed/carried_2_fixed_0.db3',
                                         description = '');
    coarse_alignment_window_size_arg = DeclareLaunchArgument('coarse_alignment_window_size',
                                                              default_value = '12700',
                                                              description = 'IMU step')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value = 'False')
    rviz_config = os.path.join(
        get_package_share_directory('dero'),
        'config',
        'dero.rviz')

    return LaunchDescription([
      est_save_dir_arg,
      bag_dir_arg,
      use_rviz_arg,
      coarse_alignment_window_size_arg,
      Node(
         package='dero',
         executable='incsl_ros2bag_node',
         namespace='incsl',
         name='rio',
         output='screen',
         parameters=[config, {
            'est_save_dir':                LaunchConfiguration('est_save_dir'),
            'bag_dir':                     LaunchConfiguration('bag_dir'),
            'coarse_alignment_window_size':LaunchConfiguration('coarse_alignment_window_size'),
            }]
      ),
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_config],
         condition=IfCondition(use_rviz)
      ),
   ])
