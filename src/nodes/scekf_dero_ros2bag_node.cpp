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

#include <rclcpp/rclcpp.hpp>

#include "dero/run_ros2bag.hpp"

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto incsl_node = std::make_shared<incsl::RunRos2Bag>("incsl_node");

  try {
    executor.add_node(incsl_node);
    executor.spin();

  } catch (std::exception const &err) {
    RCLCPP_ERROR(incsl_node->get_logger(), "&s", err.what());
  } // try / catch

  return 0;
} // int main
