cmake_minimum_required(VERSION 3.5)
project(dero)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_language(Fortran CXX)

add_library(odrpack
    src/odrpack/d_odr.f
    src/odrpack/d_mprec.f
    src/odrpack/lpkbls.f
    src/odrpack/dlunoc.f
    src/odrpack/real_precision.f
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rosbag2_cpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_eigen"
  "tf2_ros"
)

add_library(dero_lib SHARED
            src/cores/run_ros2bag.cpp
            src/cores/ekf_rio.cpp
            src/cores/scekf_dero.cpp
            src/cores/radar_estimator.cpp
            src/cores/odr.cpp
            src/utils/nav_convert.cpp
            src/utils/math_tools.cpp
)

add_executable(scekf_dero_ros2bag_node
               src/nodes/scekf_dero_ros2bag_node.cpp 
)

target_link_libraries(scekf_dero_ros2bag_node
                      dero_lib
                      ${PCL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      odrpack
)

ament_target_dependencies(scekf_dero_ros2bag_node
                          rclcpp
                          std_msgs
                          sensor_msgs
                          nav_msgs
                          rosbag2_cpp
                          geometry_msgs
                          tf2_eigen
                          tf2_ros
                          pcl_conversions
)

ament_target_dependencies(dero_lib
                          rclcpp
                          std_msgs
                          sensor_msgs
                          nav_msgs
                          rosbag2_cpp
                          geometry_msgs
                          tf2_eigen
                          tf2_ros
                          pcl_conversions
)
install(TARGETS
  scekf_dero_ros2bag_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  include/
  DESTINATION include)

install(TARGETS
  dero_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
