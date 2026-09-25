cmake_minimum_required(VERSION 3.8)
project(hb_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
include_directories(include)

add_library(hb_perception
  src/observation_buffer.cpp
  src/rgbd_acquisition.cpp
)

target_include_directories(hb_perception
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hb_perception
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_eigen
  tf2
  tf2_ros
  message_filters
)
target_link_libraries(hb_perception
Eigen3
)  

install(
  TARGETS hb_perception
  EXPORT hb_perceptionTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  hb_perceptionTargets
  HAS_TARGET_LIBRARY
)


ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_eigen
  tf2
  tf2_ros
  message_filters
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
