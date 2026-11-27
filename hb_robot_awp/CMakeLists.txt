cmake_minimum_required(VERSION 3.8)
project(hb_robot_awp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hb_robot_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/TiltSample.msg" DEPENDENCIES std_msgs geometry_msgs)
foreach(target awp_excitation_server cut_pose_experiment)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  target_compile_features(${target} PRIVATE cxx_std_17)
  ament_target_dependencies(${target} rclcpp sensor_msgs std_msgs std_srvs hb_robot_interfaces
      moveit_ros_planning_interface moveit_core moveit_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/attitude_node.py scripts/export_experiment_csv.py scripts/record_awp_excitation.sh DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/attitude_filter.py DESTINATION lib/${PROJECT_NAME})



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
ament_export_dependencies(rosidl_default_runtime)
ament_package()
