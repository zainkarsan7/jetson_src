cmake_minimum_required(VERSION 3.8)
project(hb_robot_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hb_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
add_executable(generate_explore_path_server
src/generate_explore_path_server.cpp)
add_executable(inspect_scene_server
src/inspect_scene_server.cpp)

ament_target_dependencies(generate_explore_path_server 
  rclcpp
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
)

ament_target_dependencies(inspect_scene_server 
  rclcpp
  rclcpp_action
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs
)

target_link_libraries(generate_explore_path_server
Eigen3::Eigen)

install(
  TARGETS
    generate_explore_path_server
    inspect_scene_server
  DESTINATION lib/${PROJECT_NAME}
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
