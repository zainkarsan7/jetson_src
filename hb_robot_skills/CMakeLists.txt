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
find_package(hb_robot_perception REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(hb_motion
  src/motion/cut_graph_planner.cpp
  src/motion/cut_candidate_graph.cpp
  src/motion/cut_process_trajectory_generator.cpp
  src/motion/exploration_planner.cpp
  src/motion/trajectory_validator.cpp
  src/motion/cut_planner.cpp
)

target_include_directories(hb_motion
PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hb_motion
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  hb_robot_interfaces
  tf2_eigen
  

)
target_link_libraries(hb_motion

hb_robot_perception::hb_perception
)

add_executable(cut_debug_node src/cut_debug_node.cpp)
ament_target_dependencies(cut_debug_node
  rclcpp
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  moveit_msgs
  moveit_ros_planning_interface
  ur_msgs
)

add_executable(cut_debug_node_torch src/cut_debug_node_torch.cpp)
ament_target_dependencies(cut_debug_node_torch
  rclcpp
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  moveit_msgs
  moveit_ros_planning_interface
  ur_msgs
  hb_robot_interfaces
)
target_link_libraries(cut_debug_node_torch
hb_motion
)

add_executable(approve_motion_server
src/approve_motion_server.cpp)

ament_target_dependencies(approve_motion_server
rclcpp
hb_robot_interfaces
)

add_executable(inspect_scene_server
src/inspect_scene_server.cpp)

ament_target_dependencies(inspect_scene_server 
  rclcpp
  rclcpp_action
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  sensor_msgs
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs
)

target_link_libraries(inspect_scene_server
hb_motion
hb_robot_perception::hb_perception
)

add_executable(inspect_profile_server
src/inspect_profile_server.cpp)

ament_target_dependencies(inspect_profile_server 
  rclcpp
  rclcpp_action
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  sensor_msgs
  moveit_msgs
  moveit_ros_planning
  moveit_core
  shape_msgs
  moveit_ros_planning_interface
  visualization_msgs
)

target_link_libraries(inspect_profile_server
hb_motion
hb_robot_perception::hb_perception
)

add_executable(inspect_cut_server src/inspect_cut_server.cpp)
ament_target_dependencies(inspect_cut_server
  rclcpp rclcpp_action geometry_msgs hb_robot_interfaces Eigen3 tf2_eigen
  sensor_msgs moveit_msgs moveit_ros_planning_interface)
target_link_libraries(inspect_cut_server hb_motion hb_robot_perception::hb_perception)

add_executable(task_state_server src/task_state_server.cpp)
ament_target_dependencies(task_state_server rclcpp hb_robot_interfaces moveit_msgs std_msgs)
target_link_libraries(task_state_server hb_robot_perception::hb_perception)
add_executable(revalidate_workpiece_server src/revalidate_workpiece_server.cpp)
ament_target_dependencies(revalidate_workpiece_server rclcpp rclcpp_action hb_robot_interfaces
  moveit_ros_planning_interface tf2_eigen)
target_link_libraries(revalidate_workpiece_server hb_robot_perception::hb_perception)

add_executable(cut_profile_server
src/cut_profile_server.cpp src/cut_recovery.cpp)

ament_target_dependencies(cut_profile_server
  std_srvs
  rclcpp
  rclcpp_action
  geometry_msgs
  hb_robot_interfaces
  Eigen3
  tf2_eigen
  sensor_msgs
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs
  ur_msgs
)

target_link_libraries(cut_profile_server
hb_motion
hb_robot_perception::hb_perception
)


install(
  TARGETS
    task_state_server
    revalidate_workpiece_server
    inspect_scene_server
    inspect_profile_server
    inspect_cut_server
    approve_motion_server
    cut_profile_server
    hb_motion
    cut_debug_node
    cut_debug_node_torch
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/capture_cut_environment DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/cut_environment_filter.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(cut_environment_test test/test_cut_environment.py
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=183 OPENBLAS_NUM_THREADS=1)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(recovery_test test/recovery_test.cpp)
  target_link_libraries(recovery_test hb_motion hb_robot_perception::hb_perception)
  ament_add_gtest(cut_inspection_test test/cut_inspection_test.cpp)
  target_link_libraries(cut_inspection_test hb_motion hb_robot_perception::hb_perception)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  ament_add_gtest(cut_graph_planner_test test/cut_graph_planner_test.cpp)
  target_link_libraries(cut_graph_planner_test hb_motion)
  ament_target_dependencies(cut_graph_planner_test urdf srdfdom moveit_core moveit_ros_planning_interface)
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
