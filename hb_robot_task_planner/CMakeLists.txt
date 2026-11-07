cmake_minimum_required(VERSION 3.8)
project(hb_robot_task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(hb_robot_interfaces REQUIRED)

include_directories()

add_executable(bt_executor
src/bt_executor.cpp)
ament_target_dependencies(bt_executor
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  hb_robot_interfaces

)

install(TARGETS bt_executor
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees
DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(recovery_tree_test test/recovery_tree_test.cpp)
  ament_target_dependencies(recovery_tree_test behaviortree_cpp_v3)
  target_compile_definitions(recovery_tree_test PRIVATE TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/cut_profile.xml" BASELINE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/baseline.xml")
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
