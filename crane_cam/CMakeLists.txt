cmake_minimum_required(VERSION 3.8)
project(crane_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(cam_pub src/cam_pub.cpp)
ament_target_dependencies(cam_pub rclcpp sensor_msgs cv_bridge image_transport)

add_executable(pose_est src/aruc_pose_est.cpp)
ament_target_dependencies(pose_est tf2 tf2_ros tf2_geometry_msgs geometry_msgs rclcpp sensor_msgs cv_bridge image_transport)


add_executable(beam_est src/beam_pose_est.cpp)
ament_target_dependencies(beam_est tf2 tf2_ros tf2_geometry_msgs geometry_msgs rclcpp sensor_msgs cv_bridge image_transport)




target_link_libraries(cam_pub ${OpenCV_LIBRARIES})

target_link_libraries(pose_est ${OpenCV_LIBRARIES})


target_link_libraries(beam_est ${OpenCV_LIBRARIES})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")

install(TARGETS
cam_pub
pose_est
beam_est
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
