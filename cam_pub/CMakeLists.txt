cmake_minimum_required(VERSION 3.8)
project(cam_pub)

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
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBRARY_DIRS})
# add_definitions(${OpenCV_DEFINITIONS})

add_executable(z_talk src/zain_pub.cpp)
ament_target_dependencies(z_talk rclcpp std_msgs)


add_executable(webcam_pub src/webcam_publisher.cpp)
ament_target_dependencies(webcam_pub rclcpp sensor_msgs cv_bridge image_transport)

add_executable(hough_node src/hough_node.cpp)
ament_target_dependencies(hough_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)


target_link_libraries(webcam_pub ${OpenCV_LIBRARIES})



install(TARGETS
z_talk
webcam_pub
hough_node
DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
