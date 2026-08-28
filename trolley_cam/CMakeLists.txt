cmake_minimum_required(VERSION 3.8)
project(trolley_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(spreader_pose_pub src/spreader_pose_pub.cpp)
ament_target_dependencies(spreader_pose_pub tf2 tf2_ros tf2_geometry_msgs geometry_msgs rclcpp sensor_msgs cv_bridge image_transport)

add_library(spreader_pose_component SHARED
  src/spreader_pose_node.cpp
)
ament_target_dependencies(spreader_pose_component tf2 tf2_ros tf2_geometry_msgs geometry_msgs rclcpp rclcpp_components sensor_msgs cv_bridge image_transport)




add_library(aruco_detector src/aruco_detector.cpp)
ament_target_dependencies(aruco_detector tf2 cv_bridge )

target_include_directories(aruco_detector PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_link_libraries(spreader_pose_component aruco_detector ${OpenCV_LIBRARIES})
target_link_libraries(aruco_detector ${OpenCV_LIBRARIES})
target_link_libraries(spreader_pose_pub 
aruco_detector
${OpenCV_LIBRARIES})

rclcpp_components_register_nodes(spreader_pose_component
"trolley_cam::SpreaderPoseNode"
)

install(
  TARGETS spreader_pose_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
spreader_pose_pub
aruco_detector
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
