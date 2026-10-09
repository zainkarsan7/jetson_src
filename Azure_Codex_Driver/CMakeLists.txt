cmake_minimum_required(VERSION 3.8)
project(azure_kinect_ros2_driver_codex)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package( k4a REQUIRED )
find_package( k4arecord REQUIRED )
find_path(TURBOJPEG_INCLUDE_DIR turbojpeg.h REQUIRED)
find_library(TURBOJPEG_LIBRARY NAMES turbojpeg REQUIRED)

# For Python test subscriber
find_package(rclpy REQUIRED)

if(k4a_FOUND)
  message("k4a found")
else()
  message(FATAL_ERROR "k4a not found!!")
endif()


add_executable( azure_kinect_node_codex src/k4a_ros_node.cpp src/k4a_ros_device.cpp
                src/k4a_ros_device_params.cpp src/k4a_calibration_transform_data.cpp)

target_include_directories(azure_kinect_node_codex PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(azure_kinect_node_codex PRIVATE ${TURBOJPEG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(azure_kinect_node_codex rclcpp std_msgs sensor_msgs diagnostic_msgs image_transport
                          tf2_ros tf2_geometry_msgs angles)

target_link_libraries(azure_kinect_node_codex ${OpenCV_LIBRARIES} k4a::k4a k4a::k4arecord ${TURBOJPEG_LIBRARY})

target_compile_features(azure_kinect_node_codex PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS azure_kinect_node_codex
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        scripts/test_azure_kinect_subscriber.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY launch config docs
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_timestamp_mapper test/test_timestamp_mapper.cpp)
  target_include_directories(test_timestamp_mapper PRIVATE include)
  target_compile_features(test_timestamp_mapper PRIVATE cxx_std_17)
  ament_add_gtest(test_message_pool test/test_message_pool.cpp)
  target_include_directories(test_message_pool PRIVATE include)
  target_compile_features(test_message_pool PRIVATE cxx_std_17)
  ament_add_gtest(test_latest_capture test/test_latest_capture.cpp)
  target_include_directories(test_latest_capture PRIVATE include)
  target_compile_features(test_latest_capture PRIVATE cxx_std_17)
  ament_add_gtest(test_mjpeg_decoder test/test_mjpeg_decoder.cpp)
  target_include_directories(test_mjpeg_decoder PRIVATE include ${TURBOJPEG_INCLUDE_DIR})
  target_link_libraries(test_mjpeg_decoder k4a::k4a ${TURBOJPEG_LIBRARY})
  target_compile_features(test_mjpeg_decoder PRIVATE cxx_std_17)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_library(fake_k4a SHARED test/fake_k4a.cpp)
    target_include_directories(fake_k4a PRIVATE ${TURBOJPEG_INCLUDE_DIR})
    target_link_libraries(fake_k4a k4a::k4a ${TURBOJPEG_LIBRARY})
    target_compile_features(fake_k4a PRIVATE cxx_std_17)
    ament_add_test(test_pipeline COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pipeline.py"
      "$<TARGET_FILE:azure_kinect_node_codex>" "$<TARGET_FILE:fake_k4a>"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 90)
  endif()
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
