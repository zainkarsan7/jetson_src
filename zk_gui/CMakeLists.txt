cmake_minimum_required(VERSION 3.8)
project(zk_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
include/zk_gui/zk_panel.hpp
)
add_library(zk_panel src/zk_panel.cpp ${MOC_FILES})
target_include_directories(zk_panel PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(zk_panel
pluginlib
rviz_common)

install(TARGETS zk_panel
EXPORT export_zk_gui
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include)
install(FILES rviz_common_plugins.xml
DESTINATION share/${PROJECT_NAME})
install(FILES icons/classes/zk_panel.png
DESTINATION share/${PROJECT_NAME}/icons/classes)

ament_export_include_directories(include)
ament_export_targets(export_zk_gui)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)


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
