cmake_minimum_required(VERSION 3.8)
project(percept_nav_costmap_plugin)

option(COVERAGE "Build with gcov coverage instrumentation" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

add_library(percept_nav_costmap_plugin SHARED
  src/detection_layer.cpp
)

if(COVERAGE)
  target_compile_options(percept_nav_costmap_plugin PRIVATE --coverage -O0)
  target_link_options(percept_nav_costmap_plugin PUBLIC --coverage)
endif()

target_include_directories(percept_nav_costmap_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(percept_nav_costmap_plugin
  rclcpp
  nav2_costmap_2d
  pluginlib
  vision_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d detection_layer_plugin.xml)

install(TARGETS percept_nav_costmap_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES detection_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_add_gtest(test_detection_layer test/test_detection_layer.cpp)
  target_include_directories(test_detection_layer PUBLIC include)
  target_link_libraries(test_detection_layer percept_nav_costmap_plugin)
  ament_target_dependencies(test_detection_layer rclcpp nav2_costmap_2d tf2_ros vision_msgs)
  if(COVERAGE)
    target_compile_options(test_detection_layer PRIVATE --coverage -O0)
    target_link_options(test_detection_layer PUBLIC --coverage)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(percept_nav_costmap_plugin)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib vision_msgs)

ament_package()
