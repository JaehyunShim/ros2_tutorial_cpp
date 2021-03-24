cmake_minimum_required(VERSION 3.5)
project(plugin_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build
include_directories(
  include
)

set(dependencies
  pluginlib
  rclcpp
  std_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/plugin.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(plugin_loader
  src/plugin_loader.cpp
)
ament_target_dependencies(plugin_loader ${dependencies})

# Install
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  plugin_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(plugin_tutorial_cpp plugin.xml)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
