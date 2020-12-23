################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(lifecycle_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "rcutils"
  "std_msgs"
)

# Add topic libraries
add_library(${PROJECT_NAME}_libs SHARED
  src/lifecycle_publisher.cpp
  src/lifecycle_subscriber.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})

# Register topic nodes
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "lifecycle_example::LifecyclePublisher"
  EXECUTABLE lifecycle_publisher
)
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "lifecycle_example::LifecycleSubscriber"
  EXECUTABLE lifecycle_subscriber
)

################################################################################
# Install
################################################################################
# Install libraries
install(TARGETS
  ${PROJECT_NAME}_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

#   find_package(ros_testing REQUIRED)
#   add_ros_test(
#     test/test_lifecycle.py
#     TIMEOUT 60
#   )
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(lifecycle_msgs)
ament_export_dependencies(rcutils)
ament_export_dependencies(std_msgs)
ament_package()
