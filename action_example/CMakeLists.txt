################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(action_example)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(action_files
  "action/Fetch.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ADD_LINTER_TESTS
)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

# Add action libraries
add_library(${PROJECT_NAME}_libs SHARED
  src/action_client.cpp
  src/action_server.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Register action nodes
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "action_example::ActionClient"
  EXECUTABLE action_client
)
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "action_example::ActionServer"
  EXECUTABLE action_server
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

# Install action files
install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
