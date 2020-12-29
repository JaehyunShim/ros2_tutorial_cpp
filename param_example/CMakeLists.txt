################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(param_example)

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

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "rclcpp_components"
)

# Add param libraries
add_library(${PROJECT_NAME}_libs SHARED
  src/param.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})

# Register param nodes
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "param_example::Param"
  EXECUTABLE param_example
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

# Install param files
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_package()
