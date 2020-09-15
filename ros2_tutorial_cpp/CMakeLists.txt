################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(ros2_tutorial_cpp)

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
  "std_msgs"
)


add_library(${PROJECT_NAME} SHARED
  src/talker.cpp
  src/listener.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

################################################################################
# Install
################################################################################
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

################################################################################
# Test
################################################################################
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(rclcpp REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   set(test_dependencies
#     "rclcpp"
#   )

#   ament_add_gtest(test_ros2_tutorial_cpp
#     test/test_ros2_tutorial_cpp.cpp
#     src/ros2_tutorial_cpp.cpp)
#   target_include_directories(test_ros2_tutorial_cpp PRIVATE include)
#   ament_target_dependencies(test_ros2_tutorial_cpp ${test_dependencies})
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
