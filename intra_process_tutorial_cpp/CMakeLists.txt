cmake_minimum_required(VERSION 3.5)
project(intra_process_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "std_msgs"
)

add_executable(intra_process src/intra_process.cpp)
ament_target_dependencies(intra_process ${dependencies})

# Install
install(TARGETS intra_process
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_package()
