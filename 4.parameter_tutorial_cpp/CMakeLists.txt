cmake_minimum_required(VERSION 3.5)
project(parameter_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add
include_directories(include)

set(dependencies
  rclcpp)

add_executable(parameter src/parameter.cpp)
ament_target_dependencies(parameter ${dependencies})

# install
install(TARGETS
  parameter
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Any reference???
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_parameter
    test/test_parameter.cpp)
  ament_target_dependencies(test_parameter ${dependencies})

  # Any method to test launch.py to see if param setting from a launch file works?
endif()

ament_package()
