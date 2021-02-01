cmake_minimum_required(VERSION 3.5)
project(action_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# add
include_directories(include)

set(dependecies
  action_tutorials_interfaces
  rclcpp
  rclcpp_action)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client ${dependecies})

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server ${dependecies})

# install
install(TARGETS
  action_client
  action_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_action_client_action_server
  test/test_action_client_action_server.cpp)
  ament_target_dependencies(test_action_client_action_server ${dependecies})
endif()

ament_package()
