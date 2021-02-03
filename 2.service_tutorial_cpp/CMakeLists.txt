cmake_minimum_required(VERSION 3.5)
project(service_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

#add
include_directories(include)

set(dependencies
  example_interfaces
  rclcpp)

add_executable(client src/client.cpp)
ament_target_dependencies(client ${dependencies})

add_executable(server src/server.cpp)
ament_target_dependencies(server ${dependencies})

# install
install(TARGETS
  client
  server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_client_server
    test/test_client_server.cpp)
  ament_target_dependencies(test_client_server ${dependencies})
endif()

ament_package()
