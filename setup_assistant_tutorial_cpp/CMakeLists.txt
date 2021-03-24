cmake_minimum_required(VERSION 3.5)
project(setup_assistant_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED filesystem)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# add
include_directories(include)

set(dependencies
  Boost
  ament_index_cpp
  rclcpp
  std_msgs)

add_executable(setup_assistant_tutorial src/setup_assistant_tutorial.cpp)
ament_target_dependencies(setup_assistant_tutorial ${dependencies})

# install
install(TARGETS setup_assistant_tutorial
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME})

ament_package()
