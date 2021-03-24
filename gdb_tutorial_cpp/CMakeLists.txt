cmake_minimum_required(VERSION 3.5)
project(gdb_tutorial_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# For debugger
add_compile_options(-g)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Build
include_directories(include)

set(dependencies
  rclcpp
)

add_executable(gdb src/gdb.cpp)
ament_target_dependencies(gdb ${dependencies})

# Install
install(TARGETS gdb
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Export
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
