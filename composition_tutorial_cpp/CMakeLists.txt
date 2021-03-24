cmake_minimum_required(VERSION 3.5)
project(composition_tutorial_cpp)

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
find_package(rclcpp_components REQUIRED)
find_package(interface_tutorial_cpp REQUIRED)


# add
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  interface_tutorial_cpp
)

add_library(${PROJECT_NAME}_libs SHARED
  src/publisher.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})

# Register
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "composition_tutorial_cpp::Publisher"
  EXECUTABLE publisher
)

# Install
install(TARGETS
  ${PROJECT_NAME}_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
