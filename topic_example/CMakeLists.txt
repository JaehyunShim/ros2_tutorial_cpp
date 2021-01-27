################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(topic_example)

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
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/Count.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # DEPENDENCIES imported_message_type
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
  "rclcpp_components"
)

# Add topic libraries
add_library(${PROJECT_NAME}_libs SHARED
  src/publisher.cpp
  src/subscriber.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Register topic nodes
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "topic_example::Publisher"
  EXECUTABLE publisher
)
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "topic_example::Subscriber"
  EXECUTABLE subscriber
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

# Install message files
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_publisher_subscriber test/test_publisher_subscriber.cpp)
  ament_target_dependencies(test_publisher_subscriber ${dependencies})
  rosidl_target_interfaces(test_publisher_subscriber
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
