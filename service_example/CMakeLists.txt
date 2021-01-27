################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(service_example)

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
set(srv_files
  "srv/Inquiry.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
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

# Add service libraries
add_library(${PROJECT_NAME}_libs SHARED
  src/client.cpp
  src/server.cpp
)
ament_target_dependencies(${PROJECT_NAME}_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Register service nodes
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "service_example::Client"
  EXECUTABLE client
)
rclcpp_components_register_node(${PROJECT_NAME}_libs
  PLUGIN "service_example::Server"
  EXECUTABLE server
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

# Install service files
install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_client_server test/test_client_server.cpp)
  ament_target_dependencies(test_client_server ${dependencies})
  rosidl_target_interfaces(test_client_server
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
