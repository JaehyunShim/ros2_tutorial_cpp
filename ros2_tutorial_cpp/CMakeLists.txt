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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/Count.msg"
)

set(srv_files
  "srv/Inquiry.srv"
)

set(action_files
  "action/Fetch.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  # DEPENDENCIES std_msgs
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
add_library(${PROJECT_NAME}_topic_libs SHARED
  src/talker.cpp
  src/listener.cpp
)
ament_target_dependencies(${PROJECT_NAME}_topic_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_topic_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Add service libraries
add_library(${PROJECT_NAME}_service_libs SHARED
  src/requester.cpp
  src/responsor.cpp
)
ament_target_dependencies(${PROJECT_NAME}_service_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_service_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Add param libraries
add_library(${PROJECT_NAME}_param_libs SHARED
  src/param.cpp
)
ament_target_dependencies(${PROJECT_NAME}_param_libs ${dependencies})

# Add action libraries
add_library(${PROJECT_NAME}_action_libs SHARED
  src/action_client.cpp
  src/action_server.cpp
)
ament_target_dependencies(${PROJECT_NAME}_action_libs ${dependencies})
rosidl_target_interfaces(${PROJECT_NAME}_action_libs
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Register topic nodes
rclcpp_components_register_node(${PROJECT_NAME}_topic_libs
  PLUGIN "ros2_tutorial_cpp::Talker"
  EXECUTABLE talker
)
rclcpp_components_register_node(${PROJECT_NAME}_topic_libs
  PLUGIN "ros2_tutorial_cpp::Listener"
  EXECUTABLE listener
)

# Register service nodes
rclcpp_components_register_node(${PROJECT_NAME}_service_libs
  PLUGIN "ros2_tutorial_cpp::Requester"
  EXECUTABLE requester
)
rclcpp_components_register_node(${PROJECT_NAME}_service_libs
  PLUGIN "ros2_tutorial_cpp::Responsor"
  EXECUTABLE responsor
)

# Register param nodes
rclcpp_components_register_node(${PROJECT_NAME}_param_libs
  PLUGIN "ros2_tutorial_cpp::Param"
  EXECUTABLE param
)

# Register action nodes
rclcpp_components_register_node(${PROJECT_NAME}_action_libs
  PLUGIN "ros2_tutorial_cpp::ActionClient"
  EXECUTABLE action_client
)
rclcpp_components_register_node(${PROJECT_NAME}_action_libs
  PLUGIN "ros2_tutorial_cpp::ActionServer"
  EXECUTABLE action_server
)

###########################################################################
# Install
################################################################################
# Install libraries
install(TARGETS
  ${PROJECT_NAME}_topic_libs
  ${PROJECT_NAME}_service_libs
  ${PROJECT_NAME}_param_libs
  ${PROJECT_NAME}_action_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install message, service, param files
install(
  DIRECTORY
  msg
  srv
  param
  action
  DESTINATION share/${PROJECT_NAME}
)

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
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_package()
