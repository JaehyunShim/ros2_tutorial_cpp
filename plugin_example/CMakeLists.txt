################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(plugin_example)

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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "pluginlib"
  "rclcpp"
  "std_msgs"
)

# # Add libraries
# add_library(${PROJECT_NAME} SHARED
#   src/plugin.cpp
# )
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# # Add executables
# add_executable(plugin_loader
#   src/plugin_loader.cpp
# )
# ament_target_dependencies(plugin_loader ${dependencies})

###########################################################################
# Install
################################################################################
# # Install libraries
# install(TARGETS
#   ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # Install executables
# install(TARGETS
#   plugin_loader
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install plugin files
install(FILES
  example_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(plugin_example example_plugin.xml)

################################################################################
# Test
################################################################################
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(rclcpp REQUIRED)
#   find_package(rclcpp_components REQUIRED)
#   find_package(interface_example REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   set(test_dependencies
#     "rclcpp"
#     "rclcpp_components"
#     "interface_example"
#   )

#   ament_add_gtest(test_publisher
#     test/test_publisher.cpp)
#   # target_include_directories(test_publisher PRIVATE include)
#   ament_target_dependencies(test_publisher ${test_dependencies})

#   # rosidl_target_interfaces(test_publisher
#   #   ${PROJECT_NAME}
#   #   "rosidl_typesupport_cpp"
#   # )

# endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_package()
