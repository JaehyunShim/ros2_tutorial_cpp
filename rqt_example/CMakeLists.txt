################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rqt_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(src_files
  src/rqt_example.cpp
  src/rqt_node.cpp
)

set(hd_files
  include/${PROJECT_NAME}/rqt_example.hpp
)

set(ui_files
  ui/rqt_example.ui
)

qt5_wrap_cpp(moc_files ${hd_files})
qt5_wrap_ui(ui_hd_files ${ui_files})

set(dependencies
  "rclcpp"
  "qt_gui_cpp"
  "rqt_gui_cpp"
  "std_msgs"
  "Qt5Widgets"
)

add_library(${PROJECT_NAME} SHARED
  ${src_files}
  ${moc_files}
  ${ui_hd_files}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS scripts/rqt_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch ui
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_dependencies(rclcpp)
ament_export_dependencies(qt_gui_cpp)
ament_export_dependencies(rqt_gui_cpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(Qt5Widgets)
ament_package()
