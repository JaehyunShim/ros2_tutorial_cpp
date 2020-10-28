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

# To include "ui_main_window.h" file
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

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(src_files
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(hd_files
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(ui_files
  ui/main_window.ui
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

# add_library(${PROJECT_NAME} SHARED
#   ${src_files}
#   ${moc_files}
#   ${ui_hd_files}
# )
add_executable(${PROJECT_NAME}
  ${src_files}
  ${moc_files}
  ${ui_hd_files}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  Qt5::Widgets
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib/${PROJECT_NAME}
  # LIBRARY DESTINATION lib/${PROJECT_NAME}
  # RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY ui
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(qt_gui_cpp)
ament_export_dependencies(rqt_gui_cpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(Qt5Widgets)
ament_package()
