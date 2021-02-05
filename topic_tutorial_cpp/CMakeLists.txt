cmake_minimum_required(VERSION 3.5)
project(topic_tutorial_cpp)

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
find_package(std_msgs REQUIRED)

# add
include_directories(include)

set(dependencies
  rclcpp
  std_msgs)

add_executable(publisher_old_school src/publisher_old_school.cpp)
ament_target_dependencies(publisher_old_school ${dependencies})

add_executable(subscriber_old_school src/subscriber_old_school.cpp)
ament_target_dependencies(subscriber_old_school ${dependencies})

add_executable(publisher_member_function src/publisher_member_function.cpp)
ament_target_dependencies(publisher_member_function ${dependencies})

add_executable(subscriber_member_function src/subscriber_member_function.cpp)
ament_target_dependencies(subscriber_member_function ${dependencies})

add_executable(publisher_lambda src/publisher_lambda.cpp)
ament_target_dependencies(publisher_lambda ${dependencies})

add_executable(subscriber_lambda src/subscriber_lambda.cpp)
ament_target_dependencies(subscriber_lambda ${dependencies})

# install
install(TARGETS
  publisher_old_school
  subscriber_old_school
  publisher_member_function
  subscriber_member_function
  publisher_lambda
  subscriber_lambda
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Any reference???
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_publisher_subscriber
    test/test_publisher_subscriber.cpp)
  ament_target_dependencies(test_publisher_subscriber ${dependencies})
endif()

ament_package()
