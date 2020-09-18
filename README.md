# ROS2 Tutorial
[![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/CI/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/Lint/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![codecov](https://codecov.io/gh/rjshim/ros2_tutorial/branch/master/graph/badge.svg)](https://codecov.io/gh/rjshim/ros2_tutorial) ![GitHub](https://img.shields.io/github/license/rjshim/ros2_tutorial)

## Practice
1. Write ROS Pubisher/Subscriber with Topic (Done)
2. Write ROS Client/Server with Serivce (Done)
3. Write ROS Action (Done)
4. Write ROS Parameter (Done)
5. Write ROS Launch (Done)
6. Write ROS Lifecycle (Done, TOSTUDYFURTHER)
7. Write ROS Plugin (TODO)

## Practice2
4. Write ROS in Python
1. Write URDF + Rviz
2. Write Gazebo
3. Write Ignition
4. Write ROS Control
4. Write ROS Perception
4. Write ROS Tools

## New Features
1. Component (Done)
2. Python Launch Files (Done)
3. Visibility Control (ToDo)

## TODO
1. Study Gtest Usage
2. Test Codecov
3. Test Security
4. Test Lifecycle
5. Test Doxygen
6. Create User Manual
7. Bloom
8. Write CheatSheet

## Run
```sh
# Topic examples
$ ros2 run ros2_tutorial_cpp publisher
$ ros2 run ros2_tutorial_cpp subscriber
$ ros2 launch ros2_tutorial_cpp publisher_subscriber.launch.py
$ ros2 launch ros2_tutorial_cpp publisher_subscriber.launch.xml

# Service examples
$ ros2 run ros2_tutorial_cpp client
$ ros2 run ros2_tutorial_cpp server
$ ros2 launch ros2_tutorial_cpp client_server.launch.py
$ ros2 launch ros2_tutorial_cpp client_server.launch.xml

# Action examples
$ ros2 run ros2_tutorial_cpp action_client
$ ros2 run ros2_tutorial_cpp action_server
$ ros2 launch ros2_tutorial_cpp action_client_action_server.launch.py
$ ros2 launch ros2_tutorial_cpp action_client_action_server.launch.xml

# Param examples
$ ros2 run ros2_tutorial_cpp param
$ ros2 launch ros2_tutorial_cpp param.launch.py
$ ros2 launch ros2_tutorial_cpp param.launch.xml

# Lifecycle examples
$ ros2 run lifecycle_example lifecycle_publisher
$ ros2 lifecycle set /lifecycle_publisher configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_publisher

$ ros2 run lifecycle_example lifecycle_subscriber
$ ros2 lifecycle set /lifecycle_subscriber configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_subscriber

$ ros2 launch lifecycle_example lifecycle_publisher_lifecycle_subscriber.launch.py
$ ros2 launch lifecycle_example lifecycle_publisher_lifecycle_subscriber.launch.xml
```

## Reference
1. [ROS2 Demo](https://github.com/ros2/demos)

## Issue
1. arg does not work in launch.xml
2. Action Cancel does not work [link](https://answers.ros.org/question/361666/ros2-action-goal-canceling-problem/?answer=361754#post-id-361754)
