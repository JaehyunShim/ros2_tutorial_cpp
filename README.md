# ROS2 Tutorial
[![GitHub License](https://img.shields.io/github/license/JaehyunShim/ros2_tutorial_cpp)](https://github.com/JaehyunShim/ros2_tutorial_cpp/blob/master/LICENSE)
[![GitHub CI Status](https://github.com/JaehyunShim/ros2_tutorial_cpp/workflows/CI/badge.svg)](https://github.com/JaehyunShim/ros2_tutorial_cpp/actions?query=workflow%3ACI)
[![GitHub Lint Status](https://github.com/JaehyunShim/ros2_tutorial_cpp/workflows/Lint/badge.svg)](https://github.com/JaehyunShim/ros2_tutorial_cpp/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/JaehyunShim/ros2_tutorial_cpp/branch/master/graph/badge.svg)](https://codecov.io/gh/JaehyunShim/ros2_tutorial_cpp)
[![Documentation Status](https://readthedocs.org/projects/ros2-tutorial-cpp/badge/?version=latest)](https://ros2-tutorial-cpp.readthedocs.io/en/latest/?badge=latest)
[![Doxygen](https://img.shields.io/badge/doxygen-documentation-blue.svg)](https://jaehyunshim.github.io/docs.ros2_tutorial_cpp.org/)

## Contents
- ROS2 C++ Topic (Pubisher/Subscriber)
- ROS2 C++ Service (Client/Server)
- ROS2 C++ Action (Action Client/Action Server)
- ROS2 C++ Parameter
- ROS2 C++ Interface (Message, Service, Action)
- ROS2 C++ Launch
- ROS2 C++ Composition
- ROS2 C++ Intra Process
- ROS2 C++ Lifecycle
- ROS2 C++ Plugin
- ROS2 C++ RQT
- ROS2 C++ Setup Assistant

## Run
```sh
# Topic Tutorial CPP
$ ros2 run topic_tutorial_cpp publisher_old_school
$ ros2 run topic_tutorial_cpp subscriber_old_school
$ ros2 run topic_tutorial_cpp publisher_member_function
$ ros2 run topic_tutorial_cpp subscriber_member_function
$ ros2 run topic_tutorial_cpp publisher_lambda
$ ros2 run topic_tutorial_cpp subscriber_lambda

# Service Tutorial CPP
$ ros2 run service_tutorial_cpp client 1 2
$ ros2 run service_tutorial_cpp server

# Action Tutorial CPP
$ ros2 run action_tutorial_cpp action_client
$ ros2 run action_tutorial_cpp action_server
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback

# Parameter Tutorial CPP
$ ros2 run parameter_tutorial_cpp parameter
$ ros2 launch parameter_tutorial_cpp parameter.launch.py
$ ros2 launch parameter_tutorial_cpp parameter2.launch.py
$ ros2 param get /parameter my_parameter
$ ros2 param set /parameter my_parameter "world"

# Launch Tutorial CPP
$ ros2 launch launch_tutorial_cpp robot.launch.py
$ ros2 launch launch_tutorial_cpp robot.launch.xml

# Composition Tutorial CPP
$ ros2 run composition_tutorial_cpp publisher

# Intra Process Tutorial CPP
$ ros2 run intra_process_tutorial_cpp intra_process

# Lifecycle Tutorial CPP
$ ros2 run lifecycle_tutorial_cpp lifecycle_publisher
$ ros2 lifecycle set /lifecycle_publisher configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_publisher

$ ros2 run lifecycle_tutorial_cpp lifecycle_subscriber
$ ros2 lifecycle set /lifecycle_subscriber configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_subscriber

$ ros2 launch lifecycle_tutorial_cpp lifecycle_publisher_lifecycle_subscriber.launch.py
$ ros2 launch lifecycle_tutorial_cpp lifecycle_publisher_lifecycle_subscriber.launch.xml

# Plugin Tutorial CPP
$ ros2 run plugin_tutorial_cpp plugin_loader

# RQT Tutorial CPP
$ ros2 run rqt_tutorial_cpp rqt_node
$ rqt --force-discover  # add --force-discover option when plugin is not found.

# Setup Assistant Tutorial CPP
$ ros2 run setup_assistant_tutorial_cpp setup_assistant_tutorial
```

## Reference
- [ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
- [ROS2 Examples](https://github.com/ros2/examples)
- [ROS2 Demos](https://github.com/ros2/demos)
- [ROS Pluginlib Foxy](https://github.com/ros/pluginlib/tree/foxy)
- [Read the Docs official webpage](https://readthedocs.org)
- [doxygen/doxygen/Doxyfile](https://github.com/doxygen/doxygen/blob/master/Doxyfile)

## Issue
- arg does not work in launch.xml
- pluginlib has an inner bug crash issue (commented out building process in CMakeLists.txt)
