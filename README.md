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
- ROS2 C++ Launch (TODO)
- ROS2 C++ Lifecycle (TODO)
- ROS2 C++ Plugin (TODO)
- ROS2 C++ Intra Process (TODO)
- ROS2 C++ RQT (TODO)
- ROS2 C++ Test Code (TODO)

## Run
```sh
# Topic tutorial example
$ ros2 run topic_tutorial_cpp publisher_old_school
$ ros2 run topic_tutorial_cpp subscriber_old_school
$ ros2 run topic_tutorial_cpp publisher_member_function
$ ros2 run topic_tutorial_cpp subscriber_member_function
$ ros2 run topic_tutorial_cpp publisher_lambda
$ ros2 run topic_tutorial_cpp subscriber_lambda

# Service tutorial example
$ ros2 run service_tutorial_cpp client 1 2
$ ros2 run service_tutorial_cpp server

# Action tutorial example
$ ros2 run action_tutorial_cpp action_client
$ ros2 run action_tutorial_cpp action_server
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
$ ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback

# Parameter tutorial example
$ ros2 run parameter_tutorial_cpp parameter
$ ros2 launch parameter_tutorial_cpp parameter.launch.py
$ ros2 launch parameter_tutorial_cpp parameter2.launch.py
$ ros2 param get /parameter my_parameter
$ ros2 param set /parameter my_parameter "world"
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
