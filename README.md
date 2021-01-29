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
- ROS2 C++ Launch
- ROS2 C++ Lifecycle
- ROS2 C++ Plugin
- ROS2 C++ Intra Process
- ROS2 C++ RQT
- ROS2 C++ Test Code (TODO)

## Run
```sh
# Topic examples
$ ros2 run topic_tutorial_cpp publisher_lambda
$ ros2 run topic_tutorial_cpp subscriber_lambda
$ ros2 run topic_tutorial_cpp publisher_member_function
$ ros2 run topic_tutorial_cpp subscriber_member_function

# Service examples
$ ros2 run service_example client
$ ros2 run service_example server

# Action examples
$ ros2 run action_example action_client
$ ros2 run action_example action_server

# Parameter examples
$ ros2 run param_example param_example
$ ros2 launch param_example param.launch.py
$ ros2 launch param_example param2.launch.py

# Launch examples
$ ros2 launch launch.launch.py
$ ros2 launch launch.launch.xml

# Lifecycle examples
$ ros2 run lifecycle_example lifecycle_publisher
$ ros2 lifecycle set /lifecycle_publisher configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_publisher

$ ros2 run lifecycle_example lifecycle_subscriber
$ ros2 lifecycle set /lifecycle_subscriber configure # activate, deactivate, cleanup, shutdown
$ ros2 lifecycle get /lifecycle_subscriber

$ ros2 launch lifecycle_example lifecycle_publisher_lifecycle_subscriber.launch.py
$ ros2 launch lifecycle_example lifecycle_publisher_lifecycle_subscriber.launch.xml

# Plugin examples
$ ros2 run plugin_example plugin_loader
$ ros2 launch plugin_example plugin.launch.py
$ ros2 launch plugin_example plugin.launch.xml

# Intra process examples
$ ros2 run intra_process_example intra_process_example
$ ros2 launch intra_process_example intra_process_example.launch.py
$ ros2 launch intra_process_example intra_process_example.launch.xml

# RQT examples
$ ros2 run rqt_example rqt_example
$ ros2 launch rqt_example rqt_example.launch.py
$ ros2 launch rqt_example rqt_example.launch.xml
$ rqt  # Find the example plugin on the tab
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
- action cancelling does not work [link](https://answers.ros.org/question/361666/ros2-action-goal-canceling-problem/?answer=361754#post-id-361754)
- Visibility Control usage check needed
- pluginlib has an inner bug crash issue (commented out building process in CMakeLists.txt)
- action_example has a ci build crash issue (commented out building process in CMakeLists.txt)
