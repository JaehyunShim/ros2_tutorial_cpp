# ROS2 Tutorial
[![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/CI/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/Lint/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![codecov](https://codecov.io/gh/rjshim/ros2_tutorial/branch/master/graph/badge.svg)](https://codecov.io/gh/rjshim/ros2_tutorial) ![GitHub](https://img.shields.io/github/license/rjshim/ros2_tutorial)

## Contents
- Write ROS Topic (Pubisher/Subscriber)
- Write ROS Service (Client/Server)
- Write ROS Action (Action Client/Action Server)
- Write ROS Parameter
- Write ROS Launch
- Write ROS Lifecycle
- Write ROS Plugin
- Write ROS Intra Process

## Contents2 (TODO)
- Write ROS in Python
- Write ROS URDF + Rviz
- Write ROSGazebo
- Write ROS Ignition
- Write ROS Control
- Write ROS Perception
- Write ROS Tools
- Write ROS Qt
- Write ROS Security

## Contents3 (TODO)
- Study Gtest Usage
- Study Codecov
- Study Doxygen
- Study User Manual
- Study Bloom
- Write ROS2 CheatSheet

## Run
```sh
# Topic examples
$ ros2 run topic_example publisher
$ ros2 run topic_example subscriber
$ ros2 launch topic_example publisher_subscriber.launch.py
$ ros2 launch topic_example publisher_subscriber.launch.xml

# Service examples
$ ros2 run service_example client
$ ros2 run service_example server
$ ros2 launch service_example client_server.launch.py
$ ros2 launch service_example client_server.launch.xml

# Action examples
$ ros2 run action_example action_client
$ ros2 run action_example action_server
$ ros2 launch action_example action_client_action_server.launch.py
$ ros2 launch action_example action_client_action_server.launch.xml

# Parameter examples
$ ros2 run param_example param
$ ros2 launch param_example param.launch.py
$ ros2 launch param_example param.launch.xml

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
$ ros2 run plugin_example plugin_loader
$ ros2 launch plugin_example plugin.launch.py
$ ros2 launch plugin_example plugin.launch.xml
```

## Reference
- [ROS2 Demos](https://github.com/ros2/demos)
- [ROS2 Examples](https://github.com/ros2/examples)
- [ROS Pluginlib Foxy](https://github.com/ros/pluginlib/tree/foxy)

## Issue
- arg does not work in launch.xml
- action canceling does not work [link](https://answers.ros.org/question/361666/ros2-action-goal-canceling-problem/?answer=361754#post-id-361754)
- Visibility Control Usage check needed
- pluginlib has an inner bug crash issue (removed from CI list)
