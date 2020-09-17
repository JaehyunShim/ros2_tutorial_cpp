# ROS2 Tutorial
[![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/CI/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![GitHub Action Status](https://github.com/rjshim/ros2_tutorial/workflows/Lint/badge.svg)](https://github.com/rjshim/ros2_tutorial) [![codecov](https://codecov.io/gh/rjshim/ros2_tutorial/branch/master/graph/badge.svg)](https://codecov.io/gh/rjshim/ros2_tutorial) ![GitHub](https://img.shields.io/github/license/rjshim/ros2_tutorial)

## Practice
1. Write ROS Pubisher/Subscriber (Done)
2. Write ROS Client/Server (Done)
3. Write ROS Launch (Done)
4. Write ROS Parameter (Done, TODO for param.launch.xml)
5. Write ROS Action (TODO)
6. Write ROS Plugin (TODO)
7. Write ROS Lifecycle (TODO)

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
8. ROS2 CheatSheet

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

# Param examples
$ ros2 run ros2_tutorial_cpp param
$ ros2 launch ros2_tutorial_cpp param.launch.py
$ ros2 launch ros2_tutorial_cpp param.launch.xml

# Action examples
$ ros2 run ros2_tutorial_cpp action_client
$ ros2 run ros2_tutorial_cpp action_server
$ ros2 launch ros2_tutorial_cpp action_client_action_server.launch.py
$ ros2 launch ros2_tutorial_cpp action_client_action_server.launch.xml
```

## Reference
1. [ROS2 Demo](https://github.com/ros2/demos)
