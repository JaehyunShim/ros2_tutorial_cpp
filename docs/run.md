## Run
```sh
# Topic examples
$ ros2 run topic_example publisher
$ ros2 run topic_example subscriber

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
