## Run Command
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
$ ros2 run intra_process_example intra_process_example
$ ros2 launch intra_process_example intra_process_example.launch.py
$ ros2 launch intra_process_example intra_process_example.launch.xml
```
