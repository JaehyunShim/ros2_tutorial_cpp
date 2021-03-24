# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2021 Jaehyun Shim
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='JaehyunBot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=robot_name,
            description='Robot Name'),

        Node(
            package='topic_tutorial_cpp',
            executable='publisher_old_school',
            name='changed_publisher_name',
            arguments=[robot_name],
            remappings=[('/topic_old_school', '/remapped_topic_name')],
            parameters=[{'my_parameter': 'earth'}],  # more param usage info in param_tutorial_cpp
            output='screen'),
    ])
