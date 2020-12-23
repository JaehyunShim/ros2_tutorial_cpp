# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2020, Jaehyun Shim, ROBOTIS CO., LTD.
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

    # Path to parameter files
    param_path = LaunchConfiguration(
        'param_path',
        default=os.path.join(
            get_package_share_directory('param_example'),
            'param',
            'controller_info.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=robot_name,
            description='Robot Name'),

        Node(
            package='launch_example',
            executable='launch_example',
            name='launch_example',
            remappings=[('/chatter', '/remapped_chatter')],
            arguments=[robot_name],
            parameters=[param_path],
            output='screen'),
    ])
