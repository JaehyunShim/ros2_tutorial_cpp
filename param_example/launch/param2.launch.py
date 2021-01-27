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
from launch_ros.actions import Node


def generate_launch_description():
    # Path to parameter files
    param_path = os.path.join(
        get_package_share_directory('param_example'),
        'param',
        'controller_info.yaml')

    return LaunchDescription([
        Node(
            package='param_example',
            executable='param_example',
            parameters=[param_path],
            output='screen'),
    ])
