# Copyright (c) 2026 Alberto J. Tudela Roldán
# Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launches Teleop for Pioneer robot."""

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Default filenames and where to find them
    bringup_dir = get_package_share_directory('pioneer_bringup')

    default_pioneer_teleop_file = os.path.join(bringup_dir, 'params', 'teleop.yaml')

    # Joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
        parameters=[{
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )

    # Teleop
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_twist_joy',
        name='teleop_twist_joy',
        output='screen',
        parameters=[default_pioneer_teleop_file]
    )

    return LaunchDescription([
        teleop_node
    ])
