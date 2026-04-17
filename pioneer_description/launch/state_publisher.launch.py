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

"""Creates robot state publisher topic."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Default filenames and where to find them
    description_dir = get_package_share_directory('pioneer_description')
    pioneer_xacro_file = os.path.join(description_dir, 'urdf', 'pioneer3at.xacro')

    # Create the launch configuration variables:
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Map these variables to arguments: can be set from the command line or a default will be used
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create robot description from xacro file
    robot_description_config = xacro.process_file(pioneer_xacro_file)

    # Create state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        name='state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            'robot_description': robot_description_config.toxml()
        }],
        remappings=remappings
    )

    # Publish default joint states for the wheel joints (continuous joints need this)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        namespace=namespace,
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config.toxml()
        }],
        remappings=remappings
    )

    return LaunchDescription([
        declare_namespace_arg,
        use_sim_time_launch_arg,
        joint_state_publisher_node,
        robot_state_publisher_node
    ])
