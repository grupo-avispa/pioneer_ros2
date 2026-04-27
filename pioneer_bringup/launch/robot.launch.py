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

"""
Launches nodes controlling Pioneer robot hardware.

- Robot description
- Foxglove Bridge
"""


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():

    # Default filenames and where to find them
    bringup_dir = get_package_share_directory('pioneer_bringup')
    description_dir = get_package_share_directory('pioneer_description')
    foxglove_bridge_dir = get_package_share_directory('foxglove_bridge')

    default_pioneer_param_file = os.path.join(bringup_dir, 'params', 'pioneer.yaml')
    # default_cert_file = os.path.join(os.environ['HOME'], '.ssl', 'robotavispa.crt')
    # default_key_file = os.path.join(os.environ['HOME'], '.ssl', 'robotavispa.key')

    state_publisher_launch_file = os.path.join(
        description_dir, 'launch', 'state_publisher.launch.py')
    aria_launch_file = os.path.join(
        bringup_dir, 'launch', 'aria.launch.py')
    foxglove_bridge_launch_file = os.path.join(
        foxglove_bridge_dir, 'launch', 'foxglove_bridge_launch.xml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    configured_params = RewrittenYaml(
        source_file=default_pioneer_param_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # '<robot_namespace>' keyword will be replaced by 'namespace' launch argument in config file.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=configured_params,
        replacements={'<robot_namespace>': ('/', namespace)},
    )

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

    # Aria
    aria_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aria_launch_file),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items(),
    )

    # State publisher
    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_publisher_launch_file),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ROS Bridge
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_bridge_launch_file),
        launch_arguments={
            'port': '9090',
            # 'tls': 'false',
            # 'certfile': default_cert_file,
            # 'keyfile': default_key_file
        }.items(),
    )

    return LaunchDescription([
        declare_namespace_arg,
        use_sim_time_launch_arg,
        foxglove_bridge_launch,
        state_publisher_launch,
        aria_launch
    ])
