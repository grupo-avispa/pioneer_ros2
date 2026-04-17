# Copyright (c) 2024 Alberto J. Tudela Roldán
# Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

"""Launches modern Gazebo with Pioneer 3-AT model in a world."""

import os
import re
import xml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Default filenames and where to find them
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    gazebo_dir = get_package_share_directory('pioneer_gazebo')

    gz_sim_launch_file = os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
    default_world_file = os.path.join(gazebo_dir, 'worlds', 'empty.sdf')
    pioneer_xacro_file = os.path.join(gazebo_dir, 'urdf', 'pioneer3at_gazebo.xacro')
    robot_name = 'pioneer3at'

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.1'
    spawn_yaw_val = '0.0'

    # Create robot description from xacro file
    robot_description_config = xacro.process_file(pioneer_xacro_file)

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_dir, 'worlds'))

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description_config.toxml()}
        ],
        remappings=remappings
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=remappings
    )

    # Gazebo Sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': '-r ' + default_world_file
        }.items(),
    )

    # Fix xml tags for Gazebo
    xml_parsed = xml.etree.ElementTree.fromstring(robot_description_config.toxml())
    pattern = r'package://(.*?)/meshes/'
    for mesh_element in xml_parsed.iter('mesh'):
        filename = mesh_element.get('filename')
        match = re.search(pattern, filename)
        if match:
            package_name = match.group(1)
            filename = filename.replace(match.group(0), f'file://$(find {package_name})/meshes/')
            mesh_element.set('filename', filename)
    model_xml = xml.etree.ElementTree.tostring(xml_parsed)
    if not isinstance(model_xml, str):
        model_xml = model_xml.decode(encoding='ascii')
    # Process xacro macros again
    xml_parsed = xacro.parse(model_xml)
    xacro.process_doc(xml_parsed)

    # Spawn the robot
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-string', xml_parsed.toxml(),
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    # Gz - ROS Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': os.path.join(gazebo_dir, 'configs', 'pioneer3at_bridge.yaml'),
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen'
    )

    # Gz - ROS Image Bridge
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            'realsense/image', 'realsense/depth_image'
        ],
        remappings=[
            ('/realsense/image', '/realsense/color/image_raw'),
            ('/realsense/depth_image', '/realsense/depth/image_rect_raw'),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        set_env_vars_resources,
        gazebo_launch,
        spawn_node,
        robot_state_publisher,
        joint_state_publisher,
        bridge_node,
        image_bridge_node
    ])
