# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    share_dir = get_package_share_directory('deepracing_launch')
    params_path = os.path.join(share_dir, 'config')

    argz = []

    nodez= []
    nodez.append(Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]

    ))
    nodez.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(params_path, 'ekf.yaml')],
    ))
    
    # nodez.append(Node(
    #     package='uva_iac_conversions',
    #     executable='odom_to_tf',
    #     name='odom_to_tf'
    # ))
    return LaunchDescription(argz+nodez)
