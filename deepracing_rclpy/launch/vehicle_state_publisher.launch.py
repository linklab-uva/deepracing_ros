import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition


def generate_launch_description():
   namespace = DeclareLaunchArgument("ns", default_value="")
   entries = [namespace]
   entries.append( launch_ros.actions.Node(name="vehicle_state_publisher", package='deepracing_rclpy', executable='vehicle_state_publisher', output='both', namespace=LaunchConfiguration(namespace.name) ) )
   return launch.LaunchDescription(entries)
