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
   params_file = DeclareLaunchArgument("params_file")
   vjoy = DeclareLaunchArgument("vjoy", default_value="True")
   entries = [namespace, params_file, vjoy]
   remappings = list({"car_pose" : "/ego_vehicle/pose", "car_velocity" : "/ego_vehicle/velocity"}.items())
   parameters=[LaunchConfiguration(params_file.name)]
   entries.append( launch_ros.actions.Node(name="vjoy_interface", package='deepracing_rclpy', executable='vjoy_control_node', output='both', condition = IfCondition(LaunchConfiguration(vjoy.name)) ) )
   entries.append( launch_ros.actions.Node(name="pure_pursuit_control", package='deepracing_rclpy', executable='pure_pursuit_bezier', output='both', parameters=parameters, remappings=remappings, namespace=LaunchConfiguration(namespace.name) ) )
   return launch.LaunchDescription(entries)
