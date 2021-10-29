"""Launch the cpp_code executable in this package"""

import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("deepracing_rclcpp"),"config")
    config_file = DeclareLaunchArgument("config_file", default_value=os.path.join(config_dir, "velocity_pid.yaml"))
    frequency = DeclareLaunchArgument("frequency", default_value="100.0")
    with_acceleration = DeclareLaunchArgument("with_acceleration", default_value="false")
    
    entries = [config_file, frequency, with_acceleration]
    entries.append(
        launch_ros.actions.Node(
            package='deepracing_rclcpp',
            name='velocity_control_node', 
            executable='velocity_control_node', 
            output='screen', 
            parameters=[{"frequency" : LaunchConfiguration(frequency.name), "with_acceleration" : LaunchConfiguration(with_acceleration.name)}, LaunchConfiguration(config_file.name)], 
            remappings=[("pid_state", "/ego_vehicle/speed_pid_state"), ("setpoint_in", "/ego_vehicle/speed_setpoint"), ("odom_in", "/ego_vehicle/odom"), ("accel_in", "/ego_vehicle/acceleration")]
        )
    )
    return LaunchDescription(entries)