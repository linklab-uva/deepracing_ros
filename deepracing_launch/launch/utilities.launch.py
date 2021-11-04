"""Launch the cpp_code executable in this package"""

import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("deepracing_launch"),"config")
    config_file = DeclareLaunchArgument("config_file", default_value=os.path.join(config_dir, "tf_updater.yaml"))
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    
    entries = [config_file,use_sim_time]
    entries.append(launch_ros.actions.Node(package='deepracing_rclpy', name='f1_boundary_publisher', executable='boundary_publisher', output='screen', parameters=[{"track_search_dirs": os.getenv("F1_TRACK_DIRS","").split(os.pathsep), "use_sim_time" : LaunchConfiguration(use_sim_time.name)}], remappings=[("session_data", "/f1_game/session_data")]))
    entries.append(launch_ros.actions.Node(package='deepracing_rclpy', name='vehicle_state_publisher', executable='vehicle_state_publisher', output='screen', parameters=[{"use_sim_time" : LaunchConfiguration(use_sim_time.name)}], remappings=[("motion_data", "/f1_game/motion_data"), ("lap_data", "/f1_game/lap_data")]))
    entries.append(launch_ros.actions.Node(package='deepracing_rclcpp', name='f1_tf_updater', executable='tf_updater', output='screen', parameters=[LaunchConfiguration(config_file.name), {"use_sim_time" : LaunchConfiguration(use_sim_time.name)}], remappings=[("motion_data", "/f1_game/motion_data"), ("session_data", "/f1_game/session_data")]))
    return LaunchDescription(entries)