"""Launch the cpp_code executable in this package"""

import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    deepracing_launch_dir = get_package_share_directory("deepracing_launch")
    launch_dir = os.path.join(deepracing_launch_dir,"launch")
    config_dir = os.path.join(deepracing_launch_dir,"config")
    config_file = DeclareLaunchArgument("config_file", default_value=os.path.join(config_dir, "tf_updater.yaml"))
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    with_ekf = DeclareLaunchArgument("with_ekf", default_value="true")
    boundary_pub = DeclareLaunchArgument("boundary_pub", default_value="true")
    ekf_global = DeclareLaunchArgument("ekf_global", default_value="true")
    ekf_with_angvel = DeclareLaunchArgument("ekf_with_angvel", default_value="false")
    carname = DeclareLaunchArgument("carname", default_value="")
    
    entries = [boundary_pub, carname, config_file, use_sim_time, with_ekf, ekf_global, ekf_with_angvel]

    entries.append(launch_ros.actions.Node(package='deepracing_rclpy', name='f1_boundary_publisher', executable='boundary_publisher', output='screen',\
         parameters=[{"track_search_dirs": os.getenv("F1_TRACK_DIRS","").split(os.pathsep), use_sim_time.name : LaunchConfiguration(use_sim_time.name)}],\
           condition=IfCondition(LaunchConfiguration(boundary_pub.name)), namespace=LaunchConfiguration(carname.name)))
    entries.append(launch_ros.actions.Node(package='deepracing_rclpy', name='vehicle_state_publisher', executable='vehicle_state_publisher', output='screen', parameters=[{use_sim_time.name : LaunchConfiguration(use_sim_time.name)}], namespace=LaunchConfiguration(carname.name)))

    
    entries.append(launch_ros.actions.Node(package='deepracing_rclcpp', name='f1_tf_updater', executable='tf_updater', output='screen', parameters=[LaunchConfiguration(config_file.name), {carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name), with_ekf.name : LaunchConfiguration(with_ekf.name)}], namespace=LaunchConfiguration(carname.name)))
    entries.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(os.path.join(launch_dir,"ekf.launch")),\
      launch_arguments=[("with_angvel", LaunchConfiguration(ekf_with_angvel.name)), ("global", LaunchConfiguration(ekf_global.name)), (carname.name, LaunchConfiguration(carname.name)), (use_sim_time.name, LaunchConfiguration(use_sim_time.name))], condition=IfCondition(LaunchConfiguration(with_ekf.name))))
    return LaunchDescription(entries)