"""Launch the cpp_code executable in this package"""

import os

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    deepracing_launch_dir = get_package_share_directory("deepracing_launch")
    launch_dir = os.path.join(deepracing_launch_dir,"launch")
    config_dir = os.path.join(deepracing_launch_dir,"config")
    with_ekf = DeclareLaunchArgument("with_ekf", default_value="true")
    ekf_global = DeclareLaunchArgument("ekf_global", default_value="true")
    carname = DeclareLaunchArgument("carname", default_value="player1")
    index = DeclareLaunchArgument("index", default_value="-1")
    ghost_timescale = DeclareLaunchArgument("ghost_timescale", default_value="1.0")
    ekf_with_angvel = DeclareLaunchArgument("ekf_with_angvel", default_value="false")
    boundary_pub = DeclareLaunchArgument("boundary_pub", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    
    entries = [boundary_pub, carname, index, use_sim_time, with_ekf, ekf_global, ekf_with_angvel, ghost_timescale]

    ghost_ns = "ghost"
    ego_odom_topic = PathJoinSubstitution(["/", LaunchConfiguration(carname.name), "odom", "filtered"])
    entries.append(launch_ros.actions.Node(
        package='deepracing_rclpy', name='ghost_spawner', executable='ghost_spawner',
        output='screen',
        parameters=[{use_sim_time.name : LaunchConfiguration(use_sim_time.name), "timescale" : LaunchConfiguration(ghost_timescale.name)}],
        namespace=ghost_ns,
        remappings=[("ego_odom", ego_odom_topic)]
        )) 
    entries.append(launch_ros.actions.Node(
        package='deepracing_rclpy', name='ghost_curve_viz', executable='cbc_visualizer',
        output='screen',
        parameters=[{use_sim_time.name : LaunchConfiguration(use_sim_time.name)}],
        namespace=ghost_ns,
        remappings=[("curve_in", "target_prediction"), ("marker_out", "target_prediction/markers"), ("pc2_out", "target_prediction/pc2")]
        )) 
    entries.append(launch_ros.actions.Node(
        package='deepracing_rclcpp', name='measurement_publisher', executable='measurement_publisher_exe', output='screen', 
        parameters=[os.path.join(config_dir, "measurement_publisher.yaml"), 
                    {carname.name: LaunchConfiguration(carname.name), 
                    use_sim_time.name : LaunchConfiguration(use_sim_time.name), 
                    with_ekf.name : LaunchConfiguration(with_ekf.name), 
                    index.name : LaunchConfiguration(index.name)
                    }], 
        namespace=LaunchConfiguration(carname.name)))
    entries.append(launch_ros.actions.Node(
        package='deepracing_rclpy', name='ego_lateral_error_pub', executable='lateral_error_publisher',
        output='screen',
        parameters=[{use_sim_time.name : LaunchConfiguration(use_sim_time.name), "newton_iterations" : 3, "gpu" : -1}],
        namespace=LaunchConfiguration(carname.name),
        remappings=[("odom", "odom/filtered"),]
        )) 
    entries.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(os.path.join(launch_dir,"ekf.launch")),\
      launch_arguments=[("ekf_with_angvel", LaunchConfiguration(ekf_with_angvel.name)), 
                        ("ekf_global", LaunchConfiguration(ekf_global.name)), 
                        (carname.name, LaunchConfiguration(carname.name)), 
                        (use_sim_time.name, LaunchConfiguration(use_sim_time.name)), 
                        (index.name, LaunchConfiguration(index.name))
                        ], 
                        condition=IfCondition(LaunchConfiguration(with_ekf.name))))
    

    return LaunchDescription(entries)