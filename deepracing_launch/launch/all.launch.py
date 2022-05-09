import os
import launch_ros.actions
import launch_ros
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    deepracing_launch_dir : str = get_package_share_directory("deepracing_launch")
    deepracing_launch_config_dir = os.path.join(deepracing_launch_dir,"config")
    deepracing_launch_launch_dir = os.path.join(deepracing_launch_dir,"launch")

    includez = []
    nodez = []
    argz = []
    
    boundary_pub = DeclareLaunchArgument("boundary_pub", default_value="true")
    argz.append(boundary_pub)
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)
    tf_from_odom = DeclareLaunchArgument("tf_from_odom", default_value="false")
    argz.append(tf_from_odom)
    ns = DeclareLaunchArgument("ns", default_value="")
    argz.append(ns)
    hostname = DeclareLaunchArgument("hostname", default_value="127.0.0.1")
    argz.append(hostname)
    port = DeclareLaunchArgument("port", default_value="20777")
    argz.append(port)
    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "utilities.launch.py"]))\
                    ,launch_arguments=list({boundary_pub.name: LaunchConfiguration(boundary_pub.name), ns.name: LaunchConfiguration(ns.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name), tf_from_odom.name : LaunchConfiguration(tf_from_odom.name)}.items())))

    includez.append( launch_ros.actions.Node(package='deepracing_rclcpp', name='rebroadcaster_node', executable='ros_rebroadcaster', output='screen', namespace=LaunchConfiguration(ns.name), parameters=[{port.name : LaunchConfiguration(port.name), hostname.name : LaunchConfiguration(hostname.name)}]) )
    includez.append( launch_ros.actions.Node(package='deepracing_rclpy', name='oracle_path_server', executable='oracle_path_server', output='screen', namespace=LaunchConfiguration(ns.name) ) )
    includez.append( launch_ros.actions.Node(package='deepracing_rclpy', name='pure_pursuit_node', executable='bezier_curve_pure_pursuit', output='screen', namespace=LaunchConfiguration(ns.name), remappings=[("beziercurves_in", "oraclebeziercurves")] ) )

    return LaunchDescription(argz + includez + nodez)
    