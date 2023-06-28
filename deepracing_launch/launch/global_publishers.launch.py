
import launch
import launch.actions
import launch_ros.actions
import launch.substitutions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # rebroadcasternode = launch_ros.actions.Node(package='deepracing_rclcpp', node_executable='ros_rebroadcaster', output='screen', node_name="f1_data_broadcaster")
    # return launch.LaunchDescription([rebroadcasternode,]) 
    deepracing_launch_dir = get_package_share_directory("deepracing_launch")
    deepracing_launch_launch_dir = os.path.join(deepracing_launch_dir, "launch")

    argz = []

    ip = launch.actions.DeclareLaunchArgument("ip", default_value="127.0.0.1")
    argz.append(ip)
    port = launch.actions.DeclareLaunchArgument("port", default_value="20777")
    argz.append(port)
    allcars = launch.actions.DeclareLaunchArgument("publish_all_cars", default_value="false")
    argz.append(allcars)

    ego_index = launch.actions.DeclareLaunchArgument("ego_index", default_value="-1")
    argz.append(ego_index)
    secondary_index = launch.actions.DeclareLaunchArgument("secondary_index", default_value="-2")
    argz.append(secondary_index)

    use_sim_time = launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)
    

    includez = []
    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "ros_rebroadcaster.launch.py"]))\
                    ,launch_arguments=list({ allcars.name : LaunchConfiguration(allcars.name), ip.name : LaunchConfiguration(ip.name), port.name : LaunchConfiguration(port.name)}.items())))

    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "drivers_and_boundaries.launch.py"]))\
                    ,launch_arguments=list({ego_index.name : LaunchConfiguration(ego_index.name), secondary_index.name : LaunchConfiguration(secondary_index.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name)}.items())))

                                           
    return launch.LaunchDescription(argz + includez)