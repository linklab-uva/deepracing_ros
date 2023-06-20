
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
    argz = []
    ego_index = launch.actions.DeclareLaunchArgument("ego_index", default_value="-1")
    argz.append(ego_index)
    secondary_index = launch.actions.DeclareLaunchArgument("secondary_index", default_value="-2")
    argz.append(secondary_index)
    use_sim_time = launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)

    nodez = []
    
    nodez.append(launch_ros.actions.Node(package='deepracing_rclpy', name='f1_boundary_publisher', executable='boundary_publisher', output='screen',\
         parameters=[{ use_sim_time.name : LaunchConfiguration(use_sim_time.name) }]))  
    
    nodez.append(launch_ros.actions.Node(package='deepracing_rclpy', 
                                           name='vehicle_state_publisher', 
                                           executable='vehicle_state_publisher', 
                                           output='screen', 
                                           parameters=[{
                                              use_sim_time.name : LaunchConfiguration(use_sim_time.name),
                                              ego_index.name : LaunchConfiguration(ego_index.name),
                                              secondary_index.name : LaunchConfiguration(secondary_index.name)
                                              }]
                                           )
                                           )
    
    return launch.LaunchDescription(argz +  nodez)