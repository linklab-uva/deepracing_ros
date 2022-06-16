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
    carname = DeclareLaunchArgument("carname", default_value="")
    argz.append(carname)
    default_trackfile = DeclareLaunchArgument("default_trackfile", default_value="")
    argz.append(default_trackfile)
    rate = DeclareLaunchArgument("rate", default_value="10.0")
    argz.append(rate)
    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "utilities.launch.py"]))\
                    ,launch_arguments=list({boundary_pub.name: LaunchConfiguration(boundary_pub.name), carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name), tf_from_odom.name : LaunchConfiguration(tf_from_odom.name)}.items())))
    includez.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "pure_pursuit_oracle.launch"]))\
                    ,launch_arguments=list({rate.name: LaunchConfiguration(rate.name), default_trackfile.name: LaunchConfiguration(default_trackfile.name), carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name), tf_from_odom.name : LaunchConfiguration(tf_from_odom.name)}.items())))

    return LaunchDescription(argz + includez + nodez)
    