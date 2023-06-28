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
    
    boundary_pub = DeclareLaunchArgument("boundary_pub", default_value="false")
    argz.append(boundary_pub)
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)
    with_ekf = DeclareLaunchArgument("with_ekf", default_value="true")
    argz.append(with_ekf)
    carname = DeclareLaunchArgument("carname", default_value="player1")
    argz.append(carname)
    index = DeclareLaunchArgument("index", default_value="-1")
    argz.append(index)
    default_trackfile = DeclareLaunchArgument("default_trackfile", default_value="")
    argz.append(default_trackfile)
    rate = DeclareLaunchArgument("rate", default_value="10.0")
    argz.append(rate)
    ekf_global = DeclareLaunchArgument("ekf_global", default_value="true")
    argz.append(ekf_global)
    ekf_with_angvel = DeclareLaunchArgument("ekf_with_angvel", default_value="false")
    argz.append(ekf_with_angvel)
    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "utilities.launch.py"]))\
                    ,launch_arguments=list({index.name: LaunchConfiguration(index.name), ekf_with_angvel.name: LaunchConfiguration(ekf_with_angvel.name), ekf_global.name: LaunchConfiguration(ekf_global.name), boundary_pub.name: LaunchConfiguration(boundary_pub.name), carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name), with_ekf.name : LaunchConfiguration(with_ekf.name)}.items())))
    includez.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "pure_pursuit_oracle.launch"]))\
                    ,launch_arguments=list({rate.name: LaunchConfiguration(rate.name), default_trackfile.name: LaunchConfiguration(default_trackfile.name), carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name)}.items())))
    includez.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([deepracing_launch_launch_dir, "controllers.launch.py"]))\
                    ,launch_arguments=list({carname.name: LaunchConfiguration(carname.name), use_sim_time.name : LaunchConfiguration(use_sim_time.name)}.items())))
    
    return LaunchDescription(argz + includez + nodez)
    