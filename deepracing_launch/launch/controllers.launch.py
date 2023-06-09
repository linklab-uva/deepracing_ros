import os
import launch_ros.actions
import launch_ros
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import json

def generate_launch_description():
    deepracing_launch_dir : str = get_package_share_directory("deepracing_launch")
    deepracing_launch_config_dir = os.path.join(deepracing_launch_dir,"config")
    deepracing_launch_data_dir = os.path.join(deepracing_launch_dir,"data")

    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    carname = DeclareLaunchArgument("carname")
    entries = [use_sim_time, carname]

    entries.append(launch_ros.actions.Node(package='deepracing_rclcpp', name='velocity_controller', executable='velocity_control_node', output='screen',\
        parameters=[os.path.join(deepracing_launch_config_dir, "pid_gains.yaml"), {use_sim_time.name : LaunchConfiguration(use_sim_time.name)}],\
        namespace=LaunchConfiguration(carname.name),\
        remappings=[("pid_state", "velocity_pid_state"), ("accel_in", "accel/filtered"), ("odom_in", "odom/filtered")])\
        )
    
    with open(os.path.join(deepracing_launch_data_dir, "vigem_calibration.json"), "r") as f:
        vigem_dict : dict = json.load(f)
    xinput_vals = list(vigem_dict["xinput_values"])
    steering_wheel_angles = list(vigem_dict["steering_wheel_angles"])
    entries.append(launch_ros.actions.Node(package='deepracing_rclpy',
                                           name='control_to_xinput', 
                                           executable='control_to_xinput', 
                                           output='screen', 
                                           parameters=[{"control_values" : xinput_vals, "steering_angles" : steering_wheel_angles, use_sim_time.name : LaunchConfiguration(use_sim_time.name)}], 
                                           namespace=LaunchConfiguration(carname.name)))
    
    return LaunchDescription(entries)