"""Launch the cpp_code executable in this package"""

import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    deepracing_launch_dir = get_package_share_directory("deepracing_launch")
    launch_dir = os.path.join(deepracing_launch_dir,"launch")
    with_ekf = DeclareLaunchArgument("with_ekf", default_value="true")
    ekf_global = DeclareLaunchArgument("ekf_global", default_value="true")
    ekf_with_angvel = DeclareLaunchArgument("ekf_with_angvel", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    ekf_frequency = DeclareLaunchArgument("ekf_frequency", default_value="50.0")
    
    entries = [use_sim_time, with_ekf, ekf_global, ekf_with_angvel, ekf_frequency]


    
    #Add or remove from this list as-needed
    cars = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
    for idx in cars:
        entries.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir,"utilities.launch.py")),\
        launch_arguments=[(ekf_with_angvel.name, LaunchConfiguration(ekf_with_angvel.name)), 
                          (ekf_global.name, LaunchConfiguration(ekf_global.name)),
                          (use_sim_time.name, LaunchConfiguration(use_sim_time.name)),  
                          (ekf_frequency.name, LaunchConfiguration(ekf_frequency.name)),  
                          ("carname", "car%s" % idx), 
                          ("index", str(idx)), 
                        ]
        ))
    

    return LaunchDescription(entries)