
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.descriptions
import ament_index_python
import os

def generate_launch_description():
    argz = []
    use_sim_time = launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)
    num_players = launch.actions.DeclareLaunchArgument("num_players")
    argz.append(num_players)
    share_path : str = ament_index_python.get_package_share_directory("deepracing_launch")
    param_path = os.path.join(share_path, "config")
    nodez = [
        launch_ros.actions.Node(
            package='deepracing_rclcpp',
            executable="multiagent_control_node_exe",
            name="multiagent_control_node",
            parameters=[
                os.path.join(param_path, "controllers.yaml"), 
                {
                    use_sim_time.name: launch.substitutions.LaunchConfiguration(use_sim_time.name),
                    num_players.name: launch.substitutions.LaunchConfiguration(num_players.name)
                }
            ]
        ),
    ]
    return launch.LaunchDescription(argz + nodez)