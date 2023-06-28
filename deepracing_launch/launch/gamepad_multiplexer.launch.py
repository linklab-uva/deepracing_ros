
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.descriptions
import ament_index_python
import os

def generate_launch_description():
    argz = []
    share_path : str = ament_index_python.get_package_share_directory("deepracing_launch")
    param_path = os.path.join(share_path, "config")
    nodez = [
        launch_ros.actions.Node(
            package='deepracing_rclcpp',
            executable="gamepad_multiplexer_node_exe",
            name="gamepad_multiplexer_node",
            parameters=[os.path.join(param_path, "controllers.yaml")]
        ),
    ]
    return launch.LaunchDescription(argz + nodez)