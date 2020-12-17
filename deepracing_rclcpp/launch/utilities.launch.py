"""Launch the cpp_code executable in this package"""

from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='deepracing_rclcpp', executable='boundary_publisher', output='screen', parameters=[{"track_search_dirs": os.getenv("F1_TRACK_DIRS","").split(os.pathsep)}]),
        launch_ros.actions.Node(package='deepracing_rclcpp', executable='tf_updater', output='screen')
    ])