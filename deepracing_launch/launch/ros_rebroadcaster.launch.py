
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # rebroadcasternode = launch_ros.actions.Node(package='deepracing_rclcpp', node_executable='ros_rebroadcaster', output='screen', node_name="f1_data_broadcaster")
    # return launch.LaunchDescription([rebroadcasternode,]) 
    deepracing_launch_dir = get_package_share_directory("deepracing_launch")
    config_dir = os.path.join(deepracing_launch_dir,"config")

    use_intra_process_comms = True
    argz = []
    ip = launch.actions.DeclareLaunchArgument("ip", default_value="127.0.0.1")
    argz.append(ip)
    port = launch.actions.DeclareLaunchArgument("port", default_value="20777")
    argz.append(port)
    allcars = launch.actions.DeclareLaunchArgument("publish_all_cars", default_value="false")
    argz.append(allcars)
    use_sim_time = launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false")
    argz.append(use_sim_time)

    search_dirs = []
    search_dirs_env = os.getenv("F1_MAP_DIRS", None)
    if search_dirs_env is not None:
        search_dirs.extend(search_dirs_env.split(os.pathsep))
    search_dirs.append(os.path.join(deepracing_launch_dir,"maps"))
    composable_nodez = [
        launch_ros.descriptions.ComposableNode(
            package='udp_driver',
            plugin='drivers::udp_driver::UdpReceiverNode',
            name='raw_udp_receiver_node',
            remappings=[('udp_read', '_all_udp_in')],
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         ip.name : launch.substitutions.LaunchConfiguration(ip.name), 
                         port.name : launch.substitutions.LaunchConfiguration(port.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::UdpDemuxer',
            name='udp_demuxer_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveCarSetupData',
            name='car_setup_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveCarStatusData',
            name='car_status_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveLapData',
            name='lap_data_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveMotionData',
            name='motion_data_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveMotionExData',
            name='motion_data_ex_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveSessionData',
            name='session_data_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveTelemetryData',
            name='telemetry_data_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::TrackmapPublisher',
            name='trackmap_publisher_node',
            parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                         'search_dirs': search_dirs}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}])
    ]
    container = launch_ros.actions.ComposableNodeContainer(
        name='game_udp_container',
        namespace='udp_interface',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                     "thread_num" : len(composable_nodez)+1}],
        composable_node_descriptions=composable_nodez,
        output='both'
    )
    nodez = []
    nodez.append(launch_ros.actions.Node(package='deepracing_rclpy', 
                                         name='initialize_udp_receiver', 
                                         executable='initialize_lifecycle_node', 
                                         namespace="udp_interface",
                                         parameters=[{use_sim_time.name : launch.substitutions.LaunchConfiguration(use_sim_time.name), 
                                                      "node_to_initialize" : "raw_udp_receiver_node"}],
                                         output='screen'))
    
    return launch.LaunchDescription(argz + nodez + [container,])