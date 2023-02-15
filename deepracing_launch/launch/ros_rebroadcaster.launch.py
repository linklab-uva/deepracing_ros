
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.descriptions
import launch.conditions

def generate_launch_description():
    argz = []
    ip = launch.actions.DeclareLaunchArgument("ip", default_value="127.0.0.1")
    argz.append(ip)
    port = launch.actions.DeclareLaunchArgument("port", default_value="20777")
    argz.append(port)
    carname = launch.actions.DeclareLaunchArgument("carname")
    argz.append(carname)
    allcars = launch.actions.DeclareLaunchArgument("publish_all_cars", default_value="false")
    argz.append(allcars)
    composable_api = launch.actions.DeclareLaunchArgument("composable_api", default_value="true")
    argz.append(composable_api)
    node = launch_ros.actions.Node(
        package='deepracing_rclcpp', 
        executable='ros_rebroadcaster', 
        name="f1_data_broadcaster",
        namespace=launch.substitutions.LaunchConfiguration(carname.name),
        output='screen', 
        parameters=[{
                'hostname': launch.substitutions.LaunchConfiguration(ip.name), 
                port.name: launch.substitutions.LaunchConfiguration(port.name),
                allcars.name: launch.substitutions.LaunchConfiguration(allcars.name)
            }],
        condition=launch.conditions.UnlessCondition(launch.substitutions.LaunchConfiguration(composable_api.name))
        )
    use_intra_process_comms = True
    composable_nodez = [
        launch_ros.descriptions.ComposableNode(
            package='udp_driver',
            plugin='drivers::udp_driver::UdpReceiverNode',
            name='raw_udp_receiver_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            remappings=[('udp_read', 'all_raw_udp')],
            parameters=[{'ip': launch.substitutions.LaunchConfiguration(ip.name), 'port': launch.substitutions.LaunchConfiguration(port.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::UdpDemuxer',
            name='udp_demuxer_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            remappings=[('udp_in', 'all_raw_udp')],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveCarSetupData',
            name='car_setup_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            parameters=[{'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveCarStatusData',
            name='car_status_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            parameters=[{'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveLapData',
            name='lap_data_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            parameters=[{'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveMotionData',
            name='motion_data_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            parameters=[{'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveSessionData',
            name='session_data_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
        launch_ros.descriptions.ComposableNode(
            package='deepracing_rclcpp',
            plugin='deepracing::composable_nodes::ReceiveTelemetryData',
            name='telemetry_data_node',
            namespace=launch.substitutions.LaunchConfiguration(carname.name),
            parameters=[{'all_cars': launch.substitutions.LaunchConfiguration(allcars.name)}],
            extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}])
    ]
    container = launch_ros.actions.ComposableNodeContainer(
        name='game_udp_container',
        namespace=launch.substitutions.LaunchConfiguration(carname.name),
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{"thread_num" : len(composable_nodez)+1}],
        composable_node_descriptions=composable_nodez,
        output='both',
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration(composable_api.name))
    )
    nodez = []
    nodez.append(launch_ros.actions.Node(package='deepracing_rclpy', name='initialize_udp_receiver', executable='initialize_udp_receiver', output='screen', namespace=launch.substitutions.LaunchConfiguration(carname.name)))
    
    return launch.LaunchDescription(argz + nodez + [node, container])