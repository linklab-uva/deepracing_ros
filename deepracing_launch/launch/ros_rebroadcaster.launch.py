
import launch
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    # rebroadcasternode = launch_ros.actions.Node(package='deepracing_rclcpp', node_executable='ros_rebroadcaster', output='screen', node_name="f1_data_broadcaster")
    # return launch.LaunchDescription([rebroadcasternode,]) 
    use_intra_process_comms = True
    container = launch_ros.actions.ComposableNodeContainer(
        name='game_udp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{"thread_num" : 6}],
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='udp_driver',
                plugin='drivers::udp_driver::UdpReceiverNode',
                name='raw_udp_receiver_node',
                remappings=[('udp_read', 'all_udp')],
                parameters=[{'ip': "127.0.0.1", 'port': 20777}],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
            launch_ros.descriptions.ComposableNode(
                package='deepracing_rclcpp',
                plugin='deepracing::composable_nodes::UdpDemuxer',
                name='udp_demuxer_node',
                remappings=[('udp_in', 'all_udp')],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
            launch_ros.descriptions.ComposableNode(
                package='deepracing_rclcpp',
                plugin='deepracing::composable_nodes::ReceiveMotionData',
                name='motion_data_node',
                parameters=[{'all_cars': True}],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]),
            launch_ros.descriptions.ComposableNode(
                package='deepracing_rclcpp',
                plugin='deepracing::composable_nodes::ReceiveTelemetryData',
                name='telemetry_data_node',
                parameters=[{'all_cars': True}],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}])
        ],
        output='both',
    )
    return launch.LaunchDescription([container])