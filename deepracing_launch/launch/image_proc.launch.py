
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::CropDecimateNode',
            name='crop_node',
            # Remap subscribers and publishers
            remappings=[
                
                ('in/image_raw', '/f1_game/images'),
                ('in/camera_info', '/f1_game/camera_info'),
                
                ('out/image_raw', '/f1_game/cropped/images'),
                ('out/image_raw/compressed', '/f1_game/cropped/images/compressed'),
                ('out/image_raw/compressedDepth', '/f1_game/cropped/images/compressedDepth'),
                ('out/camera_info', '/f1_game/cropped/camera_info')
            ],
            parameters=
            [
            {
                "width": 1758,
                "height": 360,
                "offset_y": 32
            }
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::ResizeNode',
            name='resize_node',
            # Remap subscribers and publishers
            remappings=[
                ('image', '/f1_game/cropped/images'),
                ('camera_info', '/f1_game/cropped/camera_info'),

                ('resize', '/f1_game/resized/images'),
                ('resize/camera_info', '/f1_game/resized/camera_info'),
                ('resize/compressed', '/f1_game/resized/compressed'),
                ('resize/compressedDepth', '/f1_game/resized/compressedDepth')
            ],
            parameters=
            [
            {
                "interpolation": 3,
                "height": 66,
                "width": 200,
                "use_scale": False
            }
            ],
        )
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return LaunchDescription([
        arg_container,
        image_processing_container,
        load_composable_nodes,
    ])
