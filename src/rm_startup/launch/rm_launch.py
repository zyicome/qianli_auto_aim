import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

def generate_launch_description():
    # Load parameters
    node_params = os.path.join(get_package_share_directory('rm_startup'), 'config', 'rm_node_parameters.yaml')

    launch_params_path = os.path.join(get_package_share_directory('rm_startup'), 'config', 'rm_launch_parameters.yaml')
    launch_params = yaml.safe_load(open(launch_params_path))

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    
    #Create camera node
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if launch_params.get('camera') == 'hik_camera':
        camera_node = hik_camera_node
    elif (launch_params.get('camera') == 'mv_camera'):
        camera_node = mv_camera_node

    detector_node = ComposableNode(
        package='rm_armor_detector',
        plugin='rm_armor_detector::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node,
            detector_node,
        ],
        output='both',
    )
    print(container)

    return LaunchDescription([
        container,
    ])