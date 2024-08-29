import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

def generate_launch_description():
    detector_node = ComposableNode(
        package='rm_armor_detector',
        plugin='rm_armor_detector::ArmorDetectorNode',
        name='armor_detector',
        parameters=[os.path.join(get_package_share_directory('rm_armor_detector'), 'config', 'detector_parameters.yaml')]
    )

    container = ComposableNodeContainer(
        name='image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            detector_node
        ],
        output='both',
    )
    print(container)

    return LaunchDescription([
        container
    ])