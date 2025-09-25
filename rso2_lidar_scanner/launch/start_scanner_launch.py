import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory for the rplidar_ros package
    rplidar_ros_path = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_ros_path, 'launch', 'rplidar_a1_launch.py')

    # Subscriber node from the current package
    subscriber_node = Node(
        package='rso2_lidar_scanner',
        executable='scanner_subscriber',
        name='scanner_subscriber',
        output='screen'
    )

    # Include the rplidar launch file
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch)
    )

    return LaunchDescription([
        rplidar_node,
        subscriber_node
    ])