from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Perception',
            executable='position_estimator',
            name='position_estimator',
            output='screen',
            remappings=[
                ('camera/left', '/fsds/cameracam2/image_color'),
                ('camera/right', '/fsds/cameracam1/image_color')
            ]
        )
    ])