from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess

def generate_launch_description():
    left = "fsds_left"
    right = "fsds_right"
    
    cam_info_left = PathJoinSubstitution([FindPackageShare('perception'), 'config',
                                                  left+'.yaml'])
    cam_info_right = PathJoinSubstitution([FindPackageShare('perception'), 'config',
                                                  right+'.yaml'])
    return LaunchDescription([
        Node(
            package='perception',
            executable='position_estimator.py',
            name='position_estimator',
            output='screen',
            remappings=[
                ('camera/left', '/fsds/cameracam2/image_color'),
                ('camera/right', '/fsds/cameracam1/image_color')
            ]
        ),
        LaunchArg('cam_info_left',default_value=['file://', cam_info_left],
                  description='camera left info with intrinsics and distortion matrix'
        ),
        LaunchArg('cam_info_right', default_value=['file://', cam_info_right], 
                  description='camera left info with intrinsics and distortion matrix'
                  ),
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                    '--yaw', '-1.570796327',
                    '--roll', '-1.5707963270',
                    '--pitch', '0',
                    '--frame-id', '/fsds/cam2',
                    '--child-frame-id', '/down'],
            output='screen'
        )
        
    ])
