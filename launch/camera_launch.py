from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')


    RealSense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("realsense2_camera"), 
                'launch', 
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30'
        }.items()
    )
    
    PavelRS = Node(
            package='ros2_realsense',
            executable='convert_image_raw',
            name='pavel_node',
                arguments=[
                    '--ros-args',
                    '-p', 'topic_color:=/camera/camera/color/image_raw',
                    '-p', 'topic_depth:=/camera/camera/depth/image_rect_raw',
                    '-p', 'topic_joy:=/joy',
                    '-p', 'use_controller:=false'
                ],
            output="screen"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        RealSense,
        PavelRS,
    ])