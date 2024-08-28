from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_tracking',
            executable='realsense_pose_node',
            name='realsense_pose_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='iAHRS_node',
            name='iAHRS_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='dynamixel_node',
            name='dynamixel_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='realtime_audio_pc_node',
            name='realtime_audio_pc_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='witmotion_imu_node',
            name='witmotion_imu_node',
            output='screen'
        ),


        Node(
            package='gps_navigation',
            executable='gps_listener_node',
            name='gps_listener_node',
            output='screen'
        ),
        Node(
            package='gps_navigation',
            executable='current_gps_node',
            name='current_gps_node',
            output='screen'
        ),
        Node(
            package='gps_navigation',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        )
    ])

