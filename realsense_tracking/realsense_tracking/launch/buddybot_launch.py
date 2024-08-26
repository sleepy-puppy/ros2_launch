from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_tracking',
            executable='1_realsense_pose.py',
            name='realsense_pose_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='2_iAHRS.py',
            name='iAHRS_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='3_dynamixel.py',
            name='dynamixel_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='4_realtime_audio_pc.py',
            name='realtime_audio_pc_node',
            output='screen'
        ),
        Node(
            package='realsense_tracking',
            executable='5_witmotion_imu_ros.py',
            name='witmotion_imu_node',
            output='screen'
        ),


        Node(
            package='gps_navigation',
            executable='openrouteservice_navigation_api.py',
            name='gps_listener_node',
            output='screen'
        ),
        Node(
            package='gps_navigation',
            executable='zed_f9p.py',
            name='current_gps_node',
            output='screen'
        ),
        Node(
            package='gps_navigation',
            executable='purepursuit.py',
            name='pure_pursuit_node',
            output='screen'
        )
    ])

