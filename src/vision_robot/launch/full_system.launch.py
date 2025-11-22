from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('vision_robot')
    
    return LaunchDescription([
        Node(
            package='vision_robot',
            executable='face_gesture_node.py',
            name='face_gesture_node',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='person_recognition_node.py',
            name='person_recognition_node',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='speech_node.py',
            name='speech_node',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='navigation_controller.py',
            name='navigation_controller',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='face_enrollment_node.py',
            name='face_enrollment_node',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='speech_recognition_node.py',
            name='speech_recognition_node',
            output='screen'
        ),
        Node(
            package='vision_robot',
            executable='llm_node.py',
            name='llm_node',
            output='screen'
        )
    ])
