from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_robot',
            executable='face_gesture_node',
            name='face_gesture_node'
        ),
        Node(
            package='vision_robot',
            executable='person_recognition_node',
            name='person_recognition_node'
        ),
        Node(
            package='vision_robot',
            executable='speech_node',
            name='speech_node'
        ),
        Node(
            package='vision_robot',
            executable='navigation_controller',
            name='navigation_controller'
        )
    ])
