from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_robot',
            executable='face_gesture_node.py',
            name='face_gesture_node'
        ),
        Node(
            package='vision_robot',
            executable='person_recognition_node.py',
            name='person_recognition_node'
        ),
        Node(
            package='vision_robot',
            executable='speech_node.py',
            name='speech_node'
        ),
        Node(
            package='vision_robot',
            executable='navigation_controller.py',
            name='navigation_controller'
        ),
        Node(
            package='vision_robot',
            executable='face_enrollment_node.py',
            name='face_enrollment_node'
        ),
        Node(
            package='vision_robot',
            executable='speech_recognition_node.py',
            name='speech_recognition_node'
        ),
        Node(
            package='vision_robot',
            executable='llm_node.py',
            name='llm_node'
        )
    ])
