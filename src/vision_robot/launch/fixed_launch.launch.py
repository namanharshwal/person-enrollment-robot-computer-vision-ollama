from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Face and gesture detection
        ExecuteProcess(
            cmd=['python3', os.path.join(os.getcwd(), 'scripts/face_gesture_node.py')],
            name='face_gesture_node',
            output='screen'
        ),
        
        # Person recognition
        ExecuteProcess(
            cmd=['python3', os.path.join(os.getcwd(), 'scripts/person_recognition_node.py')],
            name='person_recognition_node',
            output='screen'
        ),
        
        # Speech synthesis
        ExecuteProcess(
            cmd=['python3', os.path.join(os.getcwd(), 'scripts/speech_node.py')],
            name='speech_node',
            output='screen'
        ),
        
        # Navigation controller
        ExecuteProcess(
            cmd=['python3', os.path.join(os.getcwd(), 'scripts/navigation_controller.py')],
            name='navigation_controller',
            output='screen'
        )
    ])
