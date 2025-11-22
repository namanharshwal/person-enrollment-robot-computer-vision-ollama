from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vision_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include msg files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        # Include known_faces directory
        (os.path.join('share', package_name), ['known_faces']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='comp-point',
    maintainer_email='142438943+Naman-Harshwal@users.noreply.github.com',
    description='ROS2 computer vision package for mobile robot with face recognition and person identification',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_gesture_node = vision_robot.scripts.face_gesture_node:main',
            'person_recognition_node = vision_robot.scripts.person_recognition_node:main',
            'speech_node = vision_robot.scripts.speech_node:main',
            'navigation_controller = vision_robot.scripts.navigation_controller:main',
            'face_enrollment_node = vision_robot.scripts.face_enrollment_node:main',
            'speech_recognition_node = vision_robot.scripts.speech_recognition_node:main',  # New
            'llm_node = vision_robot.scripts.llm_node:main',  # New
        ],
    },
)
