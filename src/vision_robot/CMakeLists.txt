cmake_minimum_required(VERSION 3.8)
project(vision_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonInfo.msg"
)

# Install Python scripts
install(PROGRAMS
  scripts/face_gesture_node.py
  scripts/person_recognition_node.py
  scripts/speech_node.py
  scripts/navigation_controller.py
  scripts/face_enrollment_node.py
  scripts/speech_recognition_node.py  # New node
  scripts/llm_node.py                 # New node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install known_faces directory
install(
  DIRECTORY known_faces
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
