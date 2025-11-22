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

# Install Python scripts WITHOUT .py extension using a loop
set(SCRIPT_NAMES
  face_gesture_node
  person_recognition_node
  speech_node
  navigation_controller
  face_enrollment_node
  speech_recognition_node
  llm_node
)

foreach(SCRIPT_NAME ${SCRIPT_NAMES})
  install(PROGRAMS
    scripts/${SCRIPT_NAME}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${SCRIPT_NAME}
  )
endforeach()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install known_faces directory (if it exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/known_faces")
  install(
    DIRECTORY known_faces
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
