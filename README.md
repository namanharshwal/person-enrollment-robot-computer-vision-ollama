# person-enrollment-robot-computer-vision-ollama
# Person Enrollment Robot - Computer Vision with Ollama

A ROS2-based robotics system for person enrollment, face recognition, gesture detection, and conversational AI using computer vision and LLM integration.

## 🎯 Features

- **Face Enrollment**: Automatically enrolls new people by capturing 15 face images
- **Face Recognition**: Identifies enrolled persons using computer vision
- **Hand Gesture Detection**: Recognizes thumbs up and pointing gestures
- **Speech Recognition**: Voice command processing
- **Speech Synthesis**: Text-to-speech responses
- **LLM Integration**: Conversational AI using Ollama
- **Robot Navigation**: Autonomous navigation control
- **Real-time Processing**: Live camera feed processing

## 📋 System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Python**: 3.10+
- **Camera**: USB or built-in webcam
- **Microphone**: For speech recognition
- **RAM**: 8GB minimum (16GB recommended for LLM)
- **GPU**: Optional (accelerates YOLOv8 and PyTorch)

## 🚀 Quick Start

### 1. Install Dependencies
Install ROS2 Humble
sudo apt install ros-humble-desktop

Install system packages
sudo apt install -y python3-pip python3-colcon-common-extensions
ros-humble-cv-bridge libopencv-dev portaudio19-dev

Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama2

text

### 2. Setup Workspace
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/namanharshwal/person-enrollment-robot-computer-vision-ollama.git
cd ~/robot_ws
pip3 install opencv-python mediapipe ultralytics torch numpy dlib
pygame gtts pyaudio SpeechRecognition requests transformers
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

text

### 3. Run the System
Launch all nodes
ros2 launch vision_robot main_launch.py

OR launch individually
ros2 run vision_robot face_gesture_node
ros2 run vision_robot person_recognition_node
ros2 run vision_robot face_enrollment_node

text

## 📦 Package Structure

vision_robot/
├── scripts/ # ROS2 Python nodes
│ ├── face_enrollment_node.py # Person enrollment
│ ├── face_gesture_node.py # Gesture detection
│ ├── person_recognition_node.py # Face recognition
│ ├── speech_node.py # TTS
│ ├── speech_recognition_node.py # STT
│ ├── llm_node.py # Ollama integration
│ └── navigation_controller.py # Robot control
├── msg/ # Custom messages
│ └── PersonInfo.msg # Person data message
├── launch/ # Launch files
├── known_faces/ # Enrolled face database
├── package.xml # Dependencies
├── setup.py # Python setup
└── CMakeLists.txt # Build config

text

## 🔧 Configuration

### Camera Settings
Edit `face_gesture_node.py`:
self.cap = cv2.VideoCapture(0) # Change 0 to camera index

text

### LLM Model
Edit `llm_node.py`:
model = "llama2" # Change to your preferred Ollama model

text

### Face Enrollment Settings
Edit `face_enrollment_node.py`:
self.capture_count < 15 # Number of images to capture
current_time - self.last_capture > 0.2 # Capture interval (5fps)

text

## 📡 ROS2 Topics

### Published Topics
- `/processed_image` (sensor_msgs/Image) - Processed camera feed
- `/person_info` (vision_robot/PersonInfo) - Detected person data
- `/gesture_command` (std_msgs/String) - Hand gestures
- `/speech_output` (std_msgs/String) - TTS text
- `/request_speech` (std_msgs/String) - Speech requests

### Subscribed Topics
- `/voice_command` (std_msgs/String) - Voice input
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed

## 🎮 Usage Examples

### Enrolling a New Person
1. Stand in front of camera
2. Robot detects "Unknown" person
3. Say "My name is John" when prompted
4. System captures 15 face images
5. Robot greets: "It's a pleasure to meet you John"

### Gesture Commands
- **Thumbs Up**: Approval/acknowledgment
- **Pointing**: Direction/selection

### Voice Interaction
Say: "My name is Alice"
Say: "Call me Bob"
Say: "I am Charlie"

## 🐛 Troubleshooting

### Camera Issues
List cameras
v4l2-ctl --list-devices

Test camera
ros2 run vision_robot face_gesture_node

text

### Missing Dependencies
pip3 install --upgrade opencv-python mediapipe ultralytics

text

### Ollama Not Found
curl -fsSL https://ollama.com/install.sh | sh
ollama serve # Start Ollama server
ollama pull llama2

text

### Permission Denied (Microphone)
sudo usermod -a -G audio $USER

Logout and login again
text

## 📊 Performance Optimization

### For CPU-only Systems
In person_recognition_node.py
device = 'cpu' # Set explicitly

text

### For GPU Systems
Install CUDA-enabled PyTorch
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118

text

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Open pull request

## 📄 License

Apache License 2.0 - See LICENSE file

## 👤 Author

**Naman Harshwal**
- GitHub: [@namanharshwal](https://github.com/namanharshwal)
- Email: 142438943+Naman-Harshwal@users.noreply.github.com

## 🙏 Acknowledgments

- ROS2 Humble community
- MediaPipe by Google
- Ultralytics YOLOv8
- Ollama LLM framework
- OpenCV contributors

## 📚 Documentation

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [MediaPipe](https://google.github.io/mediapipe/)
- [Ollama](https://ollama.com/)
- [YOLOv8](https://docs.ultralytics.com/)

---

**Status**: Active Development
**Version**: 0.0.0
**Last Updated**: November 2025
