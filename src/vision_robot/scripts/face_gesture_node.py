#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Add for gesture commands

import sys
import os

# Fix MediaPipe protobuf issues
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

# Add package directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

class FaceGestureNode(Node):
    def __init__(self):
        super().__init__('face_gesture_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'processed_image', 10)
        
        # Add publisher for gesture commands
        self.gesture_pub = self.create_publisher(String, 'gesture_command', 10)
        
        self.cap = cv2.VideoCapture(0)
        
        # MediaPipe setup
        self.mp_face = mp.solutions.face_detection
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.face_detection = self.mp_face.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Face and Gesture Node Started")
        
        # State tracking
        self.last_gesture = None
        self.gesture_cooldown = 0

    def is_thumbs_up(self, landmarks):
        """Detect thumbs up gesture"""
        # Thumb landmarks
        thumb_tip = landmarks.landmark[4]
        thumb_ip = landmarks.landmark[3]
        
        # Index finger landmarks
        index_tip = landmarks.landmark[8]
        index_pip = landmarks.landmark[6]
        
        # Thumb is extended (tip higher than IP joint)
        thumb_extended = thumb_tip.y < thumb_ip.y
        
        # Other fingers are closed
        fingers_closed = all(landmarks.landmark[i].y > landmarks.landmark[i-2].y 
                             for i in [12, 16, 20])  # Middle, ring, pinky
        
        return thumb_extended and fingers_closed

    def is_pointing(self, landmarks):
        """Detect pointing gesture"""
        # Index finger landmarks
        index_tip = landmarks.landmark[8]
        index_dip = landmarks.landmark[7]
        index_pip = landmarks.landmark[6]
        
        # Index finger extended
        index_extended = (index_tip.y < index_dip.y and 
                          index_dip.y < index_pip.y)
        
        # Other fingers closed
        middle_closed = landmarks.landmark[12].y > landmarks.landmark[10].y
        ring_closed = landmarks.landmark[16].y > landmarks.landmark[14].y
        pinky_closed = landmarks.landmark[20].y > landmarks.landmark[18].y
        
        return index_extended and middle_closed and ring_closed and pinky_closed

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        current_gesture = None
        
        # Face detection
        face_results = self.face_detection.process(rgb_frame)
        if face_results.detections:
            for detection in face_results.detections:
                bbox = detection.location_data.relative_bounding_box
                h, w, _ = frame.shape
                x, y = int(bbox.xmin * w), int(bbox.ymin * h)
                width, height = int(bbox.width * w), int(bbox.height * h)
                cv2.rectangle(frame, (x, y), (x+width, y+height), (0, 255, 0), 2)
        
        # Hand gesture recognition
        hand_results = self.hands.process(rgb_frame)
        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())
                
                # Detect gestures
                if self.is_thumbs_up(hand_landmarks):
                    current_gesture = "thumbs_up"
                    cv2.putText(frame, "THUMBS UP", (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                elif self.is_pointing(hand_landmarks):
                    current_gesture = "pointing"
                    cv2.putText(frame, "POINTING", (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Gesture publishing logic
        if current_gesture and current_gesture != self.last_gesture:
            gesture_msg = String()
            gesture_msg.data = current_gesture
            self.gesture_pub.publish(gesture_msg)
            self.last_gesture = current_gesture
            self.gesture_cooldown = 5  # Cooldown period (0.5 seconds)
        
        # Handle gesture end
        elif not current_gesture and self.last_gesture and self.gesture_cooldown <= 0:
            end_msg = String()
            end_msg.data = f"{self.last_gesture}_end"
            self.gesture_pub.publish(end_msg)
            self.last_gesture = None
        
        # Update cooldown counter
        if self.gesture_cooldown > 0:
            self.gesture_cooldown -= 1
        
        # Publish processed image
        processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaceGestureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
