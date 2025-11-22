#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_robot.msg import PersonInfo
import face_recognition
import numpy as np
import sys
import os

class PersonRecognitionNode(Node):
    def __init__(self):
        super().__init__('person_recognition_node')
        self.bridge = CvBridge()
        
        # Subscribe to processed images
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.image_callback,
            10
        )
        
        # Publisher for person information
        self.publisher = self.create_publisher(PersonInfo, 'person_info', 10)
        
        # Load YOLOv8 model (using Ultralytics implementation)
        self.model = YOLO('yolov8n.pt')
        self.class_names = self.model.names
        
        # Load known faces database
        self.known_faces = self.load_known_faces()
        
        self.get_logger().info("Person Recognition Node Started")

    def load_known_faces(self):
        known_faces = {}
        # Get the directory of this script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        known_faces_dir = os.path.join(script_dir, "known_faces")
        
        if not os.path.exists(known_faces_dir):
            self.get_logger().warning(f"Known faces directory not found: {known_faces_dir}")
            return known_faces
        
        for file in os.listdir(known_faces_dir):
            if file.lower().endswith((".jpg", ".jpeg", ".png")):
                name = os.path.splitext(file)[0]
                image_path = os.path.join(known_faces_dir, file)
                try:
                    image = face_recognition.load_image_file(image_path)
                    encodings = face_recognition.face_encodings(image)
                    if encodings:
                        known_faces[name] = encodings[0]
                        self.get_logger().info(f"Loaded face: {name}")
                    else:
                        self.get_logger().warning(f"No face found in {file}")
                except Exception as e:
                    self.get_logger().error(f"Error loading {file}: {str(e)}")
        
        self.get_logger().info(f"Loaded {len(known_faces)} known faces")
        return known_faces

    def recognize_person(self, face_image):
        # Detect faces in the image
        face_encodings = face_recognition.face_encodings(face_image)
        
        if not face_encodings:
            return "Unknown", "UNK001"
        
        # Compare with known faces
        unknown_encoding = face_encodings[0]
        
        # Convert known_faces dictionary to lists for comparison
        known_names = list(self.known_faces.keys())
        known_encodings = list(self.known_faces.values())
        
        # Compare faces with tolerance
        matches = face_recognition.compare_faces(known_encodings, unknown_encoding, tolerance=0.5)
        face_distances = face_recognition.face_distance(known_encodings, unknown_encoding)
        
        # Find the best match
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_names[best_match_index]
            return name, name.upper().replace("_", "")
        
        return "Unknown", "UNK001"

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run person detection (class 0 = person)
        results = self.model(cv_image, classes=[0], verbose=False)
        
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            
            for i, box in enumerate(boxes):
                if confidences[i] > 0.5 and class_ids[i] == 0:  # Valid person detection
                    x1, y1, x2, y2 = map(int, box[:4])
                    
                    # Crop the face region (using the full person bounding box for simplicity)
                    face_crop = cv_image[y1:y2, x1:x2]
                    
                    # Skip if crop is empty
                    if face_crop.size == 0:
                        continue
                    
                    # Recognize the person
                    name, id_num = self.recognize_person(face_crop)
                    
                    # Prepare and publish message
                    info_msg = PersonInfo()
                    info_msg.name = name
                    info_msg.id = id_num
                    info_msg.confidence = float(confidences[i])
                    info_msg.position.x = int((x1 + x2) / 2)
                    info_msg.position.y = int((y1 + y2) / 2)
                    info_msg.bbox.width = int(x2 - x1)
                    info_msg.bbox.height = int(y2 - y1)
                    
                    self.publisher.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
