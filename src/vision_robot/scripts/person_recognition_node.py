#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_robot.msg import PersonInfo
import face_recognition
import numpy as np
import os
import cv2
import time
from ament_index_python.packages import get_package_share_directory

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
        
        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')
        self.class_names = self.model.names
        
        # Get known_faces directory
        package_share_dir = get_package_share_directory('vision_robot')
        self.known_faces_dir = os.path.join(package_share_dir, 'known_faces')
        os.makedirs(self.known_faces_dir, exist_ok=True)
        
        # Load known faces database
        self.known_faces = self.load_known_faces()
        self.known_files = set(os.listdir(self.known_faces_dir))
        
        # Create timer for reloading faces
        self.reload_timer = self.create_timer(5.0, self.reload_known_faces)  # Check every 5 seconds
        
        self.get_logger().info("Person Recognition Node Started")
        self.get_logger().info(f"Loaded {len(self.known_faces)} known faces")

    def reload_known_faces(self):
        """Reload known faces if directory contents have changed"""
        current_files = set(os.listdir(self.known_faces_dir))
        
        # Check if new files have been added
        if current_files != self.known_files:
            self.get_logger().info("Detected changes in known_faces directory. Reloading...")
            self.known_faces = self.load_known_faces()
            self.known_files = current_files
            self.get_logger().info(f"Reloaded known faces. Now {len(self.known_faces)} faces.")

    def load_known_faces(self):
        """Load all known faces from directory"""
        known_faces = {}
        try:
            self.get_logger().info(f"Loading faces from: {self.known_faces_dir}")
            
            # Create directory if it doesn't exist
            os.makedirs(self.known_faces_dir, exist_ok=True)
            
            for file in os.listdir(self.known_faces_dir):
                if file.lower().endswith((".jpg", ".jpeg", ".png")):
                    # Extract base name without count or timestamp
                    base_name = file.split('_')[0]
                    
                    # Skip "Unknown" files
                    if base_name == "Unknown":
                        continue
                        
                    image_path = os.path.join(self.known_faces_dir, file)
                    try:
                        image = face_recognition.load_image_file(image_path)
                        encodings = face_recognition.face_encodings(image)
                        if encodings:
                            # Use the base name as the person's name
                            known_faces[base_name] = encodings[0]
                            self.get_logger().info(f"Loaded face: {base_name}")
                        else:
                            self.get_logger().warning(f"No face found in {file}")
                    except Exception as e:
                        self.get_logger().error(f"Error loading {file}: {str(e)}")
            
        except Exception as e:
            self.get_logger().error(f"Error loading known faces: {str(e)}")
        
        return known_faces

    def recognize_person(self, face_image):
        """Recognize a person from face image"""
        # Return immediately if no known faces
        if not self.known_faces:
            return "Unknown", "UNK001"
        
        try:
            # Convert color space if needed (face_recognition expects RGB)
            if face_image.shape[2] == 1:  # Grayscale
                rgb_image = cv2.cvtColor(face_image, cv2.COLOR_GRAY2RGB)
            elif face_image.shape[2] == 4:  # RGBA
                rgb_image = cv2.cvtColor(face_image, cv2.COLOR_BGRA2RGB)
            else:  # BGR
                rgb_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
            
            # Detect faces in the image
            face_encodings = face_recognition.face_encodings(rgb_image)
            
            if not face_encodings:
                return "Unknown", "UNK001"
            
            # Compare with known faces
            unknown_encoding = face_encodings[0]
            known_names = list(self.known_faces.keys())
            known_encodings = list(self.known_faces.values())
            
            # Handle empty known_encodings
            if not known_encodings:
                return "Unknown", "UNK001"
            
            matches = face_recognition.compare_faces(known_encodings, unknown_encoding, tolerance=0.5)
            face_distances = face_recognition.face_distance(known_encodings, unknown_encoding)
            
            # Handle empty face_distances
            if len(face_distances) == 0:
                return "Unknown", "UNK001"
            
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_names[best_match_index]
                return name, name.upper().replace(" ", "_")
            
        except Exception as e:
            self.get_logger().error(f"Recognition error: {str(e)}")
        
        return "Unknown", "UNK001"

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run person detection (class 0 = person)
            results = self.model(cv_image, classes=[0], verbose=False)
            
            # Process detections
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
                
                for i, box in enumerate(boxes):
                    if confidences[i] > 0.5 and class_ids[i] == 0:  # Valid person detection
                        x1, y1, x2, y2 = map(int, box[:4])
                        
                        # Ensure coordinates are within image bounds
                        h, w = cv_image.shape[:2]
                        x1 = max(0, min(x1, w-1))
                        y1 = max(0, min(y1, h-1))
                        x2 = max(0, min(x2, w-1))
                        y2 = max(0, min(y2, h-1))
                        
                        # Check for valid region
                        if x2 <= x1 or y2 <= y1:
                            continue
                            
                        # Crop the face region
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
                        info_msg.bbox_x = int((x1 + x2) / 2)
                        info_msg.bbox_y = int((y1 + y2) / 2)
                        info_msg.bbox_width = int(x2 - x1)
                        info_msg.bbox_height = int(y2 - y1)
                        
                        self.publisher.publish(info_msg)
                        
        except Exception as e:
            self.get_logger().error(f"Image callback error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PersonRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
